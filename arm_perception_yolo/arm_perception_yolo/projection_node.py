import yaml
import os
import numpy as np
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CameraInfo
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_msgs.msg import Header, String
import json
import cv2
import tf2_ros
import tf_transformations
from rclpy.duration import Duration
from typing import Optional, Dict, Any, List, Tuple

# Import MeasuredObject for dual-view triangulation results
try:
    from arm_interfaces.msg import MeasuredObject
    ARM_INTERFACES_AVAILABLE = True
except ImportError:
    ARM_INTERFACES_AVAILABLE = False
    MeasuredObject = None


class ProjectionNode(Node):
    """
    ProjectionNode - Unified result publishing node (simplified version)

    Features:
    - Process dual-view measurement results (MeasuredObject)
    - Process traditional single-view detection results (Detection2DArray)
    - Unified JSON format result publishing (/detected_objects_json)
    - Publish CollisionObject to MoveIt Scene

    Subscribed Topics:
    ---------
    /measured_objects : MeasuredObject
        Dual-view triangulation results (high precision)
    /detections_triggered : Detection2DArray
        Traditional single-view detection results (backward compatible)
    /camera/camera_info : CameraInfo
        Camera calibration information

    Published Topics:
    ---------
    /detected_objects_json : String
        Unified JSON output (includes single-view and dual-view)
    /collision_object : CollisionObject
        MoveIt Scene object

    """

    def __init__(self):
        super().__init__("projection_node")
        self.declare_parameter("config", "")
        cfg_path = self.get_parameter("config").value
        self.declare_parameter("detection_mode", "triggered")  # continuous | triggered
        self.declare_parameter("allowed_sites", ["front","left","right"])  # Allowed sites

        self.detection_mode = self.get_parameter("detection_mode").value
        self.allowed_sites = set(self.get_parameter("allowed_sites").value)
        self.cfg = {}

        if cfg_path and os.path.exists(cfg_path):
            with open(cfg_path, "r") as f:
                self.cfg = yaml.safe_load(f)
        table = self.cfg.get("table", {})
        self.table_h = table.get("height", 0.0)
        self.min_conf = self.cfg.get("filter", {}).get("min_confidence", 0.5)
        # Threshold for triggered-mode (allow lower confidence on one-shot captures)
        self.min_conf_triggered = self.cfg.get("filter", {}).get("min_conf_triggered", 0.1)
        # Expose as a parameter so launch files can override at runtime
        try:
            self.declare_parameter("min_conf_triggered", float(self.min_conf_triggered))
            self.min_conf_triggered = float(self.get_parameter("min_conf_triggered").value)
        except Exception:
            # Ignore parameter plumbing failures in limited environments
            pass
        self.avg_window = self.cfg.get("filter", {}).get("avg_window", 3)
        self.publish_box = self.cfg.get("scene", {}).get("publish_box", True)
        self.object_id = self.cfg.get("scene", {}).get("object_id", "detected_object")
        # vertical offset for grasp pose (meters). Positive value moves gripper down towards object.
        # Legacy absolute offset (meters)
        self.declare_parameter("grasp_z_offset", 0.02)
        try:
            self.grasp_z_offset = float(self.get_parameter("grasp_z_offset").value)
        except Exception:
            self.grasp_z_offset = 0.02
        # Preferred: percentage of object height to offset the grasp position (e.g. 0.2 == 20%)
        # If set, this will be used in preference to the legacy absolute offset.
        self.declare_parameter("grasp_z_offset_pct", 0.2)
        try:
            self.grasp_z_offset_pct = float(self.get_parameter("grasp_z_offset_pct").value)
        except Exception:
            self.grasp_z_offset_pct = 0.2
        
        # Position correction parameters (consistent with triangulation_node)
        self.declare_parameter('position_correction_x', 0.0)
        self.declare_parameter('position_correction_y', 0.0)
        self.declare_parameter('position_correction_z', 0.0)
        self.position_correction_x = self.get_parameter('position_correction_x').value
        self.position_correction_y = self.get_parameter('position_correction_y').value
        self.position_correction_z = self.get_parameter('position_correction_z').value
        
        self.get_logger().info(
            f"📐 Position correction loaded: "
            f"X={self.position_correction_x:+.3f}m, "
            f"Y={self.position_correction_y:+.3f}m, "
            f"Z={self.position_correction_z:+.3f}m"
        )
        
        self.default_w = (
            self.cfg.get("objects", {}).get("default", {}).get("width", 0.05)
        )
        self.default_h = (
            self.cfg.get("objects", {}).get("default", {}).get("height", 0.05)
        )
        self.class_map = self.cfg.get("objects", {}).get("classes", {})
        self.cache = []

        self.cam_info = None
        # Replace subscription: subscribe to different topics based on mode
        if self.detection_mode == "continuous":
            self.sub_det = self.create_subscription(
                Detection2DArray, "/detections", self.cb_det, 10
            )
        else:
            self.sub_det = self.create_subscription(
                Detection2DArray, "/detections_triggered", self.cb_det, 10
            )

        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.cb_info, 1
        )
        self.scene_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        # publish structured detection JSON for consumers (arm planner)
        self.detected_json_pub = self.create_publisher(String, "/detected_objects_json", 10)

        # Subscribe to dual-view measurement results
        if ARM_INTERFACES_AVAILABLE:
            self.sub_measured = self.create_subscription(
                MeasuredObject,
                '/measured_objects',
                self.cb_measured,
                10
            )
            self.get_logger().info("✅ Subscribe to /measured_objects (Dual-view measurement)")
            
            # For collecting batch measurement results
            self.measured_batch = []
            self.batch_timer = None
            self.batch_timeout = 0.5  # 500ms timeout, publish after collecting a batch of results
        else:
            self.sub_measured = None
            self.get_logger().warn("⚠️  arm_interfaces unavailable, dual-view measurement features disabled")

        # Optional: listen to site requests (can keep if you want strategy at this level)
        self.site_id = None
        self.create_subscription(String, "/detection_request", self.site_req_cb, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def site_req_cb(self, msg: String):
        self.site_id = msg.data.strip()
        self.get_logger().debug(f"site request received: {self.site_id}")

    # ----------------- small helper methods for readability -----------------
    def _get_score(self, res: Any) -> Optional[float]:
        """Robustly get a numeric score/confidence from a result object.

        Tries multiple common field names and nested hypothesis.
        Returns None if not present.
        """
        for a in ("score", "confidence", "prob"):
            if hasattr(res, a):
                try:
                    return float(getattr(res, a))
                except Exception:
                    return None
        hyp = getattr(res, "hypothesis", None)
        if hyp is not None:
            for a in ("score", "confidence", "prob"):
                if hasattr(hyp, a):
                    try:
                        return float(getattr(hyp, a))
                    except Exception:
                        return None
        return None

    def _get_id(self, res: Any) -> Optional[str]:
        """Extract an id/label value (string or numeric) from result/hypothesis."""
        for a in ("id", "label", "class_id", "name"):
            if hasattr(res, a):
                return getattr(res, a)
        hyp = getattr(res, "hypothesis", None)
        if hyp is not None:
            for a in ("id", "label", "class_id", "name"):
                if hasattr(hyp, a):
                    return getattr(hyp, a)
        return None

    def _get_center_and_size(self, detection: Any) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        """Extract bbox center u,v and bbox size in pixels (w_px,h_px) from detection."""
        bbox = getattr(detection, 'bbox', None)
        if bbox is None:
            return (None, None, None, None)
        center = getattr(bbox, 'center', None)
        u = v = None
        if center is not None:
            # direct x/y
            x = getattr(center, 'x', None)
            y = getattr(center, 'y', None)
            if x is not None and y is not None:
                try:
                    u = float(x)
                    v = float(y)
                except Exception:
                    u = v = None
            # position object
            if u is None or v is None:
                pos = getattr(center, 'position', None)
                if pos is not None and hasattr(pos, 'x') and hasattr(pos, 'y'):
                    try:
                        u = float(pos.x)
                        v = float(pos.y)
                    except Exception:
                        u = v = None
            # nested pose
            if u is None or v is None:
                pose = getattr(center, 'pose', None)
                if pose is not None and hasattr(pose, 'position'):
                    try:
                        u = float(pose.position.x)
                        v = float(pose.position.y)
                    except Exception:
                        u = v = None

        # size extraction
        w_px = h_px = None
        if hasattr(bbox, 'size_x') and hasattr(bbox, 'size_y'):
            try:
                w_px = float(getattr(bbox, 'size_x'))
                h_px = float(getattr(bbox, 'size_y'))
            except Exception:
                w_px = h_px = None
        if (w_px is None or h_px is None) and hasattr(bbox, 'size'):
            size = getattr(bbox, 'size')
            if hasattr(size, 'width') and hasattr(size, 'height'):
                try:
                    w_px = float(size.width)
                    h_px = float(size.height)
                except Exception:
                    w_px = h_px = None

        return (u, v, w_px, h_px)

    def _lookup_camera_transform(self, cam_frame: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Lookup TF from camera frame to base_link and return translation and rotation matrix.

        Returns (t, R) or None on failure.
        """
        try:
            tf = self.tf_buffer.lookup_transform("base_link", cam_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed (base_link <- {cam_frame}): {e}")
            return None
            
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        q = tf.transform.rotation
        
        # Check for invalid TF values
        if np.isnan(t).any() or np.isinf(t).any():
            self.get_logger().warn(f"TF translation vector contains invalid values (NaN/Inf): t={t}")
            return None
            
        if any(np.isnan(v) or np.isinf(v) for v in [q.x, q.y, q.z, q.w]):
            self.get_logger().warn(f"TF quaternion contains invalid values (NaN/Inf): q=({q.x}, {q.y}, {q.z}, {q.w})")
            return None
            
        R = tf_transformations.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        
        if np.isnan(R).any() or np.isinf(R).any():
            self.get_logger().warn(f"TF rotation matrix contains invalid values (NaN/Inf): R={R}")
            return None
            
        return t, R

    def _project_pixel_to_table(self, u: float, v: float, fx: float, fy: float, cx: float, cy: float, t: np.ndarray, R: np.ndarray) -> Optional[np.ndarray]:
        """Project a pixel (u,v) into base_link coordinates by intersecting ray with table plane.

        Returns 3D point (numpy array) or None.
        """
        xn, yn = self._pixel_to_normalized(u, v, fx, fy, cx, cy)
        ray_cam = np.array([xn, yn, 1.0])
        ray_base = R @ ray_cam
        
        # Debug: check for invalid values
        if np.isnan(ray_base).any() or np.isinf(ray_base).any():
            self.get_logger().warn(
                f"Projection failed: ray_base contains invalid values (NaN/Inf): ray_base={ray_base}, "
                f"xn={xn}, yn={yn}, u={u}, v={v}, fx={fx}, fy={fy}, cx={cx}, cy={cy}"
            )
            return None
            
        if abs(ray_base[2]) < 1e-6:
            self.get_logger().debug(f"Projection failed: ray_base[2] too small ({ray_base[2]}), Ray almost parallel to tabletop")
            return None
            
        s = (self.table_h - t[2]) / ray_base[2]
        
        # Debug: check scale factor
        if np.isnan(s) or np.isinf(s):
            self.get_logger().warn(
                f"Projection failed: scaling factor s invalid (NaN/Inf): s={s}, "
                f"table_h={self.table_h}, t[2]={t[2]}, ray_base[2]={ray_base[2]}"
            )
            return None
            
        if s <= 0:
            self.get_logger().debug(f"Projection failed: s <= 0 ({s}), Intersection behind camera")
            return None
            
        hit = t + s * ray_base
        
        # Final check: ensure result is valid
        if np.isnan(hit).any() or np.isinf(hit).any():
            self.get_logger().warn(
                f"Projection failed: final result contains invalid values (NaN/Inf): hit={hit}, "
                f"t={t}, s={s}, ray_base={ray_base}"
            )
            return None
            
        return hit

    def _pixel_to_normalized(self, u: float, v: float, fx: float, fy: float, cx: float, cy: float) -> Tuple[float, float]:
        """Return undistorted normalized image coordinates (xn, yn).

        Uses cached camera intrinsic matrix and distortion coefficients if available
        (set in cb_info). Falls back to simple (u-cx)/fx when undistort is unavailable.
        """
        try:
            if getattr(self, '_cam_K', None) is not None and getattr(self, '_cam_D', None) is not None:
                # undistortPoints expects float32/float64 arrays
                pt = np.array([[[u, v]]], dtype=np.float32)
                und = cv2.undistortPoints(pt, self._cam_K, self._cam_D)
                # und has shape (1,1,2) with normalized coords (x = X/Z)
                xn = float(und[0, 0, 0])
                yn = float(und[0, 0, 1])
                return xn, yn
        except Exception:
            # fall through to pinhole fallback
            pass
        # fallback: simple pinhole model
        try:
            xn = (u - cx) / fx
            yn = (v - cy) / fy
            return float(xn), float(yn)
        except Exception:
            return 0.0, 0.0

    def _extract_site_id(self, frame_id: str):
        if "|site:" in frame_id:
            return frame_id.split("|site:")[-1]
        return None

    def cb_info(self, msg: CameraInfo):
        self.cam_info = msg
        
        # Validate camera intrinsics
        if not hasattr(msg, 'k') or len(msg.k) < 9:
            self.get_logger().warn(f"Camera info missing or incomplete K matrix: frame_id={msg.header.frame_id}")
            return
            
        fx, fy = msg.k[0], msg.k[4]
        cx, cy = msg.k[2], msg.k[5]
        
        # Check for invalid intrinsics
        if fx == 0 or fy == 0:
            self.get_logger().warn(
                f"Camera intrinsics invalid: fx={fx}, fy={fy}, cx={cx}, cy={cy} (Focal length cannot be 0)"
            )
            return
            
        if any(np.isnan(v) or np.isinf(v) for v in [fx, fy, cx, cy]):
            self.get_logger().warn(
                f"Camera intrinsics contain NaN/Inf: fx={fx}, fy={fy}, cx={cx}, cy={cy}"
            )
            return
        
        self.get_logger().debug(
            f"Camera info received: frame_id={msg.header.frame_id}, "
            f"fx={fx:.1f}, fy={fy:.1f}, cx={cx:.1f}, cy={cy:.1f}"
        )
        
        # cache camera matrix and distortion coefficients for undistortion
        try:
            self._cam_K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
        except Exception as e:
            self.get_logger().warn(f"Unable to build camera matrix: {e}")
            self._cam_K = None
        try:
            # msg.d is a sequence of distortion coefficients
            self._cam_D = np.array(msg.d, dtype=np.float64) if hasattr(msg, 'd') else None
        except Exception as e:
            self.get_logger().warn(f"Unable to get distortion coefficients: {e}")
            self._cam_D = None

    # --- small helpers extracted from cb_det to improve readability ---
    def _parse_int_id(self, idv: Any) -> Optional[int]:
        try:
            return int(idv)
        except Exception:
            try:
                return int(str(idv))
            except Exception:
                return None

    def _choose_label_from_results(self, results: List[Any]) -> Optional[str]:
        """Prefer a textual label found in results or hypothesis; fall back to None."""
        for r in results or []:
            for a in ("label", "name", "class_name", "label_str"):
                if hasattr(r, a):
                    cand = getattr(r, a)
                    if cand is not None and (isinstance(cand, str) and not cand.isdigit()):
                        return cand
            hyp = getattr(r, "hypothesis", None)
            if hyp is not None:
                for a in ("label", "name", "class_name", "label_str"):
                    if hasattr(hyp, a):
                        cand = getattr(hyp, a)
                        if cand is not None and (isinstance(cand, str) and not cand.isdigit()):
                            return cand
                if hasattr(hyp, 'class_id'):
                    cand = getattr(hyp, 'class_id')
                    if cand is not None and isinstance(cand, str) and not cand.isdigit():
                        return cand
        return None

    def _build_detection_obj(self, det_item: Any, i: int, fx: float, fy: float, cx: float, cy: float,
                              t: np.ndarray, R: np.ndarray, depth_est: float, real_w: float, real_h: float) -> Optional[Dict[str, Any]]:
        """Build the JSON dict for a single detection or return None if skipped."""
        if not det_item.results:
            return None
        resi = det_item.results[0]
        conf_i = self._get_score(resi)
        per_det_th = self.min_conf_triggered if self.detection_mode == "triggered" else self.min_conf
        if conf_i is None or conf_i < per_det_th:
            self.get_logger().debug(
                f"skipping detection[{i}] conf={conf_i} below per-det-th={per_det_th}"
            )
            return None

        idv_i = self._get_id(resi)
        cls_i = self._parse_int_id(idv_i)
        if cls_i is None:
            return None

        label_i = self._choose_label_from_results(getattr(det_item, 'results', []))
        if label_i is None:
            label_i = str(cls_i)

        u_i, v_i, w_px_i, h_px_i = self._get_center_and_size(det_item)
        if u_i is None or v_i is None or w_px_i is None or h_px_i is None:
            return None

        # Use depth estimation method (consistent with main detection)
        # Calculate depth for this detection using its own bbox width
        depth_i = fx * real_w / w_px_i
        
        # Convert pixel coordinates to normalized coordinates
        xn_i = (u_i - cx) / fx
        yn_i = (v_i - cy) / fy
        
        # 3D position in camera coordinate system
        point_cam_i = np.array([xn_i * depth_i, yn_i * depth_i, depth_i])
        
        # Transform to base_link coordinate system (raw TF result)
        hit_i = R @ point_cam_i + t

        # size_score uses depth_est computed from primary detection (keeps previous behavior)
        size_score = min(max((w_px_i / fx) * depth_est / (real_w + 1e-6), 0.0), 1.0)
        grasp_quality = float(min(1.0, float(conf_i) * 0.7 + 0.3 * size_score))

        # determine grasp z offset: prefer percentage-of-height if configured and valid
        use_pct = getattr(self, 'grasp_z_offset_pct', None)
        if use_pct is not None and isinstance(use_pct, (int, float)) and use_pct >= 0:
            # clamp a reasonable range [0, 1]
            pct = min(max(float(use_pct), 0.0), 1.0)
            z_offset = real_h * pct
        else:
            # fallback to legacy absolute meters
            z_offset = getattr(self, 'grasp_z_offset', 0.02)

        # Raw position from TF transform (consistent with dual-view measurement)
        raw_x = float(hit_i[0])
        raw_y = float(hit_i[1])
        raw_z = float(hit_i[2])
        
        # Apply position correction for grasp pose (consistent with dual-view measurement)
        grasp_x = raw_x + self.position_correction_x
        grasp_y = raw_y + self.position_correction_y
        # For grasp Z: use TF result and apply z_offset and position_correction
        grasp_z = (raw_z - z_offset) + self.position_correction_z

        obj = {
            "class_id": int(cls_i),
            "label": str(label_i),
            "confidence": float(conf_i),
            # Raw position: TF transform result without correction
            "position": {"x": raw_x, "y": raw_y, "z": raw_z},
            "center_px": {"u": float(u_i), "v": float(v_i)},
            "bbox_px": {"w": float(w_px_i), "h": float(h_px_i)},
            "grasp_quality": grasp_quality,
            # Grasp pose: Corrected position for actual robot motion
            "grasp_pose": {
                "position": {
                    "x": grasp_x,
                    "y": grasp_y,
                    "z": grasp_z,
                },
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            },
        }
        return obj


    def cb_det(self, msg: Detection2DArray):
        # Diagnostic: log receipt and guard conditions early so we can see why we may not publish
        num_dets = len(msg.detections) if getattr(msg, 'detections', None) is not None else 0
        frame_id = getattr(msg.header, 'frame_id', '')
        self.get_logger().debug(
            f"cb_det called: frame_id={frame_id} detections={num_dets} cam_info_set={self.cam_info is not None}"
        )

        # Guard clauses
        if self.cam_info is None:
            self.get_logger().debug("cb_det: no camera info yet, skipping")
            return
        if not msg.detections:
            self.get_logger().debug("cb_det: no detections in message, skipping")
            return

        # Primary detection checks
        det0 = msg.detections[0]
        if not getattr(det0, 'results', None):
            self.get_logger().debug("cb_det: first detection has no results, skipping")
            return

        res0 = det0.results[0]
        conf = self._get_score(res0)
        if conf is None:
            self.get_logger().warn("cb_det: detection missing confidence field, skipping")
            return
        used_th = self.min_conf_triggered if self.detection_mode == "triggered" else self.min_conf
        self.get_logger().debug(f"cb_det: first-result confidence={conf} used_threshold={used_th} (mode={self.detection_mode})")
        if conf < used_th:
            self.get_logger().debug("cb_det: confidence below threshold, skipping")
            return

        site_id = self._extract_site_id(msg.header.frame_id)
        if self.detection_mode == "triggered" and site_id and site_id not in self.allowed_sites:
            self.get_logger().debug(f"Ignoring disallowed site={site_id}")
            return

        idv = self._get_id(res0)
        if idv is None:
            self.get_logger().warn("Detection result has no id/label field, skip")
            return
        cls_id = self._parse_int_id(idv)
        if cls_id is None:
            self.get_logger().warn(f"Unable to convert detection id to integer: {idv}")
            return

        # prefer textual label when available
        label_str = self._choose_label_from_results(getattr(det0, 'results', []))
        if label_str is None:
            label_str = idv if isinstance(idv, str) else str(cls_id)

        # camera intrinsics and transform
        fx = self.cam_info.k[0]
        fy = self.cam_info.k[4]
        cx = self.cam_info.k[2]
        cy = self.cam_info.k[5]
        cam_frame = self.cam_info.header.frame_id
        tf_res = self._lookup_camera_transform(cam_frame)
        if tf_res is None:
            return
        t, R = tf_res

        # project primary detection and update cache
        u0, v0, w_px0, h_px0 = self._get_center_and_size(det0)
        if u0 is None or v0 is None or w_px0 is None or h_px0 is None:
            self.get_logger().warn("Unable to extract bbox center/size from detection message, skip projection")
            return
            
        self.get_logger().debug(
            f"Main detection projection: label={label_str}, bbox_center=({u0:.1f},{v0:.1f}), "
            f"bbox_size=({w_px0:.1f}×{h_px0:.1f}), cam_frame={cam_frame}"
        )
        
        # Get object size configuration
        dims = self.class_map.get(cls_id, {})
        real_w = dims.get("width", self.default_w)
        
        # Calculate depth using depth estimation formula (based on calibrated focal length and real object width)
        depth_est = fx * real_w / w_px0
        real_h_est = depth_est * h_px0 / fy
        real_h = dims.get("height", real_h_est)
        
        # Use depth estimation to calculate 3D position (instead of ray-plane intersection)
        # Convert pixel coordinates to normalized coordinates
        xn = (u0 - cx) / fx
        yn = (v0 - cy) / fy
        
        # 3D position in camera coordinate system
        point_cam = np.array([xn * depth_est, yn * depth_est, depth_est])
        
        # Transform to base_link coordinate system
        hit0 = R @ point_cam + t
        
        self.get_logger().info(
            f"🎯 Depth estimation: label={label_str}, "
            f"pixels=({w_px0:.1f}×{h_px0:.1f}), "
            f"real_w={real_w:.3f}m, fx={fx:.1f}, "
            f"depth_est={depth_est:.3f}m, "
            f"point_cam=[{point_cam[0]:.3f}, {point_cam[1]:.3f}, {point_cam[2]:.3f}], "
            f"R=[[{R[0,0]:.2f},{R[0,1]:.2f},{R[0,2]:.2f}],"
            f"[{R[1,0]:.2f},{R[1,1]:.2f},{R[1,2]:.2f}],"
            f"[{R[2,0]:.2f},{R[2,1]:.2f},{R[2,2]:.2f}]], "
            f"t=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}], "
            f"hit0=[{hit0[0]:.3f}, {hit0[1]:.3f}, {hit0[2]:.3f}]"
        )
        
        self.cache.append(hit0)
        if len(self.cache) > self.avg_window:
            self.cache.pop(0)
        center = np.mean(self.cache, axis=0)
        
        # Apply position correction for CollisionObject (consistent with grasp_pose)
        center_corrected = np.array([
            center[0] + self.position_correction_x,
            center[1] + self.position_correction_y,
            center[2] + self.position_correction_z
        ])

        # Publish CollisionObject only in non-scan_all mode
        # scan_all mode uses site_id: front, left, right
        is_scan_all = site_id in ["front", "left", "right"]
        
        if self.publish_box and not is_scan_all:
            self._publish_collision_object(cls_id, label_str, center_corrected, real_w, real_h)

        # build JSON list
        try:
            arr: List[Dict[str, Any]] = []
            for i, det_item in enumerate(msg.detections):
                obj = self._build_detection_obj(det_item, i, fx, fy, cx, cy, t, R, depth_est, real_w, real_h)
                if obj is None:
                    continue
                arr.append(obj)
                # Also skip CollisionObject publish in scan_all mode during iteration
                # Use grasp_pose position (corrected) for CollisionObject
                if self.publish_box and not is_scan_all:
                    grasp_pos = obj["grasp_pose"]["position"]
                    self._publish_collision_object(
                        obj["class_id"], 
                        obj["label"], 
                        np.array([grasp_pos["x"], grasp_pos["y"], grasp_pos["z"]]), 
                        real_w, 
                        real_h, 
                        idx=i
                    )

            if arr:
                self._publish_detected_json(arr, site_id)
            else:
                self.get_logger().info("no detections passed per-detection threshold; not publishing JSON")
        except Exception as e:
            self.get_logger().warn(f"Unable to publish structured detection message (JSON): {e}")

        # Info log summarizing the primary detection
        self.get_logger().info(
            f"det cls={cls_id} conf={conf:.2f} center=({center[0]:.3f},{center[1]:.3f}) "
            f"depth≈{depth_est:.3f} size≈({real_w:.3f},{real_h:.3f}) "
            f"site_id: [site={site_id}]"
        )

    def _publish_collision_object(self, cls_id: int, label: str, center: np.ndarray, real_w: float, real_h: float, idx: Optional[int] = None) -> None:
        """Publish a moveit_msgs/CollisionObject for a detected item."""
        co = CollisionObject()
        label_for_id = label if label is not None else str(cls_id)
        if idx is None:
            co.id = f"{self.object_id}|class={cls_id}|label={label_for_id}"
        else:
            co.id = f"{self.object_id}|class={cls_id}|idx={idx}|label={label_for_id}"
        co.header = Header(frame_id="base_link")
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [real_w, real_w, real_h]
        pose = Pose()
        pose.position.x = float(center[0])
        pose.position.y = float(center[1])
        pose.position.z = float(center[2])  # Use passed Z coordinate (already includes position compensation)
        pose.orientation.w = 1.0
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        self.scene_pub.publish(co)

    def cb_measured(self, msg):
        """
        Process dual-view triangulation results (batch mode)
        
        Collect multiple MeasuredObject, then batch convert to JSON and publish at once
        """
        self.get_logger().debug(
            f"📏 Received measurement result: {msg.class_name} "
            f"Position=({msg.position.x:.3f}, {msg.position.y:.3f}, {msg.position.z:.3f})"
        )
        
        # Add to batch processing cache
        self.measured_batch.append(msg)
        
        # Reset timer (if timer exists, cancel it)
        if self.batch_timer is not None:
            self.batch_timer.cancel()
        
        # Create new timer: publish batch results after timeout
        self.batch_timer = self.create_timer(
            self.batch_timeout,
            self._publish_measured_batch
        )
    
    def _publish_measured_batch(self):
        """Batch publish all collected measurement results"""
        if self.batch_timer is not None:
            self.batch_timer.cancel()
            self.batch_timer = None
        
        if not self.measured_batch:
            return
        
        # Convert all MeasuredObject to JSON objects
        arr = []
        for msg in self.measured_batch:
            # Position from triangulation (already includes dynamic compensation and position_correction)
            grasp_x = float(msg.position.x)
            grasp_y = float(msg.position.y)
            grasp_z = float(msg.position.z)
            
            self.get_logger().info(
                f"� Using compensated position from triangulation: "
                f"grasp=({grasp_x:.4f}, {grasp_y:.4f}, {grasp_z:.4f})"
            )
            
            # Get dimensions for reference
            real_h = float(msg.dimensions.z)
            
            obj = {
                "class_id": int(msg.class_id),
                "label": str(msg.class_name),
                "confidence": float(msg.confidence),
                # Position: Same as grasp_pose (already compensated in triangulation_node)
                "position": {
                    "x": grasp_x,
                    "y": grasp_y,
                    "z": grasp_z
                },
                "dimensions": {
                    "width": float(msg.dimensions.x),
                    "depth": float(msg.dimensions.y),
                    "height": float(msg.dimensions.z)
                },
                "measured": True,  # Mark as precise measurement result
                "depth_from_camera": float(msg.depth_from_camera),
                "depth_error": float(msg.depth_error),
                "size_error": float(msg.size_error),
                "matching_score": float(msg.matching_score),
                "disparity": float(msg.disparity),
                "grasp_quality": float(msg.confidence),
                # Grasp pose: Corrected position for actual robot motion
                "grasp_pose": {
                    "position": {
                        "x": grasp_x,
                        "y": grasp_y,
                        "z": grasp_z,
                    },
                    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
                },
            }
            arr.append(obj)
            
            # Publish CollisionObject (use corrected grasp position)
            center = np.array([grasp_x, grasp_y, grasp_z])
            self._publish_collision_object(
                msg.class_id,
                msg.class_name,
                center,
                float(msg.dimensions.x),
                float(msg.dimensions.z),
                idx=None
            )
        
        # Publish all results at once
        self._publish_detected_json(arr, site="dual_view")
        
        self.get_logger().info(
            f"📦 Batch publish measurement results: {len(arr)} objects "
            f"({', '.join(obj['label'] for obj in arr)})"
        )
        
        # Clear cache
        self.measured_batch = []

    def _publish_detected_json(self, arr: List[Dict[str, Any]], site: Optional[str] = None) -> None:
        """Serialize detection list to JSON and publish on `/detected_objects_json` including site metadata."""
        payload = {
            "site": site,
            "detections": arr,
        }
        js = String()
        js.data = json.dumps(payload, ensure_ascii=False)
        if hasattr(self, 'detected_json_pub') and self.detected_json_pub is not None:
            self.detected_json_pub.publish(js)
            # emit an info log so default INFO-level readers see that JSON was published
            try:
                sample = arr[0] if arr else None
                self.get_logger().info(f"published detected_objects_json count={len(arr)} sample={sample} site={site}")
            except Exception:
                self.get_logger().info(f"published detected_objects_json count={len(arr)} site={site}")


def main():
    rclpy.init()
    rclpy.spin(ProjectionNode())
    rclpy.shutdown()
