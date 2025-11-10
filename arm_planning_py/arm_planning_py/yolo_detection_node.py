#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from threading import Thread
from typing import Optional, Tuple, Any, Dict
import sys, argparse, time
from geometry_msgs.msg import Pose, Point, Quaternion
from typing import Union, Tuple, List
import math
import tf_transformations
from .arm_grasper import ExecutionMode, ArmGrasper, ArmState
from .robots import lododo_arm

try:
    from .robots.lododo_arm import init_scan_pose as _init_scan_pose
except Exception:
    _init_scan_pose = None


class YoloDetectionNode(ArmGrasper):
    """Simple node that subscribes to `/detected_objects_json` and triggers processing for optimal target.

    Design principle: Callback returns as quickly as possible, actual execution runs in background thread. This avoids
    ROS callback queue delay caused by MoveIt blocking.
    """

    def __init__(self, node_name="yolo_detection_node", is_fast_robust_plan=True):
        super().__init__(node_name=node_name, is_fast_robust_plan=is_fast_robust_plan)
        # Subscribe to JSON topic
        self.subscription = self.create_subscription(
            String, "/detected_objects_json", self.cb, 10
        )

        # Publish detection request (trigger YOLO execution)
        self._detection_request_pub = self.create_publisher(
            String, "/detection_request", 10
        )

        # When executing scan, cb only writes to self._scan_result, avoiding triggering grasp workflow
        self._scan_mode = False
        self._scan_result = None

        self.get_logger().info("Yolo detection node started; waiting for detections...")

    def set_scan_mode(self, enable: bool):
        """Set scan mode flag."""
        self._scan_mode = enable

    def get_scan_result(self):
        """Get scan result cache."""
        return self._scan_result

    def trigger_detection(self, site_id: str = "front"):
        """
        Trigger YOLO detection via topic
        
        Args:
            site_id: Station ID (view1/view2/front, etc)
        """
        msg = String()
        msg.data = site_id
        self._detection_request_pub.publish(msg)
        self.get_logger().info(f"📡 Detection triggered: {site_id}")
    
    def _wait_for_joint_state(self, timeout: float = 5.0) -> bool:
        """
        Wait for joint_state available
        
        Resolves issue: pymoveit2 joint_state subscription has delay,
                  need to wait for state to be available before calling move_to_configuration
        
        Args:
            timeout: Wait timeout duration (seconds)
            
        Returns:
            True if joint_state available, False if timeout
        """
        import time
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            js = getattr(self.arm, "joint_state", None)
            if js is not None and hasattr(js, 'position') and len(js.position) > 0:
                self.get_logger().debug("✅ joint_state is available")
                return True
            
            # Wait a short time for subscription to update
            time.sleep(0.05)
            try:
                rclpy.spin_once(self, timeout_sec=0.01)
            except:
                pass
        
        self.get_logger().warn(f"⚠️  Waiting for joint_state timeout ({timeout}s)")
        return False

    def scan_tri_view(
        self,
        angle_offset: float = 15.0,
        timeout: float = 10.0,
        wait_between_views: float = 2.0
    ) -> List[Dict[str, Any]]:
        """
        Tri-view scan - Intelligent scan to improve recognition rate and accuracy
        
        Workflow:
        1. View1: Front position (joint1 = 0°)
        2. View2: -Y direction (joint1 = -15°)  
        3. View3: +Y direction (joint1 = +15°)
        4. Intelligently select best 2 views for triangulation
        
        Advantages:
        - Improve recognition success rate: Even if one view fails, other views may succeed
        - Improve measurement accuracy: Can select 2 views with best quality
        
        Args:
            angle_offset: View offset angle (degrees), default 15 degrees
            timeout: Detection timeout for each view (seconds)
            wait_between_views: Wait time between views (seconds)
            
        Returns:
            Measurement result list
        """
        self.get_logger().info(
            f"🔍 Starting tri-view scan: angle={angle_offset}°, timeout={timeout}s"
        )
        
        # 🔧 Clear old scan results to prevent using cached data
        self._scan_result = None
        self.set_scan_mode(True)
        
        # 🔧 Synchronize MoveIt state
        try:
            self._ensure_start_state_current()
            self.get_logger().info("✅ MoveIt state synchronized")
        except Exception as e:
            self.get_logger().warn(f"⚠️  State synchronization failed: {e}")
        
        results = []
        
        # ===== View 1: Front (joint1 = 0°) =====
        self.get_logger().info("🎥 View 1: Moving to front position (joint1=0°)")
        try:
            # Wait for joint_state available
            self._wait_for_joint_state()
            
            if _init_scan_pose is not None:
                view1_cfg = list(_init_scan_pose())
                max_abs = max(abs(a) for a in view1_cfg)
                if max_abs > 2 * math.pi:
                    view1_cfg = [math.radians(a) for a in view1_cfg]
                
                self.arm.move_to_configuration(joint_positions=list(view1_cfg))
                self.arm.wait_until_executed()
            else:
                self.move_to_named_target("front_scan")
            
            # Wait for camera to stabilize and allow user to view image in RViz
            # At 15fps: 1 second = 15 frames, ensures stable image capture and visualization
            # Using _sleep_with_spin to keep ROS callbacks active during wait
            self._sleep_with_spin(1.0)
        except Exception as e:
            self.get_logger().error(f"Move to view1 failed: {e}")
            return []
        
        # trigger view1 detection
        self.trigger_detection("view1")
        time.sleep(wait_between_views)
        
        # ===== view 2: -Y direction (joint1 = -angle_offset°) =====
        self.get_logger().info(f"🎥 view 2: Rotate to -Y direction (joint1=-{angle_offset}°)")
        
        try:
            # Wait for joint_state available
            self._wait_for_joint_state()
            self._ensure_start_state_current()
        except Exception as e:
            self.get_logger().warn(f"⚠️  view2 State synchronization failed: {e}")
        
        try:
            if _init_scan_pose is not None:
                view2_cfg = list(_init_scan_pose())
                max_abs = max(abs(a) for a in view2_cfg)
                if max_abs > 2 * math.pi:
                    view2_cfg = [math.radians(a) for a in view2_cfg]
                
                # Rotate joint1 to negative direction
                view2_cfg[0] = view2_cfg[0] - math.radians(angle_offset)
                
                self.get_logger().info(
                    f"view2joint configuration: joint1={math.degrees(view2_cfg[0]):.1f}°"
                )
                
                self.arm.move_to_configuration(joint_positions=list(view2_cfg))
                self.arm.wait_until_executed()
                
                # Wait for camera to stabilize and allow user to view image in RViz
                # Using _sleep_with_spin to keep ROS callbacks active during wait
                self._sleep_with_spin(1.0)
            else:
                self.get_logger().warn("scan_pose not defined, skippingview2")
        except Exception as e:
            self.get_logger().error(f"Move to view2 failed: {e}")
        
        # trigger view2 detection
        self.trigger_detection("view2")
        time.sleep(wait_between_views)
        
        # ===== view 3: +Y direction (joint1 = +angle_offset°) =====
        self.get_logger().info(f"🎥 view 3: Rotate to +Y direction (joint1=+{angle_offset}°)")
        
        try:
            # Wait for joint_state available
            self._wait_for_joint_state()
            self._ensure_start_state_current()
        except Exception as e:
            self.get_logger().warn(f"⚠️  view3State synchronization failed: {e}")
        
        try:
            if _init_scan_pose is not None:
                view3_cfg = list(_init_scan_pose())
                max_abs = max(abs(a) for a in view3_cfg)
                if max_abs > 2 * math.pi:
                    view3_cfg = [math.radians(a) for a in view3_cfg]
                
                # Rotate joint1 to positive direction
                view3_cfg[0] = view3_cfg[0] + math.radians(angle_offset)
                
                self.get_logger().info(
                    f"view3joint configuration: joint1={math.degrees(view3_cfg[0]):.1f}°"
                )
                
                self.arm.move_to_configuration(joint_positions=list(view3_cfg))
                self.arm.wait_until_executed()
                
                # Wait for camera to stabilize and allow user to view image in RViz
                # Using _sleep_with_spin to keep ROS callbacks active during wait
                self._sleep_with_spin(1.0)
            else:
                self.get_logger().warn("scan_pose not defined, skippingview3")
        except Exception as e:
            self.get_logger().error(f"Move to view3 failed: {e}")
        
        # trigger view3 detection (this will trigger TriangulationNode intelligent pairing)
        self.trigger_detection("view3")
        
        # waitingmeasurementresult
        self.get_logger().info(f"⏳ waiting for tri-view measurement result (timeout {timeout}s)...")
        start_time = time.time()
        result = None
        
        while time.time() - start_time < timeout:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                pass
            
            if self._scan_result is not None:
                result = self._scan_result
                self.get_logger().info("✅ tri-view measurement complete")
                break
            
            time.sleep(0.01)
        
        self.set_scan_mode(False)
        
        if result is None:
            self.get_logger().warn(f"⚠️  tri-view measurement timeout ({timeout}s)")
        else:
            # extractmeasurementresult
            raw = result.get("raw")
            if isinstance(raw, dict):
                detections = raw.get("detections", [])
                for det in detections:
                    if det.get("measured") is True:
                        results.append(det)
                        self.get_logger().info(
                            f"📏 measured object: {det.get('label')} @ "
                            f"({det['position']['x']:.3f}, {det['position']['y']:.3f}, {det['position']['z']:.3f})"
                        )
        
        # ===== returnfront_scanposition =====
        self.get_logger().info("🔙 returnfront_scanposition")
        
        try:
            self._ensure_start_state_current()
        except Exception as e:
            self.get_logger().warn(f"⚠️  State synchronization before return failed: {e}")
        
        try:
            if _init_scan_pose is not None:
                front_cfg = _init_scan_pose()
                if isinstance(front_cfg, (list, tuple)):
                    max_abs = max(abs(a) for a in front_cfg)
                    if max_abs > 2 * math.pi:
                        front_cfg = [math.radians(a) for a in front_cfg]
                    self.arm.move_to_configuration(joint_positions=list(front_cfg))
                    self.arm.wait_until_executed()
            else:
                self.move_to_named_target("front_scan")
        except Exception as e:
            self.get_logger().warn(f"returnfront_scanpositionfailed: {e}")
        
        self.get_logger().info(f"✅ tri-view scan complete，measured {len(results)}  objects")
        return results

    def cb(self, msg: String):
        """Process received JSON: parse -> select optimal target -> process in background thread."""
        self.get_logger().info(
            "Yolo detection parsing detection JSON...msg[length]:" + str(len(msg.data))
        )
        try:
            arr = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Unable to parse detection JSON: {e}")
            return
        if not arr:
            return

        # Parse and try to get all matching targets (return multiple poses)
        site, poses = self.parse_all_detected_objects_json(arr, "bottle")

        if (not poses) or site is None:
            self.get_logger().warn("Valid target pose or request ID not found")
            return

        # If in scan mode, only write result to cache and return (will not trigger grasp)
        if self._scan_mode:
            self.get_logger().info("Received detection results in scan mode, writing to cache and returning")
            # New structure: poses list and original raw preserved
            self._scan_result = {
                "request_id": site,
                "poses": poses,
                "raw": arr,
            }
            return

        # # Non-scan mode: continue executing existing grasp workflow
        # self.request_id, self.detected_pose = self.request_id, detected_pose
        # # TODO self.place_pose not yet configured
        # self.place_pose = None
        # self._yolo_grasper(self.detected_pose, self.place_pose, wait_time=2.0)

    def scan_views(
        self,
        per_view_timeout: float = 10.0,
        max_attempts_per_view: int = 3,
        return_to_front_between: bool = True,
        stop_on_detection: bool = True,
        front_only: bool = False,
    ) -> List[dict]:
        """
        Perform scan in three views (front/right/left) and return detection results list。

        Return value: list of { 'view_yaw': deg, 'pose': Pose, 'detection': dict | None, 'success': bool }
        """
        results = []

        # Prioritize using robots.lododo_arm.init_scan_pose() returned joint array as front configuration
        front_cfg = None
        if _init_scan_pose is not None:
            try:
                maybe_cfg = _init_scan_pose()
                if isinstance(maybe_cfg, (list, tuple)) and len(maybe_cfg) > 0:
                    front_cfg = [float(x) for x in maybe_cfg]
            except Exception:
                front_cfg = None

        # If got joint configuration, use joint space move; otherwise fallback to original Cartesian strategy
        if front_cfg is None:
            self.get_logger().warn(
                "init_scan_pose did not return joint configuration, unable to execute joint space scan: return empty result，please check robots.lododo_arm.init_scan_pose"
            )
            return []

        # Detect units: if values look like degrees (e.g. > 2*pi), convert to radians
        max_abs = max(abs(a) for a in front_cfg)
        if max_abs > 2 * math.pi:
            self.get_logger().info("Detected init_scan_pose returned angle looks like degrees, converting to radians")
            front_cfg = [math.radians(a) for a in front_cfg]

        # Ensure joint vector length matches robot joints
        names = lododo_arm.arm_joint_names()
        if len(front_cfg) != len(names):
            self.get_logger().warn(
                f"init_scan_pose returned joint count ({len(front_cfg)}) and robot joint count ({len(names)}) mismatch, attemptingtempting to truncate/pad"
            )
            # truncate or pad with zeros
            if len(front_cfg) > len(names):
                front_cfg = front_cfg[: len(names)]
            else:
                front_cfg = front_cfg + [0.0] * (len(names) - len(front_cfg))

        # pre-move: Return to safe height/configuration of front_cfg
        try:
            self.get_logger().info("Move to initial scan joint configuration (front)")
            self.arm.move_to_configuration(joint_positions=list(front_cfg))
            self.arm.wait_until_executed()
        except Exception as e:
            self.get_logger().warn(f"Move to initial joint configuration failed: {e}")

        offset_rad = math.radians(60.0)  # User requested ±60° on joint1

        # Construct view sequence
        if return_to_front_between:
            seq = [("front", 0), ("left", +1), ("front_return", 0), ("right", -1)]
        else:
            seq = [("front", 0), ("left", +1), ("right", -1)]

        for cmd_name, _direction in seq:
            view = {
                "view_cmd": cmd_name,
                "view_yaw": None,
                "pose": None,
                "joint_positions": None,
                "detections": None,  # Changed to plural form, consistent with later assignment
                "poses": None,  # Add poses field
                "success": False,
            }

            # compute joint target
            if cmd_name == "front" or cmd_name == "front_return":
                target_cfg = list(front_cfg)
                view["view_yaw"] = 0.0
            elif cmd_name == "left":
                target_cfg = list(front_cfg)
                target_cfg[0] = target_cfg[0] + offset_rad
                view["view_yaw"] = math.degrees(offset_rad)
            else:  # right
                target_cfg = list(front_cfg)
                target_cfg[0] = target_cfg[0] - offset_rad
                view["view_yaw"] = -math.degrees(offset_rad)

            self.get_logger().info(f"scanview {cmd_name} -> joint target[0]={target_cfg[0]:.3f}")

            moved = False
            for attempt in range(max_attempts_per_view):
                try:
                    self.arm.move_to_configuration(joint_positions=list(target_cfg))
                    self.arm.wait_until_executed()
                    moved = True
                    view["joint_positions"] = list(target_cfg)
                    time.sleep(0.2)
                    break
                except Exception as e:
                    self.get_logger().warn(f"Joint space move failed attempt={attempt}: {e}")
                    time.sleep(0.1)

            if not moved:
                self.get_logger().warn(f"view {cmd_name} unreachable, skipping detection")
                results.append(view)
                continue

            # If return step, skip detection publish and waiting
            if cmd_name == "front_return":
                self.get_logger().info("Returned to front, skipping this detection publish")
                continue

            # trigger YOLO detection request（carrying direction command）
            self._scan_result = None
            try:
                req = String()
                req.data = cmd_name
                self._detection_request_pub.publish(req)
                self.get_logger().info(f"Published detection request '{cmd_name}'")
            except Exception as e:
                self.get_logger().warn(f"publish detection request failed: {e}")

            # Waiting for detection results
            start = time.time()
            detected = None
            while time.time() - start < per_view_timeout:
                try:
                    rclpy.spin_once(self, timeout_sec=0.05)
                except Exception:
                    pass
                if self._scan_result is not None:
                    detected = self._scan_result
                    break
                time.sleep(0.01)

            if detected is not None:
                raw = detected.get("raw")
                returned_cmd = None
                if isinstance(raw, dict):
                    returned_cmd = raw.get("command") or raw.get("cmd") or raw.get("request")
                self.get_logger().info(
                    f"view {cmd_name} detected target: request_id={detected.get('request_id')} returned_cmd={returned_cmd}"
                )
                # New structure: Expose complete detections list and parsed poses list in view
                view["detections"] = raw.get("detections") if isinstance(raw, dict) else None
                view["poses"] = detected.get("poses")
                view["success"] = True
            else:
                self.get_logger().info(f"view {cmd_name} target not detected (timeout {per_view_timeout}s)")

            results.append(view)
            # If required to stop scan upon detecting target, break loop
            if stop_on_detection and view.get("success"):
                self.get_logger().info(f"view {cmd_name} target detected, stopping scan")
                break
            # If required to scan only front view, break loop
            if front_only:
                self.get_logger().info(f"Only scan front view {cmd_name} ，stopping scan")
                break

        # Return to front_cfg
        try:
            self.arm.move_to_configuration(joint_positions=list(front_cfg))
            self.arm.wait_until_executed()
        except Exception:
            pass

        return results

    def _extract_pose_from_dict(self, p: Dict[str, Any]) -> Optional[Pose]:
        """
        Accept common grasp_pose layouts and convert to geometry_msgs.msg.Pose.
        Supported layouts:
        - {"position": {"x":..., "y":..., "z":...}, "orientation": {"x":..., "y":..., "z":..., "w":...}}
        - {"pose": {"position": {...}, "orientation": {...}}}
        - flat keys: {"x":..., "y":..., "z":..., "qx":..., "qy":..., "qz":..., "qw":...}
        - flat keys: {"position_x":..., ...} (attempt some common variants)
        Returns Pose or None if cannot parse.
        """
        if p is None:
            return None

        # helper to read nested values safely
        def get_num(d, *keys, default=0.0):
            for k in keys:
                if isinstance(d, dict) and k in d:
                    return float(d[k])
            return default

        # Try nested shapes
        pos = None
        ori = None
        if "pose" in p and isinstance(p["pose"], dict):
            pos = p["pose"].get("position") or p["pose"].get("pos")
            ori = p["pose"].get("orientation") or p["pose"].get("quat")
        elif "position" in p or "orientation" in p:
            pos = p.get("position")
            ori = p.get("orientation")
        else:
            # flat variants
            pos = {
                "x": get_num(p, "x", "pos_x", "position_x"),
                "y": get_num(p, "y", "pos_y", "position_y"),
                "z": get_num(p, "z", "pos_z", "position_z"),
            }
            ori = {
                "x": get_num(p, "qx", "ox", "orientation_x"),
                "y": get_num(p, "qy", "oy", "orientation_y"),
                "z": get_num(p, "qz", "oz", "orientation_z"),
                "w": get_num(p, "qw", "ow", "orientation_w"),
            }

        # If pos/orientation are dict-like, read fields
        try:
            x = (
                float(pos.get("x"))
                if isinstance(pos, dict) and "x" in pos
                else (
                    float(pos[0])
                    if isinstance(pos, (list, tuple)) and len(pos) >= 1
                    else float(pos)
                )
            )
        except Exception:
            x = None
        try:
            y = (
                float(pos.get("y"))
                if isinstance(pos, dict) and "y" in pos
                else (
                    float(pos[1])
                    if isinstance(pos, (list, tuple)) and len(pos) >= 2
                    else None
                )
            )
        except Exception:
            y = None
        try:
            z = (
                float(pos.get("z"))
                if isinstance(pos, dict) and "z" in pos
                else (
                    float(pos[2])
                    if isinstance(pos, (list, tuple)) and len(pos) >= 3
                    else None
                )
            )
        except Exception:
            z = None

        # orientation
        try:
            ox = (
                float(ori.get("x"))
                if isinstance(ori, dict) and "x" in ori
                else (
                    float(ori[0])
                    if isinstance(ori, (list, tuple)) and len(ori) >= 1
                    else None
                )
            )
        except Exception:
            ox = None
        try:
            oy = (
                float(ori.get("y"))
                if isinstance(ori, dict) and "y" in ori
                else (
                    float(ori[1])
                    if isinstance(ori, (list, tuple)) and len(ori) >= 2
                    else None
                )
            )
        except Exception:
            oy = None
        try:
            oz = (
                float(ori.get("z"))
                if isinstance(ori, dict) and "z" in ori
                else (
                    float(ori[2])
                    if isinstance(ori, (list, tuple)) and len(ori) >= 3
                    else None
                )
            )
        except Exception:
            oz = None
        try:
            ow = (
                float(ori.get("w"))
                if isinstance(ori, dict) and "w" in ori
                else (
                    float(ori[3])
                    if isinstance(ori, (list, tuple)) and len(ori) >= 4
                    else None
                )
            )
        except Exception:
            ow = None

        # If any required value missing, abort
        if x is None or y is None or z is None:
            return None
        # orientation fallback to identity quaternion if missing
        if ox is None or oy is None or oz is None or ow is None:
            ox, oy, oz, ow = 0.0, 0.0, 0.0, 1.0

        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=ox, y=oy, z=oz, w=ow)
        return pose

    def parse_all_detected_objects_json(
        self, json_payload: Any, label_name: str
    ) -> Tuple[Optional[str], List[Pose]]:
        """
        Parse detection JSON and return (site, [Pose,...]) for all detections that match label_name.
        If none found, returns (site, []). This keeps backwards compatibility while exposing
        all parsed poses for a view.
        """
        poses: List[Pose] = []
        # normalize input
        if isinstance(json_payload, str):
            try:
                data = json.loads(json_payload)
            except Exception:
                return None, []
        elif isinstance(json_payload, dict):
            data = json_payload
        else:
            return None, []

        site = data.get("site")
        detections = data.get("detections", [])
        if not isinstance(detections, list):
            return site, []

        for det in detections:
            if not isinstance(det, dict):
                continue
            det_label = (
                det.get("label") or det.get("class_name") or str(det.get("class_id", ""))
            )
            # preserve original behavior: accept any non-none label or exact match
            if det_label == label_name or det_label is not None:
                grasp = (
                    det.get("grasp_pose")
                    or det.get("pose")
                    or det.get("grasp")
                    or det.get("grasp_pose_world")
                )
                if (
                    isinstance(grasp, dict)
                    and "pose" in grasp
                    and isinstance(grasp["pose"], dict)
                ):
                    grasp = grasp["pose"]
                pose = self._extract_pose_from_dict(grasp) if grasp is not None else None
                if pose is not None:
                    poses.append(pose)

        return site, poses

    def grasping_for_scan_rs(self, grasp_pose, place_pose: Pose, wait_time=2.0):

        self.get_logger().info("Preparing to execute grasp。。。。。。。。。。")

        if not grasp_pose:
            self.get_logger().warn("Grasp pose not detected, unable to execute grasp")
            return False

        xyz = pose_to_tuple(grasp_pose)
        self.get_logger().info(f"Objects detected target pose (x,y,z) = {xyz}")

        self._yolo_grasper(grasp_pose, place_pose, wait_time)

        return True

    def grasp_and_lift(self, grasp_pose: Pose, lift_height: float = 0.15) -> bool:
        """
        Grasp object and lift to specified height
        
        :param grasp_pose: object grasp pose
        :param lift_height: Lift height (meters), default 15cm
        :return: successfulreturnTrue，failedreturnFalse
        """
        try:
            if not grasp_pose:
                self.get_logger().warn("Grasp pose not provided")
                return False
            
            xyz = pose_to_tuple(grasp_pose)
            self.get_logger().info(f"startgraspobject，position: (x,y,z) = {xyz}")
            
            # Return to home position
            self.get_logger().info("Return to home position...")
            self.go_to_home_position()
            # Use _sleep_with_spin to keep ROS callbacks active during wait
            # This ensures joint_states continue updating for next move
            self._sleep_with_spin(2.0)
            
            # Open gripper
            self._gripper_control(close=False)
            self._sleep_with_spin(1.0)
            
            # Move to pre-grasp position (above object)
            pre_grasp_offset = [0.0, 0.0, 0.10]
            self.get_logger().info(f"Move to pre-grasp position, offset: {pre_grasp_offset}")
            self._execute_move(grasp_pose, pre_grasp_offset)
            self._sleep_with_spin(1.5)
            
            # Descend to grasp position
            self.get_logger().info("Descend to grasp position...")
            grasp_offset = [0.0, 0.0, 0.05]
            self._execute_move(grasp_pose, grasp_offset)
            self._sleep_with_spin(1.0)
            
            # Close gripper
            self.get_logger().info("Close gripper...")
            self._gripper_control(close=True)
            self.is_grasped = True
            self._sleep_with_spin(2.0)
            
            # Lift object
            lift_pose = Pose()
            lift_pose.position.x = grasp_pose.position.x
            lift_pose.position.y = grasp_pose.position.y
            lift_pose.position.z = grasp_pose.position.z + lift_height
            lift_pose.orientation = grasp_pose.orientation
            
            self.get_logger().info(f"Lift object to height: {lift_height}m")
            self._execute_move(lift_pose, [0.0, 0.0, 0.0])
            self._sleep_with_spin(1.5)
            
            self.get_logger().info("grasp and lift complete")
            return True
            
        except Exception as e:
            self.get_logger().error(f"grasp and lift failed: {e}")
            return False

    def wait_for_hand_detection(self, timeout: float = 30.0, scan_label: str = "hand") -> Optional[Pose]:
        """
        Wait to detect hand position
        
        :param timeout: timeout duration (seconds)
        :param scan_label: Label name to detect, default "hand"
        :return: Detected hand pose, return None on timeout
        """
        self.get_logger().info(f"Start waiting to detect hand, timeout duration: {timeout} seconds")
        
        start_time = time.time()
        self._scan_result = None
        
        # Continuously publish detection requests and wait for result
        while time.time() - start_time < timeout:
            # Publish detection request
            try:
                req = String()
                req.data = "detect_hand"
                self._detection_request_pub.publish(req)
                self.get_logger().info("Published hand detection request")
            except Exception as e:
                self.get_logger().warn(f"Publish detection requestfailed: {e}")
            
            # Wait a moment for detection system to process
            check_start = time.time()
            while time.time() - check_start < 3.0:  # Wait 3 seconds for each detection
                try:
                    rclpy.spin_once(self, timeout_sec=0.05)
                except Exception:
                    pass
                
                if self._scan_result is not None:
                    # Check if hand detected
                    raw = self._scan_result.get("raw")
                    if isinstance(raw, dict):
                        detections = raw.get("detections", [])
                        for det in detections:
                            if not isinstance(det, dict):
                                continue
                            det_label = det.get("label") or det.get("class_name") or ""
                            if scan_label.lower() in det_label.lower():
                                # Found hand, extract pose
                                grasp = det.get("grasp_pose") or det.get("pose") or det.get("position")
                                hand_pose = self._extract_pose_from_dict(grasp) if grasp else None
                                
                                if hand_pose is not None:
                                    xyz = pose_to_tuple(hand_pose)
                                    self.get_logger().info(f"Detected hand position: {xyz}")
                                    return hand_pose
                    
                    # Clear result, prepare for next detection
                    self._scan_result = None
                
                time.sleep(0.1)
            
            # Not detected, continue loop
            elapsed = time.time() - start_time
            self.get_logger().info(f"Hand not detected, continue waiting... (elapsed time: {elapsed:.1f}s)")
        
        self.get_logger().warn(f"Waiting for hand detection timeout ({timeout} seconds)")
        return None

    def deliver_to_hand(self, hand_pose: Pose, offset_above: float = 0.10) -> bool:
        """
        Deliver object above hand and release
        
        :param hand_pose: hand position pose
        :param offset_above: Offset height above hand (meters), default 10cm
        :return: successfulreturnTrue，failedreturnFalse
        """
        try:
            if not hand_pose:
                self.get_logger().warn("Hand pose not provided")
                return False
            
            hand_xyz = pose_to_tuple(hand_pose)
            self.get_logger().info(f"Preparing to deliver to hand position: {hand_xyz}")
            
            # Create delivery position (above hand)
            delivery_pose = Pose()
            delivery_pose.position.x = hand_pose.position.x
            delivery_pose.position.y = hand_pose.position.y
            delivery_pose.position.z = hand_pose.position.z + offset_above
            delivery_pose.orientation = hand_pose.orientation
            
            delivery_xyz = pose_to_tuple(delivery_pose)
            self.get_logger().info(f"Delivery position (above hand{offset_above}m）: {delivery_xyz}")
            
            # Move above hand
            self.get_logger().info("Move above hand...")
            self._execute_move(delivery_pose, [0.0, 0.0, 0.0])
            self._sleep_with_spin(2.0)
            
            # releaseobject
            self.get_logger().info("releaseobject...")
            self._gripper_control(close=False)
            self.is_grasped = False
            self._sleep_with_spin(2.0)
            
            # Slightly raise end effector
            retreat_pose = Pose()
            retreat_pose.position.x = delivery_pose.position.x
            retreat_pose.position.y = delivery_pose.position.y
            retreat_pose.position.z = delivery_pose.position.z + 0.05
            retreat_pose.orientation = delivery_pose.orientation
            
            self.get_logger().info("Raise end effector...")
            self._execute_move(retreat_pose, [0.0, 0.0, 0.0])
            self._sleep_with_spin(1.0)
            
            # Return to home position
            self.get_logger().info("Return to home position...")
            self.go_to_home_position()
            self._sleep_with_spin(1.0)
            
            self.get_logger().info("Delivery to hand complete")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Delivery to hand failed: {e}")
            return False

    def get_end_effector_pose(self) -> Optional[Pose]:
        """
        Get current end effector pose
        
        Use pymoveit2 compute_fk() method to calculate forward kinematics,
        Get end effector pose based on current joint state
        
        :return: End effector Pose object, return None on failure
        """
        try:
            # Correct pymoveit2 method: use compute_fk()
            # compute_fk(joint_state=None) means use current joint state
            if hasattr(self.arm, 'compute_fk'):
                # Call forward kinematics, get end effector pose
                pose_stamped = self.arm.compute_fk(
                    joint_state=None,  # None = use current joint state
                    fk_link_names=None  # None = use default end effector
                )
                
                if pose_stamped is not None:
                    # compute_fk return PoseStamped，extract Pose
                    pose = pose_stamped.pose
                    self.get_logger().info(
                        f"End effector position: x={pose.position.x:.3f}, "
                        f"y={pose.position.y:.3f}, z={pose.position.z:.3f}, "
                        f"frame={pose_stamped.header.frame_id}"
                    )
                    return pose
                else:
                    self.get_logger().warn("compute_fk return None")
                    return None
            else:
                self.get_logger().error("self.arm does not have compute_fk method")
                return None
            
        except Exception as e:
            self.get_logger().error(f"Get end effector pose failed: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def log_scan_results(self, results: List[dict]) -> None:
        """Serialize and log full scan_views results including all detections and poses.

        The log will include per-view summary and per-target detailed fields
        (class_id, label, confidence, position, grasp_pose if present).
        """
        try:
            if not results:
                self.get_logger().info("log_scan_results: empty results list")
                return

            for vi, view in enumerate(results):
                view_cmd = view.get("view_cmd")
                success = view.get("success", False)
                poses = view.get("poses") or []
                detections = view.get("detections") or []
                self.get_logger().info(f"[scan_view {vi}] cmd={view_cmd} success={success} poses={len(poses)} detections={len(detections)}")

                # log per-detection details: try to use parsed poses and original detection dicts
                for ti, det in enumerate(detections):
                    try:
                        cls = det.get("class_id") if isinstance(det, dict) else None
                        label = det.get("label") if isinstance(det, dict) else None
                        conf = det.get("confidence") if isinstance(det, dict) else None
                        pos = det.get("position") if isinstance(det, dict) else None
                        grasp = det.get("grasp_pose") or det.get("pose") if isinstance(det, dict) else None
                        self.get_logger().info(
                            f"  target[{ti}] class={cls} label={label} conf={conf} pos={pos} grasp={grasp}"
                        )
                    except Exception as e:
                        self.get_logger().warn(f"  target[{ti}] logging failed: {e}")

                # also log parsed poses (geometry_msgs Pose)
                for pi, p in enumerate(poses):
                    try:
                        xyz = pose_to_tuple(p)
                        self.get_logger().info(f"  parsed_pose[{pi}] xyz={xyz}")
                    except Exception as e:
                        self.get_logger().warn(f"  parsed_pose[{pi}] logging failed: {e}")

            # full JSON dump at DEBUG level
            try:
                self.get_logger().debug(f"full_scan_results_json={json.dumps(results, default=str, ensure_ascii=False)}")
            except Exception:
                # fallback: print shallow repr
                self.get_logger().debug(f"full_scan_results_repr={repr(results)}")
        except Exception as e:
            self.get_logger().warn(f"log_scan_results failed: {e}")

    def _yolo_grasper(self, grasp_pose: Pose, place_pose: Pose, wait_time=2.0):

        self.get_logger().info("Grasper executing...")

        if place_pose is None:
            self.get_logger().info("Place pose not detected, using default pose...")
            place_pose = Pose()
            place_pose.position.x = grasp_pose.position.x - 0.1
            place_pose.position.y = grasp_pose.position.y + 0.1
            place_pose.position.z = grasp_pose.position.z + 0.1
            place_pose.orientation = grasp_pose.orientation

        # Return to home position
        self.get_logger().info("Return to home position...")
        self.go_to_home_position()
        self.current_state = ArmState.MOVE_TO_PREGRASP
        self._sleep_with_spin(5.0)

        self.move_and_grasp(
            grasp_pose=grasp_pose,
            place_pose=place_pose,
            mode=ExecutionMode.CONTINUOUS,  # Use continuous mode
            position_offset=[0.0, 0.0, 0.05],  # Pre-grasp position offset
            wait_time=wait_time,
        )

        self.get_logger().info("In continuous mode, auto-execution complete...")


def _argv_accessory(argv: List[str]) -> Tuple[List[str], bool]:
    # Parse custom parameters (before --ros-args)
    raw_args = argv[1:]
    if "--ros-args" in raw_args:
        cut = raw_args.index("--ros-args")
        custom_args = raw_args[:cut]
        ros_args = raw_args[cut:]
    else:
        custom_args = raw_args
        ros_args = []

    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument(
        "--continuous", action="store_true", help="Continuous mode (equivalent to --mode continuous)"
    )
    parser.add_argument(
        "--step", action="store_true", help="Step mode (equivalent to --mode step)"
    )
    parser.add_argument("--mode", choices=["continuous", "step"], help="Execution mode")
    parsed, _ = parser.parse_known_args(custom_args)

    # Determine mode priority: --mode > --continuous/--step > default continuous
    if parsed.mode == "continuous":
        isContinuous = True
    elif parsed.mode == "step":
        isContinuous = False
    elif parsed.continuous and not parsed.step:
        isContinuous = True
    elif parsed.step and not parsed.continuous:
        isContinuous = False
    else:
        isContinuous = True  # Default continuous

    # Remove custom parameters before passing to rclpy (avoid unknown parameters error)
    cleaned_args = []
    skip_next = False
    for a in raw_args:
        if skip_next:
            skip_next = False
            continue
        if a in ("--continuous", "--step"):
            continue
        if a == "--mode":
            skip_next = True
            continue
        # Keep ROS / other parameters
        cleaned_args.append(a)
    argv = [argv[0]] + cleaned_args

    return argv, isContinuous

def pose_to_tuple(pose):
    return (pose.position.x, pose.position.y, pose.position.z)
def main():
    # parse custom args and get cleaned argv for rclpy
    argv, isContinuous = _argv_accessory(sys.argv)
    # rclpy.init expects a list of args (or None). Pass argv (list).
    rclpy.init(args=argv)
    # rclpy.spin(YoloDetectionNode(is_fast_robust_plan=True))
    node = None
    try:
        node = YoloDetectionNode(is_fast_robust_plan=False)
        node.executor_thread_start()
        if node.executor is None:
            # fallback: spin in background thread using rclpy.spin
            node.executor_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
            node.executor_thread.start()

        # perform example scan (non-blocking w.r.t executor)
        results = node.scan_views(stop_on_detection=True, front_only=False)
        # node.log_scan_results(results)

        for vi, view in enumerate(results):
            view_cmd = view.get("view_cmd")
            success = view.get("success", False)
            poses = view.get("poses") or []
            detections = view.get("detections") or []
            node.get_logger().info(f"[scan_view {vi}] cmd={view_cmd} success={success} poses={len(poses)} detections={len(detections)}")
            # log per-detection details: try to use parsed poses and original detection dicts
            for ti, det in enumerate(detections):
                try:
                    cls = det.get("class_id") if isinstance(det, dict) else None
                    label = det.get("label") if isinstance(det, dict) else None
                    conf = det.get("confidence") if isinstance(det, dict) else None
                    pos = det.get("position") if isinstance(det, dict) else None
                    grasp = det.get("grasp_pose") or det.get("pose") if isinstance(det, dict) else None
                    node.get_logger().info(
                            f"  target[{ti}] class={cls} label={label} conf={conf} pos={pos} grasp={grasp}"
                    )
                except Exception as e:
                    node.get_logger().warn(f"  target[{ti}] logging failed: {e}")
                if det is not None and grasp is not None:
                    pose = node._extract_pose_from_dict(grasp)
                    node.get_logger().info(f"Preparing to execute grasp on target {ti} of view {view_cmd}...")
                    node.grasping_for_scan_rs(pose, None, wait_time=2.0)
                    break
        # Block main thread until rclpy requests exit (or user Ctrl-C)
        try:
            while rclpy.ok():
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass

    except Exception as e:
        # Uncaught exception during initialization or execution, logging and proceeding to cleanup
        if node:
            node.get_logger().error(f"Execution exception: {e}")
        else:
            print(f"Execution exception (node not successfully created): {e}")
    finally:
        if node:
            try:
                node.get_logger().info("Destroying node and exiting...")
                # stop executor thread and shutdown executor if started
                try:
                    # prefer node method if available
                    if hasattr(node, "executor_thread_stop"):
                        node.executor_thread_stop(join_timeout=1.0)
                    else:
                        if node.executor is not None:
                            try:
                                node.executor.shutdown()
                            except Exception:
                                pass
                        thr = getattr(node, "executor_thread", None)
                        if thr is not None and thr.is_alive():
                            try:
                                thr.join(1.0)
                            except Exception:
                                pass
                except Exception:
                    pass
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception as e:
            # rclpy may already have been shutdown by another thread/context
            try:
                if node:
                    node.get_logger().warn(f"rclpy.shutdown() raised: {e}")
                else:
                    print(f"rclpy.shutdown() raised: {e}")
            except Exception:
                pass


if __name__ == "__main__":
    main()
