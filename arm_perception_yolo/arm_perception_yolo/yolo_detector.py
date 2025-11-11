from rclpy.qos import qos_profile_sensor_data
import rclpy
import os
import json
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String, Header
from typing import TYPE_CHECKING
from rclpy.parameter import Parameter

# Import custom Detection2D messages for triangulation
try:
    from arm_interfaces.msg import (
        Detection2DArray as CustomDetection2DArray,
        Detection2D as CustomDetection2D
    )
    ARM_INTERFACES_AVAILABLE = True
except ImportError:
    ARM_INTERFACES_AVAILABLE = False
    CustomDetection2DArray = None
    CustomDetection2D = None


# Need to create a virtual environment venv_yolo to prevent conflicts with system Python packages
# The package is not available in development environment, only at runtime
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None
    _yolo_import_err = "ultralytics not available (dev mode)"
if TYPE_CHECKING:
    # Type hints only, not executed at runtime
    pass


class YoloDetector(Node):
    """
    YoloDetector.

    ROS2 node for object detection using the YOLO model from ultralytics.

    This node can operate in continuous or triggered detection modes and publishes
    detected objects as Detection2DArray messages.

    Parameters
    ----------
    model_path : str, default="yolov8n.pt"
        Path to the YOLO model file.
    min_conf : float, default=0.5
        Minimum confidence threshold for detections.
    publish_label_mode : str, default="both"
        How to publish detection labels: "id" (class id only), "name" (class name only), or "both".
    detection_mode : str, default="continuous"
        Detection mode: "continuous" (process every frame) or "triggered" (wait for trigger).
    trigger_timeout_sec : float, default=2.0
        Timeout in seconds for waiting for the first frame after a trigger.

    Subscribers
    -----------
    /camera/image_raw : Image
        Input image for object detection.
    /detection_request : String
        Trigger requests for detection in triggered mode.

    Publishers
    ----------
    /detections : Detection2DArray
        Continuous detection results (when in continuous mode).
    /detections_triggered : Detection2DArray
        Triggered detection results (when in triggered mode).

    Methods
    -------
    req_cb(msg: String)
        Callback for trigger requests.
    _make_detections(msg: Image, results, publish_label_mode: str)
        Process YOLO detection results into ROS message format.
    image_cb(msg: Image)
        Callback for processing input images.

    """

    def __init__(self):
        super().__init__("yolo_detector")

        # declare parameters with default values; they may be overridden via
        # parameter injection performed by the wrapper before node spin
        self.declare_parameter("model_path", "yolov8m.pt")
        self.declare_parameter("min_conf", 0.15)  # Lower to 0.15 to detect hard-to-recognize objects
        self.declare_parameter("publish_label_mode", "both")  # id | name | both
        self.declare_parameter("detection_mode", "triggered")  # continuous | triggered
        self.declare_parameter("trigger_timeout_sec", 3.0)  # Timeout waiting for first frame after trigger
        self.declare_parameter("min_conf_triggered", 0.20)  # Triggered mode also uses low threshold
        self.declare_parameter("inference_imgsz", 800)  # Revert to 800 (1280 makes detection harder)
        self.declare_parameter("inference_conf", 0.20)  # Inference threshold lowered to 20%
        self.declare_parameter("publish_annotated_image", True)  # Whether to publish visualization image
        self.declare_parameter("annotation_min_conf", 0.15)  # Visualize detections above 15%
        self.declare_parameter("enable_image_enhancement", True)  # Whether to enable image enhancement preprocessing
        # postpone reading parameters and loading model until after optional
        # parameter injection in main(); initialize placeholders and ROS interfaces
        self.model = None
        self.class_names = {}

        # ROS interfaces
        self.bridge = CvBridge()
        # persistent subscription for continuous mode (created after model load)
        self.sub = None
        # temporary subscription and timer for triggered (on-demand) mode
        self._pending_sub = None
        self._pending_timer = None
        # Continuous mode output
        self.pub_cont = self.create_publisher(Detection2DArray, "/detections", 10)
        # Triggered mode output
        self.pub_triggered = self.create_publisher(
            Detection2DArray, "/detections_triggered", 10
        )
        # Visualization image output (annotated image)
        self.pub_annotated_image = self.create_publisher(
            Image, "/yolo/annotated_image", 10
        )
        # Custom Detection2D output (for triangulation)
        if ARM_INTERFACES_AVAILABLE:
            self.pub_custom_detections = self.create_publisher(
                CustomDetection2DArray, "/yolo/detections", 10
            )
        else:
            self.pub_custom_detections = None
        # Trigger request (topic interface)
        self.req_sub = self.create_subscription(
            String, "/detection_request", self.req_cb, 10
        )

        # Trigger state
        self.pending = False
        self.pending_site_id = None
        self.pending_time = 0.0

    # Model loading and parameter-based logging are deferred until
    # load_model_and_apply_params() is called from main() after any
    # external parameter injection has been applied.

    def load_model_and_apply_params(self):
        """
        Read declared parameters, apply them to instance attributes and load the
        YOLO model. This should be called after any external parameter injection
        (for example via main() using the rclpy Parameter API).
        """
        # read parameters
        self.model_path = self.get_parameter("model_path").value
        self.min_conf = float(self.get_parameter("min_conf").value)
        self.detection_mode = self.get_parameter("detection_mode").value
        self.trigger_timeout = float(self.get_parameter("trigger_timeout_sec").value)
        self.min_conf_triggered = float(self.get_parameter("min_conf_triggered").value)
        # inference image size for ultralytics model
        try:
            self.infer_imgsz = int(self.get_parameter("inference_imgsz").value)
        except Exception:
            self.infer_imgsz = 640
        try:
            self.infer_conf = float(self.get_parameter("inference_conf").value)
        except Exception:
            self.infer_conf = 0.1
        
        # Visualization parameters
        try:
            self.publish_annotated_image = bool(self.get_parameter("publish_annotated_image").value)
        except Exception:
            self.publish_annotated_image = True
        try:
            self.annotation_min_conf = float(self.get_parameter("annotation_min_conf").value)
        except Exception:
            self.annotation_min_conf = 0.3
        
        # Image enhancement switch
        try:
            self.enable_image_enhancement = bool(self.get_parameter("enable_image_enhancement").value)
        except Exception:
            self.enable_image_enhancement = True

        # Load model (ultralytics may be None in dev environment)
        if YOLO is None:
            self.get_logger().error(f"ultralytics import failed: {_yolo_import_err}")
            return
        self.model = YOLO(self.model_path)
        self.class_names = self.model.names  # dict: idx -> label
        self.get_logger().info(f"Loaded model: {self.model_path}")
        self.get_logger().info(
            f"Detection mode: {self.detection_mode}, trigger timeout: {self.trigger_timeout}s"
        )
        # create persistent subscription only if continuous mode
        if self.detection_mode == "continuous" and self.sub is None:
            self.sub = self.create_subscription(
                Image, "/camera/image_raw", self.image_cb, qos_profile=qos_profile_sensor_data
            )
        # Print class mapping (count and partial entries)
        self.get_logger().info(f"class_names ({len(self.class_names)}): {self.class_names}")
        # Log all min_conf related parameters for debugging
        try:
            self.get_logger().info(f"[YOLO Params] min_conf: {self.min_conf}")
            self.get_logger().info(f"[YOLO Params] min_conf_triggered: {self.min_conf_triggered}")
            self.get_logger().info(f"[YOLO Params] inference_conf: {self.infer_conf}")
            self.get_logger().info(f"[YOLO Params] annotation_min_conf: {self.annotation_min_conf}")
        except Exception as e:
            self.get_logger().error(f"[YOLO Params] Failed to log parameters: {e}")
    def req_cb(self, msg: String):
        # Record a trigger request
        self.pending = True
        self.pending_site_id = msg.data.strip() or "unknown"
        self.pending_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f"Detection trigger received: site={self.pending_site_id}")

        # For triggered mode, create a one-shot subscription to grab the next frame
        if self.detection_mode == "triggered":
            if self._pending_sub is None:
                self._pending_sub = self.create_subscription(
                    Image, "/camera/image_raw", self._trigger_image_cb, qos_profile=qos_profile_sensor_data
                )
                # create a timeout timer to cancel pending if no frame arrives
                self._pending_timer = self.create_timer(
                    float(self.trigger_timeout), self._pending_timeout_cb
                )

    def _trigger_image_cb(self, msg: Image):
        """Callback used for one-shot triggered capture: process the frame then
        clean up the temporary subscription and timer."""
        try:
            # reuse existing image processing
            self.image_cb(msg)
        except Exception as e:
            self.get_logger().error(f"_trigger_image_cb error: {e}")

        # cleanup temporary subscription
        try:
            if self._pending_sub is not None:
                try:
                    self.destroy_subscription(self._pending_sub)
                except Exception:
                    pass
                self._pending_sub = None
        finally:
            # cancel and clear timer
            try:
                if self._pending_timer is not None:
                    try:
                        self._pending_timer.cancel()
                    except Exception:
                        pass
                    self._pending_timer = None
            finally:
                # ensure pending flag reset if still set
                if self.pending:
                    self.pending = False
                    self.pending_site_id = None


    def _pending_timeout_cb(self):
        """Called when the triggered-mode timeout expires without receiving a frame."""
        self.get_logger().warn(
            f"Trigger wait timeout: site={self.pending_site_id}, canceling this trigger"
        )
        # destroy temporary subscription if exists
        try:
            if self._pending_sub is not None:
                try:
                    self.destroy_subscription(self._pending_sub)
                except Exception:
                    pass
                self._pending_sub = None
        finally:
            # cancel and clear timer
            try:
                if self._pending_timer is not None:
                    try:
                        self._pending_timer.cancel()
                    except Exception:
                        pass
                    self._pending_timer = None
            finally:
                # clear pending state
                self.pending = False
                self.pending_site_id = None

    def _enhance_image_for_detection(self, img):
        """
        Enhance image to improve detection of blurry/low-contrast objects
        
        For hard-to-recognize objects like white tissue paper:
        1. Adaptive histogram equalization (CLAHE) - enhance local contrast
        2. Sharpening filter - enhance edges
        3. Denoising - reduce noise interference
        
        Args:
            img: OpenCV BGR image
            
        Returns:
            enhanced_img: Enhanced image
        """
        import cv2
        import numpy as np
        
        # Convert to LAB color space (better suited for brightness adjustment)
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        
        # 1. CLAHE (Contrast Limited Adaptive Histogram Equalization)
        # Use strong enhancement to ensure blurry objects can be detected
        clahe = cv2.createCLAHE(clipLimit=5.0, tileGridSize=(8, 8))
        l_enhanced = clahe.apply(l)
        
        # Merge back to LAB and convert to BGR
        enhanced_lab = cv2.merge([l_enhanced, a, b])
        enhanced_img = cv2.cvtColor(enhanced_lab, cv2.COLOR_LAB2BGR)
        
        # 2. Sharpening filter (enhance edges)
        kernel_sharpen = np.array([
            [-1, -1, -1],
            [-1,  11, -1],
            [-1, -1, -1]
        ])
        enhanced_img = cv2.filter2D(enhanced_img, -1, kernel_sharpen)
        
        # 3. Light denoising (avoid noise increase after sharpening)
        enhanced_img = cv2.fastNlMeansDenoisingColored(
            enhanced_img, None, h=3, hColor=5, templateWindowSize=7, searchWindowSize=21
        )
        
        return enhanced_img

    def _draw_annotated_image(self, img, results):
        """
        Draw detection boxes and labels on the image, return annotated image.
        
        Args:
            img: OpenCV image (BGR)
            results: YOLO detection results
            
        Returns:
            annotated_img: Annotated image
        """
        annotated_img = img.copy()
        img_h, img_w = annotated_img.shape[:2]
        
        # Color configuration (BGR format)
        box_color = (0, 0, 255)  # Red
        text_color = (255, 255, 255)  # White
        text_bg_color = (0, 0, 255)  # Red background
        thickness = 2
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        padding = 5  # Text padding
        
        for b in results.boxes:
            conf = float(b.conf)
            
            # Only show detections above visualization threshold
            if conf < self.annotation_min_conf:
                continue
                
            cls_id = int(b.cls)
            label = self.class_names.get(cls_id, f"class_{cls_id}")
            
            # Get bounding box coordinates (xyxy format)
            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            
            # Draw bounding box
            cv2.rectangle(annotated_img, (x1, y1), (x2, y2), box_color, thickness)
            
            # Prepare label text
            label_text = f"{label}: {conf:.2f}"
            
            # Calculate text size
            (text_width, text_height), baseline = cv2.getTextSize(
                label_text, font, font_scale, thickness=1
            )
            
            # Calculate text background box dimensions
            label_height = text_height + baseline + 2 * padding
            label_width = text_width + 2 * padding
            
            # Determine label position: prefer above box, if no space then below
            label_above = (y1 - label_height) >= 0
            
            if label_above:
                # Label above box
                text_bg_y1 = max(0, y1 - label_height)
                text_bg_y2 = y1
                text_y = text_bg_y2 - baseline - padding
            else:
                # Label below box (if no space below, force display at box top)
                if (y2 + label_height) <= img_h:
                    # Space below box
                    text_bg_y1 = y2
                    text_bg_y2 = min(img_h, y2 + label_height)
                else:
                    # No space below either, display at box top
                    text_bg_y1 = y1
                    text_bg_y2 = min(y1 + label_height, y2)
                text_y = text_bg_y1 + text_height + padding
            
            # Ensure text background doesn't exceed right edge
            text_bg_x1 = x1
            text_bg_x2 = min(x1 + label_width, img_w)
            
            # If background box exceeds right edge, adjust start position (left to right align)
            if text_bg_x2 >= img_w:
                overflow = text_bg_x2 - img_w
                text_bg_x1 = max(0, text_bg_x1 - overflow)
                text_bg_x2 = img_w
            
            # Draw text background
            cv2.rectangle(
                annotated_img,
                (text_bg_x1, text_bg_y1),
                (text_bg_x2, text_bg_y2),
                text_bg_color,
                -1  # Fill
            )
            
            # Draw text
            text_x = text_bg_x1 + padding
            cv2.putText(
                annotated_img,
                label_text,
                (text_x, text_y),
                font,
                font_scale,
                text_color,
                thickness=1,
                lineType=cv2.LINE_AA
            )
        
        return annotated_img

    def _make_detections(self, msg: Image, results, publish_label_mode: str):
        det_array = Detection2DArray()
        det_array.header = msg.header
        # If triggered mode and has site_id, append to frame_id
        if self.detection_mode == "triggered" and self.pending_site_id:
            det_array.header.frame_id = (
                f"{msg.header.frame_id}|site:{self.pending_site_id}"
            )
        for b in results.boxes:
            conf = float(b.conf)
            th = self.min_conf if self.detection_mode == "continuous" else self.min_conf_triggered
            if conf < th:
                continue
            cls_id = int(b.cls)
            label = self.class_names.get(cls_id, f"class_{cls_id}")
            xc, yc, w, h = b.xywh[0].tolist()
            det = Detection2D()
            det.header = msg.header
            # assign center coordinates robustly: support different message shapes
            # (Point, Pose2D, nested .position, etc.)
            try:
                det.bbox.center.x = xc
                det.bbox.center.y = yc
            except Exception:
                try:
                    # some types may have position attribute
                    det.bbox.center.position.x = xc
                    det.bbox.center.position.y = yc
                except Exception:
                    try:
                        # nested pose
                        det.bbox.center.pose.position.x = xc
                        det.bbox.center.pose.position.y = yc
                    except Exception:
                        # fallback: set attributes dynamically
                        try:
                            setattr(det.bbox.center, "x", xc)
                            setattr(det.bbox.center, "y", yc)
                        except Exception:
                            pass
            # assign size fields robustly
            try:
                det.bbox.size_x = w
                det.bbox.size_y = h
            except Exception:
                try:
                    det.bbox.size_x = float(w)
                    det.bbox.size_y = float(h)
                except Exception:
                    # last resort: try nested fields
                    try:
                        det.bbox.size.width = w
                        det.bbox.size.height = h
                    except Exception:
                        pass

            # Write according to mode
            mode = publish_label_mode
            def _assign_hypothesis_fields(hyp_wrapper, id_val, score_val):
                """Assign id/score into both hyp_wrapper.hypothesis and hyp_wrapper itself
                to be compatible with consumers that access either structure.
                hyp_wrapper is expected to be an ObjectHypothesisWithPose instance.
                """
                # target inner hypothesis object if present
                inner = getattr(hyp_wrapper, "hypothesis", None)
                # helper to try setting multiple candidate attribute names
                def _try_set(obj, names, value):
                    for name in names:
                        try:
                            setattr(obj, name, value)
                            return True
                        except Exception:
                            try:
                                # try to access attribute to see if it exists then set
                                getattr(obj, name)
                                setattr(obj, name, value)
                                return True
                            except Exception:
                                continue
                    return False

                id_candidates = ("id", "label", "class_id", "name")
                score_candidates = ("score", "confidence", "prob", "score_value")

                # first try inner hypothesis
                if inner is not None:
                    _try_set(inner, id_candidates, id_val)
                    _try_set(inner, score_candidates, score_val)
                # also try to set on wrapper for backward compatibility
                _try_set(hyp_wrapper, id_candidates, id_val)
                _try_set(hyp_wrapper, score_candidates, score_val)

            if mode == "id":
                hyp_id = ObjectHypothesisWithPose()
                try:
                    _assign_hypothesis_fields(hyp_id, str(cls_id), conf)
                except Exception:
                    pass
                det.results.append(hyp_id)
            elif mode == "name":
                hyp_name = ObjectHypothesisWithPose()
                try:
                    _assign_hypothesis_fields(hyp_name, label, conf)
                except Exception:
                    pass
                det.results.append(hyp_name)
            else:  # both
                hyp_id = ObjectHypothesisWithPose()
                hyp_name = ObjectHypothesisWithPose()
                try:
                    _assign_hypothesis_fields(hyp_id, str(cls_id), conf)
                except Exception:
                    pass
                try:
                    _assign_hypothesis_fields(hyp_name, label, conf)
                except Exception:
                    pass
                det.results.extend([hyp_id, hyp_name])
            det_array.detections.append(det)
        return det_array

    def _make_custom_detections(self, msg: Image, results):
        """
        Create custom Detection2DArray message for triangulation
        
        Args:
            msg: Original image message
            results: YOLO detection results
        
        Returns:
            CustomDetection2DArray message
        """
        if not ARM_INTERFACES_AVAILABLE:
            return None
        
        det_array = CustomDetection2DArray()
        det_array.header = Header()
        det_array.header.stamp = msg.header.stamp
        det_array.header.frame_id = msg.header.frame_id
        
        # In triggered mode, append site_id to frame_id
        if self.detection_mode == "triggered" and self.pending_site_id:
            det_array.header.frame_id = (
                f"{msg.header.frame_id}|site:{self.pending_site_id}"
            )
        
        # Get minimum confidence threshold
        min_conf_threshold = self.min_conf
        if self.detection_mode == "triggered":
            min_conf_threshold = self.min_conf_triggered
        
        # Debug: log all detection results (including filtered ones)
        total_detections = len(results.boxes)
        filtered_count = 0
        low_conf_detections = []
        
        for box in results.boxes:
            conf = float(box.conf)
            cls_id = int(box.cls)
            label = self.class_names.get(cls_id, f"class_{cls_id}")
            
            # Log low confidence detections
            if conf < min_conf_threshold:
                filtered_count += 1
                if conf >= 0.05:  # Only log above 5%
                    low_conf_detections.append(f"{label}:{conf:.2f}")
                continue
            
            cls_id = int(box.cls)
            label = self.class_names.get(cls_id, f"class_{cls_id}")
            
            # Extract bounding box coordinates
            xyxy = box.xyxy[0].tolist()  # [x1, y1, x2, y2]
            x1, y1, x2, y2 = xyxy
            
            # Create CustomDetection2D message
            det = CustomDetection2D()
            det.header = Header()
            det.header.stamp = msg.header.stamp
            det.header.frame_id = msg.header.frame_id
            
            det.class_id = cls_id
            det.class_name = label
            det.confidence = conf
            
            det.bbox = [float(x1), float(y1), float(x2), float(y2)]
            det.bbox_center = [float((x1 + x2) / 2), float((y1 + y2) / 2)]
            det.bbox_width = float(x2 - x1)
            det.bbox_height = float(y2 - y1)
            
            det.camera_frame = msg.header.frame_id
            
            det_array.detections.append(det)
        
        # Output detection statistics
        if total_detections > 0:
            self.get_logger().info(
                f"Detection stats: total={total_detections}, "
                f"passed={len(det_array.detections)}, "
                f"filtered={filtered_count}, "
                f"threshold={min_conf_threshold:.2f}"
            )
            if low_conf_detections:
                self.get_logger().info(
                    f"Low confidence detections (filtered): {', '.join(low_conf_detections)}"
                )
        
        return det_array

    def image_cb(self, msg: Image):
        if YOLO is None:
            self.get_logger().error("image_cb subscription failed: YOLO is None")
            return
        # Triggered mode: wait for pending flag
        if self.detection_mode == "triggered":
            # Timeout cancellation
            if self.pending and (
                self.get_clock().now().nanoseconds * 1e-9 - self.pending_time
                > self.trigger_timeout
            ):
                self.get_logger().warn(
                    f"Trigger timeout without image: site={self.pending_site_id} canceled"
                )
                self.pending = False
                self.pending_site_id = None
            if not self.pending:
                self.get_logger().warn(
                    f"No image received in triggered mode: site={self.pending_site_id} canceled"
                )
                return
        # Inference
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        # Image enhancement preprocessing (improve detection of blurry/low-contrast objects)
        # Can be controlled via enable_image_enhancement parameter
        if self.enable_image_enhancement:
            img_enhanced = self._enhance_image_for_detection(img)
        else:
            img_enhanced = img
        
        # Use mode-specific confidence threshold and configured imgsz so
        # triggered one-shot captures can use a lower conf cutoff similar to
        # manual debugging runs.
        # use a separate inference confidence so we can return low-conf boxes
        # and apply message-level filtering later
        try:
            conf_arg = float(self.infer_conf)
        except Exception:
            conf_arg = 0.1
        results = self.model(img_enhanced, imgsz=self.infer_imgsz, conf=conf_arg, verbose=False)[0]
        # debug: log number of boxes and confidences to help diagnosis
        try:
            num_boxes = len(results.boxes)
            self.get_logger().info(f"YOLO inference returned {num_boxes} boxes")
            for i, b in enumerate(results.boxes):
                try:
                    conf = float(b.conf)
                    cls_id = int(b.cls)
                    label = self.class_names.get(cls_id, f"class_{cls_id}")
                    self.get_logger().info(f"  box[{i}] conf={conf:.3f} cls={cls_id} name={label}")
                except Exception:
                    pass
        except Exception:
            # non-fatal debug failure
            pass
        # dump image for offline inspection when in triggered mode
        # try:
        #     if self.detection_mode == 'triggered':
        #         ts = int(self.get_clock().now().nanoseconds / 1e6)
        #         fname = f"/tmp/yolo_capture_{ts}.jpg"
        #         cv2.imwrite(fname, img)
        #         self.get_logger().info(f"Saved capture to {fname}")
        # except Exception as e:
        #     self.get_logger().warn(f"Failed to dump capture: {e}")
        publish_label_mode = self.get_parameter("publish_label_mode").value
        det_array = self._make_detections(msg, results, publish_label_mode)
        
        # Publish annotated image (if enabled)
        if self.publish_annotated_image:
            try:
                annotated_img = self._draw_annotated_image(img, results)
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_img, encoding="bgr8")
                annotated_msg.header = msg.header
                self.pub_annotated_image.publish(annotated_msg)
                self.get_logger().debug(f"Published annotated image with {len(results.boxes)} boxes")
            except Exception as e:
                self.get_logger().warn(f"Failed to publish annotated image: {e}")
        
        if not det_array.detections:
            if self.detection_mode == "triggered":
                # Publish empty result so downstream consumers (projection_node)
                # can detect that a trigger completed with zero detections.
                try:
                    self.pub_triggered.publish(det_array)
                except Exception:
                    pass
                # End this trigger even with empty result
                self.get_logger().info(
                    f"Trigger detection completed (empty result): site={self.pending_site_id}"
                )
                self.pending = False
                self.pending_site_id = None
            return

        if self.detection_mode == "continuous":
            self.pub_cont.publish(det_array)
        else:
            self.pub_triggered.publish(det_array)
            self.get_logger().info(
                f"Trigger detection completed: site={self.pending_site_id}, num={len(det_array.detections)}"
            )
            # Reset
            self.pending = False
            self.pending_site_id = None
        
        # Also publish custom Detection2D message (for triangulation)
        if self.pub_custom_detections is not None:
            custom_det_array = self._make_custom_detections(msg, results)
            if custom_det_array is not None:
                self.pub_custom_detections.publish(custom_det_array)
                self.get_logger().info(
                    f"📤 Published custom detection message to /yolo/detections: "
                    f"{len(custom_det_array.detections)} objects, "
                    f"frame_id={custom_det_array.header.frame_id}"
                )
            else:
                self.get_logger().warn("⚠️  _make_custom_detections returned None")
        else:
            self.get_logger().warn(
                "⚠️  pub_custom_detections is None (ARM_INTERFACES_AVAILABLE=False?)"
            )


def main():
    rclpy.init()
    node = YoloDetector()

    # Apply preloaded params from wrapper if present. The wrapper serializes
    # ros__parameters into YOLO_PRELOAD_PARAMS as a JSON dict.
    preload = os.environ.get("YOLO_PRELOAD_PARAMS")
    if preload:
        try:
            params = json.loads(preload)
            param_objs = []
            for k, v in params.items():
                try:
                    # use keyword 'value' to avoid ambiguity with the 'type' arg
                    param_objs.append(Parameter(k, value=v))
                except Exception:
                    # Fallback: stringify
                    try:
                        param_objs.append(Parameter(k, value=str(v)))
                    except Exception:
                        # ignore malformed params
                        pass
            if param_objs:
                node.set_parameters(param_objs)
                node.get_logger().info(f"Applied preloaded params: {list(params.keys())}")
        except Exception as e:
            node.get_logger().error(f"Failed to apply YOLO_PRELOAD_PARAMS: {e}")

    # Now load the model using possibly-updated parameters
    node.load_model_and_apply_params()

    try:
        rclpy.spin(node)
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            # ignore RCLError: shutdown already called
            pass
        
