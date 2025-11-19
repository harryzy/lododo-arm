#!/usr/bin/env python3
"""
Planar Localization Node

Calculate 3D coordinates using planar constraints and camera model.
Core: image_geometry + planar projection algorithm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Point, PointStamped, Pose
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import Header, String
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from cv_bridge import CvBridge
import numpy as np
import json
from typing import List, Optional

# Custom messages
try:
    from arm_interfaces.msg import MeasuredObject
    ARM_INTERFACES_AVAILABLE = True
except ImportError:
    ARM_INTERFACES_AVAILABLE = False
    MeasuredObject = None

# Import utility modules
from arm_perception_planar.utils import CameraModel, create_marker_array, draw_3d_info
import cv2


class PlanarLocalizationNode(Node):
    """
    Planar Projection Localization Node
    
    Subscriptions:
        /planar/cube_detections (Detection2DArray): 2D detection results
        /camera/camera_info (CameraInfo): Camera intrinsics
        /camera/image_raw (Image): Original image (for visualization)
    
    Publications:
        /planar/measured_objects (MeasuredObject): 3D localization results
        /planar/detection_markers (MarkerArray): RViz visualization markers
        /planar/result_image (Image): Image with 3D annotations
        /detection_result (String): JSON format results (compatible with YOLO interface)
        /collision_object (CollisionObject): MoveIt scene objects for motion planning
    
    Parameters:
        table_height: Table height
        camera_pose.*: Camera extrinsics
        localization.*: Localization parameters
    """
    
    def __init__(self):
        super().__init__('planar_localization_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Camera calibration
                ('use_calibrated_camera_params', True),
                ('camera_matrix.fx', 572.70774),
                ('camera_matrix.fy', 612.22574),
                ('camera_matrix.cx', 330.25436),
                ('camera_matrix.cy', 259.5),  # Precise Y-axis calibration (was 249.0)
                ('camera_pose.translation.x', -0.2745),
                ('camera_pose.translation.y', -0.0080),
                ('camera_pose.translation.z', 0.1055),
                ('camera_pose.rotation.pitch', 97.75),
                ('camera_pose.rotation.roll', 0.0),
                ('camera_pose.rotation.yaw', 0.0),
                
                # Plane parameters
                ('table_height', 0.0),
                ('table_height_offset', 0.002),
                
                # Cube size
                ('cube_detection.expected_size', 0.050),
                
                # Localization parameters
                ('localization.enable_size_validation', True),
                ('localization.size_validation_tolerance', 0.015),
                ('localization.position_correction.x', 0.0),
                ('localization.position_correction.y', 0.0),
                ('localization.position_correction.z', 0.03),
                ('localization.min_depth', 0.20),
                ('localization.max_depth', 0.80),
                ('localization.y_min', -0.02),
                ('localization.y_max', 0.02),
                
                # Visualization
                ('visualization.enable_rviz_markers', True),
                ('visualization.marker_lifetime', 2.0),
                
                # Scene object (MoveIt)
                ('scene.object_id', 'detected_cube'),
                ('scene.publish_box', True),
            ]
        )
        
        # Load parameters
        self._load_parameters()
        
        # Camera model
        self.camera_model = CameraModel()
        
        # Load intrinsics from config file
        self.camera_model.fx = self.get_parameter('camera_matrix.fx').value
        self.camera_model.fy = self.get_parameter('camera_matrix.fy').value
        self.camera_model.cx = self.get_parameter('camera_matrix.cx').value
        self.camera_model.cy = self.get_parameter('camera_matrix.cy').value
        
        self.get_logger().info(
            f'Loaded camera intrinsics from config: '
            f'fx={self.camera_model.fx:.2f}, fy={self.camera_model.fy:.2f}, '
            f'cx={self.camera_model.cx:.2f}, cy={self.camera_model.cy:.2f}'
        )
        
        # Set extrinsics
        self.camera_model.set_extrinsics(
            self.camera_tx, self.camera_ty, self.camera_tz,
            self.camera_pitch, self.camera_roll, self.camera_yaw
        )
        self.camera_info_received = False
        self.use_calibrated_params = self.get_parameter('use_calibrated_camera_params').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Latest image (for visualization)
        self.latest_image = None
        
        # Latest site_id (from triggered mode)
        self.current_site_id = None
        
        # Subscriptions
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/planar/cube_detections',
            self.detections_callback,
            10
        )
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Image subscription removed - image size extracted from detection message frame_id
        # This eliminates double subscription and improves performance
        self.image_width = 320  # Default, will be updated from detection messages
        self.image_height = 240  # Default, will be updated from detection messages
        
        # Publishers
        if ARM_INTERFACES_AVAILABLE:
            self.pub_measured_objects = self.create_publisher(
                MeasuredObject,
                '/planar/measured_objects',
                10
            )
        
        self.pub_markers = self.create_publisher(
            MarkerArray,
            '/planar/detection_markers',
            10
        )
        
        self.pub_result_image = self.create_publisher(
            Image,
            '/planar/result_image',
            10
        )
        
        # JSON result publisher (compatible with YOLO interface)
        self.pub_result_json = self.create_publisher(
            String,
            '/detected_objects_json',  # Match YOLO interface topic name
            10
        )
        
        # CollisionObject publisher (for MoveIt scene)
        self.pub_collision_object = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
        
        # Statistics
        self.localization_count = 0
        self.validation_failures = 0
        
        self.get_logger().info('Planar Localization Node initialized')
        self.get_logger().info(f'Table height: {self.table_height:.3f}m')
        self.get_logger().info(f'Camera pose: ({self.camera_tx:.3f}, {self.camera_ty:.3f}, {self.camera_tz:.3f})')
        self.get_logger().info(f'Camera pitch: {self.camera_pitch:.2f}°')
        
        # Verify rotation matrix
        R = self.camera_model.get_rotation_matrix()
        self.get_logger().info(f'Rotation matrix (row 1): [{R[0,0]:.4f}, {R[0,1]:.4f}, {R[0,2]:.4f}]')
        self.get_logger().info(f'Rotation matrix (row 2): [{R[1,0]:.4f}, {R[1,1]:.4f}, {R[1,2]:.4f}]')
        self.get_logger().info(f'Rotation matrix (row 3): [{R[2,0]:.4f}, {R[2,1]:.4f}, {R[2,2]:.4f}]')
    
    def _load_parameters(self):
        """Load all parameters"""
        # Camera extrinsics
        self.camera_tx = self.get_parameter('camera_pose.translation.x').value
        self.camera_ty = self.get_parameter('camera_pose.translation.y').value
        self.camera_tz = self.get_parameter('camera_pose.translation.z').value
        self.camera_pitch = self.get_parameter('camera_pose.rotation.pitch').value
        self.camera_roll = self.get_parameter('camera_pose.rotation.roll').value
        self.camera_yaw = self.get_parameter('camera_pose.rotation.yaw').value
        
        # Plane parameters
        base_height = self.get_parameter('table_height').value
        offset = self.get_parameter('table_height_offset').value
        self.table_height = base_height + offset
        
        # Cube size
        self.expected_size = self.get_parameter('cube_detection.expected_size').value
        
        # Localization parameters
        self.enable_size_validation = self.get_parameter('localization.enable_size_validation').value
        self.size_tolerance = self.get_parameter('localization.size_validation_tolerance').value
        self.correction_x = self.get_parameter('localization.position_correction.x').value
        self.correction_y = self.get_parameter('localization.position_correction.y').value
        self.correction_z = self.get_parameter('localization.position_correction.z').value
        self.min_depth = self.get_parameter('localization.min_depth').value
        self.max_depth = self.get_parameter('localization.max_depth').value
        self.y_min = self.get_parameter('localization.y_min').value
        self.y_max = self.get_parameter('localization.y_max').value
        
        # Visualization
        self.enable_markers = self.get_parameter('visualization.enable_rviz_markers').value
        self.marker_lifetime = self.get_parameter('visualization.marker_lifetime').value
        
        # Scene object parameters
        self.scene_object_id = self.get_parameter('scene.object_id').value
        self.publish_collision_object = self.get_parameter('scene.publish_box').value
    
    def _extract_image_size_from_frame_id(self, frame_id: str) -> tuple:
        """
        Extract image dimensions from frame_id string
        
        Expected format: camera_frame|img:WIDTHxHEIGHT
        Example: camera_link|img:320x240
        
        Returns:
            (width, height) tuple, or (self.image_width, self.image_height) if parsing fails
        """
        try:
            if '|img:' in frame_id:
                img_part = frame_id.split('|img:')[1]
                if 'x' in img_part:
                    # Extract first part before any additional | separators
                    size_str = img_part.split('|')[0]
                    width_str, height_str = size_str.split('x')
                    width = int(width_str)
                    height = int(height_str)
                    return (width, height)
        except Exception as e:
            self.get_logger().debug(f"Failed to parse image size from frame_id '{frame_id}': {e}")
        
        return (self.image_width, self.image_height)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Camera intrinsics callback"""
        if not self.camera_info_received:
            self.camera_info_received = True
            
            if self.use_calibrated_params:
                # Use calibrated params from config, ignore camera_info
                self.get_logger().info(
                    f'Camera info received but using calibrated params from config: '
                    f'fx={self.camera_model.fx:.2f}, fy={self.camera_model.fy:.2f}, '
                    f'cx={self.camera_model.cx:.2f}, cy={self.camera_model.cy:.2f}'
                )
                self.get_logger().warn(
                    f'Camera info would have set: '
                    f'fx={msg.k[0]:.2f}, fy={msg.k[4]:.2f}, '
                    f'cx={msg.k[2]:.2f}, cy={msg.k[5]:.2f} (IGNORED)'
                )
            else:
                # Override with camera_info
                self.camera_model.update_from_camera_info(msg)
                self.get_logger().info(
                    f'Camera info received and applied: '
                    f'fx={self.camera_model.fx:.1f}, fy={self.camera_model.fy:.1f}, '
                    f'cx={self.camera_model.cx:.1f}, cy={self.camera_model.cy:.1f}'
                )
    
    # image_callback removed - no longer subscribing to raw images
    # Image dimensions extracted from detection message metadata
    
    def detections_callback(self, msg: Detection2DArray):
        """Detection results callback"""
        # Extract image dimensions from frame_id
        img_width, img_height = self._extract_image_size_from_frame_id(msg.header.frame_id)
        if img_width != self.image_width or img_height != self.image_height:
            self.image_width = img_width
            self.image_height = img_height
            self.get_logger().info(f'Updated image dimensions from detection message: {img_width}x{img_height}')
        
        if not self.camera_info_received:
            self.get_logger().warn('Camera info not received yet, skipping localization')
            return
        
        if len(msg.detections) == 0:
            return
        
        # Extract site_id from frame_id (compatible with YOLO triggered mode)
        site_id = None
        if '|site:' in msg.header.frame_id:
            parts = msg.header.frame_id.split('|site:')
            if len(parts) > 1:
                site_id = parts[1]
                self.current_site_id = site_id
        
        try:
            # Process each detection
            points_3d = []
            confidences = []
            valid_detections = []
            
            for idx, detection in enumerate(msg.detections):
                # Extract pixel coordinates
                pixel_x = detection.bbox.center.position.x
                pixel_y = detection.bbox.center.position.y
                bbox_w = detection.bbox.size_x
                bbox_h = detection.bbox.size_y
                
                # Confidence score
                confidence = detection.results[0].hypothesis.score if detection.results else 0.5
                
                # Planar projection to compute 3D coordinates
                # Use debug version for first detection to verify accuracy
                if idx == 0 and True:  # Debug enabled to diagnose position 2 error
                    point_3d, debug_info = self.camera_model.intersect_plane_debug(
                        pixel_x, pixel_y, self.table_height
                    )
                    
                    # Simplified debug output
                    if 'error' in debug_info:
                        self.get_logger().error(f"Ray-plane intersection error: {debug_info['error']}")
                    else:
                        p3d = debug_info['point_3d']
                        dist = debug_info.get('distance', 0)
                        ray_norm = debug_info.get('ray_norm', 0)
                        self.get_logger().info(
                            f"[Planar Debug] pixel=({pixel_x:.0f},{pixel_y:.0f}) → "
                            f"3D=({p3d[0]:.3f},{p3d[1]:.3f},{p3d[2]:.3f}), "
                            f"dist={dist:.3f}m, ray_norm={ray_norm:.4f}"
                        )
                else:
                    point_3d = self.camera_model.intersect_plane(pixel_x, pixel_y, self.table_height)
                
                if point_3d is None:
                    self.get_logger().warn(
                        f'Failed to compute 3D intersection for pixel ({pixel_x:.1f}, {pixel_y:.1f})'
                    )
                    continue
                
                # Depth check
                depth = self.camera_model.calculate_depth(point_3d)
                
                # Debug: Simplified log (only if debug enabled)
                if False:  # Set to True to enable debug logging
                    self.get_logger().info(
                        f'[Planar] Detection #{idx+1}: pixel=({pixel_x:.0f},{pixel_y:.0f}) → '
                        f'3D=({point_3d.x:.3f},{point_3d.y:.3f},{point_3d.z:.3f}), depth={depth:.3f}m'
                    )
                
                if depth < self.min_depth or depth > self.max_depth:
                    self.get_logger().warn(
                        f'Depth {depth:.3f}m out of range [{self.min_depth}-{self.max_depth}]'
                    )
                    continue
                
                # Size validation
                if self.enable_size_validation:
                    valid, real_w, real_h = self.camera_model.validate_size(
                        int(bbox_w), int(bbox_h), point_3d,
                        self.expected_size, self.size_tolerance
                    )
                    
                    if not valid:
                        self.validation_failures += 1
                        self.get_logger().warn(
                            f'Size validation failed: ({real_w:.3f}, {real_h:.3f}) '
                            f'vs expected {self.expected_size:.3f}m'
                        )
                        continue
                
                # Position correction
                point_3d.x += self.correction_x
                point_3d.y += self.correction_y
                point_3d.z += self.correction_z
                
                # Y workspace validation (disabled after auto-calibration)
                # After calibration, Y error is within ±5mm, no need for strict filtering
                # if not (self.y_min <= point_3d.y <= self.y_max):
                #     self.get_logger().warn(
                #         f'Object Y={point_3d.y:.3f}m outside workspace '
                #         f'[{self.y_min:.3f}, {self.y_max:.3f}]. Please adjust object Y position.'
                #     )
                #     continue
                
                points_3d.append(point_3d)
                confidences.append(confidence)
                valid_detections.append(detection)
                
                self.localization_count += 1
                
                self.get_logger().info(
                    f'Cube detected at ({point_3d.x:.3f}, {point_3d.y:.3f}, {point_3d.z:.3f}), '
                    f'confidence: {confidence:.2f}, depth: {depth:.3f}m'
                )
            
            # Publish results
            if len(points_3d) > 0:
                self.get_logger().info(f'=== 3D Localization Results: {len(points_3d)} cubes ===')
                
                self._publish_measured_objects(points_3d, confidences, msg.header)
                
                # Publish JSON result (compatible with YOLO interface)
                self._publish_json_result(points_3d, confidences, valid_detections, site_id or "unknown", msg.header)
                
                # Publish CollisionObject for MoveIt scene (publish all detected cubes)
                for idx, point in enumerate(points_3d):
                    self._publish_collision_object(point, self.expected_size, idx)
                    self.get_logger().info(
                        f"[3D Localization #{idx+1}] position=({point.x:.3f}, {point.y:.3f}, {point.z:.3f})m, "
                        f"confidence={confidences[idx]:.2f}"
                    )
                
                self.get_logger().info(
                    f'Published to: /planar/measured_objects, /detected_objects_json, /collision_object'
                )
                
                if self.enable_markers:
                    self._publish_markers(points_3d, confidences, msg.header)
                
                # Note: result_image disabled since we no longer subscribe to raw images
                # If needed, could recreate visualization from detection bboxes only
        
        except Exception as e:
            self.get_logger().error(f'Error in localization: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def _publish_measured_objects(self, points: List[Point], confidences: List[float], header: Header):
        """Publish measurement results"""
        if not ARM_INTERFACES_AVAILABLE:
            return
        
        for point, conf in zip(points, confidences):
            msg = MeasuredObject()
            msg.header = header
            msg.header.frame_id = 'base_link'
            msg.header.stamp = self.get_clock().now().to_msg()
            
            msg.position = point
            msg.confidence = conf
            msg.class_name = 'cube'
            
            self.pub_measured_objects.publish(msg)
    
    def _publish_json_result(self, points: List[Point], confidences: List[float], 
                            detections: List, site_id: str, header: Header):
        """Publish JSON result (compatible with YOLO interface)"""
        import json
        
        # Build detection list in YOLO format
        detection_list = []
        
        for i, (point, conf, det) in enumerate(zip(points, confidences, detections)):
            detection_obj = {
                "class_id": 0,  # 0 for cube
                "label": "cube",
                "confidence": float(conf),
                # Raw position: Original 3D localization result (algorithm output)
                "position": {
                    "x": float(point.x),
                    "y": float(point.y),
                    "z": float(point.z)
                },
                "center_px": {
                    "u": float(det.bbox.center.position.x),
                    "v": float(det.bbox.center.position.y)
                },
                "bbox_px": {
                    "w": float(det.bbox.size_x),
                    "h": float(det.bbox.size_y)
                },
                "grasp_quality": float(conf),  # Use confidence as grasp quality
                # Grasp pose: Corrected position for robot grasping
                # Formula: grasp_pose = position + position_correction (conditional for Z)
                # Z correction only applied when calculated Z < 0.02m
                "grasp_pose": {
                    "position": {
                        "x": float(point.x) + self.correction_x,
                        "y": float(point.y) + self.correction_y,
                        "z": float(point.z) + (self.correction_z if point.z < 0.02 else 0.0)
                    },
                    "orientation": {
                        "x": 0.0,
                        "y": 0.0,
                        "z": 0.0,
                        "w": 1.0
                    }
                }
            }
            detection_list.append(detection_obj)
        
        # Build final payload matching YOLO format
        payload = {
            "site": site_id,
            "detections": detection_list
        }
        
        # Publish as JSON string
        json_msg = String()
        json_msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_result_json.publish(json_msg)
        
        # Log the complete JSON result
        self.get_logger().info('=== /detection_result JSON Output ===')
        self.get_logger().info(f'\n{json.dumps(payload, indent=2, ensure_ascii=False)}')
        self.get_logger().info('=' * 40)
    
    def _publish_collision_object(self, point: Point, size: float, idx: int = 0):
        """
        Publish CollisionObject to MoveIt scene
        
        Args:
            point: 3D position of cube center (raw position from algorithm)
            size: Cube size (meters)
            idx: Index for object ID
        """
        if not self.publish_collision_object:
            return
        
        co = CollisionObject()
        co.id = f"{self.scene_object_id}|class=cube|idx={idx}"
        co.header = Header(frame_id="base_link")
        co.header.stamp = self.get_clock().now().to_msg()
        
        # Define box primitive
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [size, size, size]  # Cube: same dimensions
        
        # Set pose with position correction
        # CollisionObject should use corrected position (same as grasp_pose)
        # Z correction only applied when calculated Z < 0.02m
        pose = Pose()
        pose.position.x = float(point.x) + self.correction_x
        pose.position.y = float(point.y) + self.correction_y
        pose.position.z = float(point.z) + (self.correction_z if point.z < 0.02 else 0.0)
        pose.orientation.w = 1.0
        
        co.primitives.append(prim)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD
        
        self.pub_collision_object.publish(co)
        self.get_logger().debug(f'Published CollisionObject: {co.id}')
    
    def _publish_markers(self, points: List[Point], confidences: List[float], header: Header):
        """Publish RViz markers"""
        marker_array = create_marker_array(
            points, confidences,
            namespace='planar_cubes',
            cube_size=self.expected_size,
            lifetime=self.marker_lifetime
        )
        
        for marker in marker_array.markers:
            marker.header = header
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
        
        self.pub_markers.publish(marker_array)
    
    def _publish_result_image(self, image: np.ndarray, detections: List, 
                             points_3d: List[Point], header: Header):
        """Publish result image with 3D annotations"""
        try:
            result_img = image.copy()
            
            for detection, point in zip(detections, points_3d):
                bbox_x = int(detection.bbox.center.position.x - detection.bbox.size_x/2)
                bbox_y = int(detection.bbox.center.position.y - detection.bbox.size_y/2)
                bbox_w = int(detection.bbox.size_x)
                bbox_h = int(detection.bbox.size_y)
                
                # Draw bounding box
                cv2.rectangle(result_img, (bbox_x, bbox_y), 
                            (bbox_x+bbox_w, bbox_y+bbox_h), (0, 255, 0), 2)
                
                # Display 3D coordinates
                text = f"({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"
                cv2.putText(result_img, text, (bbox_x, bbox_y-10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Convert and publish
            result_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
            result_msg.header = header
            self.pub_result_image.publish(result_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing result image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PlanarLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Localizations: {node.localization_count}, '
            f'Validation failures: {node.validation_failures}'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
