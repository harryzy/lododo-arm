#!/usr/bin/env python3
"""
TriangulationNode - Dual-view triangulation node

Implements dual-view stereo vision triangulation algorithm:
1. Receive 2D detection results from multiple views
2. Object matching (based on class and IoU)
3. Calculate disparity
4. Triangulate depth and real size
5. Transform coordinates to base_link
6. Publish measurement results
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
from typing import List, Optional, Dict, Tuple
import time

# ROS2 messages
from arm_interfaces.msg import Detection2DArray, Detection2D, MeasuredObject
from geometry_msgs.msg import Point, Vector3, PointStamped
from std_msgs.msg import Header
from sensor_msgs.msg import CameraInfo

# TF2
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import tf2_geometry_msgs


class TriangulationNode(Node):
    """
    Dual-view triangulation node
    
    Subscriptions:
        /yolo/detections (Detection2DArray): YOLO 2D detection results
        /camera/camera_info (CameraInfo): Camera intrinsics
    
    Publications:
        /measured_objects (MeasuredObject[]): Measured 3D object information
    
    Parameters:
        baseline: Camera translation distance (meters)
        min_disparity: Minimum disparity (pixels)
        max_depth: Maximum measurement depth (meters)
        min_depth: Minimum measurement depth (meters)
        iou_threshold: Object matching IoU threshold
        pixel_uncertainty: Pixel uncertainty (for error estimation)
    """
    
    def __init__(self):
        super().__init__('triangulation_node')
        
        # Declare parameters
        self.declare_parameter('baseline', 0.20)  # joint1 rotation angle parameter
        self.declare_parameter('camera_radius', 0.33)  # Distance from camera to joint1 axis (meters)
        self.declare_parameter('depth_calibration_factor', 1.0)  # Depth calibration factor (actual depth / calculated depth)
        self.declare_parameter('min_disparity', 10.0)  # Minimum 10 pixel disparity
        self.declare_parameter('max_depth', 1.0)  # Maximum 1 meter
        self.declare_parameter('min_depth', 0.15)  # Minimum 15cm
        self.declare_parameter('iou_threshold', 0.3)  # IoU threshold
        self.declare_parameter('pixel_uncertainty', 1.0)  # 1 pixel uncertainty
        self.declare_parameter('enable_triangulation', True)  # Enable triangulation by default
        
        # Cube detection mode
        self.declare_parameter('detect_cube_only', False)  # Whether to detect cubes only
        self.declare_parameter('cube_aspect_ratio_tolerance', 0.3)  # Cube aspect ratio tolerance
        self.declare_parameter('cube_dimension_tolerance', 0.25)  # Cube 3D dimension tolerance
        self.declare_parameter('cube_min_edge_length', 0.03)  # Cube minimum edge length (meters) - default 3cm
        self.declare_parameter('cube_max_edge_length', 0.08)  # Cube maximum edge length (meters) - default 8cm
        self.declare_parameter('cube_max_x_position', 0.5)  # Cube maximum X coordinate (meters) - beyond arm workspace
        
        # Thickness estimation parameters
        self.declare_parameter('thickness_ratio_default', 0.5)  # Default thickness ratio
        
        # Declare separate parameters for each class (ROS2 doesn't support dict parameters)
        # COCO class IDs and their default thickness ratios
        thickness_class_defaults = {
            39: 0.3,   # bottle
            40: 0.4,   # wine glass
            41: 0.5,   # cup
            45: 0.6,   # bowl
            56: 0.2,   # chair
            60: 0.1,   # dining table
            63: 0.15,  # laptop
            64: 0.6,   # mouse
            66: 0.25,  # keyboard
            73: 0.08,  # book
            75: 0.4,   # vase
        }
        
        for class_id, default_ratio in thickness_class_defaults.items():
            param_name = f'thickness_ratio_class_{class_id}'
            self.declare_parameter(param_name, default_ratio)
        
        # Read parameters
        baseline_param = self.get_parameter('baseline').value
        camera_radius = self.get_parameter('camera_radius').value
        self.depth_calibration_factor = self.get_parameter('depth_calibration_factor').value
        self.min_disparity = self.get_parameter('min_disparity').value
        self.max_depth = self.get_parameter('max_depth').value
        self.min_depth = self.get_parameter('min_depth').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.pixel_uncertainty = self.get_parameter('pixel_uncertainty').value
        
        # Matching threshold parameters
        self.declare_parameter('class_mismatch_score_threshold', 0.20)
        self.declare_parameter('same_class_score_threshold', 0.40)
        self.class_mismatch_score_threshold = self.get_parameter('class_mismatch_score_threshold').value
        self.same_class_score_threshold = self.get_parameter('same_class_score_threshold').value
        
        # Position correction parameters (compensate for URDF vs actual camera mounting offset)
        # URDF (25°+1.5cm) calculation: X≈28.2cm, Z≈4.1cm
        # Actual target: X=30-33cm, Z=0-3cm
        # Compensation: X+2cm, Y+0cm, Z+0cm
        self.declare_parameter('position_correction_x', 0.02)  # +2cm
        self.declare_parameter('position_correction_y', 0.0)
        self.declare_parameter('position_correction_z', 0.0)
        self.position_correction_x = self.get_parameter('position_correction_x').value
        self.position_correction_y = self.get_parameter('position_correction_y').value
        self.position_correction_z = self.get_parameter('position_correction_z').value
        
        # Thickness estimation parameters
        self.thickness_ratio_default = self.get_parameter('thickness_ratio_default').value
        
        # Cube detection mode parameters
        self.detect_cube_only = self.get_parameter('detect_cube_only').value
        self.cube_aspect_ratio_tolerance = self.get_parameter('cube_aspect_ratio_tolerance').value
        self.cube_dimension_tolerance = self.get_parameter('cube_dimension_tolerance').value
        self.cube_min_edge_length = self.get_parameter('cube_min_edge_length').value
        self.cube_max_edge_length = self.get_parameter('cube_max_edge_length').value
        self.cube_max_x_position = self.get_parameter('cube_max_x_position').value
        
        # Read thickness ratios for all classes
        self.thickness_ratio_by_class = {}
        for class_id in thickness_class_defaults.keys():
            param_name = f'thickness_ratio_class_{class_id}'
            try:
                ratio = self.get_parameter(param_name).value
                self.thickness_ratio_by_class[class_id] = float(ratio)
            except Exception as e:
                self.get_logger().warn(f"Unable to read parameter {param_name}: {e}")
        
        # Save rotation parameters (for rotating stereo vision formulas)
        import math
        angle_deg = baseline_param * 100  # 0.15 → 15°
        angle_rad = math.radians(angle_deg)
        
        self.camera_radius = camera_radius  # Save camera radius
        self.rotation_angle = angle_rad      # Save rotation angle (radians)
        self.rotation_angle_deg = angle_deg  # Save rotation angle (degrees)
        
        # Calculate effective baseline (for error estimation only, not for depth calculation)
        self.baseline = 2 * camera_radius * math.sin(angle_rad / 2)
        
        self.get_logger().info(
            f"📐 Rotating stereo vision parameters:\n"
            f"   Rotation angle = {angle_deg:.1f}°\n"
            f"   Camera radius = {camera_radius:.3f}m\n"
            f"   Effective baseline = {self.baseline:.3f}m (for error estimation only)\n"
            f"   Depth calibration factor = {self.depth_calibration_factor:.3f} "
            f"{'(calibrated)' if self.depth_calibration_factor != 1.0 else '(uncalibrated)'}"
        )
        self.enable_triangulation = self.get_parameter('enable_triangulation').value
        
        # Camera intrinsics
        self.camera_params = {
            'fx': 1128.1,
            'fy': 1128.1,
            'cx': 320.0,
            'cy': 240.0
        }
        self.camera_info_received = False
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Store detection results (support three views)
        self.view1_detections: Optional[Detection2DArray] = None
        self.view2_detections: Optional[Detection2DArray] = None
        self.view3_detections: Optional[Detection2DArray] = None  # Add view3
        self.view1_time = None
        self.view2_time = None
        self.view3_time = None  # Add view3 timestamp
        
        # Store latest joint states (for precise object position calculation)
        self.latest_joint_states = None
        
        # QoS configuration
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscriptions
        self.sub_detections = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detections_callback,
            qos_profile
        )
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile
        )
        
        # Subscribe to joint states (for precise object position calculation)
        from sensor_msgs.msg import JointState
        self.sub_joint_states = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            qos_profile
        )
        
        # Publishers
        self.pub_measured = self.create_publisher(
            MeasuredObject,
            '/measured_objects',
            qos_profile
        )
        
        self.get_logger().info(
            f"🔺 TriangulationNode initialization complete\n"
            f"   Baseline: {self.baseline}m\n"
            f"   Min disparity: {self.min_disparity}px\n"
            f"   Depth range: {self.min_depth}-{self.max_depth}m\n"
            f"   IoU threshold: {self.iou_threshold}\n"
            f"   Matching threshold: same class≥{self.same_class_score_threshold:.2f}, different class≥{self.class_mismatch_score_threshold:.2f}\n"
            f"   Triangulation: {'✅ Enabled' if self.enable_triangulation else '❌ Disabled'}\n"
            f"   Thickness estimation strategy: default={self.thickness_ratio_default:.2f}, "
            f"{len(self.thickness_ratio_by_class)} class-specific configs\n"
            f"   🎲 Cube detection: {'✅ Enabled' if self.detect_cube_only else '❌ Disabled'}\n"
            f"      Edge length range: {self.cube_min_edge_length*100:.1f}-{self.cube_max_edge_length*100:.1f}cm\n"
            f"      Position limit: X ≤ {self.cube_max_x_position:.2f}m"
        )
    
    def get_thickness_ratio_for_class(self, class_id: int) -> float:
        """
        Get thickness ratio for class ID
        
        Args:
            class_id: COCO dataset class ID
            
        Returns:
            Thickness ratio for this class (thickness/width)
        """
        return self.thickness_ratio_by_class.get(class_id, self.thickness_ratio_default)
    
    def camera_info_callback(self, msg: CameraInfo):
        """Receive camera intrinsics"""
        if not self.camera_info_received:
            # Update camera parameters from CameraInfo
            K = msg.k  # 3x3 camera matrix, row-major
            self.camera_params['fx'] = K[0]
            self.camera_params['fy'] = K[4]
            self.camera_params['cx'] = K[2]
            self.camera_params['cy'] = K[5]
            
            self.camera_info_received = True
            self.get_logger().info(
                f"📷 Camera intrinsics received:\n"
                f"   fx={self.camera_params['fx']:.1f}, "
                f"fy={self.camera_params['fy']:.1f}\n"
                f"   cx={self.camera_params['cx']:.1f}, "
                f"cy={self.camera_params['cy']:.1f}"
            )
    
    def joint_states_callback(self, msg):
        """Receive joint states"""
        self.latest_joint_states = msg
    
    def detections_callback(self, msg: Detection2DArray):
        """
        Receive 2D detection results
        
        Strategy:
        - Parse site_id from frame_id (format: "camera_optical_frame|site:view1")
        - site_id="view1" → store as view1
        - site_id="view2" → store as view2 and trigger triangulation
        - After completion, reset and wait for next round
        """
        if not self.enable_triangulation:
            return
        
        # Parse site_id from frame_id
        # Format: "camera_optical_frame|site:view1"
        site_id = None
        frame_id = msg.header.frame_id
        if '|site:' in frame_id:
            parts = frame_id.split('|site:')
            if len(parts) == 2:
                site_id = parts[1]
        
        if site_id is None:
            self.get_logger().warn(
                f"⚠️  Unable to parse site_id from frame_id: {frame_id}"
            )
            return
        
        current_time = time.time()
        
        if site_id == "view1":
            # View1 (front)
            self.view1_detections = msg
            self.view1_time = current_time
            self.get_logger().info(
                f"📸 View1 (front): detected {len(msg.detections)} objects"
            )
        elif site_id == "view2":
            # View2 (-Y direction, -15°)
            self.view2_detections = msg
            self.view2_time = current_time
            self.get_logger().info(
                f"📸 View2 (-Y,-15°): detected {len(msg.detections)} objects"
            )
        elif site_id == "view3":
            # View3 (+Y direction, +15°)
            self.view3_detections = msg
            self.view3_time = current_time
            self.get_logger().info(
                f"📸 View3 (+Y,+15°): detected {len(msg.detections)} objects"
            )
            
            # All three views collected, trigger intelligent triangulation
            self._trigger_tri_view_triangulation()
        else:
            self.get_logger().warn(
                f"⚠️  Unknown site_id: {site_id}, expected 'view1', 'view2' or 'view3'"
            )
    
    def _trigger_tri_view_triangulation(self):
        """
        Intelligent tri-view triangulation
        
        Strategy:
        1. If all 3 views have data, select the best 2 for measurement
        2. If only 2 views have data, use those 2
        3. If only 1 view, report detection but cannot measure
        """
        current_time = time.time()
        
        # Collect valid views
        valid_views = []
        if self.view1_detections and len(self.view1_detections.detections) > 0:
            valid_views.append(('view1', self.view1_detections, self.view1_time))
        if self.view2_detections and len(self.view2_detections.detections) > 0:
            valid_views.append(('view2', self.view2_detections, self.view2_time))
        if self.view3_detections and len(self.view3_detections.detections) > 0:
            valid_views.append(('view3', self.view3_detections, self.view3_time))
        
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"🔺 Tri-view intelligent measurement\n"
            f"   View1(front): {len(self.view1_detections.detections) if self.view1_detections else 0} objects\n"
            f"   View2(-Y,-15°): {len(self.view2_detections.detections) if self.view2_detections else 0} objects\n"
            f"   View3(+Y,+15°): {len(self.view3_detections.detections) if self.view3_detections else 0} objects\n"
            f"   Valid views: {len(valid_views)}\n"
            f"{'='*60}"
        )
        
        if len(valid_views) >= 2:
            if len(valid_views) == 3:
                # All 3 views have data, try all possible pairings
                self.get_logger().info("   📊 Trying all possible view pairs for best measurement:")
                
                all_measured = {}  # Store all measurement results
                pairs = [
                    ("view1", "view2", self.view1_detections, self.view2_detections),
                    ("view1", "view3", self.view1_detections, self.view3_detections),
                    ("view2", "view3", self.view2_detections, self.view3_detections)
                ]
                
                # Try all pairs, collect all measurements
                all_measurements = []  # [(obj, view_pair), ...]
                
                for view_a, view_b, det_a, det_b in pairs:
                    self.get_logger().info(f"   🔍 Trying pair: {view_a.upper()} + {view_b.upper()}")
                    measured_objs = self._triangulate_pair_without_publish(det_a, det_b, view_a, view_b)
                    
                    for obj in measured_objs:
                        all_measurements.append((obj, f"{view_a}+{view_b}"))
                
                self.get_logger().info(f"   📊 Got {len(all_measurements)} total measurements, starting intelligent merge...")
                
                # Step 1: Initial grouping - compare each measurement with existing groups
                merged_groups = []  # Each group contains multiple measurements of the same object
                
                for obj, view_pair in all_measurements:
                    # Check if matches any existing group
                    matched_group = None
                    for group in merged_groups:
                        # Compare with any measurement in the group
                        ref_obj, _ = group[0]
                        if self._is_same_object_3d(obj, ref_obj):
                            matched_group = group
                            self.get_logger().info(
                                f"   🔗 Identified as same object: {obj.class_name}({view_pair}) "
                                f"and {ref_obj.class_name} distance={self._calc_distance_3d(obj, ref_obj):.3f}m"
                            )
                            break
                    
                    if matched_group:
                        matched_group.append((obj, view_pair))
                    else:
                        # Create new group
                        self.get_logger().info(f"   🆕 Found new object: {obj.class_name}({view_pair})")
                        merged_groups.append([(obj, view_pair)])
                
                # Step 2: Inter-group merging - check if different groups should merge
                self.get_logger().info(f"   🔄 Initial grouping complete, {len(merged_groups)} groups, starting inter-group merge...")
                i = 0
                while i < len(merged_groups):
                    j = i + 1
                    while j < len(merged_groups):
                        # Compare representative measurements of two groups
                        obj_i, _ = merged_groups[i][0]
                        obj_j, _ = merged_groups[j][0]
                        
                        if self._is_same_object_3d(obj_i, obj_j):
                            # Merge group j into group i
                            self.get_logger().info(
                                f"   🔗 Merging groups: {obj_j.class_name} → {obj_i.class_name} "
                                f"(distance={self._calc_distance_3d(obj_i, obj_j):.3f}m)"
                            )
                            merged_groups[i].extend(merged_groups[j])
                            merged_groups.pop(j)
                            # Don't increment j since list has changed
                        else:
                            j += 1
                    i += 1
                
                self.get_logger().info(f"   ✅ Inter-group merge complete, final {len(merged_groups)} objects")
                
                # Select best measurement from each group
                final_measured = []
                for group in merged_groups:
                    if len(group) == 1:
                        obj, view_pair = group[0]
                        final_measured.append(obj)
                        self.get_logger().info(
                            f"   ✅ Object {obj.class_name}: only 1 measurement ({view_pair})"
                        )
                    else:
                        # Multiple measurements, select the one with minimum depth error
                        best_tuple = min(group, key=lambda x: x[0].depth_error)
                        best_obj, best_view_pair = best_tuple
                        final_measured.append(best_obj)
                        
                        # Output merge info
                        labels = [x[0].class_name for x in group]
                        view_pairs = [x[1] for x in group]
                        self.get_logger().info(
                            f"   🎯 Merged object: {labels[0]} ← {len(group)} measurements\n"
                            f"      Identified as: {', '.join(set(labels))}\n"
                            f"      View pairs: {', '.join(view_pairs)}\n"
                            f"      Selected: {best_view_pair} (error={best_obj.depth_error*1000:.2f}mm)"
                        )
                
                # Cube filtering mode
                if self.detect_cube_only and len(final_measured) > 0:
                    self.get_logger().info(f"   🎲 Cube detection mode: filtering most cube-like objects...")
                    
                    # Calculate cube similarity for each object and filter invalid ones
                    cube_candidates = []
                    for obj in final_measured:
                        is_cube, cube_score = self.is_cube_like(obj)
                        
                        # Check if edge length is within reasonable range
                        dims = [obj.dimensions.x, obj.dimensions.y, obj.dimensions.z]
                        avg_edge = sum(dims) / 3.0
                        edge_in_range = (self.cube_min_edge_length <= avg_edge <= self.cube_max_edge_length)
                        
                        # Check if position is within arm workspace
                        position_valid = (obj.position.x <= self.cube_max_x_position)
                        
                        # Output detailed info
                        self.get_logger().info(
                            f"      • {obj.class_name}: cube score={cube_score:.3f} "
                            f"({'✅cube' if is_cube else '❌not cube'})\n"
                            f"        Average edge={avg_edge*100:.1f}cm "
                            f"({'✅' if edge_in_range else '❌'}range {self.cube_min_edge_length*100:.1f}-{self.cube_max_edge_length*100:.1f}cm)\n"
                            f"        Position X={obj.position.x:.3f}m "
                            f"({'✅' if position_valid else '❌'}≤{self.cube_max_x_position:.2f}m)"
                        )
                        
                        # Intelligent filtering strategy:
                        # 1. Edge length and position are hard requirements (must satisfy)
                        # 2. Cube similarity is a soft requirement:
                        #    - If multiple candidates, only keep is_cube=True
                        #    - If only 1 candidate, keep even if is_cube=False (avoid false negatives)
                        if edge_in_range and position_valid:
                            # Passed hard requirements, add to candidates
                            cube_candidates.append((obj, cube_score, is_cube))
                            if is_cube:
                                self.get_logger().info(
                                    f"        ✅ Passed all filtering conditions, added to candidate list"
                                )
                            else:
                                self.get_logger().info(
                                    f"        ⚠️  Passed hard requirements (edge+position), added to candidate list (low cube score)"
                                )
                        else:
                            reasons = []
                            if not edge_in_range:
                                reasons.append(f"edge out of range({avg_edge*100:.1f}cm)")
                            if not position_valid:
                                reasons.append(f"position out of range(X={obj.position.x:.3f}m)")
                            self.get_logger().info(
                                f"        ❌ Filtered: {', '.join(reasons)}"
                            )
                    
                    # Select best candidate
                    if cube_candidates:
                        # If multiple candidates, prefer is_cube=True
                        true_cubes = [c for c in cube_candidates if c[2]]  # c[2] is is_cube
                        
                        if true_cubes:
                            # Have true cubes, select highest score
                            best_cube = max(true_cubes, key=lambda x: x[1])
                            best_obj, best_score, is_best_cube = best_cube
                            self.get_logger().info(
                                f"   🎯 Selected best cube candidate: {best_obj.class_name}\n"
                                f"      Score={best_score:.3f} (is_cube=True)\n"
                                f"      Dimensions=[{best_obj.dimensions.x*100:.1f}, "
                                f"{best_obj.dimensions.y*100:.1f}, {best_obj.dimensions.z*100:.1f}]cm\n"
                                f"      Position=[{best_obj.position.x:.3f}, "
                                f"{best_obj.position.y:.3f}, {best_obj.position.z:.3f}]m"
                            )
                        else:
                            # No true cubes, but have candidates (edge length and position OK)
                            # Select highest scoring candidate
                            best_cube = max(cube_candidates, key=lambda x: x[1])
                            best_obj, best_score, is_best_cube = best_cube
                            self.get_logger().info(
                                f"   🎯 Selected best candidate (low cube score, but dimensions/position suitable): {best_obj.class_name}\n"
                                f"      Score={best_score:.3f} (is_cube=False)\n"
                                f"      Dimensions=[{best_obj.dimensions.x*100:.1f}, "
                                f"{best_obj.dimensions.y*100:.1f}, {best_obj.dimensions.z*100:.1f}]cm\n"
                                f"      Position=[{best_obj.position.x:.3f}, "
                                f"{best_obj.position.y:.3f}, {best_obj.position.z:.3f}]m\n"
                                f"      ⚠️  Note: Object may not be standard cube, please verify grasp effect"
                            )
                        
                        # Keep only highest scoring object
                        final_measured = [best_obj]
                    else:
                        self.get_logger().warn(
                            f"   ⚠️  No cube candidates found meeting requirements\n"
                            f"      Requirements: edge length {self.cube_min_edge_length*100:.1f}-{self.cube_max_edge_length*100:.1f}cm, "
                            f"position X≤{self.cube_max_x_position:.2f}m"
                        )
                        # Clear list, don't publish any results
                        final_measured = []
                
                # Batch publish
                for obj in final_measured:
                    self.pub_measured.publish(obj)
                
                self.get_logger().info(
                    f"   ✅ Tri-view complete: {len(final_measured)} objects "
                    f"({sum(len(m) for m in all_measured.values())} attempts)\n{'='*60}"
                )

                self.get_logger().info("   ✅ All pairing attempts complete, system will automatically select best measurement")
            else:
                # Only 2 views, use these 2
                view_a, det_a, _ = valid_views[0]
                view_b, det_b, _ = valid_views[1]
                self.get_logger().info(f"   📊 Using {view_a.upper()}+{view_b.upper()} for measurement")
                self.perform_triangulation_pair(det_a, det_b, view_a, view_b)
        elif len(valid_views) == 1:
            view_name, detections, _ = valid_views[0]
            self.get_logger().warn(
                f"   ⚠️  Only {view_name.upper()} detected objects, cannot triangulate\n"
                f"      Detected: {[d.class_name for d in detections.detections]}"
            )
        else:
            self.get_logger().warn("   ⚠️  No objects detected in any view")
        
        # Reset all view states
        self.view1_detections = None
        self.view2_detections = None
        self.view3_detections = None
        self.view1_time = None
        self.view2_time = None
        self.view3_time = None
    
    def _calculate_avg_confidence(self, detections: Detection2DArray) -> float:
        """Calculate average confidence of detection results"""
        if not detections or len(detections.detections) == 0:
            return 0.0
        return sum(d.confidence for d in detections.detections) / len(detections.detections)
    
    def _calc_distance_3d(self, obj1: 'MeasuredObject', obj2: 'MeasuredObject') -> float:
        """Calculate 3D Euclidean distance between two objects"""
        import math
        dx = obj1.position.x - obj2.position.x
        dy = obj1.position.y - obj2.position.y
        dz = obj1.position.z - obj2.position.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def _is_same_object_3d(self, obj1: 'MeasuredObject', obj2: 'MeasuredObject') -> bool:
        """
        Determine if two measurements are the same object (based on 3D spatial similarity)
        
        Criteria:
        1. Class overlap (intersection of class names)
        2. 3D position proximity (Euclidean distance, adaptive threshold)
        3. 3D size similarity (volume ratio > 0.3)
        
        Args:
            obj1: First measurement object
            obj2: Second measurement object
            
        Returns:
            True if same object, False otherwise
        """
        # Calculate 3D position distance
        distance = self._calc_distance_3d(obj1, obj2)
        
        # Calculate volume ratio
        vol1 = obj1.dimensions.x * obj1.dimensions.y * obj1.dimensions.z
        vol2 = obj2.dimensions.x * obj2.dimensions.y * obj2.dimensions.z
        volume_ratio = min(vol1, vol2) / max(vol1, vol2) if max(vol1, vol2) > 0 else 0
        
        # Check class overlap
        labels1 = set(obj1.class_name.split('&'))
        labels2 = set(obj2.class_name.split('&'))
        has_label_overlap = len(labels1 & labels2) > 0
        
        # Adaptive distance threshold: adjust based on depth
        avg_depth = (obj1.depth_from_camera + obj2.depth_from_camera) / 2.0
        # Base threshold 10cm, add 10cm tolerance per meter depth
        position_threshold = 0.10 + (avg_depth * 0.10)
        
        # Volume threshold: relaxed to 30% (due to larger error from different viewpoints)
        volume_threshold = 0.3
        
        # Logic:
        # 1. If class overlap exists, relax distance requirement (1.5x)
        # 2. Must satisfy both distance and volume conditions
        if has_label_overlap:
            position_threshold *= 1.5
        
        is_same = (distance < position_threshold and volume_ratio > volume_threshold)
        
        if is_same:
            self.get_logger().debug(
                f"      ✓ Identified as same object: {obj1.class_name} ≈ {obj2.class_name} "
                f"(distance={distance*100:.1f}cm<{position_threshold*100:.1f}cm, "
                f"volume_ratio={volume_ratio:.2f}, class_overlap={has_label_overlap})"
            )
        
        return is_same
    
    def _triangulate_pair_without_publish(self, det1_array: Detection2DArray, det2_array: Detection2DArray, 
                                          view1_name: str, view2_name: str):
        """
        Execute triangulation on a pair of views, but return results without publishing
        
        Args:
            det1_array: Detection results from first view
            det2_array: Detection results from second view
            view1_name: First view name (for logging)
            view2_name: Second view name (for logging)
            
        Returns:
            List of measurement results (MeasuredObject objects)
        """
        measured_objects = []
        
        # Match and measure each detection in view1
        for det1 in det1_array.detections:
            # Find matching object in view2
            det2 = self.find_matching_object(det1, det2_array.detections)
            
            if det2 is None:
                continue
            
            # Execute triangulation
            measured_obj = self.triangulate(det1, det2)
            
            if measured_obj is not None:
                measured_objects.append(measured_obj)
        
        return measured_objects
    
    def perform_triangulation_pair(self, det1_array: Detection2DArray, det2_array: Detection2DArray, 
                                   view1_name: str, view2_name: str):
        """
        Execute triangulation on a pair of views
        
        Args:
            det1_array: Detection results from first view
            det2_array: Detection results from second view
            view1_name: First view name (for logging)
            view2_name: Second view name (for logging)
        """
        self.get_logger().info(
            f"   🔍 Pair measurement: {view1_name.upper()} ({len(det1_array.detections)} objs) + "
            f"{view2_name.upper()} ({len(det2_array.detections)} objs)"
        )
        
        # Temporarily set view1 and view2, reuse existing perform_triangulation logic
        original_view1 = self.view1_detections
        original_view2 = self.view2_detections
        
        self.view1_detections = det1_array
        self.view2_detections = det2_array
        
        # Call existing triangulation logic
        self.perform_triangulation()
        
        # Restore original values (will be reset soon anyway)
        self.view1_detections = original_view1
        self.view2_detections = original_view2
    
    def perform_triangulation(self):
        """Execute triangulation"""
        if self.view1_detections is None or self.view2_detections is None:
            return
        
        self.get_logger().info(
            f"\n{'='*60}\n"
            f"🔺 Starting triangulation\n"
            f"   View1: {len(self.view1_detections.detections)} objects\n"
            f"   View2: {len(self.view2_detections.detections)} objects\n"
            f"{'='*60}"
        )
        
        measured_count = 0
        measured_objects = []  # Collect all measurement results
        
        # Match and measure each detection in view1
        for det1 in self.view1_detections.detections:
            # Find matching object in view2
            det2 = self.find_matching_object(det1, self.view2_detections.detections)
            
            if det2 is None:
                self.get_logger().debug(
                    f"   ⚠️  {det1.class_name}: No matching object found"
                )
                continue
            
            # Calculate disparity for debugging
            u1, v1 = det1.bbox_center
            u2, v2 = det2.bbox_center
            disparity = abs(u1 - u2)
            self.get_logger().debug(
                f"   📏 {det1.class_name}: disparity={disparity:.1f}px "
                f"(u1={u1:.1f}, u2={u2:.1f})"
            )
            
            # Execute triangulation
            measured_obj = self.triangulate(det1, det2)
            
            if measured_obj is not None:
                # Collect measurement results (batch publish later)
                measured_objects.append(measured_obj)
                measured_count += 1
                
                self.get_logger().info(
                    f"   ✅ {measured_obj.class_name}:\n"
                    f"      Disparity: {measured_obj.disparity:.1f}px\n"
                    f"      Depth: {measured_obj.depth_from_camera:.3f}m\n"
                    f"      Dimensions: {measured_obj.dimensions.x:.3f}×"
                    f"{measured_obj.dimensions.y:.3f}×{measured_obj.dimensions.z:.3f}m\n"
                    f"      Position: ({measured_obj.position.x:.3f}, "
                    f"{measured_obj.position.y:.3f}, {measured_obj.position.z:.3f})\n"
                    f"      Error: depth±{measured_obj.depth_error*1000:.1f}mm, "
                    f"size±{measured_obj.size_error*1000:.1f}mm"
                )
        
        # Batch publish all measurement results
        for obj in measured_objects:
            self.pub_measured.publish(obj)
        
        self.get_logger().info(
            f"{'='*60}\n"
            f"🎯 Triangulation complete: {measured_count}/{len(self.view1_detections.detections)} objects\n"
            f"   Batch published {len(measured_objects)} measurement results\n"
            f"{'='*60}\n"
        )
    
    def find_matching_object(
        self, 
        det1: Detection2D, 
        detections2: List[Detection2D]
    ) -> Optional[Detection2D]:
        """
        Find matching object in second view
        
        Hybrid matching strategy:
        1. Must be same class
        2. Prefer IoU (suitable for large objects/small angle changes)
        3. If IoU too low, use center distance + size similarity (suitable for small objects/large angle changes)
        
        Args:
            det1: Detection result from view1
            detections2: All detection results from view2
        
        Returns:
            Matched detection result, or None if no match
        """
        best_match = None
        best_score = 0.0
        matching_method = ""
        
        self.get_logger().debug(
            f"   🔍 Matching {det1.class_name} (class_id={det1.class_id})"
        )
        
        u1, v1 = det1.bbox_center
        w1, h1 = det1.bbox_width, det1.bbox_height
        
        for det2 in detections2:
            # Allow different classes, but require stricter position and size matching
            class_mismatch = (det2.class_id != det1.class_id)
            
            if class_mismatch:
                self.get_logger().debug(
                    f"      Class difference detected: view1={det1.class_name}, view2={det2.class_name} (strict matching required)"
                )
            
            # Calculate IoU
            iou = self.calculate_iou(det1.bbox, det2.bbox)
            
            # Calculate center distance (pixels)
            u2, v2 = det2.bbox_center
            w2, h2 = det2.bbox_width, det2.bbox_height
            center_dist = np.sqrt((u1 - u2)**2 + (v1 - v2)**2)
            
            # Calculate size similarity
            size1 = w1 * h1
            size2 = w2 * h2
            size_ratio = min(size1, size2) / max(size1, size2) if max(size1, size2) > 0 else 0
            
            # If classes differ, require stricter size matching
            if class_mismatch and size_ratio < 0.6:
                # Size difference >40%, likely different objects, skip
                self.get_logger().debug(
                    f"      Skip: different class and large size difference (size_ratio={size_ratio:.2f} < 0.6)"
                )
                continue
            
            # Hybrid scoring strategy
            if iou > self.iou_threshold:
                # IoU match successful (large objects/small angle changes)
                score = iou
                method = f"IoU={iou:.3f}"
            else:
                # IoU too low, use center distance + size similarity (small objects/large angle changes)
                # Center distance threshold: 50% of image diagonal
                max_center_dist = 320  # 640x480 image, allow 50% diagonal distance
                center_score = max(0, 1 - center_dist / max_center_dist)
                
                # Combined score: center distance weight 0.7, size similarity weight 0.3
                score = center_score * 0.7 + size_ratio * 0.3
                method = f"Center={center_dist:.1f}px, Size={size_ratio:.2f}, Score={score:.3f}"
                
                # If classes differ, use stricter threshold (configurable)
                min_score = self.class_mismatch_score_threshold if class_mismatch else self.same_class_score_threshold
                if score < min_score:
                    score = 0
            
            # Output candidate info including size similarity
            candidate_info = f"{det2.class_name}, {method}"
            if class_mismatch:
                candidate_info += f" [different class, size_similarity={size_ratio:.2f}]"
            
            self.get_logger().debug(f"      Candidate: {candidate_info}")
            
            if score > best_score:
                best_score = score
                best_match = det2
                matching_method = method
        
        if best_match is not None:
            self.get_logger().debug(
                f"      ✅ Match found: {best_match.class_name}, {matching_method}"
            )
        else:
            self.get_logger().debug(
                f"      ❌ No match found (best_score={best_score:.3f})"
            )
        
        return best_match
    
    def calculate_iou(self, bbox1: List[float], bbox2: List[float]) -> float:
        """
        Calculate IoU (Intersection over Union) of two bounding boxes
        
        Args:
            bbox1: [x1, y1, x2, y2]
            bbox2: [x1, y1, x2, y2]
        
        Returns:
            IoU value (0-1)
        """
        # Calculate intersection
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        if x2 < x1 or y2 < y1:
            return 0.0
        
        intersection = (x2 - x1) * (y2 - y1)
        
        # Calculate union
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        union = area1 + area2 - intersection
        
        if union == 0:
            return 0.0
        
        return intersection / union
    
    def is_cube_like(self, obj: MeasuredObject) -> Tuple[bool, float]:
        """
        Determine if object is cube-like
        
        Cube criteria:
        1. bbox aspect ratio close to 1:1 (when viewed from above)
        2. 3D dimensions approximately equal (width ≈ depth ≈ height)
        3. Combined score above threshold
        
        Args:
            obj: Measured object
        
        Returns:
            (is_cube, cube_score): Whether it's a cube, cube similarity score (0-1)
        """
        try:
            # Get 3D dimensions
            width = obj.dimensions.x   # Width
            depth = obj.dimensions.y   # Length/depth
            height = obj.dimensions.z  # Height/thickness
            
            # Prevent division by zero
            if width == 0 or depth == 0 or height == 0:
                return False, 0.0
            
            # 1. bbox aspect ratio check (when viewed from above, cube should be square-like)
            bbox_width = obj.bbox_width if hasattr(obj, 'bbox_width') else 0
            bbox_height = obj.bbox_height if hasattr(obj, 'bbox_height') else 0
            
            if bbox_width > 0 and bbox_height > 0:
                bbox_aspect_ratio = bbox_width / bbox_height
                # Ideal value 1.0, allowed deviation controlled by cube_aspect_ratio_tolerance
                bbox_score = 1.0 - min(1.0, abs(bbox_aspect_ratio - 1.0) / self.cube_aspect_ratio_tolerance)
            else:
                bbox_score = 0.5  # No bbox info, give medium score
            
            # 2. 3D dimension ratio check
            # Calculate similarity of three edge lengths
            dims = sorted([width, depth, height])  # Sort from small to large
            min_dim, mid_dim, max_dim = dims
            
            # Relative difference (relative to smallest edge)
            if min_dim > 0:
                mid_diff = abs(mid_dim - min_dim) / min_dim
                max_diff = abs(max_dim - min_dim) / min_dim
                
                # Ideal cube: all edges equal, difference is 0
                # Maximum allowed difference controlled by cube_dimension_tolerance
                mid_score = 1.0 - min(1.0, mid_diff / self.cube_dimension_tolerance)
                max_score = 1.0 - min(1.0, max_diff / self.cube_dimension_tolerance)
                
                dim_score = (mid_score + max_score) / 2.0
            else:
                dim_score = 0.0
            
            # 3. Combined score (bbox 40%, dimensions 60%)
            cube_score = bbox_score * 0.4 + dim_score * 0.6
            
            # Judgment threshold: score > 0.6 considered cube
            is_cube = cube_score > 0.6
            
            # Debug log
            self.get_logger().debug(
                f"   🔍 Cube judgment: {obj.class_name}\n"
                f"      Dimensions: W={width*100:.1f}cm, D={depth*100:.1f}cm, H={height*100:.1f}cm\n"
                f"      bbox aspect ratio: {bbox_aspect_ratio:.2f} (score: {bbox_score:.2f})\n"
                f"      Dimension similarity: {dim_score:.2f}\n"
                f"      Combined score: {cube_score:.2f} → {'✅cube' if is_cube else '❌not cube'}"
            )
            
            return is_cube, cube_score
            
        except Exception as e:
            self.get_logger().warn(f"   ⚠️  Cube judgment failed: {e}")
            return False, 0.0
    
    def triangulate(
        self, 
        det1: Detection2D, 
        det2: Detection2D
    ) -> Optional[MeasuredObject]:
        """
        Dual-view triangulation
        
        Core formulas:
        1. Disparity: disparity = |u1 - u2|
        2. Depth: depth = (fx × baseline) / disparity
        3. Width: width = (depth × w_px) / fx
        4. Height: height = (depth × h_px) / fy
        
        Args:
            det1: Detection result from view1
            det2: Detection result from view2
        
        Returns:
            Measurement result, or None if measurement fails
        """
        fx = self.camera_params['fx']
        fy = self.camera_params['fy']
        cx = self.camera_params['cx']
        cy = self.camera_params['cy']
        
        # 1. Calculate disparity
        u1, v1 = det1.bbox_center
        u2, v2 = det2.bbox_center
        disparity = abs(u1 - u2)
        
        # Check if disparity is large enough
        if disparity < self.min_disparity:
            self.get_logger().warn(
                f"   ⚠️  {det1.class_name}: Disparity too small ({disparity:.1f}px < {self.min_disparity}px)"
            )
            return None
        
        # 2. Depth calculation
        # 
        # First query TF transform to understand actual geometric relationships
        # 
        import math
        
        # Extract real camera_frame (remove |site:xxx suffix)
        camera_frame_real = det1.camera_frame
        if '|site:' in camera_frame_real:
            camera_frame_real = camera_frame_real.split('|site:')[0]
        
        # Query and save TF transform (for subsequent size calculation)
        tf_transform = None
        try:
            tf_transform = self.tf_buffer.lookup_transform(
                'base_link',
                camera_frame_real,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # Extract transform information
            t = tf_transform.transform.translation
            r = tf_transform.transform.rotation
            
            self.get_logger().debug(
                f"   📡 TF transform ({camera_frame_real} → base_link):\n"
                f"      Translation: x={t.x:.4f}, y={t.y:.4f}, z={t.z:.4f}\n"
                f"      Rotation: x={r.x:.4f}, y={r.y:.4f}, z={r.z:.4f}, w={r.w:.4f}"
            )
        except Exception as e:
            self.get_logger().warn(f"   ⚠️  Unable to query TF: {e}")
        
        # Use pre-calculated effective baseline (temporary method)
        depth_raw = (fx * self.baseline) / disparity
        
        # Apply calibration factor
        depth = depth_raw * self.depth_calibration_factor
        
        # 3. Get bbox size (for subsequent calculation)
        w1_px = det1.bbox_width
        h1_px = det1.bbox_height
        
        # Detailed debug information
        calibration_note = ""
        if self.depth_calibration_factor != 1.0:
            calibration_note = f"\n      Calibration: {depth_raw:.4f}m × {self.depth_calibration_factor:.3f} = {depth:.4f}m"
        
        self.get_logger().info(
            f"   🔬 Depth calculation ({det1.class_name}):\n"
            f"      fx = {fx:.1f}px\n"
            f"      baseline_effective = {self.baseline:.4f}m\n"
            f"      disparity = {disparity:.1f}px\n"
            f"      Formula: depth = (fx × baseline) / disparity\n"
            f"      depth_raw = ({fx:.1f} × {self.baseline:.4f}) / {disparity:.1f}"
            f"{calibration_note}\n"
            f"      ✅ depth = {depth:.4f}m = {depth*100:.1f}cm\n"
            f"      bbox_width = {w1_px:.1f}px\n"
            f"      bbox_height = {h1_px:.1f}px"
        )
        
        # Check depth range
        if depth < self.min_depth or depth > self.max_depth:
            self.get_logger().warn(
                f"   ⚠️  {det1.class_name}: Depth out of range ({depth:.3f}m, range: {self.min_depth}-{self.max_depth}m)"
            )
            return None
        
        # 4. Real size calculation (consider camera tilt)
        
        # Method: Use TF rotation matrix to project bbox corners to 3D space, calculate real size
        if tf_transform is not None:
            try:
                self.get_logger().debug(f"   🔧 Starting geometric correction calculation...")
                
                # Get rotation matrix
                from scipy.spatial.transform import Rotation
                quat = [
                    tf_transform.transform.rotation.x,
                    tf_transform.transform.rotation.y,
                    tf_transform.transform.rotation.z,
                    tf_transform.transform.rotation.w
                ]
                rot = Rotation.from_quat(quat)
                rot_matrix = rot.as_matrix()
                
                self.get_logger().debug(f"   🔧 Rotation matrix obtained successfully")
                
                # bbox four corners in image coordinate system normalized coordinates
                # bbox format: [x1, y1, x2, y2]
                bbox = det1.bbox
                corners_2d = [
                    [(bbox[0] - cx) / fx, (bbox[1] - cy) / fy],  # Top-left
                    [(bbox[2] - cx) / fx, (bbox[1] - cy) / fy],  # Top-right
                    [(bbox[0] - cx) / fx, (bbox[3] - cy) / fy],  # Bottom-left
                    [(bbox[2] - cx) / fx, (bbox[3] - cy) / fy],  # Bottom-right
                ]
                
                self.get_logger().debug(
                    f"   🔧 Corners 2D: top_left={corners_2d[0]}, top_right={corners_2d[1]}, "
                    f"bottom_left={corners_2d[2]}, bottom_right={corners_2d[3]}"
                )
                
                # Project 2D corners to 3D (camera coordinate system)
                corners_3d_cam = []
                for x_n, y_n in corners_2d:
                    # In camera coordinate system, assume depth is depth
                    point_cam = np.array([x_n * depth, y_n * depth, depth])
                    corners_3d_cam.append(point_cam)
                
                self.get_logger().debug(f"   🔧 3D corners (camera frame): {len(corners_3d_cam)} points")
                
                # Transform to base_link coordinate system
                t_vec = np.array([
                    tf_transform.transform.translation.x,
                    tf_transform.transform.translation.y,
                    tf_transform.transform.translation.z
                ])
                
                corners_3d_base = []
                for point_cam in corners_3d_cam:
                    point_base = rot_matrix @ point_cam + t_vec
                    corners_3d_base.append(point_base)
                
                self.get_logger().debug(f"   🔧 3D corners (base frame): {len(corners_3d_base)} points")
                
                # Calculate bbox size in base_link
                corners_array = np.array(corners_3d_base)
                
                # Width: average distance of left and right column corners
                width_top = np.linalg.norm(corners_array[1] - corners_array[0])
                width_bottom = np.linalg.norm(corners_array[3] - corners_array[2])
                real_width = (width_top + width_bottom) / 2
                
                # Height: average distance of top and bottom row corners
                height_left = np.linalg.norm(corners_array[2] - corners_array[0])
                height_right = np.linalg.norm(corners_array[3] - corners_array[1])
                measured_height = (height_left + height_right) / 2
                measured_width = (width_top + width_bottom) / 2
                
                # === Reverse solve for real object size ===
                # Measurement explanation:
                # - measured_width: bbox width direction, corresponds to object real width (accurate)
                # - measured_height: bbox height direction, includes length projection + thickness projection
                
                # Camera tilt angle (calculated from rotation matrix)
                import math
                # Extract camera Z-axis direction from rotation matrix
                camera_z_in_base = rot_matrix @ np.array([0, 0, 1])
                camera_z_component = -camera_z_in_base[2]  # Downward is positive
                camera_xy_component = np.sqrt(camera_z_in_base[0]**2 + camera_z_in_base[1]**2)
                tilt_angle_rad = math.atan2(camera_z_component, camera_xy_component)
                tilt_angle_deg = math.degrees(tilt_angle_rad)
                
                # Calculate bbox aspect ratio to judge object orientation
                bbox_aspect_ratio = measured_height / measured_width if measured_width > 0.001 else 1.0
                
                # If bbox height is much greater than width (aspect_ratio > 2.5), consider it as upright elongated object
                # For example: upright bottle, toothbrush, pen, etc.
                is_vertical_object = bbox_aspect_ratio > 2.5
                
                if is_vertical_object:
                    # Vertical object processing logic
                    # Key: Vertical object bbox height directly corresponds to real height, no tilt angle correction needed!
                    # bbox height → object real height (Y-axis, because camera observes tilted)
                    # bbox width → object cross-section diameter (X and Z)
                    object_width = measured_width       # X-axis width (horizontal)
                    object_length = measured_height     # Y-axis height (vertical direction, bbox height directly maps)
                    object_thickness = measured_width   # Z-axis thickness (assume circular/square cross-section)
                    
                    self.get_logger().info(
                        f"   🔍 Detected vertical object (aspect_ratio={bbox_aspect_ratio:.2f}): "
                        f"bbox_width={measured_width*100:.1f}cm, bbox_height={measured_height*100:.1f}cm"
                    )
                else:
                    # Horizontal/flat object original processing logic
                    # measured_height = length_projection + thickness_projection
                    # measured_height = L × cos(tilt) + T × sin(tilt)
                    
                    # 🎲 Cube detection mode: force use thickness_ratio=1.0
                    if self.detect_cube_only:
                        thickness_ratio = 1.0
                    else:
                        # Get thickness ratio by class (can be adjusted in config file)
                        thickness_ratio = self.get_thickness_ratio_for_class(det1.class_id)
                    estimated_thickness = measured_width * thickness_ratio
                    
                    # Solve for length
                    # L = (measured_height - T × sin(tilt)) / cos(tilt)
                    cos_tilt = math.cos(tilt_angle_rad)
                    sin_tilt = math.sin(tilt_angle_rad)
                    
                    if cos_tilt > 0.01:  # Avoid division by zero
                        object_length = (measured_height - estimated_thickness * sin_tilt) / cos_tilt
                    else:
                        object_length = measured_height  # Fallback solution
                    
                    object_width = measured_width  # Directly use measured width (accurate)
                    object_thickness = estimated_thickness
                    
                    self.get_logger().debug(
                        f"   🔍 Horizontal object calculation details: "
                        f"thickness_ratio={thickness_ratio:.2f}, "
                        f"estimated_thickness={estimated_thickness*100:.1f}cm, "
                        f"cos(tilt)={cos_tilt:.3f}, sin(tilt)={sin_tilt:.3f}"
                    )
                
                
                object_length = max(0.01, min(1.0, object_length))
                object_width = max(0.01, min(1.0, object_width))
                object_thickness = max(0.01, min(1.0, object_thickness))
                
                # Assign to final dimensions (for JSON output)
                # dimensions: x=width, y=depth(length), z=height(thickness)
                real_width = object_width    # X direction
                real_depth = object_length   # Y direction (length)
                real_height = object_thickness  # Z direction (thickness)
                
                # Debug information (changed to debug level)
                simple_width = (depth * w1_px) / fx
                simple_height = (depth * h1_px) / fy
                
                self.get_logger().debug(
                    f"   � Geometric calculation completed: measured={measured_width*100:.2f}cm × {measured_height*100:.2f}cm"
                )
                self.get_logger().debug(
                    f"   🔧 bbox comparison: simple={simple_width*100:.1f}cm × {simple_height*100:.1f}cm, "
                    f"geometric={measured_width*100:.1f}cm × {measured_height*100:.1f}cm"
                )
                
                # Output real object size (keep info level)
                object_type = "Vertical elongated" if is_vertical_object else "Horizontal/flat"
                self.get_logger().info(
                    f"   📦 Real object dimensions ({det1.class_name}) [{object_type}]:\n"
                    f"      Camera tilt angle: {tilt_angle_deg:.1f}°\n"
                    f"      bbox aspect ratio: {bbox_aspect_ratio:.2f}\n"
                    f"      Solution result:\n"
                    f"        Width(X): {object_width*100:.1f}cm\n"
                    f"        Length(Y): {object_length*100:.1f}cm\n"
                    f"        Thickness(Z): {object_thickness*100:.1f}cm"
                )
                
            except Exception as e:
                self.get_logger().warn(
                    f"   ⚠️  3D size calculation failed, using simplified formula: {e}"
                )
                import traceback
                self.get_logger().warn(f"   Stack trace: {traceback.format_exc()}")
                # Fallback to simplified formula
                real_width = (depth * w1_px) / fx
                real_height = (depth * h1_px) / fy
                real_depth = real_width * 0.5
        else:
            # TF not obtained, using simplified formula
            self.get_logger().warn(f"   ⚠️  TF transform not obtained, using simplified formula")
            real_width = (depth * w1_px) / fx
            real_height = (depth * h1_px) / fy
            real_depth = real_width * 0.5
        
        # 5. 3D position calculation (in camera coordinate system)
        # Using pinhole camera model: 
        # X_cam = (u - cx) / fx * Z_cam
        # Y_cam = (v - cy) / fy * Z_cam
        # Z_cam = depth
        #
        # Here depth is the object depth calculated through stereo disparity
        # bbox center (u1,v1) projected to 3D space to get object center position
        #
        u1_center = u1
        v1_center = v1
        
        # Normalized coordinates (note: fx, fy, cx, cy already defined at function start)
        xn = (u1_center - cx) / fx
        yn = (v1_center - cy) / fy
        
        # 3D coordinates (camera coordinate system)
        # Note: camera coordinate system defined as X-right Y-down Z-forward
        point_cam_array = np.array([xn * depth, yn * depth, depth])
        
        self.get_logger().info(
            f"   🎯 3D position calculation details:\n"
            f"      bbox center: u={u1_center:.1f}, v={v1_center:.1f}\n"
            f"      Image center: cx={cx:.1f}, cy={cy:.1f}\n"
            f"      Normalized coordinates: xn={xn:.4f}, yn={yn:.4f}\n"
            f"      Depth: depth={depth:.3f}m\n"
            f"      Camera coordinates: point_cam=[{point_cam_array[0]:.3f}, {point_cam_array[1]:.3f}, {point_cam_array[2]:.3f}]m"
        )
        
        # 6. Transform to base_link (using TF)
        if tf_transform is None:
            # If TF not obtained before, try again
            camera_frame_real = det1.camera_frame
            if '|site:' in camera_frame_real:
                camera_frame_real = camera_frame_real.split('|site:')[0]
            
            try:
                tf_transform = self.tf_buffer.lookup_transform(
                    'base_link',
                    camera_frame_real,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
            except (LookupException, ConnectivityException, ExtrapolationException) as e:
                self.get_logger().error(
                    f"   ❌ TF transform failed: {str(e)}\n"
                    f"   Please ensure TF tree correctly publishes {camera_frame_real} -> base_link transform\n"
                    f"   Skip measurement for this object"
                )
                return None
        
        # Use obtained TF transform
        try:
            # Create PointStamped message
            point_stamped = PointStamped()
            point_stamped.header.frame_id = camera_frame_real
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = point_cam_array[0]
            point_stamped.point.y = point_cam_array[1]
            point_stamped.point.z = point_cam_array[2]
            
            # Use TF2 transform
            point_transformed = tf2_geometry_msgs.do_transform_point(
                point_stamped, 
                tf_transform
            )
            
            position_base = np.array([
                point_transformed.point.x,
                point_transformed.point.y,
                point_transformed.point.z
            ])
            
            # Manual TF transform verification (for debugging)
            from scipy.spatial.transform import Rotation
            quat = [
                tf_transform.transform.rotation.x,
                tf_transform.transform.rotation.y,
                tf_transform.transform.rotation.z,
                tf_transform.transform.rotation.w
            ]
            rot = Rotation.from_quat(quat)
            rot_matrix = rot.as_matrix()
            t_vec = np.array([
                tf_transform.transform.translation.x,
                tf_transform.transform.translation.y,
                tf_transform.transform.translation.z
            ])
            
            # Manual calculation: point_base = R @ point_cam + T
            point_base_manual = rot_matrix @ point_cam_array + t_vec
            
            # Output current joint states (for debugging)
            joint_info = "Unknown"
            if self.latest_joint_states is not None:
                try:
                    joint_dict = {}
                    for i, name in enumerate(self.latest_joint_states.name):
                        if i < len(self.latest_joint_states.position):
                            joint_dict[name] = self.latest_joint_states.position[i]
                    
                    # Extract arm joints
                    arm_joints = []
                    for j in range(1, 6):
                        joint_name = f"joint{j}"
                        if joint_name in joint_dict:
                            angle_rad = joint_dict[joint_name]
                            angle_deg = math.degrees(angle_rad)
                            arm_joints.append(f"{joint_name}={angle_deg:.1f}°")
                    
                    if arm_joints:
                        joint_info = ", ".join(arm_joints)
                except Exception as e:
                    joint_info = f"Parse failed: {e}"
            
            # Apply position compensation (compensate URDF vs actual installation deviation)
            position_base_corrected = [
                position_base[0] + self.position_correction_x,
                position_base[1] + self.position_correction_y,
                position_base[2] + self.position_correction_z
            ]
            
            self.get_logger().info(
                f"   🌍 Coordinate transform result:\n"
                f"      🤖 Current joint angles: {joint_info}\n"
                f"      \n"
                f"      TF2 result: [{position_base[0]:.3f}, {position_base[1]:.3f}, {position_base[2]:.3f}]m\n"
                f"      Manual calculation: [{point_base_manual[0]:.3f}, {point_base_manual[1]:.3f}, {point_base_manual[2]:.3f}]m\n"
                f"      Position compensation: X+{self.position_correction_x:.3f}m, Y+{self.position_correction_y:.3f}m, Z+{self.position_correction_z:.3f}m\n"
                f"      Final coordinates: [{position_base_corrected[0]:.3f}, {position_base_corrected[1]:.3f}, {position_base_corrected[2]:.3f}]m\n"
                f"      \n"
                f"      Rotation matrix:\n"
                f"        [{rot_matrix[0,0]:7.3f} {rot_matrix[0,1]:7.3f} {rot_matrix[0,2]:7.3f}]\n"
                f"        [{rot_matrix[1,0]:7.3f} {rot_matrix[1,1]:7.3f} {rot_matrix[1,2]:7.3f}]\n"
                f"        [{rot_matrix[2,0]:7.3f} {rot_matrix[2,1]:7.3f} {rot_matrix[2,2]:7.3f}]\n"
                f"      \n"
                f"      transform details:\n"
                f"        Camera coordinates: [{point_cam_array[0]:.3f}, {point_cam_array[1]:.3f}, {point_cam_array[2]:.3f}]m\n"
                f"        R @ point_cam = [{(rot_matrix @ point_cam_array)[0]:.3f}, {(rot_matrix @ point_cam_array)[1]:.3f}, {(rot_matrix @ point_cam_array)[2]:.3f}]m\n"
                f"        + T = [{t_vec[0]:.3f}, {t_vec[1]:.3f}, {t_vec[2]:.3f}]m\n"
                f"        = [{point_base_manual[0]:.3f}, {point_base_manual[1]:.3f}, {point_base_manual[2]:.3f}]m\n"
                f"      \n"
                f"      base_link coordinates (after compensation): X={position_base_corrected[0]*100:.1f}cm, Y={position_base_corrected[1]*100:.1f}cm, Z={position_base_corrected[2]*100:.1f}cm"
            )
            
            # Use compensated coordinates
            position_base = position_base_corrected
            
            # No longer use fixed correction factor, as it depends on object distance
            # TODO: Need to correct camera_joint angle definition in URDF
            
        except Exception as e:
            self.get_logger().error(
                f"   ❌ Point transform failed: {str(e)}\n"
                f"   Skip measurement for this object"
            )
            return None
        
        # 6. Estimate measurement error
        depth_error, size_error = self.estimate_accuracy(depth)
        
        # 7. Create MeasuredObject message
        measured_obj = MeasuredObject()
        measured_obj.header = Header()
        measured_obj.header.stamp = self.get_clock().now().to_msg()
        measured_obj.header.frame_id = 'base_link'
        
        # Recognition information - handle class from two views
        if det1.class_id != det2.class_id:
            # 🔄 Two views have different classes, merge display
            # class_name: "tv&snowboard" Indicates it could be one of these two objects
            # class_id: Use view with higher confidence
            if det1.confidence >= det2.confidence:
                measured_obj.class_id = det1.class_id
                primary_class = det1.class_name
                secondary_class = det2.class_name
            else:
                measured_obj.class_id = det2.class_id
                primary_class = det2.class_name
                secondary_class = det1.class_name
            
            # Merge class names
            measured_obj.class_name = f"{primary_class}&{secondary_class}"
            
            self.get_logger().info(
                f"   🔄 View classes different: View1={det1.class_name}({det1.confidence:.2f}) "
                f"vs View2={det2.class_name}({det2.confidence:.2f})\n"
                f"      Merged as: {measured_obj.class_name} (Primary class={primary_class})"
            )
        else:
            # Same class, use directly
            measured_obj.class_id = det1.class_id
            measured_obj.class_name = det1.class_name
        
        measured_obj.confidence = min(det1.confidence, det2.confidence)
        
        # Measurement result
        measured_obj.position = Point(
            x=float(position_base[0]),
            y=float(position_base[1]),
            z=float(position_base[2])
        )
        measured_obj.dimensions = Vector3(
            x=float(real_width),
            y=float(real_depth),
            z=float(real_height)
        )
        measured_obj.depth_from_camera = float(depth)
        
        # Measurement quality
        measured_obj.depth_error = float(depth_error)
        measured_obj.size_error = float(size_error)
        measured_obj.view_count = 2
        measured_obj.matching_score = float(
            self.calculate_iou(det1.bbox, det2.bbox)
        )
        
        # Debug information
        measured_obj.disparity = float(disparity)
        # view_positionsLeave empty, filled by coordinator
        
        # 🎲 Save bbox size for cube judgment
        measured_obj.bbox_width = float(det1.bbox_width)
        measured_obj.bbox_height = float(det1.bbox_height)
        
        return measured_obj
    
    def estimate_accuracy(self, depth: float) -> Tuple[float, float]:
        """
        Estimate measurement accuracy
        
        Formula:
        - Depth error: Δd = (d² × Δp) / (f × b)
        - Size error: Δw = (d × Δp) / f
        
        Args:
            depth: Measured depth (meters)
        
        Returns:
            (depth_error, size_error) Unit: meters
        """
        fx = self.camera_params['fx']
        
        # Depth error
        depth_error = (depth ** 2 * self.pixel_uncertainty) / (fx * self.baseline)
        
        # Size error
        size_error = (depth * self.pixel_uncertainty) / fx
        
        return depth_error, size_error


def main(args=None):
    rclpy.init(args=args)
    node = TriangulationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
