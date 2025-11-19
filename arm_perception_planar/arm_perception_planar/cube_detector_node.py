#!/usr/bin/env python3
"""
Cube Detector Node

Detect cubes based on geometric shape analysis, independent of YOLO classification.
Uses OpenCV contour analysis: area, aspect ratio, solidity, vertex count.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header, String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
from typing import List, Dict

# Import utility modules
from arm_perception_planar.utils import find_cube_contours, draw_contours_with_info, draw_debug_image


class CubeDetectorNode(Node):
    """
    Cube Geometric Detection Node
    
    Subscriptions:
        /camera/image_raw (Image): Input camera image
        /detection_request (String): Trigger request for detection
    
    Publications:
        /planar/cube_detections (Detection2DArray): 2D detection results
        /planar/debug_image (Image): Debug visualization image
        /detections_triggered (Detection2DArray): Triggered detection results (compatible with YOLO interface)
    
    Parameters:
        cube_detection.*: Detection parameters
        visualization.*: Visualization parameters
        detection_mode: "continuous" or "triggered" mode
    """
    
    def __init__(self):
        super().__init__('cube_detector_node')
        
        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Cube size
                ('cube_detection.expected_size', 0.050),
                ('cube_detection.size_tolerance', 0.015),
                
                # Geometric feature thresholds
                ('cube_detection.geometry_thresholds.min_area', 2000),
                ('cube_detection.geometry_thresholds.max_area', 15000),
                ('cube_detection.geometry_thresholds.min_aspect_ratio', 0.75),
                ('cube_detection.geometry_thresholds.max_aspect_ratio', 1.33),
                ('cube_detection.geometry_thresholds.min_solidity', 0.85),
                ('cube_detection.geometry_thresholds.expected_vertices', 4),
                ('cube_detection.geometry_thresholds.vertex_tolerance', 1),
                
                # Edge detection parameters
                ('cube_detection.edge_detection.use_color_segmentation', True),
                ('cube_detection.edge_detection.canny_threshold1', 50),
                ('cube_detection.edge_detection.canny_threshold2', 150),
                ('cube_detection.edge_detection.gaussian_blur_size', 5),
                ('cube_detection.edge_detection.morph_kernel_size', 3),
                
                # Color segmentation parameters
                ('cube_detection.color_segmentation.n_colors', 8),
                ('cube_detection.color_segmentation.min_saturation', 30),
                ('cube_detection.color_segmentation.min_region_pixels', 500),
                ('cube_detection.color_segmentation.max_region_pixels', 50000),
                
                # Confidence threshold
                ('cube_detection.min_confidence', 0.6),
                
                # Detection mode: "continuous" or "triggered"
                ('detection_mode', 'triggered'),
                ('trigger_timeout_sec', 3.0),
                
                # Visualization
                ('visualization.enable_debug_image', True),
                ('visualization.debug_image.show_contours', True),
                ('visualization.debug_image.show_bounding_boxes', True),
                ('visualization.debug_image.show_confidence', True),
                
                # Performance
                ('performance.image_processing_rate', 10.0),
                ('performance.queue_size', 1),
            ]
        )
        
        # Load parameters
        self._load_parameters()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Trigger state
        self.pending = False
        self.pending_site_id = None
        self.pending_time = 0.0
        self._pending_sub = None
        self._pending_timer = None
        
        # Subscription
        queue_size = self.get_parameter('performance.queue_size').value
        
        # Create subscription based on detection mode
        # Use sensor_data QoS profile for reliable high-frequency image stream
        if self.detection_mode == "continuous":
            self.sub_image = self.create_subscription(
                Image,
                '/camera/image_raw',
                self.image_callback,
                qos_profile=qos_profile_sensor_data
            )
        else:
            self.sub_image = None
        
        # Trigger request subscription (compatible with YOLO interface)
        self.req_sub = self.create_subscription(
            String,
            '/detection_request',
            self.req_cb,
            10
        )
        
        # Publishers
        self.pub_detections = self.create_publisher(
            Detection2DArray,
            '/planar/cube_detections',
            10
        )
        
        # Triggered mode output (compatible with YOLO interface)
        self.pub_triggered = self.create_publisher(
            Detection2DArray,
            '/detections_triggered',
            10
        )
        
        self.pub_debug_image = self.create_publisher(
            Image,
            '/planar/debug_image',
            10
        )
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        
        self.get_logger().info('Cube Detector Node initialized')
        self.get_logger().info(f'Detection mode: {self.detection_mode}')
        self.get_logger().info(f'Min confidence: {self.min_confidence}')
        self.get_logger().info(f'Area range: {self.geom_params["min_area"]}-{self.geom_params["max_area"]}')
    
    def _load_parameters(self):
        """Load all parameters"""
        # Geometric parameters
        self.geom_params = {
            'min_area': self.get_parameter('cube_detection.geometry_thresholds.min_area').value,
            'max_area': self.get_parameter('cube_detection.geometry_thresholds.max_area').value,
            'min_aspect_ratio': self.get_parameter('cube_detection.geometry_thresholds.min_aspect_ratio').value,
            'max_aspect_ratio': self.get_parameter('cube_detection.geometry_thresholds.max_aspect_ratio').value,
            'min_solidity': self.get_parameter('cube_detection.geometry_thresholds.min_solidity').value,
            'expected_vertices': self.get_parameter('cube_detection.geometry_thresholds.expected_vertices').value,
            'vertex_tolerance': self.get_parameter('cube_detection.geometry_thresholds.vertex_tolerance').value,
        }
        
        # Edge detection parameters
        self.edge_params = {
            'use_color_segmentation': self.get_parameter('cube_detection.edge_detection.use_color_segmentation').value,
            'canny_threshold1': self.get_parameter('cube_detection.edge_detection.canny_threshold1').value,
            'canny_threshold2': self.get_parameter('cube_detection.edge_detection.canny_threshold2').value,
            'gaussian_blur_size': self.get_parameter('cube_detection.edge_detection.gaussian_blur_size').value,
            'morph_kernel_size': self.get_parameter('cube_detection.edge_detection.morph_kernel_size').value,
        }
        
        # Color segmentation parameters
        self.color_seg_params = {
            'n_colors': self.get_parameter('cube_detection.color_segmentation.n_colors').value,
            'min_saturation': self.get_parameter('cube_detection.color_segmentation.min_saturation').value,
            'min_region_pixels': self.get_parameter('cube_detection.color_segmentation.min_region_pixels').value,
            'max_region_pixels': self.get_parameter('cube_detection.color_segmentation.max_region_pixels').value,
        }
        
        # Confidence threshold
        self.min_confidence = self.get_parameter('cube_detection.min_confidence').value
        
        # Detection mode
        self.detection_mode = self.get_parameter('detection_mode').value
        self.trigger_timeout = self.get_parameter('trigger_timeout_sec').value
        
        # Visualization
        self.enable_debug = self.get_parameter('visualization.enable_debug_image').value
    
    def req_cb(self, msg: String):
        """Callback for trigger requests (compatible with YOLO interface)"""
        self.pending = True
        self.pending_site_id = msg.data.strip() or "unknown"
        self.pending_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(f'Detection trigger received: site={self.pending_site_id}')
        
        # For triggered mode, create one-shot subscription to capture next frame
        if self.detection_mode == "triggered":
            if self._pending_sub is None:
                self._pending_sub = self.create_subscription(
                    Image,
                    '/camera/image_raw',
                    self._trigger_image_cb,
                    qos_profile=qos_profile_sensor_data
                )
                # Create timeout timer
                self._pending_timer = self.create_timer(
                    float(self.trigger_timeout),
                    self._pending_timeout_cb
                )
    
    def _trigger_image_cb(self, msg: Image):
        """Callback for one-shot triggered capture"""
        self.get_logger().info(f'Received image for trigger processing, timestamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec}')
        try:
            self.image_callback(msg, triggered=True)
            self.get_logger().info('Image processing completed successfully')
        except Exception as e:
            self.get_logger().error(f'_trigger_image_cb error: {e}')
        
        # Cleanup temporary subscription
        try:
            if self._pending_sub is not None:
                try:
                    self.destroy_subscription(self._pending_sub)
                except Exception:
                    pass
                self._pending_sub = None
        finally:
            # Cancel and clear timer
            try:
                if self._pending_timer is not None:
                    try:
                        self._pending_timer.cancel()
                    except Exception:
                        pass
                    self._pending_timer = None
            finally:
                # Reset pending flag
                if self.pending:
                    self.pending = False
                    self.pending_site_id = None
    
    def _pending_timeout_cb(self):
        """Called when trigger timeout expires without receiving frame"""
        self.get_logger().warn(
            f'Trigger wait timeout: site={self.pending_site_id}, canceling this trigger'
        )
        # Destroy temporary subscription
        try:
            if self._pending_sub is not None:
                try:
                    self.destroy_subscription(self._pending_sub)
                except Exception:
                    pass
                self._pending_sub = None
        finally:
            # Cancel and clear timer
            try:
                if self._pending_timer is not None:
                    try:
                        self._pending_timer.cancel()
                    except Exception:
                        pass
                    self._pending_timer = None
            finally:
                # Clear pending state
                self.pending = False
                self.pending_site_id = None
    
    def image_callback(self, msg: Image, triggered: bool = False):
        """Image callback"""
        # In continuous mode, only process if we have a pending trigger
        if self.detection_mode == "continuous" and not self.pending:
            return  # Skip frame if no trigger pending
        
        self.get_logger().info(f'Processing image: triggered={triggered}, frame={self.frame_count}')
        try:
            # Convert image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(f'Image converted: shape={cv_image.shape}')
            
            # Detect cubes with automatic color segmentation (returns detections and debug_image)
            detections, debug_image = find_cube_contours(cv_image, self.edge_params, self.geom_params,
                                                        self.color_seg_params,
                                                        debug=True, logger=self.get_logger())
            self.get_logger().info(f'Found {len(detections)} raw detections')
            
            # Filter by confidence threshold
            filtered_detections = [d for d in detections if d['confidence'] >= self.min_confidence]
            self.get_logger().info(f'After filtering: {len(filtered_detections)} detections (min_confidence={self.min_confidence})')
            
            # Publish detection results
            self._publish_detections(filtered_detections, msg.header, triggered)
            
            # Clear pending flag after processing (for continuous mode)
            if self.detection_mode == "continuous" and self.pending:
                self.pending = False
                self.pending_site_id = None
            
            # Publish debug image (showing color segmentation process)
            if self.enable_debug and self.pub_debug_image.get_subscription_count() > 0:
                if debug_image is not None:
                    # Use the debug image from color segmentation
                    debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
                    debug_msg.header = msg.header
                    self.pub_debug_image.publish(debug_msg)
                    self.get_logger().info('Published color segmentation debug image')
                else:
                    # Fallback: create debug image with detections only
                    self._publish_debug_image(cv_image, filtered_detections, msg.header)
            
            # Statistics
            self.frame_count += 1
            self.detection_count += len(filtered_detections)
            
            if self.frame_count % 30 == 0:  # Print every 30 frames
                self.get_logger().info(
                    f'Processed {self.frame_count} frames, '
                    f'{self.detection_count} cubes detected'
                )
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def _publish_detections(self, detections: List[Dict], header: Header, triggered: bool = False):
        """Publish detection results"""
        msg = Detection2DArray()
        msg.header = header
        msg.header.frame_id = 'camera_optical_frame'
        
        # If has site_id (triggered mode or continuous mode with pending), append to frame_id
        if self.pending_site_id:
            msg.header.frame_id = f"{msg.header.frame_id}|site:{self.pending_site_id}"
        
        for i, det in enumerate(detections):
            detection = Detection2D()
            detection.header = header
            
            # Bounding box
            x, y, w, h = det['bbox']
            cx = x + w/2
            cy = y + h/2
            detection.bbox.center.position.x = float(cx)
            detection.bbox.center.position.y = float(cy)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            
            # Hypothesis result
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = 'cube'
            hypothesis.hypothesis.score = float(det['confidence'])
            detection.results.append(hypothesis)
            
            msg.detections.append(detection)
            
            # Log 2D detection result
            self.get_logger().info(
                f"[2D Detection #{i+1}] center=({cx:.1f}, {cy:.1f})px, "
                f"size=({w}x{h})px, confidence={det['confidence']:.2f}, "
                f"aspect={det['aspect_ratio']:.2f}, solidity={det['solidity']:.2f}"
            )
        
        # Publish to internal topic
        self.pub_detections.publish(msg)
        self.get_logger().info(f'Published {len(detections)} detections to /planar/cube_detections')
        
        # If triggered mode, also publish to /detections_triggered (compatible with YOLO interface)
        if triggered:
            self.pub_triggered.publish(msg)
            self.get_logger().info(
                f'Published {len(detections)} detections to /detections_triggered '
                f'(triggered interface, site_id={self.pending_site_id})'
            )
    
    def _publish_debug_image(self, image: np.ndarray, detections: List[Dict], header: Header):
        """Publish debug image"""
        try:
            # Draw detection results
            debug_img = draw_contours_with_info(
                image, detections,
                show_confidence=True,
                show_center=True
            )
            
            # Convert and publish
            debug_msg = self.bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header = header
            self.pub_debug_image.publish(debug_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing debug image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = CubeDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
