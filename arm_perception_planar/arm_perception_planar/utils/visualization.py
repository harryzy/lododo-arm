#!/usr/bin/env python3
"""
Visualization utilities for debugging and display
"""

import cv2
import numpy as np
from typing import List, Dict, Tuple
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def draw_contours_with_info(image: np.ndarray, detections: List[Dict],
                            show_confidence: bool = True,
                            show_center: bool = True) -> np.ndarray:
    """
    Draw detection results on image
    
    Args:
        image: Input image
        detections: Detection result list
        show_confidence: Whether to display confidence
        show_center: Whether to display center point
        
    Returns:
        Annotated image
    """
    output = image.copy()
    
    for det in detections:
        contour = det['contour']
        bbox = det['bbox']
        center = det['center']
        confidence = det['confidence']
        
        x, y, w, h = bbox
        cx, cy = center
        
        # Draw contour
        cv2.drawContours(output, [contour], -1, (0, 255, 0), 2)
        
        # Draw bounding box
        cv2.rectangle(output, (x, y), (x+w, y+h), (255, 0, 0), 2)
        
        # Draw center point
        if show_center:
            cv2.circle(output, (int(cx), int(cy)), 5, (0, 0, 255), -1)
        
        # Display confidence
        if show_confidence:
            text = f"Conf: {confidence:.2f}"
            cv2.putText(output, text, (x, y-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return output


def draw_3d_info(image: np.ndarray, detections: List[Dict],
                points_3d: List[Point]) -> np.ndarray:
    """
    Draw 3D coordinate information on image
    
    Args:
        image: Input image
        detections: Detection result list
        points_3d: Corresponding 3D coordinate list
        
    Returns:
        Annotated image
    """
    output = image.copy()
    
    for det, point in zip(detections, points_3d):
        bbox = det['bbox']
        x, y, w, h = bbox
        
        # Display 3D coordinates
        text = f"({point.x:.3f}, {point.y:.3f}, {point.z:.3f})"
        cv2.putText(output, text, (x, y+h+20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
    
    return output


def create_cube_marker(point: Point, marker_id: int, namespace: str,
                       size: float = 0.05, lifetime: float = 2.0) -> Marker:
    """
    Create cube RViz marker
    
    Args:
        point: 3D position
        marker_id: Marker ID
        namespace: Namespace
        size: Cube edge length
        lifetime: Display duration
        
    Returns:
        Marker message
    """
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    
    # Position
    marker.pose.position = point
    marker.pose.orientation.w = 1.0
    
    # Size
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    
    # Color (semi-transparent green)
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.6
    
    # Lifetime
    marker.lifetime.sec = int(lifetime)
    marker.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)
    
    return marker


def create_text_marker(point: Point, text: str, marker_id: int,
                       namespace: str, lifetime: float = 2.0) -> Marker:
    """
    Create text RViz marker
    
    Args:
        point: 3D position
        text: Display text
        marker_id: Marker ID
        namespace: Namespace
        lifetime: Display duration
        
    Returns:
        Marker message
    """
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    
    # Position (text above cube)
    marker.pose.position.x = point.x
    marker.pose.position.y = point.y
    marker.pose.position.z = point.z + 0.08  # 8cm above
    marker.pose.orientation.w = 1.0
    
    # Text content
    marker.text = text
    
    # Size
    marker.scale.z = 0.02  # Text height
    
    # Color (white)
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    
    # Lifetime
    marker.lifetime.sec = int(lifetime)
    marker.lifetime.nanosec = int((lifetime - int(lifetime)) * 1e9)
    
    return marker


def create_marker_array(points_3d: List[Point], confidences: List[float],
                       namespace: str = "cubes",
                       cube_size: float = 0.05,
                       lifetime: float = 2.0) -> MarkerArray:
    """
    Create array of multiple markers
    
    Args:
        points_3d: List of 3D points
        confidences: List of confidences
        namespace: Namespace
        cube_size: Cube size
        lifetime: Display duration
        
    Returns:
        MarkerArray message
    """
    marker_array = MarkerArray()
    
    for i, (point, conf) in enumerate(zip(points_3d, confidences)):
        # Cube marker
        cube_marker = create_cube_marker(
            point, i*2, namespace, cube_size, lifetime
        )
        marker_array.markers.append(cube_marker)
        
        # Text marker
        text = f"Cube {i+1}\nConf: {conf:.2f}"
        text_marker = create_text_marker(
            point, text, i*2+1, namespace, lifetime
        )
        marker_array.markers.append(text_marker)
    
    return marker_array


def draw_debug_image(image: np.ndarray, edges: np.ndarray,
                    detections: List[Dict],
                    show_edges: bool = True) -> np.ndarray:
    """
    Create debug image showing processing pipeline
    
    Args:
        image: Original image
        edges: Edge image
        detections: Detection results
        show_edges: Whether to show edges
        
    Returns:
        Combined debug image
    """
    # Create large canvas
    h, w = image.shape[:2]
    canvas = np.zeros((h*2, w*2, 3), dtype=np.uint8)
    
    # Top-left: Original image
    canvas[0:h, 0:w] = image
    cv2.putText(canvas, "Original", (10, 30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Top-right: Edge image
    if show_edges:
        edges_color = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        canvas[0:h, w:w*2] = edges_color
        cv2.putText(canvas, "Edges", (w+10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    # Bottom-left: Detection results
    detection_img = draw_contours_with_info(image, detections)
    canvas[h:h*2, 0:w] = detection_img
    cv2.putText(canvas, f"Detections: {len(detections)}", (10, h+30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Bottom-right: Statistics
    stats_img = np.zeros((h, w, 3), dtype=np.uint8)
    y_offset = 50
    for i, det in enumerate(detections[:5]):  # Show at most 5
        text = f"#{i+1}: Conf={det['confidence']:.2f}, " \
               f"Area={det['area']:.0f}, AR={det['aspect_ratio']:.2f}"
        cv2.putText(stats_img, text, (10, y_offset),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 30
    
    canvas[h:h*2, w:w*2] = stats_img
    cv2.putText(canvas, "Statistics", (w+10, h+30),
               cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    return canvas
