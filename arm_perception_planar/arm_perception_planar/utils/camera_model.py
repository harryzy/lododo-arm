#!/usr/bin/env python3
"""
Camera model utilities for 3D projection
"""

import numpy as np
from typing import Tuple, Optional
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import CameraInfo
import math


class CameraModel:
    """
    Camera model for pixel to 3D coordinate transformation
    """
    
    def __init__(self):
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0
        
        self.distortion = np.zeros(5)
        
        # Camera extrinsics (camera to base_link)
        self.translation = np.array([0.0, 0.0, 0.0])
        self.pitch = 0.0  # degrees
        self.roll = 0.0
        self.yaw = 0.0
        
        self._rotation_matrix = None
    
    def update_from_camera_info(self, camera_info: CameraInfo):
        """
        Update intrinsics from CameraInfo message
        """
        if len(camera_info.k) >= 9:
            self.fx = camera_info.k[0]
            self.fy = camera_info.k[4]
            self.cx = camera_info.k[2]
            self.cy = camera_info.k[5]
        
        if len(camera_info.d) >= 5:
            self.distortion = np.array(camera_info.d[:5])
    
    def set_extrinsics(self, tx: float, ty: float, tz: float,
                      pitch: float = 0.0, roll: float = 0.0, yaw: float = 0.0):
        """
        Set camera extrinsics
        
        Args:
            tx, ty, tz: Translation (meters)
            pitch, roll, yaw: Rotation angles (degrees)
        """
        self.translation = np.array([tx, ty, tz])
        self.pitch = pitch
        self.roll = roll
        self.yaw = yaw
        self._rotation_matrix = None  # Reset cache
    
    def get_rotation_matrix(self) -> np.ndarray:
        """
        Get rotation matrix (camera frame to base_link frame)
        
        Returns:
            3x3 rotation matrix
        """
        if self._rotation_matrix is not None:
            return self._rotation_matrix
        
        # Convert to radians
        pitch_rad = np.radians(self.pitch)
        roll_rad = np.radians(self.roll)
        yaw_rad = np.radians(self.yaw)
        
        # Rotation matrix: Only pitch rotation (matching calibration script)
        # For planar detection with roll=yaw=0, only pitch matters
        # Around Y-axis (pitch)
        Ry = np.array([
            [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
            [0, 1, 0],
            [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
        ])
        
        # If roll or yaw are non-zero, include them
        if abs(roll_rad) > 1e-6 or abs(yaw_rad) > 1e-6:
            # Around X-axis (roll)
            Rx = np.array([
                [1, 0, 0],
                [0, np.cos(roll_rad), -np.sin(roll_rad)],
                [0, np.sin(roll_rad), np.cos(roll_rad)]
            ])
            
            # Around Z-axis (yaw)
            Rz = np.array([
                [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                [0, 0, 1]
            ])
            
            self._rotation_matrix = Rz @ Ry @ Rx
        else:
            # Only pitch (roll=yaw=0)
            self._rotation_matrix = Ry
        
        return self._rotation_matrix
    
    def pixel_to_ray(self, pixel_x: float, pixel_y: float) -> np.ndarray:
        """
        Convert pixel coordinates to normalized ray in camera frame
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            
        Returns:
            Normalized ray (x, y, z), z=1
        """
        # Normalized plane coordinates
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = (pixel_y - self.cy) / self.fy
        
        return np.array([x_norm, y_norm, 1.0])
    
    def ray_to_base_frame(self, ray_cam: np.ndarray) -> np.ndarray:
        """
        Transform ray from camera frame to base_link frame
        
        Args:
            ray_cam: Ray in camera frame
            
        Returns:
            Ray in base_link frame
        """
        R = self.get_rotation_matrix()
        ray_base = R @ ray_cam
        return ray_base
    
    def intersect_plane(self, pixel_x: float, pixel_y: float, 
                       plane_z: float) -> Optional[Point]:
        """
        Compute intersection of pixel ray with plane
        
        Args:
            pixel_x, pixel_y: Pixel coordinates
            plane_z: Plane height (base_link frame)
            
        Returns:
            Intersection point (base_link frame), None if no intersection
        """
        # Step 1: Pixel → ray in camera frame
        ray_cam = self.pixel_to_ray(pixel_x, pixel_y)
        
        # DEBUG: Print intermediate values
        print(f"\n=== DEBUG intersect_plane ===")
        print(f"pixel=({pixel_x:.1f}, {pixel_y:.1f})")
        print(f"fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}")
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = (pixel_y - self.cy) / self.fy
        print(f"x_norm={x_norm:.6f}, y_norm={y_norm:.6f}")
        print(f"ray_cam=[{ray_cam[0]:.6f}, {ray_cam[1]:.6f}, {ray_cam[2]:.6f}]")
        
        # Step 2: Transform to base_link frame
        ray_base = self.ray_to_base_frame(ray_cam)
        print(f"ray_base=[{ray_base[0]:.6f}, {ray_base[1]:.6f}, {ray_base[2]:.6f}]")
        
        # Step 3: Compute ray-plane intersection using normalized ray direction
        # ray_base represents the direction at unit depth in camera frame
        # We need to normalize it to get a true direction vector
        cam_origin = self.translation
        print(f"cam_origin=[{cam_origin[0]:.6f}, {cam_origin[1]:.6f}, {cam_origin[2]:.6f}]")
        
        # Normalize ray to get true direction vector  
        ray_norm = np.linalg.norm(ray_base)
        if ray_norm < 1e-9:
            return None
        ray_dir = ray_base / ray_norm
        
        print(f"ray_norm={ray_norm:.6f}")
        print(f"ray_dir=[{ray_dir[0]:.6f}, {ray_dir[1]:.6f}, {ray_dir[2]:.6f}]")
        
        if abs(ray_dir[2]) < 1e-6:
            # Ray parallel to plane (no z-component change)
            return None
        
        # Calculate distance along ray to reach target plane
        # P.z = camera_z + distance * ray_dir.z = plane_z
        # distance = (plane_z - camera_z) / ray_dir.z
        distance = (plane_z - cam_origin[2]) / ray_dir[2]
        
        print(f"distance = ({plane_z:.6f} - {cam_origin[2]:.6f}) / {ray_dir[2]:.6f} = {distance:.6f}m")
        
        if distance < 0:
            # Intersection behind camera (negative distance)
            return None
        
        # Compute intersection point: camera origin + distance * ray direction
        point_3d = cam_origin + distance * ray_dir
        
        print(f"point_3d=[{point_3d[0]:.6f}, {point_3d[1]:.6f}, {point_3d[2]:.6f}]")
        print(f"=== END DEBUG ===\n")
        
        return Point(x=float(point_3d[0]), 
                    y=float(point_3d[1]), 
                    z=float(point_3d[2]))
    
    def intersect_plane_debug(self, pixel_x: float, pixel_y: float, 
                             plane_z: float) -> tuple:
        """
        Compute intersection with detailed debug info
        
        Returns:
            (point, debug_info_dict)
        """
        import sys
        debug = {}
        
        # Step 1: Pixel → ray in camera frame
        ray_cam = self.pixel_to_ray(pixel_x, pixel_y)
        debug['pixel'] = (pixel_x, pixel_y)
        debug['plane_z'] = plane_z
        debug['intrinsics'] = f"fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}"
        debug['ray_cam'] = ray_cam.copy()
        
        # PRINT DEBUG to stderr (bypasses ROS logging)
        sys.stderr.write(f"\n{'='*60}\n")
        sys.stderr.write(f"DEBUG intersect_plane_debug\n")
        sys.stderr.write(f"{'='*60}\n")
        sys.stderr.write(f"pixel=({pixel_x:.1f}, {pixel_y:.1f})\n")
        sys.stderr.write(f"intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}\n")
        x_norm = (pixel_x - self.cx) / self.fx
        y_norm = (pixel_y - self.cy) / self.fy
        sys.stderr.write(f"x_norm = ({pixel_x} - {self.cx}) / {self.fx} = {x_norm:.6f}\n")
        sys.stderr.write(f"y_norm = ({pixel_y} - {self.cy}) / {self.fy} = {y_norm:.6f}\n")
        sys.stderr.write(f"ray_cam = [{ray_cam[0]:.6f}, {ray_cam[1]:.6f}, {ray_cam[2]:.6f}]\n")
        
        # Step 2: Transform to base_link frame
        ray_base = self.ray_to_base_frame(ray_cam)
        debug['camera_pose'] = self.translation.copy()
        debug['camera_rotation'] = (self.pitch, self.roll, self.yaw)
        debug['ray_base_raw'] = ray_base.copy()
        
        sys.stderr.write(f"camera_pose = [{self.translation[0]:.6f}, {self.translation[1]:.6f}, {self.translation[2]:.6f}]\n")
        sys.stderr.write(f"pitch = {self.pitch:.2f}°\n")
        sys.stderr.write(f"ray_base = [{ray_base[0]:.6f}, {ray_base[1]:.6f}, {ray_base[2]:.6f}]\n")
        
        # Step 3: Compute ray-plane intersection with normalized direction
        cam_origin = self.translation
        
        # Normalize ray for correct distance calculation
        ray_norm = np.linalg.norm(ray_base)
        if ray_norm < 1e-9:
            debug['error'] = 'Zero-length ray'
            return None, debug
        ray_dir = ray_base / ray_norm
        debug['ray_norm'] = float(ray_norm)
        debug['ray_dir_normalized'] = ray_dir.copy()
        
        sys.stderr.write(f"ray_norm = {ray_norm:.6f}\n")
        sys.stderr.write(f"ray_dir = [{ray_dir[0]:.6f}, {ray_dir[1]:.6f}, {ray_dir[2]:.6f}]\n")
        
        if abs(ray_dir[2]) < 1e-6:
            debug['error'] = 'Ray parallel to plane'
            return None, debug
        
        distance = (plane_z - cam_origin[2]) / ray_dir[2]
        debug['distance'] = float(distance)
        debug['distance_calculation'] = f"({plane_z:.3f} - {cam_origin[2]:.3f}) / {ray_dir[2]:.4f} = {distance:.3f}m"
        
        sys.stderr.write(f"distance = ({plane_z:.6f} - {cam_origin[2]:.6f}) / {ray_dir[2]:.6f} = {distance:.6f}m\n")
        
        if distance < 0:
            debug['error'] = 'Intersection behind camera'
            return None, debug
        
        # Compute intersection point
        point_3d = cam_origin + distance * ray_dir
        debug['point_3d'] = point_3d.copy()
        
        sys.stderr.write(f"point_3d = [{point_3d[0]:.6f}, {point_3d[1]:.6f}, {point_3d[2]:.6f}]\n")
        sys.stderr.write(f"{'='*60}\n\n")
        sys.stderr.flush()
        
        point = Point(x=float(point_3d[0]), 
                     y=float(point_3d[1]), 
                     z=float(point_3d[2]))
        
        return point, debug
    
    def pixel_size_to_real_size(self, pixel_size: float, depth: float,
                                is_horizontal: bool = True) -> float:
        """
        Convert pixel size to real size
        
        Args:
            pixel_size: Pixel size (pixels)
            depth: Depth (meters)
            is_horizontal: Whether horizontal direction
            
        Returns:
            Real size (meters)
        """
        focal = self.fx if is_horizontal else self.fy
        return (pixel_size * depth) / focal
    
    def real_size_to_pixel_size(self, real_size: float, depth: float,
                                is_horizontal: bool = True) -> float:
        """
        Convert real size to pixel size
        
        Args:
            real_size: Real size (meters)
            depth: Depth (meters)
            is_horizontal: Whether horizontal direction
            
        Returns:
            Pixel size (pixels)
        """
        focal = self.fx if is_horizontal else self.fy
        return (real_size * focal) / depth
    
    def calculate_depth(self, point_3d: Point) -> float:
        """
        Calculate distance from point to camera
        
        Args:
            point_3d: 3D point (base_link frame)
            
        Returns:
            Depth (meters)
        """
        point = np.array([point_3d.x, point_3d.y, point_3d.z])
        depth = np.linalg.norm(point - self.translation)
        return depth
    
    def validate_size(self, bbox_width: int, bbox_height: int,
                     point_3d: Point, expected_size: float,
                     tolerance: float) -> Tuple[bool, float, float]:
        """
        Validate detected 3D size is reasonable
        
        Args:
            bbox_width, bbox_height: Pixel bounding box size
            point_3d: 3D center point
            expected_size: Expected size (meters)
            tolerance: Tolerance (meters)
            
        Returns:
            (pass or not, actual width, actual height)
        """
        # Calculate depth
        depth = self.calculate_depth(point_3d)
        
        # Pixel size → real size
        real_width = self.pixel_size_to_real_size(bbox_width, depth, True)
        real_height = self.pixel_size_to_real_size(bbox_height, depth, False)
        
        # Validate
        width_ok = abs(real_width - expected_size) < tolerance
        height_ok = abs(real_height - expected_size) < tolerance
        
        return (width_ok and height_ok, real_width, real_height)
