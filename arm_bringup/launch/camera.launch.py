#!/usr/bin/env python3
"""
USB Camera Launch File - Using usb_cam driver
Supports MJPEG format cameras, provides stable image output for RViz display and YOLO detection

Installation: sudo apt install ros-humble-usb-cam
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Launch USB camera using usb_cam"""
    
    # Declare launch parameters
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_device",
            default_value="/dev/video0",
            description="Camera device path (e.g., /dev/video0)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "frame_id",
            default_value="camera_optical_frame",
            description="Frame ID for camera (must match URDF)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_width",
            default_value="320",
            description="Image width in pixels",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_height",
            default_value="240",
            description="Image height in pixels",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "framerate",
            default_value="15.0",
            description="Camera framerate (Hz) - 15 FPS for better stability",
        )
    )
    
    # Get parameter values
    camera_device = LaunchConfiguration("camera_device")
    frame_id = LaunchConfiguration("frame_id")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    
    # Note: Requires usb_cam package to be installed first
    # sudo apt install ros-humble-usb-cam
    # Camera calibration file path (file:// URL format)
    # This file contains camera intrinsic matrix for 3D projection calculations
    
    camera_info_url = 'file://' + os.path.join(
        get_package_share_directory('arm_bringup'),
        'config',
        'camera_info.yaml'
    )
    
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_width': image_width,  # Parameter passed via LaunchConfiguration
            'image_height': image_height,  # Parameter passed via LaunchConfiguration
            'framerate': framerate,  # Parameter passed via LaunchConfiguration
            'frame_id': frame_id,
            'pixel_format': 'yuyv',  # Use YUYV format (camera native format) mjpeg2rgb
            'io_method': 'mmap',  # Memory-mapped I/O method
            'camera_info_url': camera_info_url,  # Camera calibration file URL
            
            # 🔧 Image quality adjustment parameters (eliminate reflections/highlight overflow)
            'brightness': 20,      # Brightness (0-255) - Significantly reduced to avoid overexposure from reflections
            'contrast': 50,        # Contrast (0-255) - Increased to enhance edges
            'saturation': 60,      # Saturation (0-255) - Slightly reduced
            'sharpness': 6,        # Sharpness (0-255) - Enhanced edge detection
            'auto_exposure': 1,    # Auto exposure: 1=Manual (recommended)
            'exposure': 30,        # Exposure value (1-5000) - Significantly reduced to eliminate reflections
            'auto_white_balance': True,   # Auto white balance
            'white_balance': 4000, # White balance temperature
        }],
        remappings=[
            # Ensure topic names match Gazebo simulation
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription(declared_arguments + [camera_node])
