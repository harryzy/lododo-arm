#!/usr/bin/env python3
"""
USB Camera Launch File - Using usb_cam driver
Supports MJPEG format cameras, provides stable image output for RViz display and YOLO detection

Installation: sudo apt install ros-humble-usb-cam

Configuration:
- All parameters loaded from default_params.yaml (Single Source of Truth)
- Camera intrinsics automatically scaled based on resolution
- Launch arguments can override default values
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import tempfile


def load_yaml_config(package_name, config_file):
    """Load YAML configuration file from package"""
    config_path = os.path.join(
        get_package_share_directory(package_name),
        'config',
        config_file
    )
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def generate_camera_info_yaml(width, height, fx_base, fy_base, cx_base, cy_base, 
                                base_width=640, base_height=480):
    """
    Generate camera_info YAML content dynamically based on resolution
    Automatically scales intrinsic parameters
    
    Args:
        width, height: Target resolution
        fx_base, fy_base, cx_base, cy_base: Base intrinsic parameters (for base_width × base_height)
        base_width, base_height: Base resolution used for calibration
    
    Returns:
        Path to temporary camera_info.yaml file
    """
    # Calculate scaling factors
    scale_x = width / base_width
    scale_y = height / base_height
    
    # Scale intrinsic parameters
    fx = fx_base * scale_x
    fy = fy_base * scale_y
    cx = cx_base * scale_x
    cy = cy_base * scale_y
    
    # Generate camera_info content
    camera_info = {
        'image_width': int(width),
        'image_height': int(height),
        'camera_name': 'usb_cam',
        'camera_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [fx, 0.0, cx,
                     0.0, fy, cy,
                     0.0, 0.0, 1.0]
        },
        'distortion_model': 'plumb_bob',
        'distortion_coefficients': {
            'rows': 1,
            'cols': 5,
            'data': [0.0, 0.0, 0.0, 0.0, 0.0]
        },
        'rectification_matrix': {
            'rows': 3,
            'cols': 3,
            'data': [1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0]
        },
        'projection_matrix': {
            'rows': 3,
            'cols': 4,
            'data': [fx, 0.0, cx, 0.0,
                     0.0, fy, cy, 0.0,
                     0.0, 0.0, 1.0, 0.0]
        }
    }
    
    # Write to temporary file
    temp_file = tempfile.NamedTemporaryFile(mode='w', suffix='_camera_info.yaml', delete=False)
    yaml.dump(camera_info, temp_file, default_flow_style=False)
    temp_file.close()
    
    return temp_file.name


def generate_launch_description():
    """Launch USB camera using usb_cam with dynamic configuration"""
    
    # Load default configuration from YAML (Single Source of Truth)
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')
    
    # Declare launch parameters with defaults from YAML
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_device",
            default_value=str(default_config.get('camera_device', '/dev/video0')),
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
            default_value=str(default_config.get('camera_width', 640)),
            description="Image width in pixels",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "image_height",
            default_value=str(default_config.get('camera_height', 480)),
            description="Image height in pixels",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "framerate",
            default_value=str(default_config.get('camera_fps', 15.0)),
            description="Camera framerate (Hz)",
        )
    )
    
    # Get parameter values
    camera_device = LaunchConfiguration("camera_device")
    frame_id = LaunchConfiguration("frame_id")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    
    # Generate dynamic camera_info based on actual resolution
    # Note: This uses the default values from YAML. If launch arguments override these,
    # the camera_info might not match. Consider using OpaqueFunction for full dynamic support.
    width = default_config.get('camera_width', 640)
    height = default_config.get('camera_height', 480)
    fx_base = default_config.get('camera_fx', 1128.1)
    fy_base = default_config.get('camera_fy', 1128.1)
    cx_base = default_config.get('camera_cx', 320.0)
    cy_base = default_config.get('camera_cy', 240.0)
    
    camera_info_file = generate_camera_info_yaml(
        width, height, fx_base, fy_base, cx_base, cy_base
    )
    camera_info_url = 'file://' + camera_info_file
    
    # Get image quality parameters from YAML
    brightness = default_config.get('camera_brightness', 20)
    contrast = default_config.get('camera_contrast', 50)
    saturation = default_config.get('camera_saturation', 60)
    sharpness = default_config.get('camera_sharpness', 6)
    auto_exposure = default_config.get('camera_auto_exposure', 1)
    exposure = default_config.get('camera_exposure', 30)
    auto_white_balance = default_config.get('camera_auto_white_balance', True)
    white_balance = default_config.get('camera_white_balance', 4000)
    
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='camera',
        output='screen',
        parameters=[{
            # Device and resolution
            'video_device': camera_device,
            'image_width': image_width,
            'image_height': image_height,
            'framerate': framerate,
            'frame_id': frame_id,
            'pixel_format': 'yuyv',  # Use YUYV format (camera native format)
            'io_method': 'mmap',  # Memory-mapped I/O method
            'camera_info_url': camera_info_url,  # Dynamically generated camera calibration
            
            # Image quality parameters (loaded from default_params.yaml)
            'brightness': brightness,
            'contrast': contrast,
            'saturation': saturation,
            'sharpness': sharpness,
            'auto_exposure': auto_exposure,
            'exposure': exposure,
            'auto_white_balance': auto_white_balance,
            'white_balance': white_balance,
        }],
        remappings=[
            # Ensure topic names match Gazebo simulation
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription(declared_arguments + [camera_node])
