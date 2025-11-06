#!/usr/bin/env python3
"""
Perception Bringup Launch File
Starts only the perception system for testing and development.

This launch file includes:
- YOLO detection node (via virtual environment script)
- Projection node for 3D pose estimation
- Camera driver

Note: YOLO runs in a separate virtual environment due to dependency conflicts.

Use this for:
- Testing and debugging perception algorithms
- Calibrating cameras
- Developing new perception features
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device for perception'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLO model file'
    )

    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Confidence threshold for detections'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable visualization window'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='Publishing rate in Hz'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Get launch configurations
    camera_device = LaunchConfiguration('camera_device')
    model_path = LaunchConfiguration('model_path')
    confidence_threshold = LaunchConfiguration('confidence_threshold')
    visualize = LaunchConfiguration('visualize')
    publish_rate = LaunchConfiguration('publish_rate')
    log_level = LaunchConfiguration('log_level')

    # Find the start_yolo_node.sh script (installed in lib directory)
    # Use absolute path to the script in install directory
    workspace_root = os.path.expanduser('~/lododo-arm')
    start_yolo_script = os.path.join(
        workspace_root,
        'install',
        'arm_bringup',
        'lib',
        'arm_bringup',
        'start_yolo_node.sh'
    )

    # Launch YOLO perception system via script (runs in virtual environment)
    # The script launches both yolo_detector and projection_node
    # Note: Most parameters are handled by yolo_perception_launch.py defaults
    # We just pass visualization parameter
    perception_process = ExecuteProcess(
        cmd=['bash', start_yolo_script],
        output='screen',
        shell=False
    )

    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Starting Perception System\n',
            '='*60, '\n',
            'Camera Device: ', camera_device, '\n',
            'Model Path: ', model_path, '\n',
            'Confidence Threshold: ', confidence_threshold, '\n',
            'Visualization: ', visualize, '\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        camera_device_arg,
        model_path_arg,
        confidence_threshold_arg,
        visualize_arg,
        publish_rate_arg,
        log_level_arg,
        
        # Info
        startup_info,
        
        # Process
        perception_process,
    ])
