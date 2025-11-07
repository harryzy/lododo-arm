#!/usr/bin/env python3
"""
Lightweight Robot Side Launch File for Raspberry Pi 3B+ (1GB RAM)

This is an optimized version for resource-constrained hardware.
Reduces camera resolution and disables unnecessary features.

Differences from robot_side.launch.py:
- Lower camera resolution (640x480 -> 424x240)
- Reduced frame rate (30fps -> 15fps)
- Minimal RealSense processing filters
- Optimized for 1GB RAM devices

Launch on Raspberry Pi:
    ros2 launch arm_bringup robot_side_lite.launch.py

Launch Arguments:
    serial_port: Serial port for arm servos (default: from config file or /dev/ttyACM0)
    baud_rate: Baud rate for serial communication (default: from config file or 115200)
    use_camera: Enable/disable RealSense camera (default: true)
    camera_serial_no: Camera serial number (default: auto-detect)

Author: lododo
License: Apache 2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def load_yaml_config(package_name, config_file):
    """Load YAML configuration file and return as dict"""
    try:
        config_path = os.path.join(
            get_package_share_directory(package_name),
            'config',
            config_file
        )
        with open(config_path, 'r') as f:
            return yaml.safe_load(f)
    except Exception as e:
        print(f"Warning: Could not load config file {config_file}: {e}")
        return {}


def generate_launch_description():
    # Build MoveIt configuration (same as robot_side.launch.py)
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # Load default configuration from YAML
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')

    # Declare launch arguments with values from YAML config
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=str(default_config.get('serial_port', '/dev/ttyACM0')),
        description='Serial port for robot communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value=str(default_config.get('baud_rate', 115200)),
        description='Baud rate for serial communication'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='true',
        description='Enable RealSense camera node'
    )
    
    camera_serial_no_arg = DeclareLaunchArgument(
        'camera_serial_no',
        default_value='',
        description='RealSense camera serial number (empty for auto-detect)'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_camera = LaunchConfiguration('use_camera')
    camera_serial_no = LaunchConfiguration('camera_serial_no')

    # Package directories
    arm_driver_pkg = FindPackageShare('arm_driver_node')
    
    # 1. Static transform publisher - publishes world to base_link transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Launch arm driver node
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([arm_driver_pkg, 'launch', 'driver_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'serial_port': serial_port,
            'baud_rate': baud_rate,
        }.items(),
    )

    # 3. Delayed robot_state_publisher launch
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 15.0},  # Reduced from 20Hz to save CPU on Pi 3B
            {"ignore_timestamp": False},
        ],
    )

    delayed_robot_state_publisher = TimerAction(
        period=3.0,
        actions=[robot_state_publisher]
    )

    # 4. RealSense camera node with LITE configuration
    # Optimized for Raspberry Pi 3B+ (1GB RAM):
    # - Reduced resolution: 424x240 depth (vs 640x480)
    # - Lower framerate: 15fps (vs 30fps)
    # - Disabled RGB camera to save bandwidth/memory
    # - Minimal processing filters
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        output='screen',
        parameters=[{
            'serial_no': camera_serial_no,
            'camera_name': 'camera',
            'device_type': 'd435i',
            
            # LITE MODE: Reduced resolution and framerate
            'depth_module.profile': '424x240x15',  # Low res, 15fps
            'enable_depth': True,
            
            # Disable RGB to save memory and USB bandwidth
            'enable_color': False,
            'rgb_camera.profile': '424x240x15',
            
            # Disable extra sensors
            'enable_infra1': False,
            'enable_infra2': False,
            'enable_gyro': False,
            'enable_accel': False,
            
            # Depth processing (keep minimal for grasp planning)
            'pointcloud.enable': True,
            'align_depth.enable': False,  # Disabled (no RGB)
            'decimation_filter.enable': True,
            'spatial_filter.enable': True,
            'temporal_filter.enable': False,  # Disabled to save CPU
            'hole_filling_filter.enable': False,  # Disabled to save CPU
            
            # TF frames
            'base_frame_id': 'camera_link',
            'depth_frame_id': 'camera_depth_frame',
            'color_frame_id': 'camera_color_frame',
        }],
        condition=IfCondition(use_camera)
    )

    return LaunchDescription([
        # Launch arguments
        serial_port_arg,
        baud_rate_arg,
        use_sim_time_arg,
        use_camera_arg,
        camera_serial_no_arg,
        
        # Nodes and launch files
        static_tf,                      # Static TF publisher
        driver_launch,                  # Arm driver node
        delayed_robot_state_publisher,  # Robot state publisher (delayed)
        realsense_node,                 # RealSense camera (LITE mode, optional)
    ])

