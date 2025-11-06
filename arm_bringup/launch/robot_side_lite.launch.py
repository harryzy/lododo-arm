#!/usr/bin/env python3
"""
Lightweight Robot Side Launch File for Raspberry Pi 3B+ (1GB RAM)
Author: lododo
Contact: contect@lododo.org

This is an optimized version for resource-constrained hardware.
Reduces camera resolution and disables unnecessary features.

Launch on Raspberry Pi:
    ros2 launch arm_bringup robot_side_lite.launch.py

Parameters:
    port: Serial port for arm driver (default: /dev/ttyUSB0)
    enable_camera: Enable RealSense camera (default: true)
    camera_serial_no: Camera serial number (default: auto-detect)
    ros_domain_id: ROS Domain ID (default: 0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='Serial port for arm driver'
    )
    
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable RealSense camera node'
    )
    
    camera_serial_no_arg = DeclareLaunchArgument(
        'camera_serial_no',
        default_value='',
        description='RealSense camera serial number (empty for auto-detect)'
    )
    
    ros_domain_id_arg = DeclareLaunchArgument(
        'ros_domain_id',
        default_value='0',
        description='ROS Domain ID for distributed deployment'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    enable_camera = LaunchConfiguration('enable_camera')
    camera_serial_no = LaunchConfiguration('camera_serial_no')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    
    # Get URDF file path
    arm_description_dir = get_package_share_directory('arm_description')
    urdf_file = os.path.join(arm_description_dir, 'urdf', 'arm.urdf')
    
    # Read URDF content
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Arm driver node (serial communication with servos)
    arm_driver_node = Node(
        package='arm_driver_node',
        executable='arm_driver_node',
        name='arm_driver_node',
        output='screen',
        parameters=[{
            'port': port,
            'baudrate': 1000000,
        }]
    )
    
    # Robot state publisher (publishes TF transforms)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0,  # Reduced from 50Hz to save CPU
        }]
    )
    
    # RealSense camera node with LITE configuration
    # Optimized for Raspberry Pi 3B+ (1GB RAM):
    # - Reduced resolution: 424x240 depth (vs 640x480)
    # - Lower framerate: 15fps (vs 30fps)
    # - Disabled RGB camera to save bandwidth/memory
    # - Disabled unnecessary processing
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
        condition=IfCondition(enable_camera)
    )
    
    # Set ROS Domain ID
    set_domain_id = SetParameter(
        name='ROS_DOMAIN_ID',
        value=ros_domain_id
    )
    
    return LaunchDescription([
        # Launch arguments
        port_arg,
        enable_camera_arg,
        camera_serial_no_arg,
        ros_domain_id_arg,
        
        # Set domain ID
        set_domain_id,
        
        # Nodes
        arm_driver_node,
        robot_state_publisher_node,
        realsense_node,
    ])
