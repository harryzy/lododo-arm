#!/usr/bin/env python3
"""
Planar Perception Launch File

Launch all nodes for the planar perception system
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Package share path
    pkg_share = get_package_share_directory('arm_perception_planar')
    config_file = os.path.join(pkg_share, 'config', 'planar_params.yaml')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    # Cube detection node
    cube_detector_node = Node(
        package='arm_perception_planar',
        executable='cube_detector_node',
        name='cube_detector',
        output='screen',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', 'debug'],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ]
    )
    
    # Planar localization node
    planar_localization_node = Node(
        package='arm_perception_planar',
        executable='planar_localization_node',
        name='planar_localization',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        use_rviz_arg,
        cube_detector_node,
        planar_localization_node,
    ])
