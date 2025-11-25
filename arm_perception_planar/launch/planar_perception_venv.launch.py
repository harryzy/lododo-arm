#!/usr/bin/env python3
"""
Planar Perception Launch File with Virtual Environment
Uses venv wrapper to ensure NumPy 1.x compatibility
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
    venv_script = os.path.join(pkg_share, 'scripts', 'run_planar_venv.sh')
    
    # Launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )
    
    # Cube detection node with venv wrapper
    cube_detector_node = Node(
        package='arm_perception_planar',
        executable='run_planar_venv.sh',  # Use venv wrapper
        name='cube_detector',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
        ],
        arguments=['cube_detector']  # Pass node type as argument
    )
    
    # Planar localization node with venv wrapper
    planar_localization_node = Node(
        package='arm_perception_planar',
        executable='run_planar_venv.sh',  # Use venv wrapper
        name='planar_localization',
        output='screen',
        parameters=[config_file],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/camera/camera_info', '/camera/camera_info'),
        ],
        arguments=['localization']  # Pass node type as argument
    )
    
    return LaunchDescription([
        use_rviz_arg,
        cube_detector_node,
        planar_localization_node,
    ])