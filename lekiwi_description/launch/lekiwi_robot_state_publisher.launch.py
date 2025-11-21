#!/usr/bin/env python3
"""
Launch file to publish robot state and TF for LeKiwi arm.
This can be included in other launch files.
Usage: ros2 launch lekiwi_description lekiwi_robot_state_publisher.launch.py
"""

import os
from launch import LaunchDescription
from launch import conditions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    publish_joints_arg = DeclareLaunchArgument(
        'publish_joints',
        default_value='true',
        description='Publish joint states from GUI if true'
    )
    
    # Get package directory
    lekiwi_description_dir = get_package_share_directory('lekiwi_description')
    
    # URDF file path
    urdf_file = os.path.join(lekiwi_description_dir, 'urdf', 'lekiwi_arm.xacro')
    
    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_file).read(),
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    # Joint State Publisher node (optional, only if not using real hardware)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=conditions.IfCondition(LaunchConfiguration('publish_joints'))
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        publish_joints_arg,
        robot_state_publisher_node,
        joint_state_publisher_node
    ])
