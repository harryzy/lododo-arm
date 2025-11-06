#!/usr/bin/env python3
"""
Simulation Bringup Launch File
Starts the complete arm system in Gazebo simulation with RViz and all functionalities.

This launch file includes:
- Gazebo simulation environment
- Robot state publisher
- MoveIt move_group
- RViz with custom control panel
- YOLO perception node
- Arm planning interface
- Voice control interface (optional)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import yaml
from ament_index_python.packages import get_package_share_directory


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
    # Load default configuration from YAML
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')
    
    # Declare launch arguments with values from YAML config
    use_voice_control_arg = DeclareLaunchArgument(
        'use_voice_control',
        default_value='true',
        description='Enable voice control interface'
    )
    
    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Enable YOLO perception system'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=str(default_config.get('default_log_level', 'info')),
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Get launch configurations
    use_voice_control = LaunchConfiguration('use_voice_control')
    use_perception = LaunchConfiguration('use_perception')
    use_rviz = LaunchConfiguration('use_rviz')
    log_level = LaunchConfiguration('log_level')

    # Package directories
    moveit_config_pkg = FindPackageShare('arm_moveit_config')

    # Launch Gazebo simulation with robot (includes MoveIt and RViz internally)
    # Note: gazebo_launch.py already includes move_group and rviz, so we don't launch them separately
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'gazebo_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'true',
        }.items()
    )


    # Note: YOLO perception node is NOT launched here due to dependency conflicts
    # YOLO requires a separate virtual environment to avoid conflicts with MoveIt
    # To launch YOLO perception manually:
    #   ros2 run arm_bringup start_yolo_node.sh
    # Or in another terminal:
    #   cd ~/lododo-arm
    #   source ~/yolo_venv/bin/activate
    #   source install/setup.bash
    #   ros2 run arm_perception_yolo yolo_detection_node

    # Launch arm planning interface
    planning_node = Node(
        package='arm_planning_py',
        executable='arm_command_interface',
        name='arm_command_interface',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Note: arm_grasper is a class, not a standalone node
    # It's used by arm_command_interface internally

    # Launch voice control interface (conditional)
    voice_control_node = GroupAction(
        condition=IfCondition(use_voice_control),
        actions=[
            Node(
                package='arm_voice_interface',
                executable='arm_voice_node',
                name='arm_voice_node',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )

    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Starting Arm Simulation System\n',
            '='*60, '\n',
            'Voice Control: ', use_voice_control, '\n',
            'Perception: ', use_perception, '\n',
            'RViz: ', use_rviz, '\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        use_voice_control_arg,
        use_perception_arg,
        use_rviz_arg,
        log_level_arg,
        
        # Info
        startup_info,
        
        # Launch files
        # Note: gazebo_launch already includes MoveIt and RViz, so we only need to launch it once
        gazebo_launch,
        # rviz_launch,  # Commented out: RViz is already included in gazebo_launch
        
        # Nodes
        # Note: YOLO perception node must be started separately (see comments above)
        # Note: arm_grasper is used internally by arm_command_interface, not launched separately
        planning_node,
        voice_control_node,
    ])
