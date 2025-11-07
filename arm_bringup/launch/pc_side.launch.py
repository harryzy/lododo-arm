#!/usr/bin/env python3
"""
PC Side Launch File for Distributed Deployment

This launch file runs on the PC (control station) for distributed deployment.
Robot side must be running robot_side.launch.py on Raspberry Pi.

This launch file includes:
- MoveIt2 move_group (motion planning)
- RViz with custom control panel
- Arm planning interface node
- Voice control interface (optional)

NOTE: YOLO perception node must be started separately using:
      ros2 run arm_bringup start_yolo_cube_detect.sh
      (YOLO requires isolated virtual environment)
      
      Available YOLO scripts (in order of recommendation):
      1. start_yolo_cube_detect.sh (recommended) - Optimized for cube detection
      2. start_yolo_dual_view.sh (advanced) - Dual-view triangulation
      3. start_yolo_node.sh (basic) - Single-view projection
      
      Note: Cube detection provides best results for grasping tasks.
      Other modes require manual camera calibration and lighting tuning.

Hardware Requirements:
- PC with network connection to robot
- GPU recommended for YOLO perception (launched separately)

Network Requirements:
- Both robot and PC must be on the same network
- ROS_DOMAIN_ID must match on both sides
- Robot side (Raspberry Pi) must be running robot_side.launch.py

Launch Arguments:
- use_rviz: Launch RViz visualization (default: true)
- use_voice_control: Enable voice control interface (default: false)
- log_level: Logging level (default: info)

Author: lododo
License: Apache 2.0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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
    # Load default configuration from YAML
    default_config = load_yaml_config('arm_bringup', 'default_params.yaml')

    # Declare launch arguments with values from YAML config
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with custom control panel'
    )
    
    use_voice_control_arg = DeclareLaunchArgument(
        'use_voice_control',
        default_value='false',
        description='Enable voice control interface'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value=str(default_config.get('default_log_level', 'info')),
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Get launch configurations
    use_rviz = LaunchConfiguration('use_rviz')
    use_voice_control = LaunchConfiguration('use_voice_control')
    log_level = LaunchConfiguration('log_level')

    # Package directories
    moveit_config_pkg = FindPackageShare('arm_moveit_config')
    rviz_plugin_pkg = FindPackageShare('arm_rviz_plugin')
    arm_perception_pkg = FindPackageShare('arm_perception_yolo')

    # 1. Launch MoveIt2 move_group (reuse from arm_moveit_config)
    # Note: move_group_simple_launch.py provides just move_group without driver/rviz
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'move_group_simple_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # 2. Launch RViz with custom control panel (delayed to ensure move_group is ready)
    # Reuse arm_control_rviz.launch.py from arm_rviz_plugin
    rviz_launch = GroupAction(
        condition=IfCondition(use_rviz),
        actions=[
            LogInfo(msg='[INFO] Starting RViz with custom control panel in 5 seconds...'),
            TimerAction(
                period=5.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([
                            PathJoinSubstitution([rviz_plugin_pkg, 'launch', 'arm_control_rviz.launch.py'])
                        ])
                    )
                ]
            )
        ]
    )

    # 3. Launch arm planning interface node
    # Note: This is the high-level planning node that receives commands from RViz panel
    measurement_params_path = PathJoinSubstitution([
        arm_perception_pkg,
        'config',
        'measurement_params.yaml'
    ])
    
    planning_node = Node(
        package='arm_planning_py',
        executable='arm_command_interface',
        name='arm_command_interface',
        output='screen',
        parameters=[measurement_params_path],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # 4. Launch voice control interface (conditional)
    voice_control_node = GroupAction(
        condition=IfCondition(use_voice_control),
        actions=[
            LogInfo(msg='[INFO] Starting voice control interface...'),
            Node(
                package='arm_voice_interface',
                executable='arm_voice_node',
                name='arm_voice_node',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level]
            )
        ]
    )

    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Starting PC Side (Distributed Deployment)\n',
            '='*60, '\n',
            'RViz: ', use_rviz, '\n',
            'Voice Control: ', use_voice_control, '\n',
            '\n',
            'IMPORTANT: Robot side must be running on Raspberry Pi!\n',
            '  ssh lododo@<raspberry-pi-ip>\n',
            '  ros2 launch arm_bringup robot_side.launch.py\n',
            '\n',
            'NOTE: YOLO perception must be started separately:\n',
            '  ros2 run arm_bringup start_yolo_cube_detect.sh\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Launch arguments
        use_rviz_arg,
        use_voice_control_arg,
        log_level_arg,
        
        # Startup info
        startup_info,
        
        # Launch files (reuse existing launch files)
        move_group_launch,  # MoveIt2 move_group (from arm_moveit_config)
        rviz_launch,        # Custom RViz with control panel (from arm_rviz_plugin)
        
        # Nodes
        planning_node,      # Arm planning interface
        voice_control_node, # Voice control (optional)
    ])
