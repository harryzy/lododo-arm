#!/usr/bin/env python3
"""
Voice Headless Bringup Launch File
Starts the arm system with voice control only, no GUI (headless mode).

This is ideal for:
- Deploying on embedded systems with limited resources
- Running as a background service
- Voice-only control scenarios

This launch file includes:
- Real robot driver (serial communication)
- Robot state publisher
- MoveIt move_group (without RViz)
- Arm planning interface
- Voice control interface
- Optional perception (no visualization)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    use_perception_arg = DeclareLaunchArgument(
        'use_perception',
        default_value='true',
        description='Enable YOLO perception system (no visualization)'
    )

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device for perception'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_perception = LaunchConfiguration('use_perception')
    camera_device = LaunchConfiguration('camera_device')
    log_level = LaunchConfiguration('log_level')

    # Package directories
    moveit_config_pkg = FindPackageShare('arm_moveit_config')

    # Launch robot driver with move_group (real hardware mode, headless - no RViz)
    # Note: driver_view_launch.py already includes robot_state_publisher and move_group
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([moveit_config_pkg, 'launch', 'driver_view_launch.py'])
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'use_rviz': 'false',  # Headless mode - no GUI
        }.items()
    )

    # Note: YOLO perception node is NOT launched here due to dependency conflicts
    # YOLO requires a separate virtual environment to avoid conflicts with MoveIt
    # To launch YOLO perception manually in headless mode:
    #   ros2 run arm_bringup start_yolo_node.sh --ros-args -p visualize:=false
    # Or in another terminal:
    #   cd ~/lododo-arm
    #   source ~/yolo_venv/bin/activate
    #   source install/setup.bash
    #   ros2 run arm_perception_yolo yolo_detection_node --ros-args -p visualize:=false

    # Launch arm planning interface
    planning_node = Node(
        package='arm_planning_py',
        executable='arm_command_interface',
        name='arm_command_interface',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Note: arm_grasper is a class, not a standalone node
    # It's used by arm_command_interface internally

    # Launch voice control interface (always enabled in this mode)
    voice_control_node = Node(
        package='arm_voice_interface',
        executable='arm_voice_node',
        name='arm_voice_node',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Startup info
    startup_info = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'Starting Voice Headless Arm System\n',
            '='*60, '\n',
            'Mode: HEADLESS (No GUI)\n',
            'Serial Port: ', serial_port, '\n',
            'Baud Rate: ', baud_rate, '\n',
            'Perception: ', use_perception, '\n',
            'Voice Control: ENABLED\n',
            '='*60, '\n',
            'Say voice commands to control the arm!\n',
            '='*60, '\n'
        ]
    )

    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        use_perception_arg,
        camera_device_arg,
        log_level_arg,
        
        # Info
        startup_info,
        
        # Launch files
        driver_launch,  # Includes: driver, robot_state_publisher, move_group
        
        # Nodes
        # Note: YOLO perception node must be started separately (see comments above)
        # Note: arm_grasper is used internally by arm_command_interface, not launched separately
        planning_node,
        voice_control_node,
    ])
