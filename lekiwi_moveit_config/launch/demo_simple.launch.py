"""
Simplified demo launch for lekiwi_moveit_config without ros2_control
This is useful for visualization and planning without hardware control
"""
from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Build MoveIt config
    moveit_config = MoveItConfigsBuilder(
        "lekiwi_arm", package_name="lekiwi_moveit_config"
    ).to_moveit_configs()

    # Get URDF via xacro
    urdf_file = os.path.join(
        get_package_share_directory("lekiwi_description"), "urdf", "lekiwi_arm.xacro"
    )
    
    robot_description_content = Command(
        ["xacro ", urdf_file, " sim_gazebo:=false"]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # Static TF for virtual joint
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint State Publisher (for manual control in RViz)
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        parameters=[robot_description],
    )

    # Move Group Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"publish_robot_description_semantic": True},
        ],
    )

    # RViz Node
    rviz_config_file = os.path.join(
        get_package_share_directory("lekiwi_moveit_config"),
        "config",
        "moveit.rviz"
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_gui", default_value="true"),
        static_tf_node,
        robot_state_publisher_node,
        joint_state_publisher_node,
        move_group_node,
        rviz_node,
    ])
