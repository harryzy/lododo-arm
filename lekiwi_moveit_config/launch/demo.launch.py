from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "lekiwi_arm", package_name="lekiwi_moveit_config"
    ).to_moveit_configs()

    urdf_file = os.path.join(
        get_package_share_directory("lekiwi_description"), "urdf", "lekiwi_arm.xacro"
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_file, " sim_gazebo:=false"]), value_type=str)
    # Tell MoveIt configuration not to start the driver node
    moveit_demo = generate_demo_launch(moveit_config)

    # Create base launch description
    launch_description = LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("sim_gazebo", default_value="false"),
            # # Joint State Publisher GUI
            # Node(
            #     package="joint_state_publisher_gui",
            #     executable="joint_state_publisher_gui",
            #     name="joint_state_publisher_gui",
            #     parameters=[
            #         {"robot_description": robot_description},
            #         {"use_sim_time": True},
            #     ],
            # ),
        ]
    )

    # Add all entities from MoveIt demo to our launch description
    for entity in moveit_demo.entities:
        launch_description.add_action(entity)

    return launch_description
