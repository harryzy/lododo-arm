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
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    urdf_file = os.path.join(
        get_package_share_directory("arm_description"), "urdf", "arm.xacro"
    )
    robot_description = ParameterValue(Command(["xacro ", urdf_file, " sim_gazebo:=false"]), value_type=str)
    # 告诉MoveIt配置不要启动驱动节点
    moveit_demo = generate_demo_launch(moveit_config)

    # 创建基础启动描述
    launch_description = LaunchDescription(
        [
            # 声明启动参数
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("sim_gazebo", default_value="false"),
            # # 关节状态发布器GUI
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

    # 将MoveIt demo的所有实体添加到我们的启动描述中
    for entity in moveit_demo.entities:
        launch_description.add_action(entity)

    return launch_description
