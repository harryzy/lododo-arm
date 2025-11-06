from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
    Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    pkg_name = "arm_moveit_config"
    robot_urdf_name = "arm.xacro"
    world_file_name = "empty.world"


 # 路径设置
    pkg_name = "arm_moveit_config"  # 改为你的包名
    pkg_share = get_package_share_directory(pkg_name)

    kinematics_file = os.path.join(pkg_share, "config", "kinematics.yaml")

    # 验证文件存在
    if not os.path.exists(kinematics_file):
        raise FileNotFoundError(f"kinematics.yaml not found at {kinematics_file}")
    # 明确加载运动学配置
    with open(kinematics_file, "r") as f:
        kinematics_config = yaml.safe_load(f)

    # 1. 声明参数
    declare_world_arg = DeclareLaunchArgument(
        name="world",
        default_value=PathJoinSubstitution(
            [FindPackageShare(pkg_name), "worlds", world_file_name]
        ),
        description="Gazebo world file path",
    )
    
    declare_use_rviz_arg = DeclareLaunchArgument(
        name="use_rviz",
        default_value="true",
        description="Launch RViz visualization",
    )

    # 2. 正确的URDF加载方式 - 使用robot_state_publisher
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("arm_description"),  # 注意：使用arm_description包
                    "urdf",
                    robot_urdf_name,  # 注意：使用正确的文件名
                ]
            ),
        ]
    )

    # 3. 启动robot_state_publisher（关键！解决Gazebo插件连接问题）
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": True},
            {"publish_frequency": 30.0},
        ],
        output="screen",
        # 确保正确的话题映射
        remappings=[
            ("robot_description", "/robot_description"),
        ],
    )

    # 4. 启动Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                )
            ]
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "verbose": "true",
        }.items(),
    )

    # 5. 生成机器人模型到Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "my_robot", "-z", "0.1"],
        output="screen",
    )

    # 6.延迟启动控制器（依赖Gazebo和控制器管理器）
    delayed_controller_loader = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "arm_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],  # 使用正确的控制器名称
                output="screen",
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "hand_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
            ),
        ],
    )
    # 7.启动MoveIt2（禁用内置RViz）
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare(pkg_name),
                        "launch",
                        "move_group_simple_launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    # 8. 独立RViz2（确保加载运动规划组）
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_gazebo_moveit",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare(pkg_name),
                    "config",
                    "moveit_gazebo.rviz",  # 确认配置中已定义arm_grp规划组
                ]
            ),
        ],
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description_content},
            kinematics_config,
        ],
        output="screen",
        # 确保正确的话题重映射
        remappings=[
            ("display_planned_path", "/move_group/display_planned_path"),
            ("robot_description", "/robot_description"),
            ("/move_group/monitored_planning_scene", "/monitored_planning_scene"),
        ],
        condition=IfCondition(LaunchConfiguration("use_rviz")),  # 条件启动
    )

    return LaunchDescription(
        [
            declare_world_arg,
            declare_use_rviz_arg,  # 添加use_rviz参数声明
            robot_state_publisher_node,  # 首先启动
            gazebo_launch,  # 然后启动Gazebo
            TimerAction(period=3.0, actions=[spawn_robot]),  # 延迟生成机器人
            delayed_controller_loader,  # 延迟启动控制器
            TimerAction(period=12.0, actions=[moveit_launch]),  # 延迟启动MoveIt
            TimerAction(period=16.0, actions=[rviz_node]),
        ]
    )
