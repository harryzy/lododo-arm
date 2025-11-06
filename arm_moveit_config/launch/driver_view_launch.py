from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    # 构建MoveIt配置
    moveit_config = MoveItConfigsBuilder(
        "arm", package_name="arm_moveit_config"
    ).to_moveit_configs()

    # 声明启动参数
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config" / "moveit.rviz"),
            description="RViz configuration file",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port for robot communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Baud rate for serial communication",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz (set to false if launching custom RViz externally)",
        )
    )
    # 加载OMPL配置
    ompl_planning_yaml = os.path.join(
        get_package_share_directory("arm_moveit_config"), "config", "ompl_planning.yaml"
    )

    with open(ompl_planning_yaml, "r") as f:
        ompl_config = yaml.safe_load(f)

    # 获取启动配置
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_rviz = LaunchConfiguration("use_rviz")

    # 设置全局环境变量完全抑制不必要的日志
    qt_env = SetEnvironmentVariable("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false")
    rcutils_env = SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "FATAL")
    class_loader_env = SetEnvironmentVariable("CLASS_LOADER_LOG_LEVEL", "FATAL")

    # 1. 启动static_transform_publisher - 发布静态变换
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. 启动您的硬件驱动节点（先启动，确保joint_states话题存在）
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("arm_driver_node"),
                    "launch",
                    "driver_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "serial_port": serial_port,
            "baud_rate": baud_rate,
        }.items(),
    )

    # 延迟启动robot_state_publisher，确保驱动节点先启动
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 20.0},
            {"ignore_timestamp": False},  # 确保时间戳检查启用
        ],
    )

    delayed_robot_state_publisher = TimerAction(
        period=3.0, actions=[robot_state_publisher]
    )

    # 3. 启动move_group节点 - 完全清理版本
    # 在move_group_node中添加规划器配置
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            ompl_config,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            moveit_config.planning_scene_monitor,
            {
                "use_sim_time": use_sim_time,
                "moveit_manage_controllers": False,
                # 发布 SRDF 语义描述到参数服务器供 RViz 使用
                "publish_robot_description_semantic": True,
                # 增加轨迹执行超时时间 - 大幅放宽以适应真实硬件
                "trajectory_execution.allowed_execution_duration_scaling": 10.0,  # 从3.0增加到10.0
                "trajectory_execution.allowed_goal_duration_margin": 5.0,  # 从1.0增加到5.0秒
                "trajectory_execution.allowed_start_tolerance": 0.5,  # 放宽起始容差
                "trajectory_execution.execution_duration_monitoring": True,
                "trajectory_execution.wait_for_trajectory_completion": True,  # 等待完成确认
                # 增加步长距离 - 直接减少路点数量
                "trajectory_execution.allowed_step_interpolation": 0.3,  # 增加步长插值
                "trajectory.sample_duration": 0.1,  # 增大采样间隔到0.1秒，直接减少路点
                # 轨迹简化参数
                "trajectory.simplify_solutions": True,
                "trajectory.waypoint_reduction": True,
                "trajectory.simplification_factor": 0.1,  # 尝试减少50%的路点
                # 轨迹处理参数
                "trajectory_execution.trajectory_duration": 10.0,  # 最大轨迹时间
                "trajectory_execution.trajectory_smoothing": True,
                # 关节限制容差 - 大幅放宽
                "robot_description_planning.default_velocity_scaling_factor": 0.5,
                "robot_description_planning.default_acceleration_scaling_factor": 0.5,
                "robot_description_planning.joint_limits.default_velocity_scaling_factor": 0.5,
                "robot_description_planning.joint_limits.default_acceleration_scaling_factor": 0.5,
                # 笛卡尔限制
                "robot_description_planning.cartesian_limits.max_trans_vel": 1.0,
                "robot_description_planning.cartesian_limits.max_trans_acc": 2.25,
                "robot_description_planning.cartesian_limits.max_trans_dec": -5.0,
                "robot_description_planning.cartesian_limits.max_rot_vel": 1.57,
                # 强制使用OMPL规划器而不是CHOMP
                "default_planning_pipeline": "ompl",
                "planning_plugin": "ompl_interface/OMPLPlanner",
                # OMPL规划器特定参数
                "ompl.planning.simplify_solutions": True,
                "ompl.planning.max_waypoint_distance": 0.1,  # 增加路点间最大距离
                "ompl.planning.longest_valid_segment_fraction": 0.1,  # 调整插值步长
                "ompl.planning.simplification_time": 1.0,  # 增加简化时间
                # 禁用一些可能增加路点的功能
                "ompl.planning.interpolate": False,  # 禁用插值
                # 添加轨迹处理参数
                "time_parameterization.max_velocity_scaling_factor": 0.5,  # 增加
                "time_parameterization.max_acceleration_scaling_factor": 0.5,  # 增加
                # 状态监控配置
                "planning_scene_monitor.robot_description": "robot_description",
                "planning_scene_monitor.joint_state_topic": "/joint_states",
                "planning_scene_monitor.attached_collision_object_topic": "/move_group/planning_scene_monitor",
                "planning_scene_monitor.publish_planning_scene_topic": "/move_group/monitored_planning_scene",
                "planning_scene_monitor.publish_geometry_updates": False,
                "planning_scene_monitor.publish_state_updates": False,
                "planning_scene_monitor.publish_transforms_updates": False,
                # 增加状态监控超时时间
                "move_group.state_update_timeout": 10.0,  # 增加到10秒
                # 禁用对象识别
                "move_group.enable_object_detection": False,
                "publish_planning_scene": False,
                "publish_geometry_updates": False,
                "publish_state_updates": False,
                "publish_transforms_updates": False,
                # 关节限制检查配置
                "enforce_joint_model_state_space": False,  # 放宽关节状态空间检查
            },
        ],
    )

    delayed_move_group = TimerAction(period=4.0, actions=[move_group_node])

    # 4. 启动RViz2 - 最小化日志输出 (可选)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            rviz_config,
            "--ros-args",
            "--log-level",
            "FATAL",  # 只显示致命错误
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),  # 条件启动
    )

    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    # 组装所有节点
    nodes_to_start = [
        qt_env,
        rcutils_env,
        class_loader_env,  # 新增：抑制class_loader警告
        static_tf,
        driver_launch,
        delayed_robot_state_publisher,
        delayed_move_group,
        delayed_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
