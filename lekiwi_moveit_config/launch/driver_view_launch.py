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
    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "lekiwi_arm", package_name="lekiwi_moveit_config"
    ).to_moveit_configs()

    # Declare launch arguments
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
    # Load OMPL configuration
    ompl_planning_yaml = os.path.join(
        get_package_share_directory("lekiwi_moveit_config"), "config", "ompl_planning.yaml"
    )

    with open(ompl_planning_yaml, "r") as f:
        ompl_config = yaml.safe_load(f)

    # Get launch configurations
    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    use_rviz = LaunchConfiguration("use_rviz")

    # Set global environment variables to completely suppress unnecessary logs
    qt_env = SetEnvironmentVariable("QT_LOGGING_RULES", "*.debug=false;qt.qpa.*=false")
    rcutils_env = SetEnvironmentVariable("RCUTILS_LOGGING_SEVERITY_THRESHOLD", "FATAL")
    class_loader_env = SetEnvironmentVariable("CLASS_LOADER_LOG_LEVEL", "FATAL")

    # 1. Launch static_transform_publisher - publish static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # 2. Launch your hardware driver node (launch first to ensure joint_states topic exists)
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

    # Delayed launch of robot_state_publisher to ensure driver node starts first
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
            {"publish_frequency": 20.0},
            {"ignore_timestamp": False},  # Ensure timestamp checking is enabled
        ],
    )

    delayed_robot_state_publisher = TimerAction(
        period=3.0, actions=[robot_state_publisher]
    )

    # 3. Launch move_group node - fully cleaned version
    # Add planner configuration in move_group_node
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
                # Publish SRDF semantic description to parameter server for RViz use
                "publish_robot_description_semantic": True,
                # Increase trajectory execution timeout - greatly relaxed for real hardware
                "trajectory_execution.allowed_execution_duration_scaling": 10.0,  # Increased from 3.0 to 10.0
                "trajectory_execution.allowed_goal_duration_margin": 5.0,  # Increased from 1.0 to 5.0 seconds
                "trajectory_execution.allowed_start_tolerance": 0.5,  # Relaxed start tolerance
                "trajectory_execution.execution_duration_monitoring": True,
                "trajectory_execution.wait_for_trajectory_completion": True,  # Wait for completion confirmation
                # Increase step distance - directly reduce waypoint count
                "trajectory_execution.allowed_step_interpolation": 0.3,  # Increase step interpolation
                "trajectory.sample_duration": 0.1,  # Increase sampling interval to 0.1s to directly reduce waypoints
                # Trajectory simplification parameters
                "trajectory.simplify_solutions": True,
                "trajectory.waypoint_reduction": True,
                "trajectory.simplification_factor": 0.1,  # Try to reduce 50% of waypoints
                # Trajectory processing parameters
                "trajectory_execution.trajectory_duration": 10.0,  # Maximum trajectory time
                "trajectory_execution.trajectory_smoothing": True,
                # Joint limit tolerance - greatly relaxed
                "robot_description_planning.default_velocity_scaling_factor": 0.5,
                "robot_description_planning.default_acceleration_scaling_factor": 0.5,
                "robot_description_planning.joint_limits.default_velocity_scaling_factor": 0.5,
                "robot_description_planning.joint_limits.default_acceleration_scaling_factor": 0.5,
                # Cartesian limits
                "robot_description_planning.cartesian_limits.max_trans_vel": 1.0,
                "robot_description_planning.cartesian_limits.max_trans_acc": 2.25,
                "robot_description_planning.cartesian_limits.max_trans_dec": -5.0,
                "robot_description_planning.cartesian_limits.max_rot_vel": 1.57,
                # Force use of OMPL planner instead of CHOMP
                "default_planning_pipeline": "ompl",
                "planning_plugin": "ompl_interface/OMPLPlanner",
                # OMPL planner specific parameters
                "ompl.planning.simplify_solutions": True,
                "ompl.planning.max_waypoint_distance": 0.1,  # Increase maximum waypoint distance
                "ompl.planning.longest_valid_segment_fraction": 0.1,  # Adjust interpolation step
                "ompl.planning.simplification_time": 1.0,  # Increase simplification time
                # Disable some features that may increase waypoints
                "ompl.planning.interpolate": False,  # Disable interpolation
                # Add trajectory processing parameters
                "time_parameterization.max_velocity_scaling_factor": 0.5,  # Increased
                "time_parameterization.max_acceleration_scaling_factor": 0.5,  # Increased
                # State monitoring configuration
                "planning_scene_monitor.robot_description": "robot_description",
                "planning_scene_monitor.joint_state_topic": "/joint_states",
                "planning_scene_monitor.attached_collision_object_topic": "/move_group/planning_scene_monitor",
                "planning_scene_monitor.publish_planning_scene_topic": "/move_group/monitored_planning_scene",
                "planning_scene_monitor.publish_geometry_updates": False,
                "planning_scene_monitor.publish_state_updates": False,
                "planning_scene_monitor.publish_transforms_updates": False,
                # Increase state monitoring timeout
                "move_group.state_update_timeout": 10.0,  # Increased to 10 seconds
                # Disable object recognition
                "move_group.enable_object_detection": False,
                "publish_planning_scene": False,
                "publish_geometry_updates": False,
                "publish_state_updates": False,
                "publish_transforms_updates": False,
                # Joint limit checking configuration
                "enforce_joint_model_state_space": False,  # Relax joint state space checking
            },
        ],
    )

    delayed_move_group = TimerAction(period=4.0, actions=[move_group_node])

    # 4. Launch RViz2 - minimize log output (optional)
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
            "FATAL",  # Only show fatal errors
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(use_rviz),  # Conditional launch
    )

    delayed_rviz = TimerAction(period=6.0, actions=[rviz_node])

    # Assemble all nodes
    nodes_to_start = [
        qt_env,
        rcutils_env,
        class_loader_env,  # New: Suppress class_loader warnings
        static_tf,
        driver_launch,
        delayed_robot_state_publisher,
        delayed_move_group,
        delayed_rviz,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
