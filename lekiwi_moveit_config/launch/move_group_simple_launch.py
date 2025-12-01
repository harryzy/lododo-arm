from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def _load_yaml(pkg, rel_path):
    path = os.path.join(get_package_share_directory(pkg), rel_path)
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}


def generate_launch_description():

    # Path setup
    pkg_name = "lekiwi_moveit_config"

    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        robot_name="lekiwi_arm", package_name=pkg_name
    ).to_moveit_configs()

    kin_dict = _load_yaml(pkg_name, "config/kinematics.yaml")
    ompl_dict = _load_yaml(pkg_name, "config/ompl_planning.yaml")
    jl_dict = _load_yaml(pkg_name, "config/joint_limits.yaml")  # Load joint limits

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            kin_dict,
            ompl_dict,
            {
                # Enable automatic parameter declaration (critical fix)
                "automatically_declare_parameters_from_overrides": True,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "fake_execution": False,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "moveit_manage_controllers": True,
                # Disable object recognition capabilities (Pickup/Place)
                "move_group.disable_capabilities": "move_group/MoveGroupPickupAction move_group/MoveGroupPlaceAction",
                
                # ============ DISTRIBUTED DEPLOYMENT CLOCK SYNC FIX ============
                # CRITICAL: Set to 0.0 to completely disable trajectory start validation
                # This bypasses the 1-second timestamp check that fails with clock drift
                "trajectory_execution.allowed_start_tolerance": 0.0,
                # Increase timeout for receiving current state (default is 1.0s which is too short)
                "trajectory_execution.wait_for_trajectory_completion": True,
                "trajectory_execution.execution_duration_monitoring": False,
                # ============ END DISTRIBUTED FIX ============
                
                # Trajectory execution config for distributed deployment
                "trajectory_execution.allowed_execution_duration_scaling": 15.0,
                "trajectory_execution.allowed_goal_duration_margin": 10.0,
                "trajectory_execution.execution_velocity_scaling": 1.0,
                "trajectory_execution.execution_acceleration_scaling": 1.0,
                
                # Ensure listening to correct joint state topic
                "planning_scene_monitor.joint_state_topic": "/joint_states",
                # Relax current_state_monitor timestamp validation for distributed deployment
                "planning_scene_monitor.wait_for_initial_state_timeout": 10.0,
                
                # Key 1: Explicitly set default planning pipeline
                "planning_pipeline": "ompl",
                # Key 2: Set adapters under correct namespace (double safety: set in both places)
                "ompl.request_adapters": "default_planner_request_adapters/FixWorkspaceBounds "
                "default_planner_request_adapters/FixStartStateBounds "
                "default_planner_request_adapters/FixStartStateCollision "
                "default_planner_request_adapters/FixStartStatePathConstraints "
                "default_planner_request_adapters/AddTimeOptimalParameterization",
            },
            # Critical: Inject joint_limits into robot_description_planning (your yaml top-level is joint_limits)
            {
                "robot_description_planning": {
                    "joint_limits": jl_dict.get("joint_limits", {})
                }
            },
            {"ompl.planning_time": 5.0},  # Set OMPL planning time
        ],
        # Important: Remap robot_description topic
        remappings=[
            ("robot_description", "/robot_description"),
            ("robot_description_semantic", "/robot_description_semantic"),
        ],
        arguments=[
            "--ros-args",
            # "--log-level", "moveit_ros.planning:=debug",
            # "--log-level", "moveit_ros.move_group:=debug",
            # "--log-level", "pluginlib.ClassLoader:=debug",
            "--log-level",
            "moveit_trajectory_processing.iterative_time_parameterization:=debug",
            "--log-level",
            "moveit_trajectory_processing.time_optimal_trajectory_generation:=debug",
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            move_group_node,
        ]
    )
