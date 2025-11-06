#!/usr/bin/env python3
"""
Arm Driver Node for Feetech Servos with MoveIt2 Integration.

This node provides ROS2 interface for controlling a 6-DOF robotic arm
equipped with Feetech ST3215 serial servos. It translates joint trajectory
commands from MoveIt2 into servo control signals and publishes joint states.

Author: lododo
License: Apache 2.0
"""

import traceback
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState as JointStateMsg
import time
import threading
import math
import numpy as np
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

# Import Feetech servo controller
from .sdk.ftservo.ft_arm_controller import ServoArmController
from .gripper_adapter import normalize_gripper_input, angle_to_servo_position


class ArmDriverNode(Node):
    """
    ROS2 driver node for Feetech servo-based robotic arm.
    
    Provides:
    - Joint state publishing for ROS2 Control/MoveIt2
    - Trajectory execution via FollowJointTrajectory action
    - Gripper control with position feedback
    - Calibration and home position reset services
    """
    
    def __init__(self):
        super().__init__("arm_driver_node")

        # Calibration and reset services
        self.calibration_service = self.create_service(
            Trigger, "/arm/calibrate", self.calibration_callback
        )

        self.reset_service = self.create_service(
            Trigger, "/arm/reset_to_home", self.reset_to_home_callback
        )

        # Safe initial position (all joints at 0 radians)
        self.safe_position = [0.0, 0.0, 0.0, 0.0, 0.0]

        # ROS2 parameters declaration
        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 1000000)  # Feetech default: 1M baud
        self.declare_parameter("update_rate", 20.0)  # Hz, joint state publishing rate
        self.declare_parameter("cmd_delay_time", 0.002)  # Delay between servo commands (s)

        # Fast trajectory execution mode parameters
        self.declare_parameter("fast_trajectory_execution", False)
        self.declare_parameter(
            "fast_trajectory_threshold", 10
        )  # Enable fast mode when waypoints > threshold

        self.declare_parameter("auto_reset_home", False)  # Auto-reset to home on startup

        # Load fast execution mode settings
        self.use_fast_mode = self.get_parameter("fast_trajectory_execution").value
        self.fast_threshold = self.get_parameter("fast_trajectory_threshold").value
        self.auto_reset_home = self.get_parameter("auto_reset_home").value

        # Joint names configuration
        self.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]
        self.finger_joint_name = "finger_joint1"

        # Servo position to radians conversion constants
        self.CENTER_POSITION = 2048  # Center position (0 radians)
        self.POSITION_PER_RAD = 2048 / math.pi  # Position units per radian
        self.POINTS_BATCH_SIZE = 20  # Feedback publishing interval (waypoints)

        # Initialize joint state variables
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # Radians
        self.joint_velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Currently unused
        self.gripper_position = 0.0  # Radians

        # Initialize hardware connection
        self.init_hardware()

        # Create QoS profile compatible with robot_state_publisher
        state_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Critical for TF synchronization
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Joint state publisher for ROS2 Control/MoveIt2
        self.joint_state_publisher = self.create_publisher(
            JointState, "/joint_states", qos_profile=state_qos
        )
        
        # Callback group for concurrent action handling
        self.callback_group = ReentrantCallbackGroup()

        # Action server for arm trajectory execution
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            self.execute_trajectory_callback,
            callback_group=self.callback_group,
        )

        # Action server for gripper control
        self._gripper_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/hand_controller/follow_joint_trajectory",
            self.execute_gripper_callback,
            callback_group=self.callback_group,
        )
        
        # Joint command subscriber (alternative to action interface)
        self.joint_command_subscription = self.create_subscription(
            Float64MultiArray,
            "/arm_controller/commands",
            self.joint_command_callback,
            10,
        )

        # Gripper command subscriber
        self.gripper_command_subscription = self.create_subscription(
            Float64MultiArray,
            "/hand_controller/commands",
            self.gripper_command_callback,
            10,
        )

        # Timer for periodic joint state reading and publishing
        self.timer = self.create_timer(
            1.0 / self.get_parameter("update_rate").value, self.timer_callback
        )

        # Hardware access lock to prevent concurrent access
        self.hw_lock = threading.Lock()

        self.get_logger().info("Feetech arm driver node initialized (action servers ready)")

    def init_hardware(self):
        """Initialize servo controller"""
        try:
            serial_port = self.get_parameter("serial_port").value
            baud_rate = self.get_parameter("baud_rate").value

            # Add debug log
            self.get_logger().info(
                f"Attempting to connect to servos, port: {serial_port}, baud rate: {baud_rate}"
            )

            # Create servo controller instance
            self.servo_controller = ServoArmController(
                self,  # Pass current ROS2 node to use its logging system
                port_name=serial_port,
                baudrate=baud_rate,
                servo_count=6,  # 5 joint servos + 1 gripper servo
                cmd_delay_time=(
                    self.get_parameter("cmd_delay_time").value
                    if self.has_parameter("cmd_delay_time")
                    else 0.02
                ),  # Default 0.02 seconds
            )

            # Connect to servos
            if not self.servo_controller.connect():
                raise Exception("Cannot connect to servo controller")

            # Enable servo torque
            self.servo_controller.setup_torque(None, enable_value=1)

            # Initialize state
            self.hardware_connected = True
            self.get_logger().info("Connected to Feetech servo arm hardware")

            # Auto-reset to home position
            if self.auto_reset_home:
                result = self.reset_to_home_callback()
                if result:
                    self.get_logger().info("Reset to home position completed")
                else:
                    self.get_logger().error("Failed to reset to home position")

        except Exception as e:
            self.get_logger().error(f"Failed to connect to servo controller: {str(e)}")
            self.hardware_connected = False
            self.servo_controller = None

    def servo_initialize_to_center_position(self):
        """Initialize servos to center position"""
        try:
            self.servo_controller.initialize_to_center()
            self.get_logger().info("Servos initialized to center position")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize servos: {str(e)}")

    def position_to_radians(self, position, joint_index=0):
        """Convert servo position to radians"""
        # Center position 2048 corresponds to 0 radians
        # Basic conversion
        rad = (position - self.CENTER_POSITION) / self.POSITION_PER_RAD

        # Define conservative joint limits (within URDF constraints)
        joint_limits = [
            (-2.96, 2.96),   # joint1: ~±170deg (10deg margin)
            (-2.35, 2.35),   # joint2: ~±135deg (10deg margin)
            (-0.01, 3.01),   # joint3: ~0-172deg
            (-1.57, 1.57),   # joint4: ~±90deg
            (-3.14, 3.14),   # joint5: ~±180deg
            (-1.60, 0.05),   # joint6 (finger_joint1): -91.5° ~ 2.9° gripper protection
        ]

        # Apply strict limits
        if 0 <= joint_index < len(joint_limits):
            min_limit, max_limit = joint_limits[joint_index]
            rad = max(min_limit, min(max_limit, rad))

            # Log warning if exceeds limits
            original_rad = (position - self.CENTER_POSITION) / self.POSITION_PER_RAD
            if original_rad < min_limit or original_rad > max_limit:
                self.get_logger().warn(
                    f"Joint{joint_index+1}exceeds limits: {original_rad:.3f} -> {rad:.3f}"
                )

        return rad

    def radians_to_position(self, radians):
        """Convert radians to servo position"""
        # Limit to valid servo range
        position = int(self.CENTER_POSITION + radians * self.POSITION_PER_RAD)
        return max(0, min(4095, position))  # Limit to 0-4095 range

    def timer_callback(self):
        """Periodically read and publish servo state"""
        if not self.hardware_connected:
            # If hardware disconnected, attempt to reconnect
            try:
                self.init_hardware()
            except Exception:
                return

        try:
            with self.hw_lock:
                # Read current position from servos
                positions = self.servo_controller.read_position()

                # If read fails
                if positions is None:
                    self.get_logger().warn("Failed to read servo positions")
                    return

                # Convert position to radians and apply joint limits
                for i in range(1, 6):
                    if i in positions:
                        self.joint_positions[i - 1] = self.position_to_radians(
                            positions[i], i - 1  # Pass joint index
                        )

                        # Gripper position (if available)
                        if 6 in positions:
                            # Use adapter to convert servo position to radians (reverse mapping)
                            gripper_servo_pos = positions[6]
                            gripper_rad = self.position_to_radians(gripper_servo_pos, 5)
                            self.gripper_position = gripper_rad

            # Use sequence number to generate monotonic timestamp
            # self.sequence_number += 1
            # timestamp_ns = self.base_time.nanoseconds + (self.sequence_number * 50_000_000)  # Increment by 50ms each time

            # current_time = rclpy.time.Time(nanoseconds=timestamp_ns)
            # Check if all joint values are in reasonable range
            valid_state = True
            for i, pos in enumerate(self.joint_positions):
                if abs(pos) > 3.14:  # Exceeds 180 degrees
                    self.get_logger().error(f"Joint{i+1}position abnormal: {pos:.3f} rad")
                    valid_state = False

            if not valid_state:
                self.get_logger().error("Detected abnormal joint state, skipping publish")
                return
            # Use actual current time instead of sequence time
            current_time = self.get_clock().now()
            # Publish joint state
            joint_state = JointState()
            joint_state.header.stamp = current_time.to_msg()
            joint_state.header.frame_id = ""  # Empty frame_id
            joint_state.name = self.joint_names + [self.finger_joint_name]
            joint_state.position = self.joint_positions + [self.gripper_position]
            joint_state.velocity = [0.0] * len(joint_state.name)
            joint_state.effort = [0.0] * len(joint_state.name)

            self.joint_state_publisher.publish(joint_state)

            self.get_logger().debug(
                f"Published servo state: {joint_state.name} - position: {joint_state.position}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to read servo state: {str(e)}")
            self.hardware_connected = False

    def joint_command_callback(self, msg):
        """Handle joint commands from MoveIt/controller"""
        if not self.hardware_connected:
            return

        try:
            with self.hw_lock:
                # Convert radian commands to servo positions
                positions = {}
                for i, rad in enumerate(msg.data):
                    if i < 5:  # 5 joint servos
                        servo_id = i + 1
                        position = self.radians_to_position(rad)
                        positions[servo_id] = position

                # Set servo positions
                if positions:
                    if self.servo_controller.set_all_positions(positions):
                        # Execute movement
                        self.servo_controller.execute_movement()
                        self.get_logger().debug(f"Sent joint command: {positions}")
                    else:
                        self.get_logger().warn("Failed to set servo positions")

        except Exception as e:
            self.get_logger().error(f"Failed to send joint command: {str(e)}")

    def gripper_command_callback(self, msg):
        """Handle gripper commands"""
        if not self.hardware_connected or not msg.data:
            return

        try:
            with self.hw_lock:
                # Compatible with multiple inputs: angle (rad), legacy width (m), or list
                rad = normalize_gripper_input(msg.data, max_width_m=0.04)
                gripper_position = angle_to_servo_position(rad)

                # Set gripper servo position
                if self.servo_controller.set_position(6, gripper_position):
                    # Execute movement
                    self.servo_controller.execute_movement()
                    self.get_logger().debug(f"Sent gripper command: {gripper_position} (rad={rad:.3f})")
                else:
                    self.get_logger().warn("Failed to set gripper position")

        except Exception as e:
            self.get_logger().error(f"Failed to send gripper command: {str(e)}")

    def execute_trajectory_callback(self, goal_handle):
        """Handle FollowJointTrajectory action request"""
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(f"Received trajectory execution request: {len(trajectory.points)} waypoints")

        # Create result object
        result = FollowJointTrajectory.Result()

        try:
            total_points = len(trajectory.points)
            # Decide which waypoints to execute
            if self.use_fast_mode and total_points > self.fast_threshold:
                self.get_logger().info(
                    f"Using fast execution mode: executing only first and last waypoints (skipped {total_points-2} intermediate waypoints)"
                )
                points_to_execute = [trajectory.points[0], trajectory.points[-1]]
            else:
                self.get_logger().info(
                    f"Using full execution mode: executing all {total_points} waypoints"
                )
                points_to_execute = trajectory.points

            # Process each point in trajectory
            for i, point in enumerate(points_to_execute):

                # Index of current point in original trajectory (for logging)
                point_idx = (
                    0
                    if i == 0
                    else total_points - 1 if i == len(points_to_execute) - 1 else i
                )

                # Check cancel request (reduced frequency)
                if i % self.POINTS_BATCH_SIZE == 0 and goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = (
                        FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                    )
                    result.error_string = "Trajectory execution canceled"
                    return result

                # Send feedback (reduced frequency)
                if i % self.POINTS_BATCH_SIZE == 0:
                    feedback_msg = FollowJointTrajectory.Feedback()
                    feedback_msg.joint_names = trajectory.joint_names
                    feedback_msg.actual.positions = self.joint_positions
                    feedback_msg.desired.positions = list(point.positions)
                    feedback_msg.error.positions = [
                        d - a for d, a in zip(point.positions, self.joint_positions)
                    ]
                    goal_handle.publish_feedback(feedback_msg)

                # Prepare position data
                positions = {}
                for j, joint_name in enumerate(trajectory.joint_names):
                    if joint_name in self.joint_names:
                        joint_index = self.joint_names.index(joint_name)
                        servo_id = joint_index + 1
                        target_rad = point.positions[j]
                        position = self.radians_to_position(target_rad)
                        positions[servo_id] = position

                # Set servo positions - critical fix
                with self.hw_lock:
                    success = True
                    # Fallback to original method
                    success = self.servo_controller.set_all_positions(positions)
                    if not success:
                        result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                        result.error_string = f"Failed to set servo positions at waypoint {point_idx+1}"
                        goal_handle.abort()
                        return result

                    # Step 2: Execute movement command (critical!)
                    self.get_logger().debug("Executing servo movement command...")
                    if not self.servo_controller.execute_movement():
                        self.get_logger().error("Failed to execute servo movement command")
                        result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                        result.error_string = f"Failed to execute movement at waypoint{point_idx+1}"
                        goal_handle.abort()
                        return result

                # Reduce log output
                if i % self.POINTS_BATCH_SIZE == 0 or i == total_points - 1:
                    self.get_logger().info(
                        f"Trajectory progress: {point_idx+1}/{total_points}"
                    )
                # # Wait for movement completion - adjust based on trajectory time
                # expected_duration = point.time_from_start.sec + point.time_from_start.nanosec / 1e9
                # if i == 0:
                #     wait_time = max(1.0, expected_duration)
                # else:
                #     prev_duration = trajectory.points[i-1].time_from_start.sec + trajectory.points[i-1].time_from_start.nanosec / 1e9
                #     wait_time = max(0.5, expected_duration - prev_duration)

                # self.get_logger().info(f'Waiting {wait_time:.2f} seconds for movement completion...')
                # time.sleep(wait_time)

            # Trajectory execution successful
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            result.error_string = "Trajectory execution successful"
            goal_handle.succeed()
            self.get_logger().info("Trajectory execution completed!")
            return result

        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {str(e)}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"Trajectory execution exception: {str(e)}"
            goal_handle.abort()
            return result

    def execute_gripper_callback(self, goal_handle):
        """Handle gripper FollowJointTrajectory action request"""
        trajectory = goal_handle.request.trajectory
        self.get_logger().info("Received gripper trajectory execution request")

        # Create result object
        result = FollowJointTrajectory.Result()

        try:
            # Take only the last waypoint position
            if not trajectory.points:
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory is empty"
                goal_handle.abort()
                return result

            # Get final position point
            final_point = trajectory.points[-1]

            # Assume gripper joint is the first joint
            gripper_rad = normalize_gripper_input(final_point.positions[0], max_width_m=0.04)

            # Map to servo position
            gripper_position = angle_to_servo_position(gripper_rad)

            self.get_logger().info(
                f"Gripper target: {gripper_rad:.3f}rad -> position{gripper_position}"
            )

            # Set gripper servo position
            with self.hw_lock:
                # Step 1: Set positions
                if not self.servo_controller.set_position(6, gripper_position):
                    self.get_logger().warn("Failed to set gripper position")
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    result.error_string = "Failed to set gripper position"
                    goal_handle.abort()
                    return result

                # Step 2: Execute movement (critical!)
                if not self.servo_controller.execute_movement():
                    self.get_logger().error("Failed to execute gripper movement command")
                    result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
                    result.error_string = "Failed to execute gripper movement command"
                    goal_handle.abort()
                    return result

                self.get_logger().debug(f"Gripper command completed: {gripper_position}")

                # Wait for movement completion
                time.sleep(1.0)

                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Gripper command execution successful"
                goal_handle.succeed()
                return result

        except Exception as e:
            self.get_logger().error(f"Failed to execute gripper command: {str(e)}")
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = f"Gripper command execution exception: {str(e)}"
            goal_handle.abort()
            return result

    def calibration_callback(self, request, response):
        """Calibrate joints to safe position"""
        try:
            self.get_logger().info("Starting calibration to safe position...")

            # Set all joints to center position
            safe_positions = {}
            for i in range(1, 7):
                safe_positions[i] = self.CENTER_POSITION  # 2048 is center position

            # Send to hardware
            with self.hw_lock:
                result = self.servo_controller.set_all_positions(safe_positions)

            if result and self.servo_controller.execute_movement():
                # Update internal state
                self.joint_positions = [0.0] * 5
                self.gripper_position = 0.0

                response.success = True
                response.message = "Calibration completed, arm moved to safe position"
                self.get_logger().info("Calibration completed")
            else:
                response.success = False
                response.message = "Calibration failed：cannot control servos"

        except Exception as e:
            response.success = False
            response.message = f"Calibration failed：{str(e)}"
            self.get_logger().error(f"Calibration failed：{str(e)}")

        return response

    def reset_to_home_callback(self):
        """Reset to home position"""
        try:
            self.get_logger().info("Resetting to home position...")

            # Define home position (adjust for your arm)
            home_positions = {
                1: 2048,  # joint1: 0deg
                2: 2048,  # joint2: 0deg
                3: 2048,  # joint3: 0deg
                4: 2048,  # joint4: 0deg
                5: 2048,  # joint5: 0deg
                6: 2048,  # Gripper: Open
            }

            positions = {}
            positions = self.servo_controller.read_position()
            # Get servo ID and position together
            for servo_id, position in positions.items():
                rad_value = self.position_to_radians(position, servo_id - 1)
                self.get_logger().info(
                    f"Servo {servo_id}: position {position}，radians {rad_value}"
                )

            # with self.hw_lock:
            result = self.servo_controller.set_all_positions(home_positions)

            if result and self.servo_controller.execute_movement():
                return True

            else:
                return False

        except Exception as e:
            # Print detailedStack trace
            stack_trace = traceback.format_exc()
            self.get_logger().error(f"Reset failed：{str(e)}\nStack trace:\n{stack_trace}")
            return False

    def cleanup(self):
        """Cleanup resources"""
        if hasattr(self, "servo_controller") and self.servo_controller:
            try:
                # Disable servo torque
                self.servo_controller.setup_torque(None, enable_value=0)
                # Disconnect
                self.servo_controller.disconnect()
                self.get_logger().info("Disconnected from servo controller")
            except Exception as e:
                self.get_logger().error(f"Failed to disconnect from servos: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = ArmDriverNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
