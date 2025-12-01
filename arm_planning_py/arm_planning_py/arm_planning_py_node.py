#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2 import GripperInterface
from rclpy.callback_groups import ReentrantCallbackGroup
from .robots import lododo_arm  # Import the Lododo arm definitions
from scipy.spatial.transform import Rotation
import time
import tf_transformations
from typing import Union, Tuple, List
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
import math

import rclpy.executors as rex
from threading import Thread, Event
from moveit_msgs.msg import MoveItErrorCodes
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration


class ArmPlanningPyNode(Node):
    def __init__(self, node_name="arm_planning_py_node"):
        super().__init__(node_name)

        # Create callback group that allows execution of callbacks in parallel without restrictions

        self.callback_group = ReentrantCallbackGroup()

        # 1. Initialize the arm_grp in the robotic arm controlled by move group
        self.arm = MoveIt2(
            node=self,
            joint_names=lododo_arm.arm_joint_names(),  # Modify according to actual joint names
            base_link_name=lododo_arm.base_link_name(),
            end_effector_name=lododo_arm.end_effector_name(),  # Modify according to actual end effector link name
            group_name=lododo_arm.MOVE_GROUP_ARM,
            callback_group=self.callback_group,
        )
        # self.arm.allowed_planning_time = 5.0
        self.arm.max_velocity = 0.5
        self.arm.max_acceleration = 0.5
        self.arm.planner_id = "RRTConnectkConfigDefault"  # Modify according to actual planner ID
        self.arm.allowed_planning_time = 5.0
        self.arm.num_planning_attempts = 10  # Default 10[7,8](@ref)

        # self.arm_position_tolerance = 0.1  # Set position tolerance (unit: meters)
        # self.arm_orientation_tolerance = 0.5  # Set orientation tolerance (unit: radians)
        # Change inverse kinematics solver
        # self.arm.planner_id = "BKPIECE"  # or "PRMstar"
        # # Relax position and orientation tolerance (unit: meters/radians)
        # self.arm.set_goal_position_tolerance(0.02)
        # self.arm.set_orientation_goal  # Default 0.01
        # self.arm.set_goal_orientation_tolerance(0.1)  # Default 0.01
        # Extend planning time and increase number of attempts

        # 2. Initialize the hand_grp in the robotic arm controlled by move group
        # Create gripper interface
        self.gripper_interface = GripperInterface(
            node=self,
            gripper_joint_names=lododo_arm.hand_joint_names(),
            open_gripper_joint_positions=lododo_arm.OPEN_HAND_JOINT_POSITIONS,
            closed_gripper_joint_positions=lododo_arm.CLOSED_HAND_JOINT_POSITIONS,
            gripper_group_name=lododo_arm.MOVE_GROUP_HAND,
            callback_group=self.callback_group,
            gripper_command_action_name=lododo_arm.GRIPPER_COMMAND_ACTION_NAME,
        )
        self.gripper_interface.max_velocity = 0.5
        self.gripper_interface.max_acceleration = 0.5
        self.gripper_interface.planner_id = (
            "RRTConnectkConfigDefault"  # Modify according to actual planner ID
        )
        # Add functionality to wait for joint state availability
        self.joint_states_available = False
        # self._executor_thread_start()

        self.get_logger().info("MoveIt 2 Initialization completed!")

    def executor_thread_start(self):

        # # Start executor callback loop
        # self.executor_thread = Thread(target=rclpy.spin, args=(self,), daemon=True)
        # self.executor_thread.start()
        # Three-layer fallback: MT -> ST -> rclpy.spin(self)
        # Use SingleThreadedExecutor to avoid action client wait-set races seen with
        # multi-threaded executors in some rclpy versions. This is more stable for
        # action clients created during node init.
        try:
            ExecST = getattr(rex, "SingleThreadedExecutor", None)
            if ExecST is None or not callable(ExecST):
                raise RuntimeError(f"Invalid SingleThreadedExecutor symbol: {ExecST}")
            self.executor = ExecST(context=self.context)
            self.executor.add_node(self)
            self._spin_stop = Event()
            self.executor_thread = Thread(target=self._spin_loop, daemon=False)
            self.executor_thread.start()
            self.get_logger().info("Create SingleThreadedExecutor and start executor thread")
        except Exception as e:
            # Last resort: do not create background executor, use rclpy.spin directly in main thread
            self.get_logger().warn(
                f"Failed to create SingleThreadedExecutor: {e}; Please call rclpy.spin in main process"
            )
            self.executor = None

    def _spin_loop(self):
        while rclpy.ok(context=self.context) and not self._spin_stop.is_set():
            # guard against executor being None due to runtime errors
            if getattr(self, "executor", None) is None:
                time.sleep(0.05)
                continue
            try:
                self.executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                self.get_logger().warn(f"executor.spin_once exception: {e}")
                time.sleep(0.1)

    def executor_thread_stop(self, join_timeout: float = 1.0):
        """
        Safely stop the executor thread and shutdown the executor.

        This will set the _spin_stop event, join the background thread, remove the
        node from the executor (if present) and shutdown the executor.
        """
        try:
            # signal loop to stop
            if getattr(self, "_spin_stop", None) is not None:
                try:
                    self._spin_stop.set()
                except Exception:
                    pass

            # join thread
            thr = getattr(self, "executor_thread", None)
            if thr is not None and thr.is_alive():
                try:
                    thr.join(join_timeout)
                except Exception:
                    pass

            # remove node from executor and shutdown executor
            if getattr(self, "executor", None) is not None:
                try:
                    # remove_node may not be available on some rclpy executor implementations
                    try:
                        self.executor.remove_node(self)
                    except Exception:
                        pass
                    try:
                        self.executor.shutdown()
                    except Exception:
                        pass
                finally:
                    self.executor = None
        except Exception as e:
            try:
                self.get_logger().warn(f"executor_thread_stop exception: {e}")
            except Exception:
                pass

    def _sleep_with_spin(self, duration: float):
        """
        Sleep while keeping ROS message loop active
        
        This ensures joint_states and other subscriptions continue to update
        during the wait period, preventing stale state issues in MoveIt.
        
        Args:
            duration: Sleep duration in seconds
        """
        import rclpy
        start_time = time.time()
        while time.time() - start_time < duration:
            try:
                rclpy.spin_once(self, timeout_sec=0.05)
            except Exception:
                pass
            time.sleep(0.01)

    def _ensure_start_state_current(self):
        """
        Force synchronize MoveIt internal state with actual arm state
        
        Solve problem: MoveIt state becomes outdated when scan_front after grasp then home
        "Invalid Trajectory: start point deviates"error
        """
        import time
        from builtin_interfaces.msg import Time as TimeMsg
        
        self.get_logger().info("🔄 Starting state synchronization...")
        
        # CRITICAL FIX: joint_states publishes at ~1.6Hz (every 0.62s)
        # Wait 1.5s to guarantee at least 2 fresh updates
        # This ensures MoveIt validation (which requires state within 1s) will pass
        self.get_logger().info("⏳ Waiting 1.5s for fresh joint_state (publish rate: ~1.6Hz)...")
        self._sleep_with_spin(1.5)
        
        self.get_logger().info("✅ Wait completed, verifying timestamp...")
        
        # method2: Read latest joint_state and verify timestamp
        js = getattr(self.arm, "joint_state", None)
        if js is None:
            self.get_logger().warn("⚠️  joint_state is empty, skip state synchronization")
            return
        
        # method2.5: Verify joint_state timestamp is recent (within 0.5s)
        current_time = self.get_clock().now()
        js_time_sec = js.header.stamp.sec + js.header.stamp.nanosec * 1e-9
        current_time_sec = current_time.seconds_nanoseconds()[0] + current_time.seconds_nanoseconds()[1] * 1e-9
        time_diff = current_time_sec - js_time_sec
        
        self.get_logger().info(
            f"🕒 joint_state timestamp age: {time_diff:.3f}s (should be < 0.5s for MoveIt validation)"
        )
        
        if time_diff > 0.5:
            self.get_logger().error(
                f"❌ joint_state timestamp is too old: {time_diff:.3f}s! This will likely fail MoveIt validation."
            )
        
        # method3: Write to MoveIt start_state
        if hasattr(self.arm, "_MoveIt2__move_action_goal"):
            try:
                self.arm._MoveIt2__move_action_goal.request.start_state.joint_state = js
                self.get_logger().info(
                    f"✅ Synchronized joint_state to start_state, timestamp_age={time_diff:.3f}s"
                )
            except Exception as e:
                self.get_logger().warn(f"⚠️  Failed to write start_state: {e}")
        else:
            self.get_logger().warn("⚠️  Cannot find _MoveIt2__move_action_goal")
        
        # method4: Force refresh moveit2 internal state cache
        if hasattr(self.arm, 'joint_state'):
            try:
                # Force update moveit2 joint state cache
                self.arm.joint_state = js
            except Exception:
                pass
        
        self.get_logger().info("🏁 State synchronization finished")

    def _retime_traj(self, traj, dt: float = 0.2):
        """
        Add strictly increasing time_from_start to trajectory without timestamps (uniform interval dt seconds)
        """
        try:
            jt = traj.joint_trajectory
        except AttributeError:
            self.get_logger().error("Input trajectory does not contain joint_trajectory field, cannot retime")
            return traj

        t = 0.0
        for p in jt.points:
            sec = int(t)
            nsec = int((t - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nsec)
            t += dt

        return traj

    def _error_code_to_text(self, code: int) -> str:
        # MoveItErrorCodes mapping (subset, sufficient for troubleshooting)
        m = {
            MoveItErrorCodes.SUCCESS: "SUCCESS",
            MoveItErrorCodes.PLANNING_FAILED: "PLANNING_FAILED",
            MoveItErrorCodes.INVALID_MOTION_PLAN: "INVALID_MOTION_PLAN",
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            MoveItErrorCodes.CONTROL_FAILED: "CONTROL_FAILED",
            MoveItErrorCodes.TIMED_OUT: "TIMED_OUT",
            MoveItErrorCodes.START_STATE_IN_COLLISION: "START_STATE_IN_COLLISION",
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            MoveItErrorCodes.GOAL_IN_COLLISION: "GOAL_IN_COLLISION",
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: "GOAL_VIOLATES_PATH_CONSTRAINTS",
            MoveItErrorCodes.INVALID_GROUP_NAME: "INVALID_GROUP_NAME",
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: "INVALID_GOAL_CONSTRAINTS",
            MoveItErrorCodes.INVALID_ROBOT_STATE: "INVALID_ROBOT_STATE",
            MoveItErrorCodes.INVALID_LINK_NAME: "INVALID_LINK_NAME",
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: "FRAME_TRANSFORM_FAILURE",
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: "COLLISION_CHECKING_UNAVAILABLE",
            MoveItErrorCodes.ROBOT_STATE_STALE: "ROBOT_STATE_STALE",
            MoveItErrorCodes.SENSOR_INFO_STALE: "SENSOR_INFO_STALE",
            MoveItErrorCodes.NO_IK_SOLUTION: "NO_IK_SOLUTION",
        }
        return m.get(int(code), f"UNKNOWN({code})")

    def log_last_moveit_error(self, prefix: str = "Planning failed"):
        # Read pymoveit2 private result (try to be compatible with field differences)
        res = getattr(self.arm, "_MoveIt2__move_action_result", None)
        if not res:
            self.get_logger().warn(f"{prefix}: Not found move_action_result")
            return
        try:
            # Compatible with two field naming conventions
            rsp = getattr(res.result, "motion_plan_response", None) or getattr(
                res.result, "planning_result", None
            )
            ec = None
            if rsp is not None and hasattr(rsp, "error_code"):
                ec = (
                    getattr(rsp.error_code, "val", None)
                    if hasattr(rsp.error_code, "val")
                    else int(rsp.error_code)
                )
            elif hasattr(res.result, "error_code"):
                ec = (
                    getattr(res.result.error_code, "val", None)
                    if hasattr(res.result.error_code, "val")
                    else int(res.result.error_code)
                )
            if ec is None:
                self.get_logger().warn(f"{prefix}: No error_code field")
                return
            self.get_logger().error(
                f"{prefix}: MoveIt error_code={ec} ({self._error_code_to_text(ec)})"
            )
        except Exception as e:
            self.get_logger().warn(f"{prefix}: Failed to parse MoveIt result: {e}")

    def robust_plan_to_pose(
        self,
        position: list[float],
        orientation: list[float],
        tolerance_position: Union[float, List[float]] = 0.02,
        tolerance_orientation: Union[float, List[float]] = 0.2,
        cartesian: bool = False,
        frame_id: str | None = None,
        max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.0,
    ):
        """
        Robust plan wrapper (enhanced version):
        1) Multiple planner + multiple tolerance combinations retry (pass tolerance_position / tolerance_orientation as list)
        2) intermediate waypoints
        3) slight perturbation search

        parameter:
            tolerance_position:
                - float: single position tolerance
                - List[float]: multiple position tolerance sequence (attempt in order, wider towards the end)
            tolerance_orientation:
                - same as above, orientation tolerance sequence
            combination rules:
                - if both are lists and have the same length => pair by index
                - if one is a single value and the other is a list => pair single value with each element of the list
                - if lengths are different and both >1 => use Cartesian product (full combination)

        Usage example:
            robust_plan_to_pose(...,
                tolerance_position=[0.01,0.02,0.03],
                tolerance_orientation=[0.2,0.4,0.6]
            )

        return:
            trajectory object if plan succeeds or None
        """
        frame_id = frame_id or self.arm.base_link_name
        orientation = self.normalize_quat(orientation)

        # Normalize to list
        def to_list(v):
            if isinstance(v, (list, tuple)):
                return [float(x) for x in v]
            return [float(v)]

        tp_list = to_list(tolerance_position)
        to_list_ = to_list(tolerance_orientation)

        # Generate combinations
        pairs: List[Tuple[float, float]] = []
        if len(tp_list) == 1 and len(to_list_) == 1:
            pairs = [(tp_list[0], to_list_[0])]
        elif len(tp_list) == 1 and len(to_list_) > 1:
            pairs = [(tp_list[0], o) for o in to_list_]
        elif len(tp_list) > 1 and len(to_list_) == 1:
            pairs = [(p, to_list_[0]) for p in tp_list]
        elif len(tp_list) == len(to_list_):
            pairs = list(zip(tp_list, to_list_))
        else:
            # Different lengths and both >1 -> Cartesian product
            pairs = [(p, o) for p in tp_list for o in to_list_]

        # Helper: ensure any tolerance-like value becomes a plain Python float
        def _tol_scalar(v):
            try:
                # if it's a list/tuple/np.ndarray, take the largest (most permissive) element
                if isinstance(v, (list, tuple)):
                    return float(max([float(x) for x in v]))
                # numpy scalar or other numeric
                return float(v)
            except Exception:
                # Fallback safe default
                return float(0.05)

        planner_ids = [
            "RRTConnectkConfigDefault",
            "RRTstarkConfigDefault",
            "PRMkConfigDefault",
            "BFMTkConfigDefault",
        ]

        # 1) multiple planner + relax tolerance retry
        for pid in planner_ids:
            self.arm.planner_id = pid
            for idx, (tp, to_) in enumerate(pairs, start=1):
                try:
                    self.get_logger().info(
                        f"[robust][planner={pid}][combo {idx}/{len(pairs)}] "
                        f"attempt tol_pos={tp}, tol_ori={to_}"
                    )
                    plan = self.arm.plan(
                        position=position,
                        quat_xyzw=orientation,
                        tolerance_position=tp,
                        tolerance_orientation=to_,
                        cartesian=cartesian,
                        frame_id=frame_id,
                        max_step=max_step,
                        cartesian_fraction_threshold=cartesian_fraction_threshold,
                    )
                    if plan and len(getattr(plan, "points", [])) >= 2:
                        self.get_logger().info(
                            f"[robust] success: planner={pid}, tol_pos={tp}, tol_ori={to_}, points={len(plan.points)}"
                        )
                        return plan
                except Exception as e:
                    self.get_logger().warn(
                        f"[robust] planner={pid} combination({idx}) callexception: {e}"
                    )

        # if all attempts failed, print more detailed diagnostic info
        try:
            self.log_last_moveit_error(
                prefix="[robust] All planner attempts failed, last MoveIt error"
            )
            self.get_logger().warn(
                f"[robust] Total number of already attempted combinations={len(pairs)}，combination examples={pairs[:10]}"
            )
        except Exception:
            pass

        # 2) intermediate waypoints: first go above target, then descend (commonly used to avoid collisions/singularities)
        pre = [position[0], position[1], min(position[2] + 0.12, position[2] + 0.20)]
        self.get_logger().info("attempt two-stage plan: first to pre-grasp position, then to target")
        plan1 = self.arm.plan(
            position=pre,
            quat_xyzw=orientation,
            tolerance_position=_tol_scalar(tolerance_position),
            tolerance_orientation=_tol_scalar(tolerance_orientation),
            cartesian=cartesian,
            frame_id=frame_id,
            max_step=max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        if plan1 and len(getattr(plan1, "points", [])) >= 2:
            # Attempt second stage using straight line (if your MoveIt configuration supports Cartesian)
            plan2 = self.arm.plan(
                position=position,
                quat_xyzw=orientation,
                tolerance_position=_tol_scalar(tolerance_position),
                tolerance_orientation=_tol_scalar(tolerance_orientation),
                cartesian=cartesian,
                frame_id=frame_id,
                max_step=0.005,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )
            if plan2 and len(getattr(plan2, "points", [])) >= 2:
                # Simple merge (timestamps will be uniformly retimed before execute)
                plan1.points.extend(plan2.points)
                return plan1

        # 3) slight perturbation search (small scan of yaw/xy within tolerance range)
        self.get_logger().info("attempt small perturbation search for target pose")
        import numpy as np

        yaw_offsets = np.deg2rad([0, 5, -5, 10, -10]).tolist()
        xy_offsets = [(0, 0), (0.01, 0), (-0.01, 0), (0, 0.01), (0, -0.01)]
        # will convert quat to rpy, perturb yaw then convert back
        rpy = tf_transformations.euler_from_quaternion(orientation)
        for dy in yaw_offsets:
            q = tf_transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2] + dy)
            for dx, dy_ in xy_offsets:
                p = [position[0] + dx, position[1] + dy_, position[2]]
                plan = self.arm.plan(
                    position=p,
                    quat_xyzw=q,
                    tolerance_position=max(_tol_scalar(tolerance_position), 0.03),
                    tolerance_orientation=max(_tol_scalar(tolerance_orientation), 0.3),
                    cartesian=cartesian,
                    frame_id=frame_id,
                    max_step=max_step,
                    cartesian_fraction_threshold=cartesian_fraction_threshold,
                )
                if plan and len(getattr(plan, "points", [])) >= 2:
                    self.get_logger().info(
                        f"small perturbation search succeeded, position={p},quat_xyzw={q},tolerance_position={max(_tol_scalar(tolerance_position), 0.03)}, tolerance_orientation={max(_tol_scalar(tolerance_orientation), 0.3)}"
                    )
                    return plan

        return None

    def check_joint_states(self):
        """Check if joint state is available"""
        self.get_logger().info("Current State: " + str(self.arm.query_state()))
        if self.arm.query_state() != MoveIt2State.IDLE:
            self.get_logger().info("current joint state not yet available, currently waiting...")
            return False
        else:
            self.get_logger().info("joint state already available！")
            self.joint_states_available = True
            return True

    def wait_for_joint_states(self):
        """waiting for joint state to be available, with timeout"""
        self.check_joint_states()
        if self.joint_states_available:
            self.get_logger().info("joint state already available，no need to wait")
            return True

        self.get_logger().info(f"waiting for joint state to be available...")
        self.get_logger().info("Current State: " + str(self.arm.query_state()))

        future = self.arm.get_execution_future()

        # Add code to check if future is None
        if future is None:
            self.get_logger().warn("No active motion target, cannot get future")
            return False

        rate = self.create_rate(10)

        # Wait until the future is done
        while not future.done():
            rate.sleep()

        self.joint_states_available = True
        # Print the result
        self.get_logger().info("Result status: " + str(future.result().status))
        self.get_logger().info(
            "Result error code: " + str(future.result().result.error_code)
        )

        return self.joint_states_available

    def ik_feasible(self, position, orientation, is_compute_ik=False):
        # Simple IK feasibility pre-check (pymoveit2 does not provide explicit IK, can directly attempt plan and see result)
        qn = self.normalize_quat(orientation)
        # Here only record normalization result
        if qn != orientation:
            self.get_logger().info(f"quaternion already normalized: {orientation} -> {qn}")

        if not is_compute_ik:
            self.get_logger().info("IK calculation not requested, directly return feasibility check result")
            return True
        retval = self.arm.compute_ik(position, orientation)
        if retval is None:
            self.get_logger().warn("ik_feasible Failed.")
            return False
        else:
            self.get_logger().info("ik_feasible Succeeded. Result: " + str(retval))
            return True

    def normalize_quat(self, q):
        import math

        norm = math.sqrt(sum(c * c for c in q))
        if norm < 1e-6:
            return [0, 0, 0, 1]
        return [c / norm for c in q]

    def point_to_list(
        self, p: Union[Point, Tuple[float, float, float], List[float]]
    ) -> list[float]:
        if hasattr(p, "x"):  # geometry_msgs/Point
            return [float(p.x), float(p.y), float(p.z)]
        return [float(p[0]), float(p[1]), float(p[2])]

    def list_to_point(
        self, p: Union[Point, Tuple[float, float, float], list[float], List[float]]
    ) -> Point:
        if hasattr(p, "x"):  # geometry_msgs/Point
            return Point(x=float(p.x), y=float(p.y), z=float(p.z))
        return Point(x=float(p[0]), y=float(p[1]), z=float(p[2]))

    def quat_to_list(
        self, q: Union[Quaternion, Tuple[float, float, float, float], List[float]]
    ) -> list[float]:
        if hasattr(q, "x"):  # geometry_msgs/Quaternion
            return [float(q.x), float(q.y), float(q.z), float(q.w)]
        return [float(q[0]), float(q[1]), float(q[2]), float(q[3])]

    def list_to_quat(
        self,
        q: Union[
            Quaternion, Tuple[float, float, float, float], list[float], List[float]
        ],
    ) -> Quaternion:
        if hasattr(q, "x"):  # geometry_msgs/Quaternion
            return Quaternion(x=float(q.x), y=float(q.y), z=float(q.z), w=float(q.w))
        return Quaternion(x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3]))

    def pose_to_lists(
        self, pose: Union[Pose, PoseStamped]
    ) -> Tuple[list[float], list[float]]:
        if isinstance(pose, PoseStamped):
            pose = pose.pose
        return self.point_to_list(pose.position), self.quat_to_list(pose.orientation)

    def lists_to_pose(self, position: list[float], orientation: list[float]) -> Pose:
        return Pose(
            position=self.list_to_point(position),
            orientation=self.list_to_quat(orientation),
        )

    def go_to_home_position(self):
        """Return arm to home position using unified joint configuration method"""
        self.get_logger().info("Returning to home position...")
        
        home_cfg = lododo_arm.arm_joint_home_positions()
        names = lododo_arm.arm_joint_names()
        
        if (not home_cfg) or len(home_cfg) != len(names):
            self.get_logger().error(
                f"Home joint array invalid: {home_cfg} (expected length {len(names)})"
            )
            return False
        
        # Use unified method with retry
        success = self.move_to_joint_configuration(
            joint_positions=home_cfg,
            description="home position"
        )
        
        if success:
            # Open gripper at home position
            self.control_gripper(position=0.0)
        
        return success

    def move_to_joint_configuration(
        self,
        joint_positions: List[float],
        max_attempts: int = 2,
        execution_timeout: float = 15.0,
        future_timeout: float = 2.0,
        description: str = "joint configuration"
    ) -> bool:
        """
        Unified method for moving to joint configuration with robust error handling
        
        Replaces direct calls to arm.move_to_configuration() + arm.wait_until_executed()
        with proper future-based result checking and retry mechanism.
        
        Args:
            joint_positions: Target joint positions (radians)
            max_attempts: Maximum retry attempts (default: 2)
            execution_timeout: Timeout for motion execution (seconds, default: 15)
            future_timeout: Timeout for future acquisition (seconds, default: 2)
            description: Human-readable description for logging
            
        Returns:
            True if successful, False if all attempts failed
            
        Example:
            # Old way (unreliable):
            self.arm.move_to_configuration(joint_positions=cfg)
            self.arm.wait_until_executed()
            
            # New way (reliable):
            if not self.move_to_joint_configuration(cfg, description="scan view1"):
                self.get_logger().error("Failed to move to scan view1")
                return False
        """
        from action_msgs.msg import GoalStatus
        
        # Synchronize state before planning
        self._ensure_start_state_current()
        
        for attempt in range(max_attempts):
            try:
                if attempt > 0:
                    self.get_logger().info(
                        f"Retry moving to {description} (attempt {attempt + 1}/{max_attempts})"
                    )
                else:
                    self.get_logger().info(f"Moving to {description}...")
                
                # Plan and execute motion
                self.arm.move_to_configuration(joint_positions=list(joint_positions))
                
                # Wait for future with timeout
                future = self.arm.get_execution_future()
                if future is None:
                    self.get_logger().warn("Future not immediately available, waiting...")
                    start_time = time.time()
                    while time.time() - start_time < future_timeout:
                        future = self.arm.get_execution_future()
                        if future is not None:
                            break
                        rclpy.spin_once(self, timeout_sec=0.01)
                        time.sleep(0.01)
                    
                    if future is None:
                        self.get_logger().error(
                            f"❌ Failed to get execution future for {description}"
                        )
                        if attempt < max_attempts - 1:
                            time.sleep(0.5)
                            continue
                        return False
                
                # Wait for execution completion
                wait_start = time.time()
                last_log = wait_start
                
                while not future.done():
                    elapsed = time.time() - wait_start
                    
                    # Timeout check
                    if elapsed > execution_timeout:
                        self.get_logger().error(
                            f"❌ Execution timeout ({execution_timeout}s) for {description}"
                        )
                        break
                    
                    # Progress logging every 2 seconds
                    if time.time() - last_log > 2.0:
                        self.get_logger().info(
                            f"⏳ Still executing {description}... ({elapsed:.1f}s elapsed)"
                        )
                        last_log = time.time()
                    
                    # Keep ROS spinning
                    try:
                        rclpy.spin_once(self, timeout_sec=0.01)
                    except Exception as e:
                        self.get_logger().warn(f"spin_once error: {e}")
                    
                    time.sleep(0.01)
                
                # Check result
                if not future.done():
                    self.get_logger().error(
                        f"❌ Motion to {description} did not complete within timeout"
                    )
                    if attempt < max_attempts - 1:
                        time.sleep(0.5)
                        continue
                    return False
                
                # Get result status
                try:
                    result = future.result()
                    if result.status == GoalStatus.STATUS_SUCCEEDED:
                        self.get_logger().info(f"✅ Successfully moved to {description}")
                        return True
                    else:
                        self.get_logger().warn(
                            f"⚠️ Motion to {description} returned status: {result.status}"
                        )
                        if attempt < max_attempts - 1:
                            time.sleep(0.5)
                            continue
                        return False
                        
                except Exception as e:
                    self.get_logger().error(
                        f"❌ Error getting result for {description}: {e}"
                    )
                    if attempt < max_attempts - 1:
                        time.sleep(0.5)
                        continue
                    return False
                    
            except Exception as e:
                self.get_logger().error(f"❌ Exception during motion to {description}: {e}")
                if attempt < max_attempts - 1:
                    time.sleep(0.5)
                    continue
                return False
        
        self.get_logger().error(
            f"❌ Failed to move to {description} after {max_attempts} attempts"
        )
        return False

    def _clear_joint5_constraint_if_locked(self, lock_joint5: bool):
        """Clear path constraints if joint5 was locked"""
        if lock_joint5:
            try:
                self.arm.clear_path_constraints()
                self.get_logger().debug("🔓 Cleared joint5 path constraint")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to clear path constraints: {e}")

    def move_arm_to_pose(
        self,
        position: list[float],
        orientation: list[float],
        cartesian=True,
        tolerance_position: Union[float, List[float]] = 0.02,
        tolerance_orientation: Union[float, List[float]] = 0.2,
        max_step=0.01,
        cartesian_fraction_threshold=0.0,
        wait=True,
        lock_joint5: bool = True,
        joint5_tolerance: float = 0.05,
    ):
        """
        Move arm to specified pose

        parameter:
            position (list): target position [x, y, z]，unit is meter
            orientation (list): target orientation，quaternion representation [x, y, z, w]
            cartesian (bool): whether to use Cartesian path planning (straight line motion)
            wait (bool): whether to wait for execution to complete
            lock_joint5 (bool): whether to lock joint5 orientation during motion (default: True)
            joint5_tolerance (float): tolerance for joint5 constraint in radians (default: 0.05)

        return:
            bool: whether execution succeeded
        """
        self.get_logger().info(f"🎯 move_arm_to_pose ENTRY: position={position}, orientation={orientation}")
        self.get_logger().info(f"Plan arm move to position: {position}, orientation: {orientation}")
        
        try:
            self.get_logger().info("📞 Calling _ensure_start_state_current()...")
            self._ensure_start_state_current()
            self.get_logger().info("✅ _ensure_start_state_current() completed")
        except Exception as e:
            self.get_logger().error(f"❌ Exception in _ensure_start_state_current(): {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
        
        # Set joint5 path constraint if lock_joint5 is enabled
        if lock_joint5:
            try:
                js = getattr(self.arm, "joint_state", None)
                if js is not None:
                    joint_names = list(js.name)
                    joint_positions = list(js.position)
                    # Find joint5 index
                    if "joint5" in joint_names:
                        j5_idx = joint_names.index("joint5")
                        j5_current = joint_positions[j5_idx]
                        self.get_logger().info(
                            f"🔒 Locking joint5 at current position: {j5_current:.4f} rad "
                            f"(tolerance: ±{joint5_tolerance:.4f} rad)"
                        )
                        # Set path constraint for joint5
                        self.arm.set_path_joint_constraint(
                            joint_positions=[j5_current],
                            joint_names=["joint5"],
                            tolerance=joint5_tolerance,
                            weight=1.0
                        )
                    else:
                        self.get_logger().warn("⚠️ joint5 not found in joint_state, cannot lock")
                else:
                    self.get_logger().warn("⚠️ joint_state is None, cannot lock joint5")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Failed to set joint5 constraint: {e}")
        
        # Parameter validation
        if len(position) != 3:
            self._clear_joint5_constraint_if_locked(lock_joint5)
            self.get_logger().error("position parameter must be a list containing 3 elements [x, y, z]")
            raise Exception("position parameter must be a list containing 3 elements [x, y, z]")

        if len(orientation) != 4:
            self._clear_joint5_constraint_if_locked(lock_joint5)
            self.get_logger().error("orientation parameter must be a quaternion containing 4 elements [x, y, z, w]")
            raise Exception("orientation parameter must be a quaternion containing 4 elements [x, y, z, w]")

        check_result, details = self.check_within_workspace(position, verbose=True)
        # Check if position is within workspace
        if not check_result:
            self._clear_joint5_constraint_if_locked(lock_joint5)
            self.get_logger().error(f"target position exceeds workspace, cannot execute：" + str(details))
            raise Exception("target position exceeds workspace, cannot execute")

        if not self.ik_feasible(position, orientation):
            self._clear_joint5_constraint_if_locked(lock_joint5)
            raise RuntimeError("IK pre-check failed")

        plan = self.arm.plan(
            position=position,
            quat_xyzw=orientation,
            cartesian=cartesian,
            frame_id=self.arm.base_link_name,
            max_step=max_step,
            cartesian_fraction_threshold=cartesian_fraction_threshold,
        )
        self.get_logger().debug(
            f"Initial plan call: tolerance_position={repr(tolerance_position)} (type={type(tolerance_position)}), "
            f"tolerance_orientation={repr(tolerance_orientation)} (type={type(tolerance_orientation)})"
        )
        if plan is None or len(getattr(plan, "points", [])) < 2:
            self.get_logger().warn(
                f"First attempt with default tolerance plan failed/too few trajectory points (tol_pos=0.001, tol_ori=0.001, planner={self.arm.planner_id}）, "
                "Warning: Next will perform more robust plan attempts: 1. explicit IK calculation result, 2. use higher tolerance combinations for multiple attempts (may take longer), 3. use two-stage planning algorithm, 4. use small perturbation search for target pose (may take longer)"
            )
            self.log_last_moveit_error("First plan failed")
            # Diagnostic: attempt explicit IK calculation and print current joint_state to help locate unreachable or collision causes
            try:
                ik_ok = self.ik_feasible(position, orientation, is_compute_ik=True)
                self.get_logger().info(f"Explicit IK calculation result: {ik_ok}")
            except Exception as _e:
                self.get_logger().warn(f"Explicit IK calculation threw exception: {_e}")

            try:
                js = getattr(self.arm, "joint_state", None)
                if js is not None:
                    names = getattr(js, "name", [])
                    pos = getattr(js, "position", [])
                    self.get_logger().info(f"current JointState names={names}")
                    self.get_logger().info(f"current JointState positions={pos}")
                else:
                    self.get_logger().warn("Cannot read current joint_state (None)")
            except Exception as _e:
                self.get_logger().warn(f"Exception when reading joint_state: {_e}")

            # Use robust plan retry
            plan = self.robust_plan_to_pose(
                position=position,
                orientation=orientation,
                tolerance_position=tolerance_position,
                tolerance_orientation=tolerance_orientation,
                cartesian=cartesian,
                frame_id=self.arm.base_link_name,
                max_step=max_step,
                cartesian_fraction_threshold=cartesian_fraction_threshold,
            )

        if plan is None or len(getattr(plan, "points", [])) < 2:
            self.get_logger().error("All strategies failed, cannot obtain valid trajectory")
            # Fallback: if pose plan failed, attempt explicit compute_ik and directly move execute in joint space
            try:
                self.get_logger().info(
                    "attempt fallback: obtain joint solution through compute_ik and execute in joint space"
                )
                ik_sol = self.arm.compute_ik(position, orientation)
                if ik_sol:
                    # Ensure solution length matches number of joints
                    names = lododo_arm.arm_joint_names()
                    if len(ik_sol) == len(names):
                        self.get_logger().info(
                            "compute_ik returned joint solution, attempt move_to_configuration execute (joint space)"
                        )
                        # Use unified method for joint configuration
                        if self.move_to_joint_configuration(
                            list(map(float, ik_sol)), 
                            description="IK fallback solution"
                        ):
                            self.get_logger().info("joint space execution complete (fallback)")
                            return True
                        else:
                            self.get_logger().warn("joint space execution failed (fallback)")
                    else:
                        self.get_logger().warn(
                            f"compute_ik returned solution length does not match number of joints: len(ik)={len(ik_sol)} vs names={len(names)})"
                        )
                else:
                    self.get_logger().warn(
                        "compute_ik did not return valid solution, cannot perform joint space fallback"
                    )
            except Exception as e:
                self.get_logger().warn(
                    f"exception occurred during fallback compute_ik/ move_to_configuration process: {e}"
                )

            self._clear_joint5_constraint_if_locked(lock_joint5)
            raise RuntimeError("Path planning failed")

        self.get_logger().info("plansuccess，startexecute...")
        # Clear constraint before execution (planning is done)
        self._clear_joint5_constraint_if_locked(lock_joint5)
        
        # execute
        self.arm.execute(plan)
        # waitingexecutecomplete
        if wait:
            # Note: the same functionality can be achieved by setting
            # `synchronous:=false` and `cancel_after_secs` to a negative value.
            
            # Wait for action to be accepted first (give it time to register)
            # Use time-based waiting to avoid blocking ROS callbacks
            start_time = time.time()
            timeout = 2.0  # 2 seconds timeout
            future = None
            
            while time.time() - start_time < timeout:
                future = self.arm.get_execution_future()
                if future is not None:
                    elapsed = time.time() - start_time
                    self.get_logger().info(f"Got execution future after {elapsed:.3f}s")
                    break
                # Keep ROS spinning to process callbacks - use shorter delay
                try:
                    rclpy.spin_once(self, timeout_sec=0.01)
                except Exception:
                    pass
                time.sleep(0.01)  # Reduced from 0.05 to 0.01 for faster response
            
            if future is None:
                self.get_logger().error(f"No active motion target after {timeout}s, cannot get execution future")
                return False
            
            # Instead of using wait_until_executed, wait directly on the future
            # This avoids race conditions with fast-completing motions
            wait_timeout = 30.0  # 30 second timeout for motion completion
            wait_start = time.time()
            last_log_time = wait_start
            
            self.get_logger().info("Waiting for motion to complete...")
            
            while not future.done():
                elapsed = time.time() - wait_start
                
                # Log progress every 2 seconds
                if time.time() - last_log_time > 2.0:
                    self.get_logger().info(f"Still waiting for motion... ({elapsed:.1f}s elapsed)")
                    last_log_time = time.time()
                
                if elapsed > wait_timeout:
                    self.get_logger().error(f"Motion execution timeout after {wait_timeout}s")
                    return False
                
                # Keep ROS spinning while waiting
                try:
                    rclpy.spin_once(self, timeout_sec=0.05)
                except Exception as e:
                    self.get_logger().warn(f"spin_once error: {e}")
                    pass
                
                time.sleep(0.01)  # Small sleep to avoid busy-waiting
            
            self.get_logger().info(f"Motion completed after {time.time() - wait_start:.3f}s")
            
            # Future is done, check result
            try:
                result = future.result()
                if result.status == GoalStatus.STATUS_SUCCEEDED:
                    self.get_logger().info("Arm successfully reached target pose")
                    return True
                else:
                    self.get_logger().error(f"Arm execution failed with status: {result.status}")
                    return False
            except Exception as e:
                self.get_logger().error(f"Failed to get future result: {e}")
                return False
        else:
            # Wait for the request to get accepted (i.e., for execution to start)
            self.get_logger().info(
                "MoveIt2State Current State: " + str(self.arm.query_state())
            )
            rate = self.create_rate(10)
            while (
                self.arm.query_state() != MoveIt2State.EXECUTING
                and self.arm.query_state() != MoveIt2State.IDLE
            ):
                self.get_logger().info("Waiting for execution to start...")
                rate.sleep()

            # Get the future
            self.get_logger().info(
                "future Current State: " + str(self.arm.query_state())
            )
            future = self.arm.get_execution_future()

            # Wait until the future is done
            if future is None:
                self.get_logger().warn("No active motion target, cannot get future")
                return False

            while not future.done():
                rate.sleep()
            
            # Check execution result
            result = future.result()
            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Arm successfully reached target pose")
                return True
            else:
                self.get_logger().error(f"Arm execution failed with status: {result.status}")
                return False

    def move_arm_to_pose_euler(
        self, position, euler_angles, cartesian=False, wait=True
    ):
        """
        Use Euler angle representation for orientation and move arm to specified pose

        parameter:
            position (list): target position [x, y, z]，unit is meter
            euler_angles (list): target orientation，Euler angle representation [roll, pitch, yaw]，unit is radian
            cartesian (bool): whether to use Cartesian path planning
            wait (bool): whether to wait for execution to complete
            timeout (float): execution timeout time, unit is seconds

        return:
            bool: whether execution succeeded
        """
        # Convert Euler angles to quaternion
        quaternion = tf_transformations.quaternion_from_euler(
            euler_angles[0], euler_angles[1], euler_angles[2]  # roll  # pitch  # yaw
        )
        self.get_logger().info(f"Using Euler angle representation for orientation, converted quaternion: {quaternion}")
        # callmove_arm_to_posemethod
        return self.move_arm_to_pose(
            position=position, orientation=quaternion, cartesian=cartesian, wait=wait
        )

    def control_gripper(self, position: float = 0.0, timeout: float = 8.0) -> bool:
        """
        Control gripper position
        
        Args:
            position: Gripper position (0.0=open, 1.0=closed)
            timeout: Maximum wait time in seconds
            
        Returns:
            bool: True if gripper command succeeded, False otherwise
        """
        # 4. Set gripper target position and control gripper motion
        self.get_logger().info(f"Set gripper motion ratio: {position}")
        
        # Critical: Synchronize state before gripper motion (same as move_to_joint_configuration)
        # Fix: "Found empty JointState message" and timestamp validation errors
        try:
            self._ensure_start_state_current()
        except Exception as e:
            self.get_logger().warn(f"⚠️  Gripper state synchronization failed: {e}")

        # Track execution success
        execution_success = [True]  # Use list to allow modification in nested function
        
        try:
            # --- Diagnostic: check whether gripper action servers are available ---
            try:
                # GripperCommand action client (name varies by installation)
                if hasattr(self.gripper_interface, "gripper_command_action_client"):
                    try:
                        ok = self.gripper_interface.gripper_command_action_client.wait_for_server(
                            timeout_sec=1.0
                        )
                    except Exception:
                        ok = False
                    self.get_logger().debug(
                        f"gripper_command_action_client available: {ok}"
                    )
                # MoveIt execute trajectory action client
                if hasattr(self.gripper_interface, "_execute_trajectory_action_client"):
                    try:
                        ok2 = self.gripper_interface._execute_trajectory_action_client.wait_for_server(
                            timeout_sec=0.5
                        )
                    except Exception:
                        ok2 = False
                    self.get_logger().debug(
                        f"execute_trajectory_action_client available: {ok2}"
                    )
            except Exception as _e:
                self.get_logger().warn(
                    f"gripper action availability check failed: {_e}"
                )

            if position == 0.0:
                self.gripper_interface.open()
            elif position == 1.0:
                self.gripper_interface.close()
            elif position > 1.0 or position < 0.0:
                raise ValueError("Gripper position must be between 0.0 and 1.0")
            else:
                open_pos = lododo_arm.OPEN_HAND_JOINT_POSITIONS[0]
                close_pos = lododo_arm.CLOSED_HAND_JOINT_POSITIONS[0]
                t_position = open_pos + (close_pos - open_pos) * position
                self.get_logger().debug(f"Set gripper joint position: {t_position}")
                self.gripper_interface.move_to_position(t_position)
        except Exception as e:
            self.get_logger().error(f"Failed to send gripper command: {e}")
            return False

        # Use separate thread to execute blocking wait and record warning after timeout
        def _wait_exec():
            try:
                iface = getattr(self.gripper_interface, "_interface", None)
                if iface is None:
                    # Interface not determined, record and exit
                    self.get_logger().warn(
                        "gripper interface could not be determined before waiting; no active action server?"
                    )
                    execution_success[0] = False
                    return

                ok = self.gripper_interface.wait_until_executed()
                if not ok:
                    self.get_logger().warn(
                        "gripper wait_until_executed returned False (failed or timed out on server side)"
                    )
                    execution_success[0] = False
            except Exception as ex:
                try:
                    self.get_logger().warn(
                        f"gripper wait_until_executed threw exception: {ex}"
                    )
                    execution_success[0] = False
                except Exception:
                    pass

        waiter = Thread(target=_wait_exec, daemon=True)
        waiter.start()
        waiter.join(timeout)
        if waiter.is_alive():
            self.get_logger().warn(
                f"Gripper command waiting timeout ({timeout}s)，Possibly controller did not respond or action name does not match"
            )
            execution_success[0] = False
            # Do not crash directly, return and continue; can choose to retry or report error
        else:
            self.get_logger().debug("Gripper command execution complete (or already confirmed)")
        
        return execution_success[0]

    def shutdown(self):
        self.wait_for_joint_states()
        # Distinguish whether fallback spin was used
        if getattr(self, "_fallback_spin", False):
            # Fallback mode: spin relies on rclpy.shutdown() to exit, called by outer finally
            pass
        else:
            try:
                if hasattr(self, "_spin_stop"):
                    self._spin_stop.set()
            except Exception:
                pass
            if hasattr(self, "executor") and self.executor is not None:
                try:
                    self.executor.shutdown()
                except Exception as e:
                    self.get_logger().warn(f"executor.shutdown() failed: {e}")
        if hasattr(self, "executor_thread") and getattr(self, "executor_thread", None):
            if self.executor_thread.is_alive():
                self.executor_thread.join(timeout=2.0)
                if self.executor_thread.is_alive():
                    self.get_logger().warn("Executor thread cannot close within 2 seconds")
            self.get_logger().info("Close MoveIt and exit...")

    def set_workspace(self, x, y, z):
        """Dynamically set workspace boundaries, parameter is (min,max) tuple"""
        self._ws_x = x
        self._ws_y = y
        self._ws_z = z
        self.get_logger().info(f"Update workspace: X{x} Y{y} Z{z}")

    def check_within_workspace(self, position, verbose=True) -> Tuple[bool, dict]:
        """Check if position is within workspace and return (bool, detail)"""
        # Allow dynamic adjustment
        x_min, x_max = getattr(self, "_ws_x", (-0.5, 0.5))
        y_min, y_max = getattr(self, "_ws_y", (-0.4, 0.4))
        z_min, z_max = getattr(self, "_ws_z", (-0.10, 0.5))

        px, py, pz = position
        ok_x = x_min <= px <= x_max
        ok_y = y_min <= py <= y_max
        ok_z = z_min <= pz <= z_max
        ok = ok_x and ok_y and ok_z
        if verbose and not ok:
            self.get_logger().error(
                f"Workspace check failed: pos={position} "
                f"[X {ok_x} range({x_min},{x_max}) Δ={0 if ok_x else min(px-x_min, px-x_max, key=abs)}; "
                f"Y {ok_y} range({y_min},{y_max}) Δ={0 if ok_y else min(py-y_min, py-y_max, key=abs)}; "
                f"Z {ok_z} range({z_min},{z_max}) Δ={0 if ok_z else min(pz-z_min, pz-z_max, key=abs)}]"
            )
            # Give additional hints for outrageous values
            if abs(py) > 1.0 or abs(px) > 1.0 or abs(pz) > 1.0:
                self.get_logger().warn(
                    "coordinate absolute value >1.0m, suspected unit or coordinate system error, please check if mm→m conversion or TF transformation is needed."
                )
        return ok, {"x_ok": ok_x, "y_ok": ok_y, "z_ok": ok_z}

    def calculate_facing_orientation(robot_pos, object_pos):
        """Calculate orientation for end effector to face object"""
        # Calculate orientation vector
        direction = [
            object_pos[0] - robot_pos[0],
            object_pos[1] - robot_pos[1],
            object_pos[2] - robot_pos[2],
        ]

        # Calculate horizontal plane angle and pitch angle
        yaw = math.atan2(direction[1], direction[0])
        horizontal_dist = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
        pitch = math.atan2(direction[2], horizontal_dist)

        # Convert to quaternion
        return tf_transformations.quaternion_from_euler(0, pitch, yaw)


def main():
    return


if __name__ == "__main__":
    main()
