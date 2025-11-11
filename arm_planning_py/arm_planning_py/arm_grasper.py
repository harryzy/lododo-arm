from .arm_planning_py_node import ArmPlanningPyNode
from enum import Enum
from geometry_msgs.msg import Pose
import time
from moveit_msgs.msg import RobotState
from typing import Union, Tuple, List


class ExecutionMode(Enum):
    """Execution mode enumeration"""

    STEP = 0  # Step-by-step execution (each step requires manual trigger)
    CONTINUOUS = 1  # Continuous execution (fully automatic completion)


class ArmState(Enum):
    """Arm state machine"""

    IDLE = 0
    MOVE_TO_PREGRASP = 1  # Move to pre-grasp position
    DESCEND_AND_GRASP = 2  # Descend and grasp
    ASCEND_WITH_OBJECT = 3  # Ascend with object
    MOVE_TO_PLACE = 4  # Move to place position
    RELEASE_OBJECT = 5  # Release object
    RETURN_HOME = 6  # Return to home position


class ArmGrasper(ArmPlanningPyNode):
    def __init__(self, node_name="arm_grasper_node", is_fast_robust_plan=True):
        """
        Initialize the arm grasper.

        This constructor sets up the arm grasper with the specified planning mode and initializes
        the state control variables.

        Parameters
        ----------
        is_fast_robust_plan : bool, default=True
            If True, uses faster but less accurate robust planning parameters.
            If False, uses slower but more accurate robust planning parameters.

        Notes
        -----
        The initialization process:
        1. Inherits from the parent class
        2. Sets the initial arm state to MOVE_TO_PREGRASP
        3. Sets the execution mode to CONTINUOUS
        4. Initializes the gripper state tracking
        5. Configures robust planning parameters based on the is_fast_robust_plan parameter
        """
        super().__init__(node_name=node_name)
        # Initialize MoveIt control group
        # State control variables
        self.current_state = ArmState.MOVE_TO_PREGRASP
        self.execution_mode = ExecutionMode.CONTINUOUS
        self.is_grasped = False  # Gripper state tracking
        tolerance_position, tolerance_orientation = self._default_robust_tolerance(
            is_fast_robust_plan
        )
        self.set_robust_plan_parameters(
            tolerance_orientation=tolerance_orientation,
            tolerance_position=tolerance_position,
            cartesian=True,
            max_step=0.01,
            cartesian_fraction_threshold=0.0,
        )

    def _default_robust_tolerance(
        self, is_fast: bool
    ) -> Tuple[Union[float, List[float]], Union[float, List[float]]]:
        """Return default robust planning tolerance"""
        if is_fast:
            tolerance_position = [0.005, 0.01]
            tolerance_orientation = [0.12, 0.15, 0.2]
            return tolerance_position, tolerance_orientation

        tolerance_position = [0.005, 0.01, 0.02, 0.05, 0.1]
        tolerance_orientation = [0.005, 0.01, 0.02, 0.05, 0.1, 0.15, 0.2]
        return tolerance_position, tolerance_orientation

    def set_robust_plan_parameters(
        self,
        tolerance_position: Union[float, List[float]] = 0.02,
        tolerance_orientation: Union[float, List[float]] = 0.2,
        cartesian: bool = True,
        max_step: float = 0.01,
        cartesian_fraction_threshold: float = 0.0,
    ):
        """
        Set robust planning parameters
        """
        self.tolerance_position = tolerance_position  # Set position tolerance
        self.tolerance_orientation = tolerance_orientation  # Set target orientation tolerance
        self.cartesian = cartesian  # Set target position tolerance
        self.max_step = max_step  # Set maximum velocity scaling factor
        self.cartesian_fraction_threshold = (
            cartesian_fraction_threshold  # Set maximum acceleration scaling factor
        )
        self.get_logger().info(
            f"Robust planning parameters set: cartesian:{self.cartesian}, orientation tolerance:{self.tolerance_orientation}, position tolerance:{self.tolerance_position}, velocity and acceleration scaling factor:{self.max_step}, jump threshold:{self.cartesian_fraction_threshold}"
        )

    def _ensure_start_state_current(self):
        """Set MoveIt request's start_state to current joint_state, avoiding start point inconsistent with current state"""
        js = getattr(self.arm, "joint_state", None)
        if js is None or not getattr(js, "name", []) or not getattr(js, "position", []):
            self.get_logger().warn("JointState not available yet, skipping start_state synchronization")
            return False

        goal = getattr(self.arm, "_MoveIt2__move_action_goal", None)
        if goal is None or not hasattr(goal, "request"):
            self.get_logger().warn("move_action_goal not found, request not yet built, try again later")
            return False

        try:
            rs = RobotState()
            rs.joint_state = js
            # Explicitly set as non-differential (complete override)
            rs.is_diff = False
            goal.request.start_state = rs
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to set start_state: {e}")
            return False

    def _sync_before_new_plan(self, timeout: float = 10.0):
        """Before initiating new planning, ensure last execution is finished and synchronize start_state"""
        try:
            fut = getattr(self.arm, "get_execution_future", None)
            if callable(fut):
                f = fut()
                if f is not None and not f.done():
                    self.get_logger().info("Waiting for previous motion to complete...")
                    # NOTE: This wait_until_executed is for PREVIOUS motion, not new motion
                    # For new motions, use move_to_joint_configuration() instead
                    self.arm.wait_until_executed()
                    # Additional wait to ensure state update - use _sleep_with_spin
                    self._sleep_with_spin(0.3)
        except Exception:
            pass
        # Safety: stop residual motion
        try:
            if hasattr(self.arm, "stop_motion"):
                self.arm.stop_motion()
        except Exception:
            pass
        # Synchronize start_state
        self._ensure_start_state_current()

    def move_and_grasp(
        self,
        grasp_pose: Pose,
        place_pose: Pose,
        mode: ExecutionMode = ExecutionMode.CONTINUOUS,
        position_offset: list[float] = [0.0, 0.0, 0.0],  # Default 10cm offset above
        wait_time: float = 0.5,
    ):
        """
        Core grasp function
        :param grasp_pose: Target grasp pose (geometry_msgs/Pose)
        :param place_pose: Target place pose
        :param mode: Execution mode (step/continuous)
        """
        # Cache pose for step execution
        self.cached_grasp_pose = grasp_pose
        self.cached_place_pose = place_pose

        self.execution_mode = mode
        self.position_offset = position_offset
        self.wait_time = wait_time
        # self.current_state = ArmState.MOVE_TO_PREGRASP

        # Main loop state machine
        if self.current_state != ArmState.IDLE:
            if self.current_state == ArmState.MOVE_TO_PREGRASP:
                self._execute_move(grasp_pose, position_offset)  # Move to 10cm above grasp point
                self._gripper_control(close=False)  # Ensure gripper is open
                self.is_grasped = False  # Reset grasp state

                self.get_logger().info(
                    f"MOVE_TO_PREGRASP moved to pre-grasp position: {grasp_pose} offset: {position_offset}"
                )
                self._sleep_with_spin(wait_time)

                # self.current_state = ArmState.DESCEND_AND_GRASP  # Switch to next state
                # self.get_logger().info(f"Current state: {self.current_state.name}")

            elif self.current_state == ArmState.DESCEND_AND_GRASP:
                # self._execute_move(grasp_pose)  # Precisely move to grasp point TODO temporarily skip precise point
                self._gripper_control(close=True)  # Close gripper
                self.get_logger().info(f"DESCEND_AND_GRASP moved to grasp position: {grasp_pose}")
                self._sleep_with_spin(wait_time)
                # self.current_state = ArmState.ASCEND_WITH_OBJECT  # Switch to ascend state
                # self.get_logger().info(f"Current state: {self.current_state.name}")

            elif self.current_state == ArmState.ASCEND_WITH_OBJECT:
                self._execute_move(grasp_pose, position_offset)  # Ascend with object
                self.get_logger().info(
                    f"ASCEND_WITH_OBJECT moved object to ascend position: {grasp_pose} offset: {position_offset}"
                )
                self._sleep_with_spin(wait_time)
                # self.current_state = ArmState.MOVE_TO_PLACE  # Switch to move to place state
                # self.get_logger().info(f"Current state: {self.current_state.name}")

            elif self.current_state == ArmState.MOVE_TO_PLACE:
                self._execute_move(place_pose, position_offset)  # Move to above place point
                self.get_logger().info(
                    f"MOVE_TO_PLACE moved to above place position: {place_pose} offset: {position_offset}"
                )
                time.sleep(wait_time)
                # self.current_state = ArmState.RELEASE_OBJECT  # Switch to release object state
                # self.get_logger().info(f"Current state: {self.current_state.name}")

            elif self.current_state == ArmState.RELEASE_OBJECT:
                self._execute_move(place_pose)  # Precisely move to place point
                self._gripper_control(close=False)  # Release object
                self.get_logger().info(
                    f"RELEASE_OBJECT moved precisely to place position: {place_pose}"
                )
                time.sleep(wait_time)
                # self.current_state = ArmState.RETURN_HOME
                # self.get_logger().info(f"Current state: {self.current_state.name}")

            elif self.current_state == ArmState.RETURN_HOME:
                if not self.go_to_home_position():  # Return to home position
                    self.get_logger().error("❌ Failed to return to home position")
                    return False
                # home_pose = Pose()
                # self._execute_move(home_pose)
                self.get_logger().info(
                    f"RELEASE_OBJECT returned to home position: {self.arm.query_state()}"
                )
                time.sleep(wait_time)
                # self.current_state = ArmState.IDLE  # Switch to idle state
                # self.get_logger().info(f"Current state: {self.current_state.name}")
            # # Step mode requires external trigger for state transition
            # if mode == ExecutionMode.STEP:
            #     break
            if self.execution_mode == ExecutionMode.STEP:
                self.get_logger().info("ExecutionMode in step mode, waiting for next step...")
                return
            self.get_logger().info(
                "ExecutionMode currently in automatic state, completed current state action, automatically advancing to next state."
            )

            self.advance_step()  # Automatically advance to next state

        else:
            self.get_logger().info("ArmState current state is IDLE.")
            return

    def _execute_move(
        self, target_pose: Pose, position_offset: list[float] = [0.0, 0.0, 0.0]
    ) -> bool:
        """Motion execution helper function
        
        Returns:
            bool: True if movement succeeded, False if failed
        """
        # New: synchronization barrier before each planning
        self._sync_before_new_plan()
        adjusted_pose = self._apply_position_offset(target_pose, position_offset)

        position = [
            adjusted_pose.position.x,
            adjusted_pose.position.y,
            adjusted_pose.position.z,
        ]
        orientation = [
            adjusted_pose.orientation.x,
            adjusted_pose.orientation.y,
            adjusted_pose.orientation.z,
            adjusted_pose.orientation.w,
        ]

        success = self.move_arm_to_pose(
            position=position,
            orientation=orientation,
            cartesian=self.cartesian,
            tolerance_position=self.tolerance_position,
            tolerance_orientation=self.tolerance_orientation,
            max_step=self.max_step,
            cartesian_fraction_threshold=self.cartesian_fraction_threshold,
            wait=True,
        )
        
        if not success:
            self.get_logger().error(f"Failed to move arm to position {position}")
            return False
        
        # Additional wait to ensure state is fully stable before next step
        # Resolves "Failed to receive current joint state" warning - use _sleep_with_spin
        self._sleep_with_spin(0.5)
        return True

    def _gripper_control(self, close: bool):
        """Gripper control"""
        if close:
            self.control_gripper(position=1.0)  # Close gripper
            self.is_grasped = True
        else:
            self.control_gripper(position=0.0)  # Open gripper
            self.is_grasped = False

    def _apply_position_offset(self, pose: Pose, offset: list[float]) -> Pose:
        """Position offset compensation (collision avoidance)"""
        temp_pose = Pose()
        temp_pose.position.x = (
            pose.position.x + offset[0] if isinstance(offset, list) else offset
        )
        temp_pose.position.y = (
            pose.position.y + offset[1] if isinstance(offset, list) else offset
        )
        temp_pose.position.z = (
            pose.position.z + offset[2] if isinstance(offset, list) else offset
        )
        temp_pose.orientation = pose.orientation  # Keep original orientation
        return temp_pose

    def advance_step(self):
        """
        State advance function in step mode
        Each call advances the state machine to the next state and triggers corresponding action execution
        """
        # # Check if in step mode
        # if self.execution_mode != ExecutionMode.STEP:
        #     self.get_logger().warn("This method is only available in step mode")
        #     return False

        # State transition mapping table (current state -> next state)
        state_mapping = {
            ArmState.IDLE: ArmState.MOVE_TO_PREGRASP,
            ArmState.MOVE_TO_PREGRASP: ArmState.DESCEND_AND_GRASP,
            ArmState.DESCEND_AND_GRASP: ArmState.ASCEND_WITH_OBJECT,
            ArmState.ASCEND_WITH_OBJECT: ArmState.MOVE_TO_PLACE,
            ArmState.MOVE_TO_PLACE: ArmState.RELEASE_OBJECT,
            ArmState.RELEASE_OBJECT: ArmState.RETURN_HOME,
            ArmState.RETURN_HOME: ArmState.IDLE,
        }

        # Record current state for logging
        current_state_name = self.current_state.name
        self.get_logger().debug(f"advance_step current state: {current_state_name}")

        # State transition logic
        if self.current_state in state_mapping:
            # Execute state transition
            next_state = state_mapping[self.current_state]
            self.current_state = next_state
            self.get_logger().debug(
                f"advance_step state advance: {current_state_name} → {next_state.name}"
            )
            self.get_logger().debug(
                f"advance_step reset current state: {self.current_state.name}"
            )

            # Execute action corresponding to new state
            try:
                # Re-trigger action execution using cached pose
                self.get_logger().debug(
                    f"advance_step re-trigger action execution using cached pose: {self.cached_grasp_pose}, {self.cached_place_pose}, mode: {self.execution_mode.name}"
                )
                self.move_and_grasp(
                    self.cached_grasp_pose,
                    self.cached_place_pose,
                    self.execution_mode,
                    self.position_offset,
                    self.wait_time,
                )
                return True
            except Exception as e:
                self.get_logger().error(f"advance_step state execution failed: {str(e)}")
                raise Exception("State execution failed: " + str(e))
        elif self.current_state == ArmState.IDLE:
            self.get_logger().warn("advance_step already in idle state, no need to advance")
            return False
        else:
            self.get_logger().error("advance_step unknown state, unable to advance")
            return False
