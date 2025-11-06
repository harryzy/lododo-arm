#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Feetech ST3215 Servo Controller for Robotic Arm.

This module provides a high-level interface for controlling Feetech ST3215
serial servos using the SCS communication protocol. Designed for use with
ROS 2 nodes to control robotic arm joints.

Author: lododo
License: Apache 2.0
"""

import time
import sys
import os

from .scservo_sdk import *


class ServoArmController:
    """
    Controller class for Feetech ST3215 servo-based robotic arm.
    
    Provides high-level methods for servo control including position setting,
    torque management, and synchronized multi-servo movements.
    """

    def __init__(
        self,
        ros_node,
        port_name="/dev/ttyACM0",
        baudrate=1000000,
        servo_count=6,
        cmd_delay_time=0.1,
    ):
        """
        Initialize servo controller.
        
        Args:
            ros_node: ROS 2 node instance for logging
            port_name (str): Serial port device name (e.g., /dev/ttyACM0)
            baudrate (int): Serial communication baudrate (default: 1M for Feetech)
            servo_count (int): Number of servos to control
            cmd_delay_time (float): Delay between commands in seconds
        """
        self.node = ros_node
        self.logger = ros_node.get_logger()

        self.cmd_delay_time = cmd_delay_time  # Command delay time
        self.logger.debug(f"Servo command delay time: {cmd_delay_time}s")
        
        # Control parameters
        self.DEVICENAME = port_name
        self.BAUDRATE = baudrate
        self.SERVO_COUNT = servo_count
        self.CENTER_POSITION = 2048  # Center position value (0 radians)

        # Servo motion parameters
        self.MOVING_SPEED = 800  # Default moving speed
        self.INIT_MOVING_TIME = 2000  # Initial moving time (ms)
        self.MOVING_ACC = 80  # Default acceleration

        # Initialize port handler
        self.portHandler = PortHandler(self.DEVICENAME)

        # Initialize packet handler
        self.packetHandler = sms_sts(self.portHandler)

        # Servo ID list (1 to servo_count)
        self.servo_ids = list(range(1, self.SERVO_COUNT + 1))

        self.logger.debug("Servo controller initialized")

    def connect(self):
        """
        Connect to servo controller via serial port.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        # Open serial port
        if not self.portHandler.openPort():
            self.logger.error("Failed to open serial port")
            return False

        # Set baudrate
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            self.logger.error("Failed to set baudrate")
            self.portHandler.closePort()
            return False

        self.logger.info(
            f"Connected to servo controller (port: {self.DEVICENAME}, baudrate: {self.BAUDRATE})"
        )
        return True

    def disconnect(self):
        """Close connection to servo controller."""
        self.portHandler.closePort()
        self.logger.debug("Servo controller connection closed")

    def setup_torque(self, servo_id=None, enable_value=0):
        """
        Control servo torque output state.
        
        Args:
            servo_id (int, optional): Servo ID, None for all servos
            enable_value (int): Torque control value:
                0: Disable torque output
                1: Enable torque output
                2: Damping mode (STS servo specific)
                128: Calibrate current position to 2048
        
        Returns:
            bool: True if all specified servos set successfully
        """
        ids = [servo_id] if servo_id is not None else self.servo_ids
        success = True

        for id in ids:
            scs_comm_result, scs_error = self.packetHandler.WriteTorqueEnable(
                id, enable_value
            )

            if scs_comm_result != COMM_SUCCESS:
                self.logger.error(
                    f"Servo {id}: {self.packetHandler.getTxRxResult(scs_comm_result)}"
                )
                success = False
            elif scs_error != 0:
                self.logger.error(
                    f"Servo {id}: {self.packetHandler.getRxPacketError(scs_error)}"
                )
                success = False
            else:
                self.logger.debug(f"Servo {id}: Torque register set, state={enable_value}")

            time.sleep(0.1)

        return success

    def read_position(self, servo_id=None):
        """
        Read current position of specified servo(s)

        Args:
            servo_id (int, optional): Servo ID, None for all servos

        Returns:
            dict: Dict of servo IDs and positions, or single position value
        """
        if servo_id is not None:
            scs_present_position, scs_present_speed, scs_comm_result, scs_error = (
                self.packetHandler.ReadPosSpeed(servo_id)
            )

            if scs_comm_result != COMM_SUCCESS:
                self.logger.error(
                    f"Servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}"
                )
                return None
            elif scs_error != 0:
                self.logger.error(
                    f"Servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}"
                )
                return None

            self.logger.debug(
                f"Servo {servo_id}: current position = {scs_present_position}, current speed = {scs_present_speed}"
            )
            return scs_present_position
        else:
            positions = {}
            for id in self.servo_ids:
                position = self.read_position(id)
                if position is not None:
                    positions[id] = position
                time.sleep(0.1)

            return positions

    def set_position(self, servo_id, position, speed=None, acceleration=None):
        """
        Set singleServotarget position

        Args:
            servo_id (int): ServoID
            position (int): target position
            speed (int, optional): moving speed
            acceleration (int, optional): acceleration

        Returns:
            bool: Whether successfully set
        """
        speed = speed or self.MOVING_SPEED
        acceleration = acceleration or self.MOVING_ACC

        scs_comm_result, scs_error = self.packetHandler.RegWritePosEx(
            servo_id, position, speed, acceleration
        )

        if scs_comm_result != COMM_SUCCESS:
            self.logger.error(
                f"Servo {servo_id}: {self.packetHandler.getTxRxResult(scs_comm_result)}"
            )
            return False
        elif scs_error != 0:
            self.logger.error(
                f"Servo {servo_id}: {self.packetHandler.getRxPacketError(scs_error)}"
            )
            return False

        self.logger.debug(
            f"Servo {servo_id}: Set position={position}, speed={speed}, acceleration={acceleration}"
        )
        return True

    def set_all_positions(self, positions, speed=None, acceleration=None):
        """
        Set multipleServoto their respectivetarget position

        Args:
            positions (dict): Dict containing servo IDs and target positions, format like {1: 2048, 2: 3000, ...}
            speed (int, optional): moving speed
            acceleration (int, optional): acceleration

        Returns:
            bool: Whether successfully setallServo
        """
        success = True

        # Check ifallServoIDhave specified positions
        for servo_id in self.servo_ids:
            if servo_id not in positions:
                self.logger.debug(f"Servo {servo_id} No position specified, will be skipped")

        # Set position for each servo
        for servo_id, position in positions.items():
            if servo_id in self.servo_ids:
                if not self.set_position(servo_id, position, speed, acceleration):
                    success = False
                time.sleep(self.cmd_delay_time)  # Ensure sufficient delay between servos
            else:
                self.logger.warn(f"ServoID {servo_id} not in configuredServolist, will be skipped")

        return success

    def execute_movement(self):
        """
        Execute previously set movement commands

        Returns:
            bool: Whether successfully executed
        """
        # Check RegAction actual return type
        result = self.packetHandler.RegAction()

        # If return is integer (error code), simple processing
        if isinstance(result, int):
            if result != 0:  # Assume 0 is success code
                self.logger.error(f"Execute command failed: error code {result}")
                return False
        # If return is tuple, process as original code
        else:
            scs_comm_result, scs_error = result
            if scs_comm_result != COMM_SUCCESS:
                self.logger.error(
                    f"Execute command failed: {self.packetHandler.getTxRxResult(scs_comm_result)}"
                )
                return False
            elif scs_error != 0:
                self.logger.error(
                    f"Execute command error: {self.packetHandler.getRxPacketError(scs_error)}"
                )
                return False

        self.logger.debug("allServostart executing movement")
        return True

    def verify_positions(self, target_positions, tolerance=10):
        """
        VerifyallServowhether reachedtarget position

        Args:
            target_positions (dict): Dict containing servo IDs and target positions, format like {1: 2048, 2: 3000, ...}
            tolerance (int): Allowed position error

        Returns:
            bool: Whether all servos at target position
        """
        all_correct = True
        current_positions = self.read_position()

        for id, pos in current_positions.items():
            # Check if this ID has corresponding target position
            if id in target_positions:
                target_pos = target_positions[id]
                if abs(pos - target_pos) > tolerance:
                    self.logger.warn(
                        f"Servo {id}: Position deviation too large (target: {target_pos}, actual: {pos})"
                    )
                    all_correct = False
                else:
                    self.logger.debug(
                        f"Servo {id}: Position correct (target: {target_pos}, actual: {pos})"
                    )
            else:
                # If no specifiedtarget position, Log but does not affect result
                self.logger.warn(f"Servo {id}: No specified target position (current position: {pos})")

        # Check if any servo ID failed to read position
        for id in target_positions:
            if id not in current_positions:
                self.logger.error(f"Servo {id}: Cannot readcurrent position")
                all_correct = False

        return all_correct

    def initialize_to_center(self):
        """
        InitializeallServoto center position complete process

        Returns:
            bool: Whether successfullycompleted initialization
        """
        self.logger.info("===== StartServocenterInitialize =====")

        # 1. Connect to controller
        if not self.connect():
            return False

        try:
            # 2. Set servo torque
            self.logger.debug("----- Set servo torque -----")
            self.setup_torque(None, enable_value=128)

            # 3. Readcurrent position
            self.logger.debug("----- Readcurrent position -----")
            self.read_position()

            # 4. Execute movement command
            self.logger.debug("----- Execute movement command -----")
            if not self.execute_movement():
                return False

            # 5. Wait for movement completion
            self.logger.debug(f"----- Wait for movement completion ({self.INIT_MOVING_TIME/1000}seconds) -----")
            time.sleep(self.INIT_MOVING_TIME / 1000 + 1)

            # 6. Verify position
            self.logger.debug("----- Verify servo position -----")

            target_positions = {
                1: self.CENTER_POSITION,
                2: self.CENTER_POSITION,
                3: self.CENTER_POSITION,
                4: self.CENTER_POSITION,
                5: self.CENTER_POSITION,
                6: self.CENTER_POSITION,
            }  # Assume all servos to center position

            result = self.verify_positions(target_positions)

            if result:
                self.logger.info("✓ All servos successfully initialized to center position")
            else:
                self.logger.warn("✗ Some servos failed to accurately reach center position")

            return result

        finally:
            # 8. Close connection
            self.disconnect()

    def move_to_center_position(self, speed=None, acceleration=None):
        """
        Move all servos to center position

        Returns:
            bool: Whether successfully moved all servos
        """
        self.logger.info("----- Move all servos to center position -----")
        positions = {
            1: self.CENTER_POSITION,
            2: self.CENTER_POSITION,
            3: self.CENTER_POSITION,
            4: self.CENTER_POSITION,
            5: self.CENTER_POSITION,
            6: self.CENTER_POSITION,
        }
        return self.set_all_positions(positions, speed, acceleration)

    def move_to_stop_position(self, speed=None, acceleration=None):
        """
        Move all servos to specified stop position

        Args:
            position (int): target position

        Returns:
            bool: Whether successfully moved all servos
        """
        positions = {1: 2027, 2: 956, 3: 3936, 4: 2905, 5: 3111, 6: 805}
        self.logger.info(f"----- Move all servos to stop position {positions} -----")
        return self.set_all_positions(positions, speed, acceleration)


def main():
    """Main function"""
    # Create servo controller instance
    arm = ServoArmController(port_name="/dev/ttyACM0", baudrate=1000000, servo_count=6)

    # Execute center initialization process
    success = arm.initialize_to_center()

    # Output final result
    if success:
        print("\nProgram successfully completed! All servos initialized to center position.")
    else:
        print("\nProgram completed with issues, please check logs.")

    return 0 if success else 1


if __name__ == "__main__":
    main()
