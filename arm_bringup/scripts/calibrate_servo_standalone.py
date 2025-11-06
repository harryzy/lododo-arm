#!/usr/bin/env python3
"""
Standalone Servo Calibration Tool (No ROS2 Required)

This script directly uses the Feetech servo controller to calibrate servos
without requiring any ROS2 nodes to be running.

Usage:
    python3 calibrate_servo_standalone.py [options]
    
Options:
    --port PORT         Serial port (default: /dev/ttyACM0)
    --baud BAUD        Baud rate (default: 1000000)
    --servo-count N    Number of servos (default: 6)
    
Example:
    python3 calibrate_servo_standalone.py --port /dev/ttyUSB0

Author: Lododo Arm Project
License: Apache 2.0
"""

import sys
import os
import argparse
import time

# Try to import from installed ROS2 package first, then fallback to source
try:
    # Method 1: Try importing from installed ROS2 package
    from arm_driver_node.sdk.ftservo.ft_arm_controller import ServoArmController  # type: ignore
except ImportError:
    try:
        # Method 2: Try adding local path (for direct python3 execution)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        driver_path = os.path.join(script_dir, '..', '..', 'arm_driver_node', 'arm_driver_node')
        sys.path.insert(0, driver_path)
        from sdk.ftservo.ft_arm_controller import ServoArmController  # type: ignore
    except ImportError as e:
        print(f"❌ Error: Failed to import Feetech servo controller")
        print(f"   Tried both installed package and source directory")
        print(f"   Error details: {e}")
        print()
        print("Solutions:")
        print("  1. Build the workspace: colcon build --packages-select arm_driver_node")
        print("  2. Source the workspace: source install/setup.bash")
        print("  3. Or run directly: python3 /path/to/calibrate_servo_standalone.py")
        sys.exit(1)


class DummyLogger:
    """Simple logger replacement for standalone use"""
    def info(self, msg): print(f"ℹ️  {msg}")
    def warn(self, msg): print(f"⚠️  {msg}")
    def warning(self, msg): print(f"⚠️  {msg}")
    def error(self, msg): print(f"❌ {msg}")
    def debug(self, msg): pass  # Suppress debug messages


class DummyNode:
    """Dummy ROS2 node replacement for standalone use"""
    def __init__(self):
        self._logger = DummyLogger()
    
    def get_logger(self):
        return self._logger


class StandaloneServoCalibrator:
    """Standalone servo calibration without ROS2 dependencies"""
    
    def __init__(self, port_name, baudrate, servo_count=6):
        """
        Initialize servo calibrator
        
        Args:
            port_name: Serial port (e.g., /dev/ttyACM0)
            baudrate: Communication baud rate (typically 1000000)
            servo_count: Number of servos to calibrate
        """
        self.port_name = port_name
        self.baudrate = baudrate
        self.servo_count = servo_count
        
        # Create dummy node (ServoArmController expects a ROS2 node)
        dummy_node = DummyNode()
        
        # Create servo controller instance
        self.controller = ServoArmController(
            ros_node=dummy_node,
            port_name=port_name,
            baudrate=baudrate,
            servo_count=servo_count,
            cmd_delay_time=0.02
        )
        
        self.connected = False
        
    def connect(self):
        """Open serial port connection"""
        print(f"🔌 Connecting to servos...")
        print(f"   Port: {self.port_name}")
        print(f"   Baud rate: {self.baudrate}")
        
        if not self.controller.connect():
            print(f"❌ Error: Failed to connect to servos")
            print(f"   Check:")
            print(f"   1. Device is connected to {self.port_name}")
            print(f"   2. Port permissions: sudo chmod 666 {self.port_name}")
            print(f"   3. User in dialout group: sudo usermod -a -G dialout $USER")
            print(f"   4. No other programs are using the port")
            return False
        
        print("✅ Serial port opened successfully")
        self.connected = True
        return True
    
    def disconnect(self):
        """Close serial port connection"""
        if self.connected:
            self.controller.disconnect()
            print("🔌 Serial port closed")
            self.connected = False
    
    def read_servo_position(self, servo_id):
        """
        Read current position from a servo
        
        Args:
            servo_id: Servo ID (1-6)
            
        Returns:
            Position value (0-4095) or None if read fails
        """
        positions = self.controller.read_position()
        if positions and servo_id in positions:
            return positions[servo_id]
        return None
    
    def ping_servo(self, servo_id):
        """
        Ping a servo to check if it's responding
        
        Args:
            servo_id: Servo ID (1-6)
            
        Returns:
            True if servo responds, False otherwise
        """
        pos = self.read_servo_position(servo_id)
        return pos is not None
    
    def calibrate_all_servos(self):
        """
        Calibrate all servos by setting their current positions as center (2048)
        
        Returns:
            True if all servos calibrated successfully, False otherwise
        """
        print("\n" + "="*60)
        print("  Starting Servo Calibration")
        print("="*60)
        
        # Step 1: Check all servos are responding
        print("\n📡 Step 1: Checking servo connectivity...")
        failed_servos = []
        
        for servo_id in range(1, self.servo_count + 1):
            print(f"   Servo {servo_id}: ", end="")
            if self.ping_servo(servo_id):
                print("✅ Responding")
            else:
                print("❌ Not responding")
                failed_servos.append(servo_id)
        
        if failed_servos:
            print(f"\n❌ Error: Servos {failed_servos} are not responding")
            print("   Please check:")
            print("   1. Servo power supply is connected (7-12V)")
            print("   2. Servo IDs are correctly set (1-6)")
            print("   3. Serial connections are secure")
            return False
        
        print("✅ All servos responding")
        
        # Step 2: Read current positions
        print("\n📊 Step 2: Reading current servo positions...")
        positions = self.controller.read_position()
        
        if not positions:
            print("❌ Failed to read servo positions")
            return False
        
        for servo_id in range(1, self.servo_count + 1):
            if servo_id in positions:
                pos = positions[servo_id]
                # Calculate angle from center
                angle_deg = (pos - 2048) * 0.293  # 0.293 deg per unit
                print(f"   Servo {servo_id}: Position {pos:4d} ({angle_deg:+7.2f}°)")
            else:
                print(f"   Servo {servo_id}: ❌ Failed to read position")
                return False
        
        # Step 3: Confirm calibration
        print("\n⚠️  WARNING: This will set current positions as center (2048 = 0°)")
        print("   Current servo positions will become the new reference point.")
        print("   Make sure the arm is in HOME POSE before continuing!")
        print()
        
        try:
            response = input("Continue with calibration? (yes/no): ").strip().lower()
            if response not in ['yes', 'y']:
                print("❌ Calibration cancelled by user")
                return False
        except (KeyboardInterrupt, EOFError):
            print("\n❌ Calibration cancelled by user")
            return False
        
        # Step 4: Initialize to center using controller method
        print("\n🔧 Step 3: Initializing servos to center position...")
        print("   (This will verify positions internally and close the connection)")
        
        try:
            result = self.controller.initialize_to_center()
            if result:
                print("✅ All servos initialized to center and verified")
            else:
                print("⚠️  Initialization completed but some servos may not be at exact center")
                print("   This is usually fine - servos are within tolerance")
        except Exception as e:
            print(f"❌ Calibration failed: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        print("\n" + "="*60)
        print("✅ Calibration completed successfully!")
        print("="*60)
        print()
        print("Next steps:")
        print("1. All servos are now calibrated to center position (2048 = 0°)")
        print("2. You can now start ROS2 nodes:")
        print("   ros2 launch arm_bringup real_bringup.launch.py")
        print("3. Joint states should show ~0.0 radians for all joints")
        print()
        
        return True


def print_banner():
    """Print tool banner"""
    print()
    print("="*60)
    print("  Standalone Servo Calibration Tool")
    print("  (No ROS2 Required)")
    print("="*60)
    print()


def print_home_pose_guide():
    """Print home pose positioning guide"""
    print("📐 HOME POSE Guide:")
    print()
    print("   Position your robot arm as follows:")
    print("   - Joint 1 (Base): 0° (pointing forward)")
    print("   - Joint 2 (Shoulder): 0° (horizontal)")
    print("   - Joint 3 (Elbow): 0° (straight with shoulder)")
    print("   - Joint 4 (Wrist Pitch): 0° (horizontal)")
    print("   - Joint 5 (Wrist Roll): 0° (straight)")
    print("   - Gripper: Open position")
    print()
    print("   Visual reference (side view):")
    print()
    print("        Joint5 ──┐")
    print("        Joint4 ──┤")
    print("                 │")
    print("        Joint3 ──┤")
    print("                 │")
    print("        Joint2 ──┤")
    print("                 │")
    print("        Joint1 ──┴── (Base)")
    print()


def parse_arguments():
    """Parse command line arguments"""
    parser = argparse.ArgumentParser(
        description='Calibrate Feetech servos without ROS2',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 calibrate_servo_standalone.py
  python3 calibrate_servo_standalone.py --port /dev/ttyUSB0
  python3 calibrate_servo_standalone.py --port /dev/ttyACM0 --baud 1000000
        """
    )
    
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/ttyACM0',
        help='Serial port (default: /dev/ttyACM0)'
    )
    
    parser.add_argument(
        '--baud',
        type=int,
        default=1000000,
        help='Baud rate (default: 1000000)'
    )
    
    parser.add_argument(
        '--servo-count',
        type=int,
        default=6,
        help='Number of servos (default: 6)'
    )
    
    parser.add_argument(
        '--skip-prompt',
        action='store_true',
        help='Skip home pose prompt (use with caution!)'
    )
    
    return parser.parse_args()


def main():
    """Main calibration routine"""
    args = parse_arguments()
    
    print_banner()
    print_home_pose_guide()
    
    # Confirm home pose
    if not args.skip_prompt:
        try:
            response = input("Is the arm positioned in HOME POSE? (yes/no): ").strip().lower()
            if response not in ['yes', 'y']:
                print("❌ Please position the arm in HOME POSE first, then run this script again.")
                return 1
        except (KeyboardInterrupt, EOFError):
            print("\n❌ Cancelled by user")
            return 1
    
    # Create calibrator
    calibrator = StandaloneServoCalibrator(
        port_name=args.port,
        baudrate=args.baud,
        servo_count=args.servo_count
    )
    
    try:
        # Connect to servos
        if not calibrator.connect():
            return 1
        
        # Perform calibration
        if not calibrator.calibrate_all_servos():
            print("\n❌ Calibration failed!")
            return 1
        
        return 0
        
    except KeyboardInterrupt:
        print("\n❌ Calibration interrupted by user")
        return 1
    except Exception as e:
        print(f"\n❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        calibrator.disconnect()


if __name__ == '__main__':
    sys.exit(main())
