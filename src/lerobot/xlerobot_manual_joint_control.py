#!/usr/bin/env python
"""
Manual Joint Control Script for XLerobot

This script allows you to manually set the angle for any joint of the robot.

Usage:
    PYTHONPATH=src python examples/xlerobot_manual_joint_control.py

Modes:
    Normal mode: Uses normalized values (degrees or percentage)
    Raw mode (--raw): Uses direct motor values (0-4095) bypassing calibration
    Speed control: Adjusts P_Coefficient to control speed response
                 (slow=8, medium=16, fast=25, custom 1-32)
"""

import argparse
import sys
import time
from lerobot.robots.xlerobot import XLerobotSingleArmConfig, XLerobotSingleArm


def print_available_joints(robot_config, use_right_arm=False):
    """Print list of available joints with their descriptions."""
    arm_prefix = "right_arm" if use_right_arm else "left_arm"

    print("\n" + "="*60)
    print("Available Joints")
    print("="*60)

    print("\n--- Arm Joints (Position Control) ---")
    arm_joints = [
        (f"{arm_prefix}_shoulder_pan", "Shoulder pan rotation (horizontal rotation)"),
        (f"{arm_prefix}_shoulder_lift", "Shoulder lift (up/down)"),
        (f"{arm_prefix}_elbow_flex", "Elbow flex (bend)"),
        (f"{arm_prefix}_wrist_flex", "Wrist flex (up/down)"),
        (f"{arm_prefix}_wrist_roll", "Wrist roll (rotation)"),
        (f"{arm_prefix}_gripper", "Gripper (open/close)"),
    ]

    for joint, desc in arm_joints:
        print(f"  {joint:30} - {desc}")

    print("\n--- Base Velocities (Velocity Control) ---")
    base_joints = [
        ("x.vel", "Linear velocity X (m/s)"),
        ("y.vel", "Linear velocity Y (m/s)"),
        ("theta.vel", "Angular velocity (deg/s)"),
    ]

    for joint, desc in base_joints:
        print(f"  {joint:30} - {desc}")

    print("="*60 + "\n")


def set_joint_angle(robot, joint_name, value, use_right_arm=False):
    """
    Set a specific joint to a target angle/velocity.

    Args:
        robot: XLerobotSingleArm instance
        joint_name: Name of the joint to control
        value: Target angle (for arm joints) or velocity (for base)
        use_right_arm: Whether to use right arm prefix
    """
    arm_prefix = "right_arm" if use_right_arm else "left_arm"

    # Check if it's a base velocity command
    if joint_name in ["x.vel", "y.vel", "theta.vel"]:
        action = {joint_name: float(value)}
        robot.send_action(action)
        print(f"✓ Set {joint_name} to {value}")
        return

    # Check if joint name starts with arm prefix
    if not joint_name.startswith(f"{arm_prefix}_"):
        # Try to add prefix
        if joint_name in ["shoulder_pan", "shoulder_lift", "elbow_flex",
                        "wrist_flex", "wrist_roll", "gripper"]:
            joint_name = f"{arm_prefix}_{joint_name}"
        else:
            print(f"✗ Invalid joint name: {joint_name}")
            print("  Use full joint name like 'left_arm_shoulder_pan' or 'right_arm_elbow_flex'")
            return False

    # Add .pos suffix if not present
    if not joint_name.endswith(".pos"):
        full_joint_name = f"{joint_name}.pos"
    else:
        full_joint_name = joint_name

    # Send action
    action = {full_joint_name: float(value)}
    robot.send_action(action)
    print(f"✓ Set {full_joint_name} to {value}")
    return True


def set_joint_raw(robot, joint_name, value, use_right_arm=False):
    """
    Set a specific joint to a raw motor value (0-4095).

    Args:
        robot: XLerobotSingleArm instance
        joint_name: Name of the joint to control
        value: Raw motor value (0-4095)
        use_right_arm: Whether to use right arm prefix
    """
    arm_prefix = "right_arm" if use_right_arm else "left_arm"

    # Check if joint name starts with arm prefix
    if not joint_name.startswith(f"{arm_prefix}_"):
        # Try to add prefix
        if joint_name in ["shoulder_pan", "shoulder_lift", "elbow_flex",
                        "wrist_flex", "wrist_roll", "gripper"]:
            joint_name = f"{arm_prefix}_{joint_name}"
        else:
            print(f"✗ Invalid joint name: {joint_name}")
            print("  Use full joint name like 'left_arm_shoulder_pan' or 'right_arm_elbow_flex'")
            return False

    # Validate raw value range
    try:
        raw_value = int(value)
    except (ValueError, TypeError):
        print(f"✗ Invalid raw value: {value}. Must be an integer (0-4095).")
        return False

    if raw_value < 0 or raw_value > 4095:
        print(f"✗ Raw value {raw_value} out of range. Must be between 0 and 4095.")
        return False

    # Write raw value to motor
    robot.bus.write("Goal_Position", joint_name, raw_value, normalize=False)
    print(f"✓ Set {joint_name} to raw value {raw_value}")
    return True


def set_motor_speed(robot, speed_level="medium", joint_name=None):
    """
    Set motor moving speed by adjusting P_Coefficient.

    P_Coefficient controls speed/stiffness:
    - Lower value = slower response, less overshoot, smoother
    - Higher value = faster response, more responsive

    Args:
        robot: XLerobotSingleArm instance
        speed_level: Speed level ('slow', 'medium', 'fast', or custom value 1-32)
        joint_name: Name of the joint (if None, sets for all arm motors)
    """
    # P_Coefficient values for different speeds
    speed_config = {
        "slow": 8,
        "medium": 16,
        "fast": 25,
    }

    if speed_level.lower() in speed_config:
        p_coeff = speed_config[speed_level.lower()]
    else:
        try:
            p_coeff = int(speed_level)
            if p_coeff < 1 or p_coeff > 32:
                print(f"✗ Speed must be between 1 and 32. Got {p_coeff}")
                return False
        except ValueError:
            print(f"✗ Invalid speed: {speed_level}. Use 'slow', 'medium', 'fast', or 1-32")
            return False

    # Determine which motors to set speed for
    if joint_name:
        arm_prefix = "right_arm" if robot.config.use_right_arm else "left_arm"
        if not joint_name.startswith(f"{arm_prefix}_"):
            if joint_name in ["shoulder_pan", "shoulder_lift", "elbow_flex",
                            "wrist_flex", "wrist_roll", "gripper"]:
                joint_name = f"{arm_prefix}_{joint_name}"
            else:
                print(f"✗ Invalid joint name: {joint_name}")
                return False
        motors = [joint_name]
    else:
        motors = robot.arm_motors

    # Set P_Coefficient to control speed response
    print(f"Setting P_Coefficient to {p_coeff}...")
    for motor in motors:
        try:
            robot.bus.write("P_Coefficient", motor, p_coeff, normalize=False)
        except Exception as e:
            print(f"  Warning: Failed to set speed for {motor}: {e}")

    print(f"✓ Set P_Coefficient to {p_coeff} for: {', '.join(motors)}")
    return True


def interactive_mode(robot, use_right_arm=False, use_raw=False):
    """Interactive mode for setting joint angles."""
    mode_str = "RAW" if use_raw else "ANGLE"
    print(f"\nInteractive Mode ({mode_str})")
    print("Enter 'quit' or 'exit' to stop")
    print("Enter 'list' to see available joints")
    print("Enter 'speed slow/medium/fast' to adjust motor speed response")
    if use_raw:
        print("Enter 'raw' to switch to raw mode, 'angle' to switch to angle mode")
    print()

    while True:
        try:
            prompt = f"Enter joint name and raw value (0-4095): " if use_raw else f"Enter joint name and value: "
            user_input = input(prompt).strip()

            if not user_input:
                continue

            if user_input.lower() in ['quit', 'exit', 'q']:
                print("Exiting interactive mode...")
                break

            if user_input.lower() == 'list':
                print_available_joints(robot.config, use_right_arm)
                continue

            # Set speed
            if user_input.lower().startswith('speed'):
                parts = user_input.split()
                if len(parts) == 2:
                    set_motor_speed(robot, parts[1])
                else:
                    print("  Usage: speed <slow|medium|fast|1-32>")
                continue

            # Toggle mode
            if user_input.lower() == 'raw':
                use_raw = True
                print("Switched to RAW mode (0-4095)")
                continue
            if user_input.lower() == 'angle':
                use_raw = False
                print("Switched to ANGLE mode")
                continue

            # Parse input
            parts = user_input.split()
            if len(parts) != 2:
                print("✗ Invalid format. Use: <joint_name> <value>")
                if use_raw:
                    print("  Example: left_arm_shoulder_pan 2047 (raw value)")
                else:
                    print("  Example: left_arm_shoulder_pan 10.0")
                continue

            joint_name = parts[0]
            try:
                value = parts[1]
                if not use_raw:
                    value = float(value)
            except ValueError:
                print(f"✗ Invalid value: {parts[1]}.")
                if use_raw:
                    print("  Must be an integer (0-4095) for raw mode.")
                else:
                    print("  Must be a number for angle mode.")
                continue

            if use_raw:
                set_joint_raw(robot, joint_name, value, use_right_arm)
            else:
                set_joint_angle(robot, joint_name, value, use_right_arm)

        except KeyboardInterrupt:
            print("\nExiting interactive mode...")
            break
        except Exception as e:
            print(f"✗ Error: {e}")


def main():
    parser = argparse.ArgumentParser(description="Manual joint control for XLerobot")
    parser.add_argument("--port", type=str, default="/dev/ttyACM0",
                        help="USB port for the robot (default: /dev/ttyACM0)")
    parser.add_argument("--use-right-arm", action="store_true",
                        help="Use right arm instead of left arm")
    parser.add_argument("--raw", action="store_true",
                        help="Use raw motor values (0-4095) instead of normalized values")
    parser.add_argument("--speed", type=str, default=None,
                        help="Motor speed response: 'slow', 'medium', 'fast', or 1-32 (P_Coefficient)")
    parser.add_argument("--joint", type=str, default=None,
                        help="Joint name to control (e.g., left_arm_shoulder_pan)")
    parser.add_argument("--value", type=str, default=None,
                        help="Value to set (angle/velocity for normal mode, raw integer for raw mode)")
    parser.add_argument("--interactive", action="store_true",
                        help="Enter interactive mode for multiple commands")
    parser.add_argument("--list", action="store_true",
                        help="List all available joints")

    args = parser.parse_args()

    # Create robot configuration
    robot_config = XLerobotSingleArmConfig(
        port1=args.port,
        use_right_arm=args.use_right_arm,
    )

    # Initialize robot
    robot = XLerobotSingleArm(robot_config)

    print(f"Connecting to robot on {args.port}...")
    try:
        robot.connect()
        print(f"✓ Robot connected successfully")
        if args.raw:
            print("  Mode: RAW (direct motor values 0-4095)")
        else:
            print("  Mode: Normalized (degrees or percentage)")
    except Exception as e:
        print(f"✗ Failed to connect to robot: {e}")
        sys.exit(1)

    # List joints if requested
    if args.list:
        print_available_joints(robot_config, args.use_right_arm)
        robot.disconnect()
        return

    # Set speed if requested (applies to joint command or alone)
    if args.speed is not None and args.interactive is False and args.joint is None and args.value is None:
        # Speed only mode
        set_motor_speed(robot, args.speed, args.joint)
        robot.disconnect()
        return

    # Interactive mode
    if args.interactive:
        interactive_mode(robot, args.use_right_arm, args.raw)
    # Single command mode
    elif args.joint and args.value is not None:
        # Set speed before moving if specified
        if args.speed is not None:
            set_motor_speed(robot, args.speed, args.joint)
        print_available_joints(robot_config, args.use_right_arm)
        if args.raw:
            success = set_joint_raw(robot, args.joint, args.value, args.use_right_arm)
        else:
            try:
                value = float(args.value)
                success = set_joint_angle(robot, args.joint, value, args.use_right_arm)
            except ValueError:
                print(f"✗ Invalid value: {args.value}. Must be a number for normal mode.")
                success = False

        if success:
            # Wait for motor to complete movement before disconnecting
            print("\nWaiting for motor to complete movement...")
            time.sleep(2.0)  # Give motor time to reach target position
        else:
            print_available_joints(robot_config, args.use_right_arm)
    # No arguments provided
    else:
        print_available_joints(robot_config, args.use_right_arm)
        print("\nExamples:")
        print("  # Set shoulder pan to 10 degrees (normalized mode)")
        print(f"  python xlerobot_manual_joint_control.py --joint left_arm_shoulder_pan --value 10.0")
        print()
        print("  # Set shoulder pan to raw value 2047 (raw mode)")
        print(f"  python xlerobot_manual_joint_control.py --raw --joint left_arm_shoulder_pan --value 2047")
        print()
        print("  # Set motor speed to fast (affects all arm motors)")
        print(f"  python xlerobot_manual_joint_control.py --speed fast")
        print()
        print("  # Set motor speed to custom value (1-32)")
        print(f"  python xlerobot_manual_joint_control.py --speed 25 --joint left_arm_shoulder_pan")
        print()
        print("  # Interactive mode")
        print("  python xlerobot_manual_joint_control.py --interactive")
        print()
        print("  # Interactive mode with raw values")
        print("  python xlerobot_manual_joint_control.py --raw --interactive")
        print()
        print("  # List all joints")
        print("  python xlerobot_manual_joint_control.py --list")
        print()

    # Disconnect
    robot.disconnect()
    print("✓ Robot disconnected")


if __name__ == "__main__":
    main()
