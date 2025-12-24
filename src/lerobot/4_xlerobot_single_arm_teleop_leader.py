#!/usr/bin/env python
"""
XLerobot Single Arm Teleoperation with Leader Arm

Hardware Setup:
- /dev/ttyACM0: Follower arm (6 motors) + Mobile base (3 wheel motors) - Total 9 motors on same bus
  * Arm motors (ID 1-6): shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
  * Base motors (ID 7-9): base_left_wheel, base_back_wheel, base_right_wheel
- /dev/ttyACM1: Leader arm (SO101 - 6 motors)

Usage:
    # Terminal 1: Run this teleoperation script
    PYTHONPATH=src python examples/4_xlerobot_single_arm_teleop_leader.py
"""
import os
# Fix OpenMP library conflict on Windows
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import time
from lerobot.robots.xlerobot import XLerobotSingleArmConfig, XLerobotSingleArm
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

def main():
    # Configuration
    FPS = 30
    
    # Leader arm configuration (SO101 - teleop master arm)
    # Connects to /dev/ttyACM1
    leader_config = SO101LeaderConfig(
        port="/dev/ttyACM0",
        id="my_leader_arm",
    )
    
    # Follower robot configuration (single arm + base on same bus)
    # Connects to /dev/ttyACM0 (arm + base, total 9 motors)
    robot_config = XLerobotSingleArmConfig(
        port1="/dev/ttyACM1",  # Follower arm + base on same bus
        use_right_arm=False,  # Set to True if using right arm instead of left
    )
    
    # Keyboard configuration for base control
    keyboard_config = KeyboardTeleopConfig()
    
    # Initialize devices
    robot = XLerobotSingleArm(robot_config)
    leader_arm = SO101Leader(leader_config)
    keyboard = KeyboardTeleop(keyboard_config)
    
    print("Connecting to devices...")
    print(f"  - Leader arm: {leader_config.port}")
    print(f"  - Follower robot (arm+base): {robot_config.port1}")
    
    try:
        robot.connect()
        print(f"[MAIN] ✓ Follower robot connected")
    except Exception as e:
        print(f"[MAIN] ✗ Failed to connect to robot: {e}")
        return
    
    try:
        leader_arm.connect()
        print(f"[MAIN] ✓ Leader arm connected")
    except Exception as e:
        print(f"[MAIN] ✗ Failed to connect to leader arm: {e}")
        robot.disconnect()
        return
    
    try:
        keyboard.connect()
        print(f"[MAIN] ✓ Keyboard connected")
    except Exception as e:
        print(f"[MAIN] ✗ Failed to connect to keyboard: {e}")
        robot.disconnect()
        leader_arm.disconnect()
        return
    
    init_rerun(session_name="xlerobot_single_arm_teleop")
    
    arm_prefix = "right_arm" if robot_config.use_right_arm else "left_arm"
    print(f"\n[MAIN] Starting teleoperation...")
    print(f"[MAIN] Follower arm: {arm_prefix}")
    print(f"[MAIN] ===========================")
    print(f"[MAIN] Control instructions:")
    print(f"[MAIN] - Arm: Move leader arm to control follower arm")
    print(f"[MAIN] - Base: i/k (forward/back), j/l (left/right)")
    print(f"[MAIN] - Base: u/o (rotate left/right)")
    print(f"[MAIN] - Speed: n (increase), m (decrease)")
    print(f"[MAIN] - Quit: Ctrl+C")
    print(f"[MAIN] ===========================\n")
    
    try:
        while True:
            t0 = time.perf_counter()
            
            # Get robot observation
            observation = robot.get_observation()
            
            # Get leader arm action
            leader_action = leader_arm.get_action()
            
            # Map leader action to follower arm format
            # Leader arm uses: shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
            # Follower arm uses: {arm_prefix}_shoulder_pan, {arm_prefix}_shoulder_lift, etc.
            arm_action = {}
            for key, value in leader_action.items():
                if key.endswith(".pos"):
                    motor_name = key.replace(".pos", "")
                    # Rename to follower arm format
                    follower_key = f"{arm_prefix}_{motor_name}.pos"
                    arm_action[follower_key] = value
            
            # Get keyboard action for base control
            keyboard_keys = keyboard.get_action()
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)
            
            # Combine actions
            action = {**arm_action, **base_action}
            
            # Send action to robot
            robot.send_action(action)
            
            # Visualize
            log_rerun_data(observation, action)
            
            # Maintain control frequency
            busy_wait(max(1.0 / FPS - (time.perf_counter() - t0), 0.0))
            
    except KeyboardInterrupt:
        print("\n[MAIN] Teleoperation stopped by user")
    finally:
        robot.disconnect()
        leader_arm.disconnect()
        keyboard.disconnect()
        print("[MAIN] All devices disconnected")

if __name__ == "__main__":
    main()
