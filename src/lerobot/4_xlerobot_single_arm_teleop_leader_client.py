#!/usr/bin/env python
"""
XLerobot Single Arm Teleoperation with Leader Arm (Remote Client Mode)

Hardware Setup:
- Local: Leader arm (SO101 - 6 motors) connected via serial port
- Remote: Follower arm + Mobile base on remote host (connected via network)

Usage:
    # Terminal 1 (Remote Host): Run the host
    python -m lerobot.robots.xlerobot.xlerobot_single_arm_host --robot.id=my_xlerobot_single_arm

    # Terminal 2 (Local): Run this teleoperation script
    python -m lerobot.4_xlerobot_single_arm_teleop_leader_client
"""
import os
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"
import time
from lerobot.robots.xlerobot import XLerobotSingleArmClient, XLerobotSingleArmClientConfig
from lerobot.teleoperators.so101_leader import SO101Leader, SO101LeaderConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import KeyboardTeleop, KeyboardTeleopConfig
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.visualization_utils import init_rerun, log_rerun_data

def main():
    # Configuration
    FPS = 30
    
    # Remote host IP (change this to your actual remote host IP)
    ip = "192.168.110.29"
    robot_name = "None"
    
    # Leader arm configuration (SO101 - teleop master arm)
    # Windows: Use COMx format, e.g., "COM4"
    # Linux: Use /dev/ttyACMx format, e.g., "/dev/ttyACM0"
    leader_config = SO101LeaderConfig(
        port="COM3",  # Change to your leader arm port
        id="my_leader_arm",
    )
    
    # Remote follower robot configuration (via network)
    robot_config = XLerobotSingleArmClientConfig(
        remote_ip=ip,
        id=robot_name,
    )
    
    # Keyboard configuration for base control
    keyboard_config = KeyboardTeleopConfig()
    
    # Initialize devices
    robot = XLerobotSingleArmClient(robot_config)
    leader_arm = SO101Leader(leader_config)
    keyboard = KeyboardTeleop(keyboard_config)
    
    print("Connecting to devices...")
    print(f"  - Leader arm (local): {leader_config.port}")
    print(f"  - Follower robot (remote): {ip}")
    
    try:
        robot.connect()
        print(f"[MAIN] ✓ Follower robot connected (remote)")
    except Exception as e:
        print(f"[MAIN] ✗ Failed to connect to robot: {e}")
        print(f"[MAIN] Make sure the host is running on {ip}")
        return
    
    try:
        leader_arm.connect()
        print(f"[MAIN] ✓ Leader arm connected (local)")
    except Exception as e:
        print(f"[MAIN] ✗ Failed to connect to leader arm: {e}")
        print(f"[MAIN] Check if the leader arm is connected to {leader_config.port}")
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
    
    init_rerun(session_name="xlerobot_single_arm_leader_client_teleop")
    
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
