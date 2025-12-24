#!/usr/bin/env python

# Copyright 2024 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import logging
import time
from functools import cached_property
from itertools import chain
from typing import Any

import numpy as np

from lerobot.cameras.utils import make_cameras_from_configs
from lerobot.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_xlerobot_single_arm import XLerobotSingleArmConfig

logger = logging.getLogger(__name__)


class XLerobotSingleArm(Robot):
    """
    XLerobot with a single follower arm and mobile base on the same bus.
    
    Hardware configuration:
    - port1 (/dev/ttyACM0): Follower arm (6 motors) + Mobile base (3 wheel motors) - Total 9 motors
    
    Motor ID assignment on the same bus:
    - Arm motors (ID 1-6): shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
    - Base motors (ID 7-9): base_left_wheel, base_back_wheel, base_right_wheel
    
    Designed to work with a separate leader arm (SO101Leader) for teleoperation.
    """

    config_class = XLerobotSingleArmConfig
    name = "xlerobot_single_arm"

    def __init__(self, config: XLerobotSingleArmConfig):
        super().__init__(config)
        self.config = config
        self.teleop_keys = config.teleop_keys
        
        # Define three speed levels and a current index for base control
        self.speed_levels = [
            {"xy": 0.1, "theta": 30},  # slow
            {"xy": 0.2, "theta": 60},  # medium
            {"xy": 0.3, "theta": 90},  # fast
        ]
        self.speed_index = 0  # Start at slow
        
        norm_mode_body = MotorNormMode.DEGREES if config.use_degrees else MotorNormMode.RANGE_M100_100
        
        # Determine arm prefix based on configuration
        arm_prefix = "right_arm" if config.use_right_arm else "left_arm"
        
        # Setup calibration for all motors on the same bus
        all_motor_names = [
            f"{arm_prefix}_shoulder_pan",
            f"{arm_prefix}_shoulder_lift",
            f"{arm_prefix}_elbow_flex",
            f"{arm_prefix}_wrist_flex",
            f"{arm_prefix}_wrist_roll",
            f"{arm_prefix}_gripper",
            "base_left_wheel",
            "base_back_wheel",
            "base_right_wheel",
        ]
        
        # Use existing calibration if available
        if self.calibration.get(f"{arm_prefix}_shoulder_pan") is not None:
            calibration = {}
            for motor_name in all_motor_names:
                calibration[motor_name] = self.calibration.get(motor_name)
        else:
            calibration = self.calibration
        
        # Define motor lists
        self.arm_motors = [
            f"{arm_prefix}_shoulder_pan",
            f"{arm_prefix}_shoulder_lift",
            f"{arm_prefix}_elbow_flex",
            f"{arm_prefix}_wrist_flex",
            f"{arm_prefix}_wrist_roll",
            f"{arm_prefix}_gripper",
        ]
        self.base_motors = ["base_left_wheel", "base_back_wheel", "base_right_wheel"]
        
        # Single bus with both arm and base motors
        # Arm motors: ID 1-6 (position mode)
        # Base motors: ID 7-9 (velocity mode)
        self.bus = FeetechMotorsBus(
            port=self.config.port1,
            motors={
                # Arm motors (ID 1-6)
                f"{arm_prefix}_shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                f"{arm_prefix}_shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                f"{arm_prefix}_elbow_flex": Motor(3, "sts3215", norm_mode_body),
                f"{arm_prefix}_wrist_flex": Motor(4, "sts3215", norm_mode_body),
                f"{arm_prefix}_wrist_roll": Motor(5, "sts3215", norm_mode_body),
                f"{arm_prefix}_gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
                # Base motors (ID 7-9)
                "base_left_wheel": Motor(7, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_back_wheel": Motor(8, "sts3215", MotorNormMode.RANGE_M100_100),
                "base_right_wheel": Motor(9, "sts3215", MotorNormMode.RANGE_M100_100),
            },
            calibration=calibration,
        )
        
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def _state_ft(self) -> dict[str, type]:
        arm_prefix = "right_arm" if self.config.use_right_arm else "left_arm"
        return dict.fromkeys(
            (
                f"{arm_prefix}_shoulder_pan.pos",
                f"{arm_prefix}_shoulder_lift.pos",
                f"{arm_prefix}_elbow_flex.pos",
                f"{arm_prefix}_wrist_flex.pos",
                f"{arm_prefix}_wrist_roll.pos",
                f"{arm_prefix}_gripper.pos",
                "x.vel",
                "y.vel",
                "theta.vel",
            ),
            float,
        )

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.config.cameras[cam].height, self.config.cameras[cam].width, 3) 
            for cam in self.cameras
        }

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._state_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._state_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(
            cam.is_connected for cam in self.cameras.values()
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        
        # Check if calibration file exists
        if self.calibration_fpath.is_file():
            logger.info(f"Calibration file found at {self.calibration_fpath}")
            user_input = input(
                f"Press ENTER to restore calibration from file, or type 'c' and press ENTER to run manual calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info("Attempting to restore calibration from file...")
                try:
                    # Load calibration data into bus memory
                    self.bus.calibration = {
                        k: v for k, v in self.calibration.items()
                        if k in self.bus.motors
                    }
                    logger.info("Calibration data loaded into bus memory successfully!")
                    
                    # Write calibration data to motors
                    self.bus.write_calibration({
                        k: v for k, v in self.calibration.items()
                        if k in self.bus.motors
                    })
                    logger.info("Calibration restored successfully from file!")
                    
                except Exception as e:
                    logger.warning(f"Failed to restore calibration from file: {e}")
                    if calibrate:
                        logger.info("Proceeding with manual calibration...")
                        self.calibrate()
            else:
                logger.info("User chose manual calibration...")
                if calibrate:
                    self.calibrate()
        elif calibrate:
            logger.info("No calibration file found, proceeding with manual calibration...")
            self.calibrate()

        for cam in self.cameras.values():
            cam.connect()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def calibrate(self) -> None:
        logger.info(f"\nRunning calibration of {self}")
        
        # Disable torque first
        self.bus.disable_torque()
        
        # Set operating modes for arm motors (position)
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
        
        # Calibrate arm motors
        input(f"Move arm motors to the middle of their range of motion and press ENTER....")
        homing_offsets_arm = self.bus.set_half_turn_homings(self.arm_motors)
        
        print(
            f"Move all arm joints sequentially through their "
            "entire ranges of motion.\nRecording positions. Press ENTER to stop..."
        )
        range_mins_arm, range_maxes_arm = self.bus.record_ranges_of_motion(self.arm_motors)
        
        # Set operating modes for base motors (velocity)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        
        # Base motors use full turn range (0-4095)
        homing_offsets_base = dict.fromkeys(self.base_motors, 0)
        range_mins_base = dict.fromkeys(self.base_motors, 0)
        range_maxes_base = dict.fromkeys(self.base_motors, 4095)
        
        # Combine all calibration data
        self.calibration = {}
        for name, motor in self.bus.motors.items():
            if name in self.arm_motors:
                self.calibration[name] = MotorCalibration(
                    id=motor.id,
                    drive_mode=0,
                    homing_offset=homing_offsets_arm[name],
                    range_min=range_mins_arm[name],
                    range_max=range_maxes_arm[name],
                )
            elif name in self.base_motors:
                self.calibration[name] = MotorCalibration(
                    id=motor.id,
                    drive_mode=0,
                    homing_offset=homing_offsets_base[name],
                    range_min=range_mins_base[name],
                    range_max=range_maxes_base[name],
                )
        
        # Write calibration to bus
        self.bus.write_calibration(self.calibration)
        self._save_calibration()
        print("Calibration saved to", self.calibration_fpath)

    def configure(self):
        # Disable torque first
        self.bus.disable_torque()
        self.bus.configure_motors()
        
        # Setup arm actuators (position mode)
        for name in self.arm_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.POSITION.value)
            # Set P_Coefficient to lower value to avoid shakiness (Default is 32)
            self.bus.write("P_Coefficient", name, 16)
            # Set I_Coefficient and D_Coefficient to default value 0 and 32
            self.bus.write("I_Coefficient", name, 0)
            self.bus.write("D_Coefficient", name, 43)
        
        # Setup base actuators (velocity mode)
        for name in self.base_motors:
            self.bus.write("Operating_Mode", name, OperatingMode.VELOCITY.value)
        
        self.bus.enable_torque()

    def setup_motors(self) -> None:
        # Setup all motors (arm first, then base)
        for motor in reversed(self.arm_motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")
        
        for motor in reversed(self.base_motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    @staticmethod
    def _degps_to_raw(degps: float) -> int:
        steps_per_deg = 4096.0 / 360.0
        speed_in_steps = degps * steps_per_deg
        speed_int = int(round(speed_in_steps))
        # Cap the value to fit within signed 16-bit range (-32768 to 32767)
        if speed_int > 0x7FFF:
            speed_int = 0x7FFF
        elif speed_int < -0x8000:
            speed_int = -0x8000
        return speed_int

    @staticmethod
    def _raw_to_degps(raw_speed: int) -> float:
        steps_per_deg = 4096.0 / 360.0
        magnitude = raw_speed
        degps = magnitude / steps_per_deg
        return degps

    def _body_to_wheel_raw(
        self,
        x: float,
        y: float,
        theta: float,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
        max_raw: int = 3000,
    ) -> dict:
        """
        Convert desired body-frame velocities into wheel raw commands.
        """
        theta_rad = theta * (np.pi / 180.0)
        velocity_vector = np.array([x, y, theta_rad])
        
        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
        
        wheel_linear_speeds = m.dot(velocity_vector)
        wheel_angular_speeds = wheel_linear_speeds / wheel_radius
        
        wheel_degps = wheel_angular_speeds * (180.0 / np.pi)
        
        # Scaling
        steps_per_deg = 4096.0 / 360.0
        raw_floats = [abs(degps) * steps_per_deg for degps in wheel_degps]
        max_raw_computed = max(raw_floats)
        if max_raw_computed > max_raw:
            scale = max_raw / max_raw_computed
            wheel_degps = wheel_degps * scale
        
        wheel_raw = [self._degps_to_raw(deg) for deg in wheel_degps]
        
        return {
            "base_left_wheel": wheel_raw[0],
            "base_back_wheel": wheel_raw[1],
            "base_right_wheel": wheel_raw[2],
        }

    def _wheel_raw_to_body(
        self,
        left_wheel_speed,
        back_wheel_speed,
        right_wheel_speed,
        wheel_radius: float = 0.05,
        base_radius: float = 0.125,
    ) -> dict[str, Any]:
        """Convert wheel raw command feedback back into body-frame velocities."""
        wheel_degps = np.array([
            self._raw_to_degps(left_wheel_speed),
            self._raw_to_degps(back_wheel_speed),
            self._raw_to_degps(right_wheel_speed),
        ])
        
        wheel_radps = wheel_degps * (np.pi / 180.0)
        wheel_linear_speeds = wheel_radps * wheel_radius
        
        angles = np.radians(np.array([240, 0, 120]) - 90)
        m = np.array([[np.cos(a), np.sin(a), base_radius] for a in angles])
        
        m_inv = np.linalg.inv(m)
        velocity_vector = m_inv.dot(wheel_linear_speeds)
        x, y, theta_rad = velocity_vector
        theta = theta_rad * (180.0 / np.pi)
        return {
            "x.vel": x,
            "y.vel": y,
            "theta.vel": theta,
        }
    
    def _from_keyboard_to_base_action(self, pressed_keys: np.ndarray):
        # Speed control
        if self.teleop_keys["speed_up"] in pressed_keys:
            self.speed_index = min(self.speed_index + 1, 2)
        if self.teleop_keys["speed_down"] in pressed_keys:
            self.speed_index = max(self.speed_index - 1, 0)
        speed_setting = self.speed_levels[self.speed_index]
        xy_speed = speed_setting["xy"]
        theta_speed = speed_setting["theta"]

        x_cmd = 0.0
        y_cmd = 0.0
        theta_cmd = 0.0

        if self.teleop_keys["forward"] in pressed_keys:
            x_cmd += xy_speed
        if self.teleop_keys["backward"] in pressed_keys:
            x_cmd -= xy_speed
        if self.teleop_keys["left"] in pressed_keys:
            y_cmd += xy_speed
        if self.teleop_keys["right"] in pressed_keys:
            y_cmd -= xy_speed
        if self.teleop_keys["rotate_left"] in pressed_keys:
            theta_cmd += theta_speed
        if self.teleop_keys["rotate_right"] in pressed_keys:
            theta_cmd -= theta_speed
            
        return {
            "x.vel": x_cmd, 
            "y.vel": y_cmd,
            "theta.vel": theta_cmd,
        }

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        
        # Read arm positions
        arm_pos = self.bus.sync_read("Present_Position", self.arm_motors)
        
        # Read base wheel velocities
        base_wheel_vel = self.bus.sync_read("Present_Velocity", self.base_motors)
        
        # Convert wheel velocities to body velocities
        base_vel = self._wheel_raw_to_body(
            base_wheel_vel["base_left_wheel"],
            base_wheel_vel["base_back_wheel"],
            base_wheel_vel["base_right_wheel"],
        )
        
        arm_state = {f"{k}.pos": v for k, v in arm_pos.items()}
        obs_dict = {**arm_state, **base_vel}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read state: {dt_ms:.1f}ms")

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.async_read()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        return obs_dict

    def send_action(self, action: dict[str, Any]) -> dict[str, Any]:
        """Command robot to move to a target joint configuration."""
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")
        
        arm_pos = {k: v for k, v in action.items() if k.endswith(".pos")}
        base_goal_vel = {k: v for k, v in action.items() if k.endswith(".vel")}
        base_wheel_goal_vel = self._body_to_wheel_raw(
            base_goal_vel.get("x.vel", 0.0),
            base_goal_vel.get("y.vel", 0.0),
            base_goal_vel.get("theta.vel", 0.0),
        )
        
        # Apply max_relative_target if configured
        if self.config.max_relative_target is not None and arm_pos:
            present_pos_arm = self.bus.sync_read("Present_Position", self.arm_motors)
            goal_present_pos = {
                key: (g_pos, present_pos_arm[key.replace(".pos", "")]) 
                for key, g_pos in arm_pos.items()
            }
            safe_goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)
            arm_pos = {k: v for k, v in safe_goal_pos.items()}
        
        # Send arm positions
        arm_pos_raw = {k.replace(".pos", ""): v for k, v in arm_pos.items()}
        if arm_pos_raw:
            self.bus.sync_write("Goal_Position", arm_pos_raw)
        
        # Send base velocities
        if base_wheel_goal_vel:
            self.bus.sync_write("Goal_Velocity", base_wheel_goal_vel)
        
        return {
            **arm_pos,
            **base_goal_vel,
        }

    def stop_base(self):
        self.bus.sync_write("Goal_Velocity", dict.fromkeys(self.base_motors, 0), num_retry=5)
        logger.info("Base motors stopped")

    def disconnect(self):
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.stop_base()
        self.bus.disconnect(self.config.disable_torque_on_disconnect)
        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
