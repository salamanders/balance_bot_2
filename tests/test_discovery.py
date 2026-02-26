import pytest
import glm
from typing import List, Tuple, Optional
from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware, MeasureResult, IMUReading
from balance_bot.discovery.pipeline import SelfDiscoveryPipeline
from balance_bot.discovery.protocol import StepStatus

class MockRobotHardware(RobotHardware):
    def __init__(self, config: HardwareConfig):
        super().__init__(config)
        self.stop_called = False
        self.applied_config = config

        # Simulation state
        self.friction_threshold = 15.0 # Min power to move
        self.lean_angle_back = 20.0
        self.lean_angle_front = -15.0
        self.current_bumper = 'back'
        self.drift_bias = 0.05 # Natural drift (turning left/right). Small enough to pass polarity check.

    def apply_config(self, config: HardwareConfig):
        self.applied_config = config
        self.config = config

    def stop(self):
        self.stop_called = True

    def drive_and_measure(self, left: float, right: float, duration: float, sample_interval: float = 0.01, wait_for_stability: bool = False, trim_override: Optional[float] = None) -> MeasureResult:
        # Simulate physics based on inputs

        # 1. Determine effective power
        trim = trim_override if trim_override is not None else self.config.trim_bias

        # Assumption for DriveTrimStep logic:
        # Step uses: new_trim = trim - drift * kp
        # This implies positive trim causes positive drift (+Z).
        # Positive Drift (+Z) = Turn Left.
        # Turn Left happens if Right > Left.
        # So Trim must add to Right (or subtract from Left).

        eff_left = left
        eff_right = right + trim # Trim adds to Right

        samples = []
        num_samples = int(duration / sample_interval)
        if num_samples < 1: num_samples = 1

        for _ in range(num_samples):
            gyro = glm.vec3(0.0)
            accel = glm.vec3(0.0)

            # Stationary (Gravity/Lean)
            if left == 0 and right == 0:
                # Accel reflects lean
                import math
                angle = self.lean_angle_back if self.current_bumper == 'back' else self.lean_angle_front
                rad = math.radians(angle)
                accel = glm.vec3(math.sin(rad) * 9.81, 0.0, math.cos(rad) * 9.81)

            # Moving
            else:
                # Check stiction per motor or max power
                max_power = max(abs(left), abs(right))
                if max_power < self.friction_threshold:
                    # Stuck
                    accel = glm.vec3(0.0, 0.0, 9.81) # Stationary
                else:
                    # Moving
                    speed = (max_power - self.friction_threshold) * 0.1

                    # Gyro Z (Yaw)
                    diff = (eff_right - eff_left)

                    # drift_bias adds to diff
                    effective_turn = diff + self.drift_bias

                    gyro.z = effective_turn * 5.0 # Scale factor

                    # Add vibration noise to satisfy FrictionThresholdStep (mag > 15)
                    # It checks gyro magnitude.
                    # We need mag > 15.0 when moving.
                    # If turning is small, we need noise.
                    import random
                    noise_x = 16.0 if max_power > self.friction_threshold else 0.0
                    gyro.x = noise_x

                    # Accel X (Forward)
                    accel.x = speed
                    accel.z = 9.81 # Gravity

            samples.append(IMUReading(0.0, gyro, accel))

        return MeasureResult(duration, samples)

def test_pipeline_success():
    config = HardwareConfig()
    state = LearningState()
    hw = MockRobotHardware(config)

    pipeline = SelfDiscoveryPipeline(hw, config, state)

    # Run pipeline
    pipeline.run()

    # Verify State
    assert state.initial_assumptions_verified
    assert state.hardware_verified
    assert state.gravity_vector_verified
    assert state.friction_threshold_verified
    assert state.min_power_visible >= 15.0 # Since mock threshold is 15.0
    assert state.polarity_verified
    assert state.turn_direction_verified
    assert state.lean_verified
    assert state.lean_angle_back > 0 # Should measure ~20
    assert state.trim_verified

    # Verify Trim
    # drift_bias = 0.05.
    # We want effective_turn = diff + bias = 0.
    # diff = right - left = (p+trim) - p = trim.
    # trim + bias = 0 -> trim = -bias = -0.05.
    # Pipeline should find trim approx -0.05.
    assert abs(config.trim_bias + 0.05) < 0.1

    assert state.balance_point_verified
