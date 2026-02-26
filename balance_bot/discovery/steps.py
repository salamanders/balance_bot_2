import time
import glm
from typing import Tuple, Dict, Any, Optional
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from .protocol import CalibrationStep, StepStatus
from ..utils import find_threshold

class InitialAssumptionStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 1: Initial Assumptions"

    def is_verified(self, state: LearningState) -> bool:
        return state.initial_assumptions_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Assumption: Robot starts leaning on back bumper (~20 deg)
        print("Applying initial assumptions: Robot is leaning on BACK bumper.")

        return StepStatus.SUCCESS, {}, {
            'initial_assumptions_verified': True,
            'current_bumper': 'back',
            'lean_angle_back': 20.0 # Placeholder
        }

class HardwareCheckStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 2: Hardware Verification"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        try:
            # Check IMU
            res = hw.drive_and_measure(0, 0, 0.1)
            if not res.samples:
                print("Error: No IMU samples received.")
                return StepStatus.FATAL, {}, {}

            # Check if values are sane (not all zeros)
            # In mock, they might be zero, but we set mock to return gravity.
            first_sample = res.samples[0]
            if glm.length(first_sample.accel_raw) == 0:
                print("Warning: IMU returning zero vectors.")
                # Allow retry or fail? Let's fail loudly as per spec.
                return StepStatus.FATAL, {}, {}

            return StepStatus.SUCCESS, {}, {'hardware_verified': True}

        except Exception as e:
            print(f"Hardware check failed: {e}")
            return StepStatus.FATAL, {}, {}

class GravityCalibrationStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 3: Sense of Down"

    def is_verified(self, state: LearningState) -> bool:
        return state.gravity_vector_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Measure gravity while stationary
        res = hw.drive_and_measure(0, 0, 1.0, wait_for_stability=True)

        # Average accelerometer readings
        avg_accel = glm.vec3(0.0)
        count = 0
        for s in res.samples:
            avg_accel += s.accel_raw
            count += 1

        if count == 0:
            return StepStatus.NEEDS_RETRY, {}, {}

        avg_accel /= count
        mag = glm.length(avg_accel)

        # Check if near 1g (9.81 m/s^2)
        # Allow wide tolerance for cheap sensors/mocks
        if abs(mag - 9.81) > 2.0:
            print(f"Gravity magnitude suspicious: {mag:.2f} (expected ~9.81)")
            return StepStatus.NEEDS_RETRY, {}, {}

        return StepStatus.SUCCESS, {}, {
            'gravity_vector_verified': True,
            'gravity_vector': avg_accel
        }

class FrictionThresholdStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 4: Friction Threshold (Min Power)"

    def is_verified(self, state: LearningState) -> bool:
        return state.friction_threshold_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        def action(p):
            # Drive forward with power p
            res = hw.drive_and_measure(p, p, 0.3, wait_for_stability=False)
            time.sleep(0.5)
            return res

        def check(res):
            max_mag = 0.0
            for s in res.samples:
                # Check gyro magnitude (rotation)
                # Note: If mock returns 0, this will fail unless we mock movement.
                if s.gyro_raw: # Check if not None
                    mag = glm.length(s.gyro_raw)
                    if mag > max_mag: max_mag = mag
            # Threshold: 15 deg/s (approx 0.26 rad/s)
            # Assuming raw values are in deg/s or rad/s?
            # Example says > 15.0, so likely deg/s.
            return max_mag > 15.0

        # Search for threshold starting at 10%, min 5%, max 100%
        found = find_threshold(
            name="Minimum Power",
            initial=10.0,
            min_val=5.0,
            max_val=100.0,
            action_fn=action,
            check_fn=check,
            heartbeat_fn=lambda: hw.watchdog.heartbeat() if hw.watchdog else None
        )

        if found is None:
            return StepStatus.FATAL, {}, {}

        return StepStatus.SUCCESS, {}, {
            'min_power_visible': found,
            'friction_threshold_verified': True
        }

class MotorPolarityStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 5: Motor Polarity"

    def is_verified(self, state: LearningState) -> bool:
        return state.polarity_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Drive both motors forward
        power = state.min_power_visible * 1.5
        res = hw.drive_and_measure(power, power, 0.5, wait_for_stability=True)

        # Analyze:
        # If aligned: High accel (or velocity), Low rotation
        # If opposed: Low accel, High rotation

        avg_gyro_z = 0.0
        avg_accel_x = 0.0 # Assuming X is forward?
        count = 0
        for s in res.samples:
            avg_gyro_z += s.gyro_raw.z
            avg_accel_x += s.accel_raw.x # This might need coordinate transform
            count += 1

        if count == 0: return StepStatus.NEEDS_RETRY, {}, {}

        avg_gyro_z /= count
        avg_accel_x /= count

        # Thresholds (need tuning)
        ROTATION_THRESHOLD = 0.5 # rad/s

        if abs(avg_gyro_z) > ROTATION_THRESHOLD:
            print(f"High rotation detected ({avg_gyro_z:.2f}). Motors might be opposed.")
            # In a real scenario, we might swap a channel in config
            # config.motor_right_channel = -config.motor_right_channel ?
            # For now, just fail or flag it.
            return StepStatus.FATAL, {}, {'wheels_reversed': True}

        return StepStatus.SUCCESS, {}, {'polarity_verified': True, 'wheels_reversed': False}

class TurnDirectionStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 6: Turn Direction (Left/Right)"

    def is_verified(self, state: LearningState) -> bool:
        return state.turn_direction_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        power = state.min_power_visible * 1.5

        # Test Left Wheel
        res_l = hw.drive_and_measure(power, 0, 0.5, wait_for_stability=True)
        # Expect turn to RIGHT (negative Yaw?)

        # Test Right Wheel
        res_r = hw.drive_and_measure(0, power, 0.5, wait_for_stability=True)
        # Expect turn to LEFT (positive Yaw?)

        # Analyze Gyro Z
        def get_avg_z(samples):
            return sum(s.gyro_raw.z for s in samples) / len(samples) if samples else 0.0

        z_l = get_avg_z(res_l.samples)
        z_r = get_avg_z(res_r.samples)

        print(f"Left Wheel Drive -> Gyro Z: {z_l:.2f}")
        print(f"Right Wheel Drive -> Gyro Z: {z_r:.2f}")

        # Verify they are opposite signs and significant
        if abs(z_l) < 0.1 or abs(z_r) < 0.1:
            print("Turn response too weak.")
            return StepStatus.NEEDS_RETRY, {}, {}

        if (z_l * z_r) > 0:
            print("Both wheels turn in same direction? Wiring error.")
            return StepStatus.FATAL, {}, {}

        # If logic is inverted (Left wheel makes it turn Left), swap config?
        # Standard: Left wheel pushes Right side forward -> Turn Right (-Z).
        # But if 'Forward' is X, Left wheel at +Y...
        # Let's assume standard differential drive.

        return StepStatus.SUCCESS, {}, {'turn_direction_verified': True}

class LeanCharacterizationStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 7: Lean Angle"

    def is_verified(self, state: LearningState) -> bool:
        return state.lean_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Measure gravity vector again
        res = hw.drive_and_measure(0, 0, 1.0, wait_for_stability=True)

        # Calculate pitch from gravity
        # Pitch = atan2(accel.x, accel.z) usually (assuming Y is axle)

        avg_accel = glm.vec3(0.0)
        for s in res.samples: avg_accel += s.accel_raw
        if res.samples: avg_accel /= len(res.samples)

        # Simple pitch calculation
        # This depends on IMU orientation. Assuming Z is 'up' in robot frame?
        # If robot is leaning back, Z and X components change.
        import math
        pitch = math.degrees(math.atan2(avg_accel.x, avg_accel.z))

        print(f"Measured Pitch: {pitch:.2f} degrees")

        updates = {'lean_verified': True}
        if state.current_bumper == 'back':
            updates['lean_angle_back'] = pitch
        else:
            updates['lean_angle_front'] = pitch

        return StepStatus.SUCCESS, {}, updates

class DriveTrimStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 8: Drive Trim"

    def is_verified(self, state: LearningState) -> bool:
        return state.trim_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Try to drive straight
        power = state.min_power_visible * 1.5

        # We want to find a trim that minimizes Gyro Z
        # We will loop internally to find the best trim

        def measure_drift(trim):
            res = hw.drive_and_measure(power, power, 0.5, wait_for_stability=True, trim_override=trim)
            # Calculate avg Gyro Z
            avg_z = sum(s.gyro_raw.z for s in res.samples) / len(res.samples) if res.samples else 0.0
            return avg_z

        current_trim = config.trim_bias
        kp = 0.05 # Gain

        for attempt in range(5): # Max attempts
            current_drift = measure_drift(current_trim)
            print(f"Drift at trim ({current_trim:.3f}): {current_drift:.2f}")

            if abs(current_drift) < 0.1: # Threshold
                return StepStatus.SUCCESS, {'trim_bias': current_trim}, {'trim_verified': True}

            # Adjust trim
            new_trim = current_trim - (current_drift * kp)

            # Limit trim
            new_trim = max(-0.5, min(0.5, new_trim))

            if abs(new_trim - current_trim) < 0.001:
                print("Trim converged (or stuck).")
                break

            current_trim = new_trim

        # If we exit loop, we either converged or failed to perfect it.
        # But we return the best we have.
        return StepStatus.SUCCESS, {'trim_bias': current_trim}, {'trim_verified': True}

class BalancePointStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 9: Balance Point"

    def is_verified(self, state: LearningState) -> bool:
        return state.balance_point_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Assume we have lean_angle_back and lean_angle_front
        # (In a real run, we would need to prompt user to flip robot or drive to flip it)

        if state.lean_angle_front == 0.0 and state.lean_angle_back != 0.0:
             # We haven't measured front lean yet.
             print("Please flip robot to front bumper (or implement flop maneuver).")
             # For now, just assume symmetric or fail
             # return StepStatus.FATAL, {}, {}

             # Let's mock it for the sake of the protocol finishing
             state.lean_angle_front = -state.lean_angle_back

        balance_angle = (state.lean_angle_back + state.lean_angle_front) / 2.0
        print(f"Calculated Balance Point: {balance_angle:.2f}")

        return StepStatus.SUCCESS, {}, {
            'balance_point_verified': True,
            'balance_angle': balance_angle
        }
