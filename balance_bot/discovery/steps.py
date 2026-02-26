import time
import math
import logging
from typing import Tuple, Dict, Any, Optional

import glm
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from .protocol import CalibrationStep, StepStatus
from ..utils import find_threshold

logger = logging.getLogger(__name__)

class InitialAssumptionStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 1: Initial Assumptions"

    def is_verified(self, state: LearningState) -> bool:
        return state.initial_assumptions_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("[Deduction] Setting initial axiom: Robot is resting on rear bumper.")
        return StepStatus.SUCCESS, {}, {
            'initial_assumptions_verified': True,
            'current_bumper': 'back',
            'lean_angle_back': 20.0  # Safe initial assumption
        }


class HardwareCheckStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 2: Hardware Verification"

    def is_verified(self, state: LearningState) -> bool:
        return state.hardware_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("[Action] Pinging IMU for 0.1s to verify data stream.")
        try:
            res = hw.drive_and_measure(0, 0, 0.1)
            if not res.samples:
                logger.error("[Deduction] FAILED: No IMU samples received. Check I2C wiring.")
                return StepStatus.FATAL, {}, {}

            first_sample = res.samples[0]
            if glm.length(first_sample.accel_raw) == 0.0:
                logger.error("[Deduction] FAILED: IMU returning all zeros. Sensor is dead.")
                return StepStatus.FATAL, {}, {}

            logger.info("[Deduction] IMU is actively returning non-zero data.")
            return StepStatus.SUCCESS, {}, {'hardware_verified': True}
        except Exception as e:
            logger.error(f"[Deduction] FAILED: Hardware check exception: {e}")
            return StepStatus.FATAL, {}, {}


class GravityCalibrationStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 3: Sense of Down"

    def is_verified(self, state: LearningState) -> bool:
        return state.gravity_vector_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("[Action] Measuring gravity vector while stationary.")
        res = hw.drive_and_measure(0, 0, 1.0, wait_for_stability=True)

        avg_accel = glm.vec3(0.0)
        for s in res.samples:
            avg_accel += s.accel_raw
        
        if len(res.samples) == 0:
            return StepStatus.NEEDS_RETRY, {}, {}
            
        avg_accel /= len(res.samples)
        mag = glm.length(avg_accel)

        if abs(mag - 9.81) > 2.0:
            logger.warning(f"[Deduction] FAILED: Gravity magnitude suspicious ({mag:.2f}). Is robot moving?")
            return StepStatus.NEEDS_RETRY, {}, {}

        logger.info(f"[Deduction] Gravity vector established: {avg_accel.x:.2f}, {avg_accel.y:.2f}, {avg_accel.z:.2f}")
        return StepStatus.SUCCESS, {}, {
            'gravity_vector_verified': True,
            'gravity_vector': avg_accel
        }


class FrictionThresholdStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 4: Friction Threshold (Stiction)"

    def is_verified(self, state: LearningState) -> bool:
        return state.friction_threshold_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("[Action] Ramping PWM to find minimum power for movement.")

        def action(p):
            return hw.drive_and_measure(p, p, 0.2, wait_for_stability=False)

        def check(res):
            max_mag = 0.0
            for s in res.samples:
                mag = glm.length(s.gyro_raw)
                if mag > max_mag: 
                    max_mag = mag
            return max_mag > 15.0  # 15 deg/s or rad/s threshold

        found = find_threshold(
            name="Stiction",
            initial=10.0,
            min_val=5.0,
            max_val=100.0,
            action_fn=action,
            check_fn=check
        )

        hw.stop()

        if found is None:
            logger.error("[Deduction] FAILED: Could not overcome friction at 100% power.")
            return StepStatus.FATAL, {}, {}

        logger.info(f"[Deduction] Stiction overcome at {found:.2f}% power.")
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
        power = state.min_power_visible + 15.0
        logger.info(f"[Action] Pulsing motors at {power:.2f}% to check for rotation vs translation.")
        
        res = hw.drive_and_measure(power, power, 0.5, wait_for_stability=True)
        hw.stop()

        avg_gyro_z = sum(s.gyro_raw.z for s in res.samples) / max(len(res.samples), 1)

        # If applying equal positive power results in significant yaw, motors are opposed
        if abs(avg_gyro_z) > 1.0: # high threshold
            logger.info(f"[Deduction] Motors are fighting each other! Detected Yaw: {avg_gyro_z:.2f}. Inverting right motor.")
            return StepStatus.SUCCESS, {'motor_right_invert': not config.motor_right_invert}, {}
            
        logger.info("[Deduction] Motors are pushing together.")
        return StepStatus.SUCCESS, {}, {'polarity_verified': True, 'wheels_reversed': False}


class TurnDirectionStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 6: Turn Direction"

    def is_verified(self, state: LearningState) -> bool:
        return state.turn_direction_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        power = state.min_power_visible + 15.0
        
        logger.info("[Action] Driving left wheel to observe rotation.")
        res_l = hw.drive_and_measure(power, 0, 0.5, wait_for_stability=True)
        hw.stop()
        
        logger.info("[Action] Driving right wheel to observe rotation.")
        res_r = hw.drive_and_measure(0, power, 0.5, wait_for_stability=True)
        hw.stop()

        z_l = sum(s.gyro_raw.z for s in res_l.samples) / max(len(res_l.samples), 1)
        z_r = sum(s.gyro_raw.z for s in res_r.samples) / max(len(res_r.samples), 1)

        logger.info(f"[Observation] Left wheel alone -> Gyro Z: {z_l:.2f}")
        logger.info(f"[Observation] Right wheel alone -> Gyro Z: {z_r:.2f}")

        if (z_l * z_r) > 0 and abs(z_l) > 0.1 and abs(z_r) > 0.1:
            logger.error("[Deduction] FAILED: Both wheels turn the robot the same way. Wiring is seriously wrong.")
            return StepStatus.FATAL, {}, {}

        logger.info("[Deduction] Differential drive is confirmed functional.")
        return StepStatus.SUCCESS, {}, {'turn_direction_verified': True}


class LeanCharacterizationStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 7: Lean Angle"

    def is_verified(self, state: LearningState) -> bool:
        return state.lean_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        logger.info("[Action] Measuring static lean angle on current bumper.")
        res = hw.drive_and_measure(0, 0, 1.0, wait_for_stability=True)

        avg_accel = glm.vec3(0.0)
        for s in res.samples: 
            avg_accel += s.accel_raw
        if res.samples: 
            avg_accel /= len(res.samples)

        pitch = math.degrees(math.atan2(avg_accel.y, avg_accel.z))
        logger.info(f"[Deduction] Measured static pitch is {pitch:.2f} degrees.")

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
        power = state.min_power_visible + 15.0
        current_trim = config.trim_bias
        
        logger.info(f"[Action] Searching for zero-drift straight line trim starting at {current_trim:.3f}")

        res = hw.drive_and_measure(power, power, 0.5, wait_for_stability=True, trim_override=current_trim)
        hw.stop()
        
        avg_z = sum(s.gyro_raw.z for s in res.samples) / max(len(res.samples), 1)
        
        logger.info(f"[Observation] Drift at trim {current_trim:.3f} was {avg_z:.2f} yaw/s")
        
        if abs(avg_z) < 0.2:
            logger.info("[Deduction] Trim is perfectly acceptable.")
            return StepStatus.SUCCESS, {'trim_bias': current_trim}, {'trim_verified': True}

        # Small proportional adjustment for the next loop run
        kp = 0.05
        new_trim = current_trim - (avg_z * kp)
        new_trim = max(-0.5, min(0.5, new_trim))
        
        logger.info(f"[Deduction] Adjusting trim to {new_trim:.3f}. Will re-test.")
        return StepStatus.SUCCESS, {'trim_bias': new_trim}, {} # NOT setting verified so it loops


class BalancePointStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 9: Balance Point"

    def is_verified(self, state: LearningState) -> bool:
        return state.balance_point_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Need both leans
        if state.lean_angle_front == 0.0:
            logger.info("[Action] We don't know the front lean angle. Flop the robot over.")
            # Simple Flop Maneuver
            power = state.min_power_visible + 30.0 
            hw.drive_and_measure(power, power, 0.7, wait_for_stability=True)
            hw.stop()
            time.sleep(1.0)
            
            # Now we are on the front bumper
            res = hw.drive_and_measure(0, 0, 1.0, wait_for_stability=True)
            avg_accel = glm.vec3(0.0)
            for s in res.samples: avg_accel += s.accel_raw
            if res.samples: avg_accel /= len(res.samples)
            pitch = math.degrees(math.atan2(avg_accel.y, avg_accel.z))
            
            logger.info(f"[Deduction] Flopped to front bumper. Measured pitch is {pitch:.2f} degrees.")
            return StepStatus.SUCCESS, {}, {'lean_angle_front': pitch, 'current_bumper': 'front'}
            
        balance_angle = (state.lean_angle_back + state.lean_angle_front) / 2.0
        logger.info(f"[Deduction] Balance point calculated exactly midway between bumpers: {balance_angle:.2f} degrees.")

        return StepStatus.SUCCESS, {}, {
            'balance_point_verified': True,
            'balance_angle': balance_angle
        }
