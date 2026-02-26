import time
import math
import logging
from typing import Tuple, Dict, Any, Optional

import glm
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from .protocol import CalibrationStep, StepStatus
from ..utils import find_threshold, shortest_angular_distance

logger = logging.getLogger(__name__)

class DiscoverBusesStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 0: Discover I2C Buses"

    def is_verified(self, state: LearningState) -> bool:
        return state.buses_discovered

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        import os
        if os.environ.get("ALLOW_MOCK_FALLBACK"):
            logger.info("[Deduction] Mocks allowed. Skipping I2C bus scan.")
            return StepStatus.SUCCESS, {'motor_i2c_bus': 1, 'imu_i2c_bus': 1}, {'buses_discovered': True}

        logger.info("[Action] Scanning I2C buses (1 and 3) for IMU (0x68) and Motor Driver (0x22).")
        try:
            import smbus2 as smbus
        except ImportError:
            logger.error("[Deduction] FAILED: smbus2 not installed.")
            return StepStatus.FATAL, {}, {}

        found_imu = None
        found_motor = None

        for bus_id in [1, 3]:
            try:
                bus = smbus.SMBus(bus_id)
                
                # Check for IMU (0x68)
                if found_imu is None:
                    try:
                        bus.read_byte(0x68)
                        found_imu = bus_id
                        logger.info(f"[Observation] Found IMU at 0x68 on bus {bus_id}")
                    except OSError:
                        pass
                
                # Check for Motor Driver (0x22)
                if found_motor is None:
                    try:
                        bus.read_byte(0x22)
                        found_motor = bus_id
                        logger.info(f"[Observation] Found Motor Driver at 0x22 on bus {bus_id}")
                    except OSError:
                        pass
                        
                bus.close()
            except Exception:
                pass
        
        if found_imu is None or found_motor is None:
            logger.error("[Deduction] FAILED: Could not find both devices. Check wiring.")
            return StepStatus.FATAL, {}, {}

        logger.info(f"[Deduction] Configured IMU on bus {found_imu} and Motor Driver on bus {found_motor}.")
        # We assign Left=0 and Right=1 as an initial working hypothesis. 
        # TurnDirectionStep will verify this and swap them if the hypothesis is wrong.
        return StepStatus.SUCCESS, {
            'motor_i2c_bus': found_motor, 
            'imu_i2c_bus': found_imu,
            'motor_left_channel': 0,
            'motor_right_channel': 1
        }, {'buses_discovered': True}


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
        
        logger.info("[Action] Driving right wheel to observe rotation.")
        res_r = hw.drive_and_measure(0, power, 0.5, wait_for_stability=True)

        z_l = sum(s.gyro_raw.z for s in res_l.samples) / max(len(res_l.samples), 1)
        z_r = sum(s.gyro_raw.z for s in res_r.samples) / max(len(res_r.samples), 1)

        logger.info(f"[Observation] Left wheel alone -> Gyro Z: {z_l:.2f}")
        logger.info(f"[Observation] Right wheel alone -> Gyro Z: {z_r:.2f}")

        if (z_l * z_r) > 0 and abs(z_l) > 0.1 and abs(z_r) > 0.1:
            logger.error("[Deduction] FAILED: Both wheels turn the robot the same way. Wiring is seriously wrong.")
            return StepStatus.FATAL, {}, {}

        # Assuming positive Z is turning left (Right Hand Rule on Z Up).
        # A wheel pushing forward that turns robot left (Positive Z) is the RIGHT wheel.
        # A wheel pushing forward that turns robot right (Negative Z) is the LEFT wheel.
        # Currently, config.motor_left_channel was driven for res_l.
        # If z_l > 0, the "left" wheel turned the robot left, which means it is actually the RIGHT wheel!
        
        updates = {'turn_direction_verified': True}
        config_updates = {}
        
        if z_l > 0:
            logger.info("[Deduction] The currently assigned Left wheel turned the robot left. It is actually the Right wheel. Swapping channels.")
            config_updates['motor_left_channel'] = config.motor_right_channel
            config_updates['motor_right_channel'] = config.motor_left_channel
            # We must also swap their inversion states!
            config_updates['motor_left_invert'] = config.motor_right_invert
            config_updates['motor_right_invert'] = config.motor_left_invert
        else:
            logger.info("[Deduction] Turn directions align with Left/Right assumptions. No swap needed.")

        return StepStatus.SUCCESS, config_updates, updates


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

        kp = 0.02  # Lower gain to prevent wild oscillation
        
        for attempt in range(10):  # More attempts to converge
            hw.wait_for_stability()
            res = hw.drive_and_measure(power, power, 0.5, trim_override=current_trim)
            
            avg_z = sum(s.gyro_raw.z for s in res.samples) / max(len(res.samples), 1)
            logger.info(f"[Observation] Drift at trim {current_trim:.3f} was {avg_z:.2f} yaw/s")
            
            if abs(avg_z) < 2.0:  # Relaxed acceptable threshold to 2.0 deg/s
                logger.info("[Deduction] Trim is perfectly acceptable.")
                return StepStatus.SUCCESS, {'trim_bias': current_trim}, {'trim_verified': True}

            # Proportional adjustment. 
            # If Yaw Z is negative (e.g. -11.4), we are turning Right.
            # This means the Left wheel is driving faster than the Right wheel.
            # To fix this, we need to add negative trim to slow down the Left wheel.
            # new_trim = current_trim + (avg_z * kp)
            
            new_trim = current_trim + (avg_z * kp)
            new_trim = max(-0.4, min(0.4, new_trim))
            
            if abs(new_trim - current_trim) < 0.005:
                logger.info(f"[Deduction] Trim converged as best it can at {current_trim:.3f}. Final drift: {avg_z:.2f} yaw/s")
                break
                
            current_trim = new_trim
            logger.info(f"[Deduction] Adjusting trim to {new_trim:.3f} for next attempt.")
            time.sleep(0.5)
            
        # If we exit the loop, we either converged to a non-perfect value or exhausted attempts.
        # We must be pessimistic. If the final drift is still awful (e.g. > 10 deg/s), we must abort.
        if abs(avg_z) > 10.0:
            logger.error(f"[Deduction] FAILED: Could not achieve straight-line driving. Drift stuck at {avg_z:.2f} yaw/s.")
            return StepStatus.FATAL, {}, {}

        logger.warning(f"[Deduction] Exhausted trim attempts, but drift is manageable. Settling on {current_trim:.3f}")
        return StepStatus.SUCCESS, {'trim_bias': current_trim}, {'trim_verified': True}


class BalancePointStep(CalibrationStep):
    @property
    def name(self) -> str: return "Phase 9: Balance Point"

    def is_verified(self, state: LearningState) -> bool:
        return state.balance_point_verified

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]:
        # Need both leans
        if state.lean_angle_front == 0.0:
            logger.info("[Action] We don't know the front lean angle. Attempting to flop the robot over.")
            
            base_power = state.min_power_visible + 20.0 
            
            for attempt in range(5):
                power = base_power + (attempt * 15.0) 
                if power > 100.0: power = 100.0
                
                logger.info(f"[Action] Flop attempt {attempt+1} at {power:.2f}% power...")
                # If pushing from a standstill fails to tip the robot over the center of mass, 
                # accelerate into the current bumper first (forward power to pitch back), 
                # then violently reverse (backward power to pitch forward).
                hw.execute_maneuver([
                    (power * 0.8, power * 0.8, 0.4),
                    (-power, -power, 0.7)
                ])
                time.sleep(1.5)
                
                # Measure new pitch
                res = hw.measure_only(1.0)
                avg_accel = glm.vec3(0.0)
                for s in res.samples: avg_accel += s.accel_raw
                if res.samples: avg_accel /= len(res.samples)
                new_pitch = math.degrees(math.atan2(avg_accel.y, avg_accel.z))
                
                # Did we actually flop? We need to have moved significantly from the back bumper
                pitch_diff = abs(shortest_angular_distance(state.lean_angle_back, new_pitch))
                logger.info(f"[Observation] New pitch is {new_pitch:.2f} degrees (Difference: {pitch_diff:.2f})")
                
                if pitch_diff > 15.0: # Significant change indicates it landed on the other bumper
                    logger.info(f"[Deduction] Successfully flopped to front bumper. Measured pitch is {new_pitch:.2f} degrees.")
                    return StepStatus.SUCCESS, {}, {'lean_angle_front': new_pitch, 'current_bumper': 'front'}
            
            logger.error("[Deduction] FAILED: Could not flop the robot over even at max power.")
            return StepStatus.FATAL, {}, {}
            
        # We need to average the two leans safely around the circle
        # e.g., averaging 170 and -170 should be 180, not 0.
        diff = shortest_angular_distance(state.lean_angle_back, state.lean_angle_front)
        balance_angle = (state.lean_angle_back + (diff / 2.0)) % 360.0
        if balance_angle > 180.0: balance_angle -= 360.0
        
        logger.info(f"[Deduction] Balance point calculated exactly midway between bumpers: {balance_angle:.2f} degrees.")

        return StepStatus.SUCCESS, {}, {
            'balance_point_verified': True,
            'balance_angle': balance_angle
        }
