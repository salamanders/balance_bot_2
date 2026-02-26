import time
import logging
import os
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any

import glm
from ..configuration import HardwareConfig

logger = logging.getLogger(__name__)

MOTOR_MIN_OUTPUT = -100
MOTOR_MAX_OUTPUT = 100

@dataclass(frozen=True)
class IMUReading:
    timestamp: float
    gyro_raw: glm.vec3
    accel_raw: glm.vec3
    # Helper to check if data is valid (non-zero)
    def is_valid(self) -> bool:
        return glm.length(self.accel_raw) > 0.0

@dataclass(frozen=True)
class MeasureResult:
    duration: float
    samples: List[IMUReading] = field(default_factory=list)

class RobotHardware:
    """
    Hardware Abstraction Layer for the Balance Bot.
    """
    def __init__(self, config: HardwareConfig):
        self.config = config
        self.watchdog = None 
        self.pz = None
        self.sensor = None
        self.initialize_drivers()

    def apply_config(self, config: HardwareConfig):
        """Updates the hardware configuration."""
        self.config = config
        logger.info(f"Applied new configuration: {config}")

    def initialize_drivers(self):
        """Initializes the PiconZero and MPU6050 drivers."""
        if os.environ.get("ALLOW_MOCK_FALLBACK"):
            logger.warning("Hardware Init: Mock Mode Requested via Environment.")
            self._init_mock_hardware()
            return

        try:
            import sys
            import smbus2
            sys.modules['smbus'] = smbus2
            from .piconzero import PiconZero
            from mpu6050 import mpu6050
            
            if self.config.motor_i2c_bus is not None:
                self.pz = PiconZero(bus_number=self.config.motor_i2c_bus)
                self.pz.init()
            else:
                logger.info("Skipping PiconZero init (Bus Unknown)")

            if self.config.imu_i2c_bus is not None:
                self.sensor = mpu6050(0x68, bus=self.config.imu_i2c_bus)
            else:
                logger.info("Skipping MPU6050 init (Bus Unknown)")

            logger.info("Hardware initialized.")
        except Exception as e:
            logger.error(f"CRITICAL: Failed to initialize hardware: {e}")
            # If initialization fails, fallback to mocks if permitted
            if os.environ.get("ALLOW_MOCK_FALLBACK"):
                logger.warning("Falling back to mocks after failed initialization.")
                self._init_mock_hardware()
            else:
                raise e

    def _init_mock_hardware(self):
        from .mocks import MockPiconZero, MockMPU6050
        self.pz = MockPiconZero()
        self.sensor = MockMPU6050(0x68)
        logger.info("Mock hardware initialized.")

    def read_imu_raw(self) -> Tuple[glm.vec3, glm.vec3]:
        """Reads raw accelerometer and gyro data from MPU6050."""
        if self.sensor is None:
            raise RuntimeError("IMU Sensor not initialized")

        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data()

            accel_vec = glm.vec3(accel['x'], accel['y'], accel['z'])
            gyro_vec = glm.vec3(gyro['x'], gyro['y'], gyro['z'])

            # Apply gyro offsets from config
            gyro_vec.x -= self.config.gyro_offset_x
            gyro_vec.y -= self.config.gyro_offset_y
            gyro_vec.z -= self.config.gyro_offset_z

            return accel_vec, gyro_vec
        except OSError as e:
            logger.error(f"IMU reading failed: {e}")
            raise RuntimeError(f"IMU Hardware failure: {e}")

    def set_motors(self, left: float, right: float, trim_override: Optional[float] = None) -> None:
        """Sets motor speeds considering configuration (channels, inversions, trim)."""
        if self.pz is None:
            raise RuntimeError("Motor driver not initialized")
        
        if self.config.motor_left_channel is None or self.config.motor_right_channel is None:
            raise RuntimeError("Motor channels not configured")

        trim = trim_override if trim_override is not None else self.config.trim_bias

        if trim > 0:
            right *= (1.0 - trim)
        elif trim < 0:
            left *= (1.0 - abs(trim))

        if self.config.motor_left_invert:
            left = -left
        if self.config.motor_right_invert:
            right = -right

        left_val = int(max(min(left, MOTOR_MAX_OUTPUT), MOTOR_MIN_OUTPUT))
        right_val = int(max(min(right, MOTOR_MAX_OUTPUT), MOTOR_MIN_OUTPUT))
        
        # Log motor outputs explicitly
        logger.debug(f"Motor Command - Left (Ch {self.config.motor_left_channel}): {left_val}, Right (Ch {self.config.motor_right_channel}): {right_val}")

        val_0 = 0
        val_1 = 0

        if self.config.motor_left_channel == 0:
            val_0 = left_val
        elif self.config.motor_left_channel == 1:
            val_1 = left_val

        if self.config.motor_right_channel == 0:
            val_0 = right_val
        elif self.config.motor_right_channel == 1:
            val_1 = right_val

        self.pz.set_motors(val_0, val_1)

    def stop(self):
        """Stops all motors immediately."""
        if self.pz is not None:
            self.pz.stop()

    def wait_for_stability(self):
        """
        Wait until the robot is physically stable (not moving).
        """
        logger.info("Waiting for stability...")
        time.sleep(0.5)

    def execute_maneuver(self, maneuver: List[Tuple[float, float, float]], sample_interval: float = 0.01, trim_override: Optional[float] = None) -> MeasureResult:
        """
        Executes a sequence of motor commands and records IMU data.
        
        Args:
            maneuver: List of (left_power, right_power, duration) tuples.
            sample_interval: Time between IMU samples.
            trim_override: Optional trim value to apply.
        """
        samples = []
        total_duration = 0.0

        try:
            for left, right, duration in maneuver:
                start_time = time.time()
                end_time = start_time + duration

                self.set_motors(left, right, trim_override=trim_override)

                while time.time() < end_time:
                    try:
                        accel, gyro = self.read_imu_raw()
                        # Logging sensor inputs and outputs
                        logger.debug(f"Sensor Read - Accel: ({accel.x:.2f}, {accel.y:.2f}, {accel.z:.2f}), Gyro: ({gyro.x:.2f}, {gyro.y:.2f}, {gyro.z:.2f})")
                        
                        reading = IMUReading(
                            timestamp=time.time(),
                            gyro_raw=gyro,
                            accel_raw=accel
                        )
                        samples.append(reading)
                    except Exception as e:
                        logger.warning(f"Failed to read IMU during maneuver: {e}")
                    
                    time.sleep(sample_interval)

                total_duration += duration
        finally:
            self.stop()

        return MeasureResult(duration=total_duration, samples=samples)

    def drive_and_measure(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01, wait_for_stability: bool = False, trim_override: Optional[float] = None) -> MeasureResult:
        """
        Standard measurement function.
        """
        if wait_for_stability:
            self.wait_for_stability()

        return self.execute_maneuver([(left_power, right_power, duration)], sample_interval, trim_override=trim_override)
