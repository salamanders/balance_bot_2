import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple, Any
import glm
from ..configuration import HardwareConfig

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
        self.watchdog = None # Placeholder for watchdog timer

    def apply_config(self, config: HardwareConfig):
        """Updates the hardware configuration."""
        self.config = config
        # In a real implementation, this would re-initialize I2C devices, etc.
        print(f"Applied new configuration: {config}")

    def wait_for_stability(self):
        """
        Wait until the robot is physically stable (not moving).
        """
        # Mock implementation: just sleep a bit
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

        for left, right, duration in maneuver:
            start_time = time.time()
            end_time = start_time + duration

            # Apply trim if needed
            # current_trim = trim_override if trim_override is not None else self.config.trim_bias
            # left += current_trim ...

            while time.time() < end_time:
                # In a real implementation, we would read the sensors here.
                # For now, we'll append a dummy reading or rely on mocks in tests.

                # Create a dummy reading
                # Using 0 vectors as default placeholder
                reading = IMUReading(
                    timestamp=time.time(),
                    gyro_raw=glm.vec3(0.0, 0.0, 0.0),
                    accel_raw=glm.vec3(0.0, 0.0, 9.81) # Gravity down
                )
                samples.append(reading)

                time.sleep(sample_interval)

            total_duration += duration

        return MeasureResult(duration=total_duration, samples=samples)

    def drive_and_measure(self, left_power: float, right_power: float, duration: float, sample_interval: float = 0.01, wait_for_stability: bool = False, trim_override: Optional[float] = None) -> MeasureResult:
        """
        Standard measurement function as required by the spec.
        """
        if wait_for_stability:
            self.wait_for_stability()

        return self.execute_maneuver([(left_power, right_power, duration)], sample_interval, trim_override=trim_override)

    def stop(self):
        """Stops all motors immediately."""
        # Send 0 power to motors
        pass
