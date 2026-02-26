from pydantic import BaseModel, Field
from typing import Optional, Dict, Any

class HardwareConfig(BaseModel):
    """
    Configuration for the robot hardware.
    """
    imu_i2c_bus: Optional[int] = None
    motor_i2c_bus: Optional[int] = None

    motor_left_channel: Optional[int] = None
    motor_right_channel: Optional[int] = None
    motor_left_invert: bool = False
    motor_right_invert: bool = False

    gyro_offset_x: float = 0.0
    gyro_offset_y: float = 0.0
    gyro_offset_z: float = 0.0

    trim_bias: float = 0.0  # Power adjustment to correct drift

    def save(self):
        # Mock save functionality
        pass

class LearningState(BaseModel):
    """
    State tracking for the discovery process.
    """
    # Phase 0: Bus Discovery
    buses_discovered: bool = False

    # Phase 1: Initial Assumptions
    initial_assumptions_verified: bool = False

    # Phase 2: Hardware Check
    hardware_verified: bool = False

    # Phase 3: Sense of Down
    gravity_vector_verified: bool = False
    gravity_vector: Optional[Any] = None # Will store vector representation

    # Phase 4: Stiction
    friction_threshold_verified: bool = False
    min_power_visible: float = 0.0

    # Phase 5: Polarity
    polarity_verified: bool = False
    wheels_reversed: bool = False

    # Phase 6: Turn Direction
    turn_direction_verified: bool = False

    # Phase 7: Lean
    lean_verified: bool = False
    lean_angle_front: float = 0.0
    lean_angle_back: float = 0.0
    current_bumper: str = "unknown"

    # Phase 8: Trim
    trim_verified: bool = False

    # Phase 9: Balance Point
    balance_point_verified: bool = False
    balance_angle: float = 0.0

    def save(self):
        # Mock save functionality
        pass
