from enum import Enum, auto
from typing import Protocol, Tuple, Dict, Any
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware

class StepStatus(Enum):
    SUCCESS = auto()
    NEEDS_RETRY = auto()
    FATAL = auto()

class CalibrationStep(Protocol):
    @property
    def name(self) -> str: ...

    def is_verified(self, state: LearningState) -> bool: ...

    def run(self, hw: RobotHardware, config: HardwareConfig, state: LearningState) -> Tuple[StepStatus, Dict[str, Any], Dict[str, Any]]: ...
