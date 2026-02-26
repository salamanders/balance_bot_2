import time
from typing import List
from ..configuration import HardwareConfig, LearningState
from ..hardware.robot_hardware import RobotHardware
from .protocol import CalibrationStep, StepStatus
from .steps import (
    InitialAssumptionStep,
    HardwareCheckStep,
    GravityCalibrationStep,
    FrictionThresholdStep,
    MotorPolarityStep,
    TurnDirectionStep,
    LeanCharacterizationStep,
    DriveTrimStep,
    BalancePointStep
)

class SelfDiscoveryPipeline:
    def __init__(self, hw: RobotHardware, config: HardwareConfig, state: LearningState):
        self.hw = hw
        self.config = config
        self.state = state
        self.steps: List[CalibrationStep] = [
            InitialAssumptionStep(),
            HardwareCheckStep(),
            GravityCalibrationStep(),
            FrictionThresholdStep(),
            MotorPolarityStep(),
            TurnDirectionStep(),
            LeanCharacterizationStep(),
            DriveTrimStep(),
            BalancePointStep()
        ]

    def run(self):
        print("Starting Self-Discovery Pipeline...")

        for step in self.steps:
            print(f"\n--- Checking Step: {step.name} ---")

            while True: # Retry Loop
                if step.is_verified(self.state):
                    print(f"Step {step.name} already verified.")
                    break

                print(f"Running {step.name}...")
                status, config_updates, state_updates = step.run(self.hw, self.config, self.state)

                if status == StepStatus.SUCCESS:
                    print(f"Step {step.name} SUCCEEDED.")

                    if state_updates:
                        for k, v in state_updates.items():
                            setattr(self.state, k, v)
                        self.state.save()

                    if config_updates:
                        # Create a copy with updates
                        # Pydantic's copy(update=...) is deprecated in v2, use model_copy(update=...)
                        # Prompt used model_copy
                        new_config = self.config.model_copy(update=config_updates)

                        if new_config != self.config:
                            print(f"Updating configuration: {config_updates}")
                            new_config.save()
                            self.config = new_config
                            self.hw.apply_config(self.config)

                    # Verify immediately?
                    # The loop checks is_verified at start.
                    # So if we break here, we move to next step.
                    # But if the step logic relies on is_verified returning true now, it should work.
                    # However, if 'run' returns SUCCESS but 'is_verified' is still False (e.g. DriveTrimStep returning SUCCESS but verification needs another run?),
                    # then we should probably NOT break and let the loop run again?
                    # The prompt implementation:
                    # if status == SUCCESS: ... break
                    # So it breaks the retry loop and moves to next step.
                    break

                elif status == StepStatus.NEEDS_RETRY:
                    print(f"Step {step.name} requested RETRY.")
                    self.hw.stop()
                    time.sleep(2.0)
                    continue

                elif status == StepStatus.FATAL:
                    self.hw.stop()
                    raise RuntimeError(f"Pipeline halted at {step.name}")

        print("\nSelf-Discovery Pipeline COMPLETED Successfully.")
