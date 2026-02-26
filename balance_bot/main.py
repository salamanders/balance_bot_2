import argparse
import logging
import os

from balance_bot.configuration import HardwareConfig, LearningState
from balance_bot.hardware.robot_hardware import RobotHardware
from balance_bot.discovery.pipeline import SelfDiscoveryPipeline

def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )

def main():
    setup_logging()
    parser = argparse.ArgumentParser(description="Balance Bot 2 Discovery & Control")
    parser.add_argument("--allow-mocks", action="store_true", help="Allow fallback to mock hardware")
    args = parser.parse_args()

    if args.allow_mocks:
        os.environ["ALLOW_MOCK_FALLBACK"] = "1"

    print("=== Balance Bot 2 Initialization ===")
    config = HardwareConfig()
    state = LearningState()
    
    # Initialize Hardware Abstraction Layer
    hw = RobotHardware(config)

    # Run Discovery Pipeline
    print("
--- Starting Tabula Rasa Discovery ---")
    pipeline = SelfDiscoveryPipeline(hw, config, state)
    pipeline.run()
    
    print("
=== All systems go! ===")

if __name__ == "__main__":
    main()