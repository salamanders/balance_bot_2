import glm
import math

class MockPiconZero:
    def init(self) -> None:
        pass

    def stop(self) -> None:
        pass

    def set_retries(self, retries: int) -> None:
        pass

    def set_motor(self, motor: int, value: int) -> None:
        pass

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        pass

    def cleanup(self) -> None:
        pass

class MockMPU6050:
    def __init__(self, address: int, bus: int = 1):
        self.address = address

    def get_accel_data(self) -> dict:
        pitch = 0.0
        rad = math.radians(pitch)
        y = math.sin(rad) * 9.8
        z = math.cos(rad) * 9.8
        return {'x': 0.0, 'y': y, 'z': z}

    def get_gyro_data(self) -> dict:
        return {'x': 0.0, 'y': 0.0, 'z': 0.0}