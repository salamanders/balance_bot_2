import smbus2 as smbus
import time
from typing import Callable, Any

class PiconZero:
    """
    Driver for 4tronix Picon Zero Motor HAT.
    Implements the MotorDriver protocol.
    Replaces the legacy module-based piconzero driver and adapter.
    """
    I2C_ADDRESS = 0x22
    CMD_RESET = 20

    def __init__(self, bus_number: int = 1):
        self.bus_number = bus_number
        self.retries = 10
        self.debug = False
        self.bus = None
        self._open_bus()

    def _open_bus(self):
        """Open or re-open the I2C bus."""
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass
        self.bus = smbus.SMBus(self.bus_number)

    def _retry(self, func: Callable[[], Any], name: str) -> Any:
        """Internal helper to retry I2C operations."""
        for _ in range(self.retries):
            try:
                return func()
            except Exception:
                if self.debug:
                    print(f"Error in {name}(), retrying")
                time.sleep(0.005)
        raise OSError(f"PiconZero {name}() failed after {self.retries} retries")

    def init(self, debug: bool = False) -> None:
        """Initialize the motor driver hardware."""
        self.debug = debug
        self._retry(lambda: self.bus.write_byte_data(self.I2C_ADDRESS, self.CMD_RESET, 0), "init")
        time.sleep(0.1)
        if self.debug:
            print("PiconZero Debug is", self.debug)

    def cleanup(self) -> None:
        """Release hardware resources."""
        # Reset the board
        try:
            self._retry(lambda: self.bus.write_byte_data(self.I2C_ADDRESS, self.CMD_RESET, 0), "cleanup")
        except OSError:
            pass # Best effort cleanup
        time.sleep(0.001)
        if self.bus:
            try:
                self.bus.close()
            except Exception:
                pass

    def stop(self) -> None:
        """Stop all motors immediately."""
        self.set_motor(0, 0)
        self.set_motor(1, 0)

    def set_retries(self, retries: int) -> None:
        """Set the number of I2C retries."""
        self.retries = retries

    def set_motor(self, motor: int, value: int) -> None:
        """
        Set speed for a specific motor.
        :param motor: Motor channel index (0 or 1).
        :param value: Speed (-100 to 100).
        """
        if 0 <= motor <= 1 and -128 <= value < 128:
            self._retry(lambda: self.bus.write_byte_data(self.I2C_ADDRESS, motor, value), "set_motor")

    def set_motors(self, motor_0_val: int, motor_1_val: int) -> None:
        """
        Set speed for both motors.
        The module doesn't support block write for motors, so we call set_motor twice.
        :param motor_0_val: Speed for Motor 0 (-100 to 100).
        :param motor_1_val: Speed for Motor 1 (-100 to 100).
        """
        self.set_motor(0, motor_0_val)
        self.set_motor(1, motor_1_val)
