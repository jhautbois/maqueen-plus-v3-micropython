"""Type stubs for micro:bit MicroPython
Provides autocomplete and type hints for Zed editor

This stub covers the most commonly used microbit module features.
For full documentation, see: https://microbit-micropython.readthedocs.io/
"""

from typing import Tuple, List, Optional, Union


# ========== Display ==========
class Display:
    """5x5 LED matrix display"""

    def show(
        self,
        value: Union[str, int, float, 'Image'],
        delay: int = 400,
        wait: bool = True,
        loop: bool = False,
        clear: bool = False
    ) -> None:
        """Show a value on the display"""
        ...

    def scroll(
        self,
        value: Union[str, int, float],
        delay: int = 150,
        wait: bool = True,
        loop: bool = False,
        monospace: bool = False
    ) -> None:
        """Scroll text across the display"""
        ...

    def clear(self) -> None:
        """Clear the display"""
        ...

    def set_pixel(self, x: int, y: int, value: int) -> None:
        """Set brightness of pixel at (x, y) from 0-9"""
        ...

    def get_pixel(self, x: int, y: int) -> int:
        """Get brightness of pixel at (x, y)"""
        ...

    def on(self) -> None:
        """Turn on all LEDs"""
        ...

    def off(self) -> None:
        """Turn off all LEDs"""
        ...

    def is_on(self) -> bool:
        """Check if display is on"""
        ...


display: Display


# ========== Buttons ==========
class Button:
    """Physical button (A or B)"""

    def is_pressed(self) -> bool:
        """Check if button is currently pressed"""
        ...

    def was_pressed(self) -> bool:
        """Check if button was pressed since last call"""
        ...

    def get_presses(self) -> int:
        """Get number of presses since last call"""
        ...


button_a: Button
button_b: Button


# ========== Accelerometer ==========
class Accelerometer:
    """Motion sensor"""

    def get_x(self) -> int:
        """Get X axis acceleration in milli-g"""
        ...

    def get_y(self) -> int:
        """Get Y axis acceleration in milli-g"""
        ...

    def get_z(self) -> int:
        """Get Z axis acceleration in milli-g"""
        ...

    def get_values(self) -> Tuple[int, int, int]:
        """Get (x, y, z) acceleration tuple"""
        ...

    def current_gesture(self) -> str:
        """Get current gesture name"""
        ...

    def is_gesture(self, name: str) -> bool:
        """Check if given gesture is active"""
        ...

    def was_gesture(self, name: str) -> bool:
        """Check if gesture occurred since last call"""
        ...

    def get_gestures(self) -> Tuple[str, ...]:
        """Get list of recent gestures"""
        ...


accelerometer: Accelerometer


# ========== Compass ==========
class Compass:
    """Magnetometer / compass"""

    def calibrate(self) -> None:
        """Start calibration sequence"""
        ...

    def is_calibrated(self) -> bool:
        """Check if compass is calibrated"""
        ...

    def clear_calibration(self) -> None:
        """Reset calibration"""
        ...

    def get_x(self) -> int:
        """Get X axis reading in nano tesla"""
        ...

    def get_y(self) -> int:
        """Get Y axis reading in nano tesla"""
        ...

    def get_z(self) -> int:
        """Get Z axis reading in nano tesla"""
        ...

    def heading(self) -> int:
        """Get compass heading in degrees (0-359)"""
        ...

    def get_field_strength(self) -> int:
        """Get magnetic field strength in nano tesla"""
        ...


compass: Compass


# ========== I2C ==========
class I2C:
    """I2C bus communication"""

    def read(self, addr: int, n: int, repeat: bool = False) -> bytes:
        """Read n bytes from device at addr"""
        ...

    def write(self, addr: int, buf: bytes, repeat: bool = False) -> None:
        """Write buffer to device at addr"""
        ...

    def scan(self) -> List[int]:
        """Scan for I2C devices, returns list of addresses"""
        ...


i2c: I2C


# ========== Pins ==========
class MicroBitDigitalPin:
    """Digital pin"""

    def read_digital(self) -> int:
        """Read digital value (0 or 1)"""
        ...

    def write_digital(self, value: int) -> None:
        """Write digital value (0 or 1)"""
        ...

    def set_pull(self, value: int) -> None:
        """Set pull mode (NO_PULL, PULL_UP, PULL_DOWN)"""
        ...

    def get_pull(self) -> int:
        """Get current pull mode"""
        ...

    def get_mode(self) -> str:
        """Get current pin mode"""
        ...

    def is_touched(self) -> bool:
        """Check if pin is touched (capacitive)"""
        ...


class MicroBitAnalogDigitalPin(MicroBitDigitalPin):
    """Pin with analog capabilities"""

    def read_analog(self) -> int:
        """Read analog value (0-1023)"""
        ...

    def write_analog(self, value: int) -> None:
        """Write analog value (0-1023) as PWM"""
        ...

    def set_analog_period(self, period: int) -> None:
        """Set PWM period in milliseconds"""
        ...

    def set_analog_period_microseconds(self, period: int) -> None:
        """Set PWM period in microseconds"""
        ...


class MicroBitTouchPin(MicroBitAnalogDigitalPin):
    """Pin with touch sensing"""
    ...


# Pin instances
pin0: MicroBitTouchPin
pin1: MicroBitTouchPin
pin2: MicroBitTouchPin
pin3: MicroBitAnalogDigitalPin
pin4: MicroBitAnalogDigitalPin
pin5: MicroBitDigitalPin
pin6: MicroBitDigitalPin
pin7: MicroBitDigitalPin
pin8: MicroBitDigitalPin
pin9: MicroBitDigitalPin
pin10: MicroBitAnalogDigitalPin
pin11: MicroBitDigitalPin
pin12: MicroBitDigitalPin
pin13: MicroBitDigitalPin
pin14: MicroBitDigitalPin
pin15: MicroBitDigitalPin
pin16: MicroBitDigitalPin
pin19: MicroBitDigitalPin
pin20: MicroBitDigitalPin


# ========== Images ==========
class Image:
    """5x5 LED image"""

    def __init__(
        self,
        string: Optional[str] = None,
        width: Optional[int] = None,
        height: Optional[int] = None,
        buffer: Optional[bytearray] = None
    ) -> None:
        ...

    def width(self) -> int:
        """Get image width"""
        ...

    def height(self) -> int:
        """Get image height"""
        ...

    def get_pixel(self, x: int, y: int) -> int:
        """Get pixel brightness (0-9)"""
        ...

    def set_pixel(self, x: int, y: int, value: int) -> None:
        """Set pixel brightness (0-9)"""
        ...

    def shift_left(self, n: int) -> 'Image':
        """Return image shifted left by n pixels"""
        ...

    def shift_right(self, n: int) -> 'Image':
        """Return image shifted right by n pixels"""
        ...

    def shift_up(self, n: int) -> 'Image':
        """Return image shifted up by n pixels"""
        ...

    def shift_down(self, n: int) -> 'Image':
        """Return image shifted down by n pixels"""
        ...

    def crop(self, x: int, y: int, w: int, h: int) -> 'Image':
        """Return cropped image"""
        ...

    def copy(self) -> 'Image':
        """Return a copy of the image"""
        ...

    def invert(self) -> 'Image':
        """Return inverted image"""
        ...

    # Built-in images
    HEART: 'Image'
    HEART_SMALL: 'Image'
    HAPPY: 'Image'
    SMILE: 'Image'
    SAD: 'Image'
    CONFUSED: 'Image'
    ANGRY: 'Image'
    ASLEEP: 'Image'
    SURPRISED: 'Image'
    SILLY: 'Image'
    FABULOUS: 'Image'
    MEH: 'Image'
    YES: 'Image'
    NO: 'Image'
    SQUARE: 'Image'
    SQUARE_SMALL: 'Image'
    TRIANGLE: 'Image'
    TRIANGLE_LEFT: 'Image'
    DIAMOND: 'Image'
    DIAMOND_SMALL: 'Image'
    TARGET: 'Image'
    ARROW_N: 'Image'
    ARROW_NE: 'Image'
    ARROW_E: 'Image'
    ARROW_SE: 'Image'
    ARROW_S: 'Image'
    ARROW_SW: 'Image'
    ARROW_W: 'Image'
    ARROW_NW: 'Image'
    CLOCK1: 'Image'
    CLOCK2: 'Image'
    CLOCK3: 'Image'
    CLOCK4: 'Image'
    CLOCK5: 'Image'
    CLOCK6: 'Image'
    CLOCK7: 'Image'
    CLOCK8: 'Image'
    CLOCK9: 'Image'
    CLOCK10: 'Image'
    CLOCK11: 'Image'
    CLOCK12: 'Image'


# ========== Utility Functions ==========
def sleep(milliseconds: int) -> None:
    """Sleep for given milliseconds"""
    ...


def running_time() -> int:
    """Get milliseconds since boot"""
    ...


def temperature() -> int:
    """Get CPU temperature in Celsius"""
    ...


def panic(error_code: int) -> None:
    """Enter panic mode with error code"""
    ...


def reset() -> None:
    """Reset the micro:bit"""
    ...


# ========== Constants ==========
NO_PULL: int
PULL_UP: int
PULL_DOWN: int
