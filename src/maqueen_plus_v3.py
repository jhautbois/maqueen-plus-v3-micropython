"""Maqueen Plus V3 MicroPython Library (minified)"""
from microbit import i2c, pin1, sleep
import neopixel

try:
    from laser_matrix import LaserMatrix
except ImportError:
    LaserMatrix = None

class MaqueenPlusV3:
    STM8_ADDR = 0x10
    REG_MOTOR_LEFT = 0x00
    REG_MOTOR_RIGHT = 0x02
    REG_HEADLIGHT = 0x0B
    DIR_FORWARD = 0x00
    DIR_REVERSE = 0x01
    # Headlight color indices (not RGB!)
    HL_COLORS = {
        'off': 0, 'red': 1, 'green': 2, 'yellow': 3,
        'blue': 4, 'cyan': 5, 'purple': 6, 'magenta': 6, 'white': 7,
    }

    def __init__(self, laser_enabled=True):
        if self.STM8_ADDR not in i2c.scan():
            raise RuntimeError("STM8 not found at 0x10")
        self.laser = None
        if laser_enabled and LaserMatrix:
            try:
                self.laser = LaserMatrix()
            except:
                pass
        self.underglow_leds = neopixel.NeoPixel(pin1, 4)
        self.underglow_off()

    def _write_reg(self, reg, data):
        if isinstance(data, int):
            data = bytes([data])
        i2c.write(self.STM8_ADDR, bytes([reg]) + data)

    def motors(self, left_speed, right_speed):
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))
        left_dir = self.DIR_REVERSE if left_speed < 0 else self.DIR_FORWARD
        self._write_reg(self.REG_MOTOR_LEFT, bytes([left_dir, abs(left_speed)]))
        right_dir = self.DIR_REVERSE if right_speed < 0 else self.DIR_FORWARD
        self._write_reg(self.REG_MOTOR_RIGHT, bytes([right_dir, abs(right_speed)]))

    def drive(self, speed):
        self.motors(speed, speed)

    def turn(self, speed):
        self.motors(-speed, speed)

    def stop(self):
        self.motors(0, 0)

    def headlights(self, left_color, right_color):
        # Convert color names to indices
        if isinstance(left_color, str):
            left_color = self.HL_COLORS.get(left_color.lower(), 0)
        if isinstance(right_color, str):
            right_color = self.HL_COLORS.get(right_color.lower(), 0)
        try:
            # Separate registers: 0x0B=left, 0x0C=right
            self._write_reg(0x0B, left_color)
            sleep(10)
            self._write_reg(0x0C, right_color)
        except:
            pass

    def underglow(self, index, r, g, b):
        if index == 'all':
            for i in range(4):
                self.underglow_leds[i] = (r, g, b)
        else:
            self.underglow_leds[index] = (r, g, b)
        self.underglow_leds.show()

    def underglow_off(self):
        self.underglow('all', 0, 0, 0)

    def laser_zones(self):
        if not self.laser:
            return None
        return self.laser.read_zones()
