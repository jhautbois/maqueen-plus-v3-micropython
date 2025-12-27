"""Maqueen Plus V3 MicroPython Library
Complete API for DFRobot Maqueen Plus V3 robot

Based on DFRobot's official MakeCode library:
https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20

Features:
- Motor control with encoders
- Line tracking sensors (5x IR, analog/digital)
- Light sensors (ambient light)
- Laser ToF matrix sensor (8x8 LIDAR)
- RGB headlights (STM8-controlled)
- WS2812 underglow LEDs (4x, pin15)

Note: Protocol based on V2, tested for V3 compatibility.
Some features (encoders, light sensors) need hardware validation.
See SOURCES.md for detailed protocol documentation.
"""
from microbit import i2c, pin15, sleep
import neopixel

try:
    from laser_matrix import LaserMatrix
except ImportError:
    LaserMatrix = None
    print("Warning: laser_matrix module not found")


class MaqueenPlusV3:
    """Main class for Maqueen Plus V3 robot control"""

    # I2C addresses
    STM8_ADDR = 0x10  # Motor controller & sensors

    # STM8 Register addresses (from official DFRobot MakeCode library)
    # Source: https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20
    REG_MOTOR_LEFT = 0x00
    REG_MOTOR_RIGHT = 0x02
    REG_ENCODER_LEFT = 0x04
    REG_ENCODER_RIGHT = 0x06
    REG_RGB_LEFT = 0x0B
    REG_RGB_RIGHT = 0x0C  # Fixed: was 0x0D
    REG_LINE_STATE = 0x1D  # Line sensor digital state (bitfield)
    REG_LINE_R2 = 0x1E     # Line sensor R2 (ADC, 16-bit)
    REG_LINE_R1 = 0x20     # Line sensor R1 (ADC, 16-bit)
    REG_LINE_M = 0x22      # Line sensor Middle (ADC, 16-bit)
    REG_LINE_L1 = 0x24     # Line sensor L1 (ADC, 16-bit)
    REG_LINE_L2 = 0x26     # Line sensor L2 (ADC, 16-bit)
    REG_VERSION = 0x32     # Version count
    REG_VERSION_DATA = 0x33  # Version data
    REG_RESET = 0x49       # System reset (write 1)

    # Motor directions
    DIR_FORWARD = 0x00
    DIR_REVERSE = 0x01

    # RGB color presets
    COLORS = {
        'off': (0, 0, 0),
        'red': (255, 0, 0),
        'green': (0, 255, 0),
        'blue': (0, 0, 255),
        'yellow': (255, 255, 0),
        'cyan': (0, 255, 255),
        'magenta': (255, 0, 255),
        'white': (255, 255, 255),
        'orange': (255, 128, 0),
        'purple': (128, 0, 255),
    }

    def __init__(self, laser_enabled=True):
        """Initialize Maqueen Plus V3

        Args:
            laser_enabled: Enable laser ToF sensor (default True)
        """
        self._check_stm8()
        self.laser = None

        # Initialize laser sensor if enabled
        if laser_enabled and LaserMatrix:
            try:
                self.laser = LaserMatrix()
            except:
                print("Warning: Failed to init laser sensor")

        # Initialize underglow LEDs (4 WS2812 on pin15)
        self.underglow_leds = neopixel.NeoPixel(pin15, 4)
        self.underglow_off()

        # Encoder state
        self._last_encoder_left = 0
        self._last_encoder_right = 0

    def _check_stm8(self):
        """Check STM8 motor controller connection"""
        devices = i2c.scan()
        if self.STM8_ADDR not in devices:
            raise RuntimeError(f"STM8 not found at 0x{self.STM8_ADDR:02X}")

    def _write_reg(self, reg, data):
        """Write data to STM8 register"""
        if isinstance(data, int):
            data = bytes([data])
        i2c.write(self.STM8_ADDR, bytes([reg]) + data)

    def _read_reg(self, reg, length=1):
        """Read data from STM8 register"""
        i2c.write(self.STM8_ADDR, bytes([reg]))
        return i2c.read(self.STM8_ADDR, length)

    def _read_u16(self, reg):
        """Read 16-bit unsigned value (big-endian)"""
        data = self._read_reg(reg, 2)
        return (data[0] << 8) | data[1]

    def _read_u16_le(self, reg):
        """Read 16-bit unsigned value (little-endian)"""
        data = self._read_reg(reg, 2)
        return (data[1] << 8) | data[0]

    def _read_s16(self, reg):
        """Read 16-bit signed value (big-endian)"""
        val = self._read_u16(reg)
        return val if val < 32768 else val - 65536

    # ========== Motor Control ==========

    def motors(self, left_speed, right_speed):
        """Control both motors independently

        Args:
            left_speed: -255 to 255 (negative = reverse)
            right_speed: -255 to 255 (negative = reverse)
        """
        # Clamp speeds
        left_speed = max(-255, min(255, left_speed))
        right_speed = max(-255, min(255, right_speed))

        # Set left motor
        left_dir = self.DIR_REVERSE if left_speed < 0 else self.DIR_FORWARD
        left_speed = abs(left_speed)
        self._write_reg(self.REG_MOTOR_LEFT, bytes([left_dir, left_speed]))

        # Set right motor
        right_dir = self.DIR_REVERSE if right_speed < 0 else self.DIR_FORWARD
        right_speed = abs(right_speed)
        self._write_reg(self.REG_MOTOR_RIGHT, bytes([right_dir, right_speed]))

    def drive(self, speed):
        """Drive straight forward or backward

        Args:
            speed: -255 to 255 (negative = reverse)
        """
        self.motors(speed, speed)

    def turn(self, speed):
        """Turn in place

        Args:
            speed: -255 to 255 (negative = left, positive = right)
        """
        self.motors(-speed, speed)

    def stop(self):
        """Stop both motors"""
        self.motors(0, 0)

    # ========== Sensor Reading ==========

    def line_sensors(self):
        """Read line tracking sensors (analog)

        Returns:
            List [L2, L1, M, R1, R2] with values 0-1023
            Higher values = lighter surface
        """
        try:
            # Read each sensor separately (16-bit little-endian)
            l2 = self._read_u16_le(self.REG_LINE_L2)
            l1 = self._read_u16_le(self.REG_LINE_L1)
            m = self._read_u16_le(self.REG_LINE_M)
            r1 = self._read_u16_le(self.REG_LINE_R1)
            r2 = self._read_u16_le(self.REG_LINE_R2)
            return [l2, l1, m, r1, r2]
        except:
            return [0, 0, 0, 0, 0]

    def line_digital(self, threshold=512):
        """Read line sensors as digital (0/1)

        Args:
            threshold: Value above which sensor reads as 1 (default 512)

        Returns:
            List [L2, L1, M, R1, R2] with values 0 or 1
        """
        analog = self.line_sensors()
        return [1 if v > threshold else 0 for v in analog]

    def light_sensors(self):
        """Read ambient light sensors

        Returns:
            Tuple (left, right) with values 0-1023
            Higher values = brighter light
        """
        try:
            left = self._read_u16(self.REG_LIGHT_LEFT)
            right = self._read_u16(self.REG_LIGHT_RIGHT)
            return (left, right)
        except:
            return (0, 0)

    # ========== Laser ToF Sensor ==========

    def laser_distance(self):
        """Get center distance from laser sensor

        Returns:
            Distance in mm (20-4000), or -1 if sensor not available
        """
        if not self.laser:
            return -1
        zones = self.laser.read_zones()
        return zones['center'] if zones else -1

    def laser_matrix(self):
        """Get full 8x8 distance matrix

        Returns:
            List of 64 distances in mm, or None if not available
        """
        if not self.laser:
            return None
        return self.laser.read_matrix()

    def laser_zones(self):
        """Get simplified zone distances (left, center, right)

        Returns:
            Dict with keys 'left', 'center', 'right' (mm)
            Returns None if sensor not available
        """
        if not self.laser:
            return None
        return self.laser.read_zones()

    # ========== RGB LEDs ==========

    def headlights(self, left_color, right_color):
        """Set RGB headlight colors

        Args:
            left_color: Color name or (r, g, b) tuple
            right_color: Color name or (r, g, b) tuple
        """
        # Convert color names to RGB
        if isinstance(left_color, str):
            left_color = self.COLORS.get(left_color.lower(), (0, 0, 0))
        if isinstance(right_color, str):
            right_color = self.COLORS.get(right_color.lower(), (0, 0, 0))

        # Send to STM8
        try:
            self._write_reg(self.REG_RGB_LEFT, bytes(left_color))
            self._write_reg(self.REG_RGB_RIGHT, bytes(right_color))
        except:
            pass  # Ignore errors if not supported

    def underglow(self, index, r, g, b):
        """Set underglow LED color

        Args:
            index: LED index (0-3) or 'all' for all LEDs
            r, g, b: RGB values (0-255)
        """
        if index == 'all':
            for i in range(4):
                self.underglow_leds[i] = (r, g, b)
        else:
            self.underglow_leds[index] = (r, g, b)
        self.underglow_leds.show()

    def underglow_off(self):
        """Turn off all underglow LEDs"""
        self.underglow('all', 0, 0, 0)

    # ========== Encoders ==========

    def read_encoders(self):
        """Read encoder values

        Returns:
            Tuple (left, right) encoder counts
        """
        try:
            left = self._read_s16(self.REG_ENCODER_LEFT)
            right = self._read_s16(self.REG_ENCODER_RIGHT)
            return (left, right)
        except:
            return (0, 0)

    def reset_encoders(self):
        """Reset encoder counters to zero"""
        # Command depends on STM8 firmware
        # May need to write specific reset command
        self._last_encoder_left = 0
        self._last_encoder_right = 0

    def speed(self):
        """Get current wheel speeds from encoders

        Returns:
            Tuple (left_speed, right_speed) in counts per second
            Note: Actual cm/min depends on wheel diameter and gear ratio
        """
        left, right = self.read_encoders()
        left_delta = left - self._last_encoder_left
        right_delta = right - self._last_encoder_right
        self._last_encoder_left = left
        self._last_encoder_right = right
        return (left_delta, right_delta)

    # ========== Utility Methods ==========

    def get_version(self):
        """Get STM8 firmware version

        Returns:
            Version number or 0 if not available
        """
        try:
            return self._read_reg(self.REG_VERSION)[0]
        except:
            return 0

    def self_test(self):
        """Run self-test routine

        Tests all components and prints results
        """
        from microbit import display, Image

        print("\n" + "="*40)
        print("Maqueen Plus V3 Self-Test")
        print("="*40)

        # Test STM8
        print("\n1. STM8 Controller:")
        version = self.get_version()
        print(f"   Version: {version if version else 'Unknown'}")
        display.show(Image.HEART)

        # Test line sensors
        print("\n2. Line Sensors:")
        line = self.line_sensors()
        print(f"   L2:{line[0]:4d} L1:{line[1]:4d} M:{line[2]:4d} R1:{line[3]:4d} R2:{line[4]:4d}")

        # Test light sensors
        print("\n3. Light Sensors:")
        left, right = self.light_sensors()
        print(f"   Left:{left:4d}  Right:{right:4d}")

        # Test laser
        print("\n4. Laser ToF Sensor:")
        if self.laser:
            zones = self.laser_zones()
            if zones:
                print(f"   L:{zones['left']:4d} C:{zones['center']:4d} R:{zones['right']:4d} mm")
            else:
                print("   Error reading sensor")
        else:
            print("   Not initialized")

        # Test encoders
        print("\n5. Encoders:")
        left, right = self.read_encoders()
        print(f"   Left:{left:6d}  Right:{right:6d}")

        # Test LEDs
        print("\n6. Testing LEDs...")
        self.headlights('red', 'green')
        self.underglow('all', 0, 0, 255)
        sleep(500)
        self.headlights('off', 'off')
        self.underglow_off()

        # Test motors (brief)
        print("\n7. Testing motors (2 sec)...")
        self.drive(50)
        sleep(1000)
        self.drive(-50)
        sleep(1000)
        self.stop()

        print("\n" + "="*40)
        print("Self-test complete!")
        print("="*40 + "\n")
        display.show(Image.HAPPY)


# Convenience function for testing
def test():
    """Quick test of robot functionality"""
    robot = MaqueenPlusV3()
    robot.self_test()


if __name__ == "__main__":
    test()
