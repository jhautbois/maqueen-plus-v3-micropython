"""Maqueen Plus V3 - Light Seeker Example
Robot that follows bright light using ambient light sensors

Behavior:
- Reads left and right light sensors
- Turns toward brighter light source
- Drives forward when light is centered
- Stops when light is very bright (close to source)

Controls:
- Button A: Start/Stop
- Button B: Emergency stop
"""
from microbit import display, button_a, button_b, sleep, Image
from maqueen_plus_v3 import MaqueenPlusV3


# ========== Configuration ==========
DRIVE_SPEED = 100          # Forward speed
TURN_SPEED = 80            # Turning speed

LIGHT_DIFF_THRESHOLD = 50  # Difference to trigger turn
BRIGHT_THRESHOLD = 800     # Stop when this bright (0-1023)
MIN_LIGHT = 100            # Minimum light to be considered "found"


class LightSeeker:
    """Light-seeking robot"""

    def __init__(self):
        """Initialize robot"""
        print("Initializing Light Seeker...")
        display.show(Image.HEART)

        self.robot = MaqueenPlusV3(laser_enabled=False)
        self.running = False

        print("Ready! Press button A to start")
        print("Point a flashlight at the robot to attract it")
        display.show(Image.HAPPY)

    def read_light(self):
        """Read light sensors

        Returns:
            Tuple (left, right, difference)
            difference > 0 means right is brighter
        """
        left, right = self.robot.light_sensors()
        diff = right - left

        print(f"Light: L={left:4d} R={right:4d} diff={diff:+5d}")

        # Show light levels on underglow (brightness)
        left_brightness = min(255, left // 4)
        right_brightness = min(255, right // 4)

        self.robot.underglow(0, left_brightness, left_brightness, 0)
        self.robot.underglow(1, left_brightness, left_brightness, 0)
        self.robot.underglow(2, right_brightness, right_brightness, 0)
        self.robot.underglow(3, right_brightness, right_brightness, 0)

        return (left, right, diff)

    def decide_action(self, left, right, diff):
        """Decide what to do based on light readings

        Returns:
            Action: 'forward', 'turn_left', 'turn_right', 'stop', 'search'
        """
        max_light = max(left, right)

        # Very bright - reached light source
        if max_light > BRIGHT_THRESHOLD:
            print("  -> Found light source!")
            return 'stop'

        # No significant light detected - search
        if max_light < MIN_LIGHT:
            print("  -> Searching for light...")
            return 'search'

        # Significant difference - turn toward brighter side
        if abs(diff) > LIGHT_DIFF_THRESHOLD:
            if diff > 0:  # Right is brighter
                print("  -> Turn right (toward light)")
                return 'turn_right'
            else:  # Left is brighter
                print("  -> Turn left (toward light)")
                return 'turn_left'

        # Light roughly centered - drive forward
        print("  -> Drive forward (light ahead)")
        return 'forward'

    def execute_action(self, action):
        """Execute the decided action"""

        if action == 'forward':
            self.robot.drive(DRIVE_SPEED)
            self.robot.headlights('green', 'green')
            display.show(Image.ARROW_N)

        elif action == 'turn_left':
            # Differential drive - slow left side
            self.robot.motors(TURN_SPEED // 2, TURN_SPEED)
            self.robot.headlights('yellow', 'green')
            display.show(Image.ARROW_W)

        elif action == 'turn_right':
            # Differential drive - slow right side
            self.robot.motors(TURN_SPEED, TURN_SPEED // 2)
            self.robot.headlights('green', 'yellow')
            display.show(Image.ARROW_E)

        elif action == 'search':
            # Rotate slowly to find light
            self.robot.turn(TURN_SPEED // 2)
            self.robot.headlights('blue', 'blue')
            display.show(Image.CONFUSED)

        else:  # stop
            self.robot.stop()
            self.robot.headlights('red', 'red')
            display.show(Image.HAPPY)
            sleep(2000)
            # Back up a bit
            self.robot.drive(-DRIVE_SPEED // 2)
            sleep(500)
            self.robot.stop()

    def run(self):
        """Main control loop"""
        print("\nLight Seeker Active")
        print("Button A: Start/Stop")
        print("Button B: Emergency Stop")
        print("-" * 40)

        while True:
            # Check buttons
            if button_a.was_pressed():
                self.running = not self.running
                if self.running:
                    print("Seeking light...")
                else:
                    print("Stopped")
                    self.robot.stop()
                    self.robot.underglow_off()
                    display.show(Image.HAPPY)

            if button_b.was_pressed():
                print("Emergency stop!")
                self.running = False
                self.robot.stop()
                self.robot.underglow_off()
                display.show(Image.NO)

            if not self.running:
                sleep(100)
                continue

            # Read sensors
            left, right, diff = self.read_light()

            # Decide action
            action = self.decide_action(left, right, diff)

            # Execute
            self.execute_action(action)

            # Update rate
            sleep(100)


def main():
    """Main program"""
    try:
        seeker = LightSeeker()
        seeker.run()

    except KeyboardInterrupt:
        print("\nStopped by user")

    except Exception as e:
        print(f"Error: {e}")
        display.show(Image.SAD)

    finally:
        # Cleanup
        try:
            robot = MaqueenPlusV3(laser_enabled=False)
            robot.stop()
            robot.underglow_off()
        except:
            pass


if __name__ == "__main__":
    main()
