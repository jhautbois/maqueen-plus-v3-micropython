"""Maqueen Plus V3 - Obstacle Avoidance Program
Uses laser ToF matrix sensor to navigate and avoid obstacles

Behavior:
- Drives forward continuously
- Scans environment with 8x8 LIDAR matrix
- Stops if obstacle detected in center zone < 25cm
- Reverses if obstacle < 15cm
- Turns toward clearest direction
- Shows status on LED matrix and underglow LEDs
"""
from microbit import display, button_a, button_b, sleep, Image, running_time
from maqueen_plus_v3 import MaqueenPlusV3


# ========== Configuration ==========
DRIVE_SPEED = 100          # Normal driving speed (0-255)
TURN_SPEED = 80            # Turning speed
REVERSE_SPEED = 80         # Reverse speed

OBSTACLE_STOP = 250        # Stop if obstacle < 25cm
OBSTACLE_REVERSE = 150     # Reverse if obstacle < 15cm
OBSTACLE_SLOW = 400        # Slow down if obstacle < 40cm

REVERSE_TIME = 500         # Time to reverse (ms)
TURN_TIME = 400            # Time to turn (ms)

UPDATE_INTERVAL = 66       # ~15Hz sensor reading (ms)


# ========== LED Display Patterns ==========
ARROW_FORWARD = Image("00900:09990:90909:00900:00900")
ARROW_LEFT = Image("00900:09000:99999:09000:00900")
ARROW_RIGHT = Image("00900:00090:99999:00090:00900")
ARROW_BACK = Image("00900:00900:90909:09990:00900")
EXCLAMATION = Image("00900:00900:00900:00000:00900")


class ObstacleAvoidance:
    """Main obstacle avoidance controller"""

    def __init__(self):
        """Initialize robot and state"""
        print("Initializing Maqueen Plus V3...")
        display.show(Image.HEART)

        self.robot = MaqueenPlusV3(laser_enabled=True)

        if not self.robot.laser:
            print("ERROR: Laser sensor required!")
            display.show(Image.SAD)
            raise RuntimeError("Laser sensor not available")

        self.running = False
        self.last_update = 0
        self.state = "stopped"

        print("Ready! Press button A to start/stop")
        display.show(Image.HAPPY)

    def check_buttons(self):
        """Check for button presses"""
        if button_a.was_pressed():
            self.running = not self.running
            if self.running:
                print("Starting...")
                self.state = "driving"
            else:
                print("Stopped")
                self.robot.stop()
                self.robot.underglow_off()
                self.state = "stopped"
                display.show(Image.HAPPY)

        if button_b.was_pressed():
            # Emergency stop
            print("Emergency stop!")
            self.running = False
            self.robot.stop()
            self.robot.underglow_off()
            display.show(Image.NO)

    def read_sensors(self):
        """Read laser sensor zones

        Returns:
            Dict with 'left', 'center', 'right' distances (mm)
            or None on error
        """
        zones = self.robot.laser_zones()

        if zones:
            print(f"L:{zones['left']:4d} C:{zones['center']:4d} R:{zones['right']:4d} mm")

        return zones

    def decide_action(self, zones):
        """Decide what action to take based on sensor data

        Args:
            zones: Dict with left, center, right distances

        Returns:
            Action string: 'drive', 'reverse', 'turn_left', 'turn_right', 'stop'
        """
        if not zones:
            return 'stop'

        left = zones['left']
        center = zones['center']
        right = zones['right']

        # Find minimum distance
        min_dist = min(left, center, right)

        # Critical: Very close obstacle - reverse!
        if min_dist < OBSTACLE_REVERSE:
            print(f"  -> REVERSE! (obstacle at {min_dist}mm)")
            return 'reverse'

        # Center obstacle - need to turn
        if center < OBSTACLE_STOP:
            # Decide turn direction based on clearest side
            if left > right:
                print(f"  -> TURN LEFT (center blocked)")
                return 'turn_left'
            else:
                print(f"  -> TURN RIGHT (center blocked)")
                return 'turn_right'

        # Side obstacle - gentle turn away
        if left < OBSTACLE_STOP and left < right:
            print(f"  -> Drift right (left obstacle)")
            return 'drift_right'

        if right < OBSTACLE_STOP and right < left:
            print(f"  -> Drift left (right obstacle)")
            return 'drift_left'

        # All clear - drive forward
        # Slow down if obstacles nearby
        if min_dist < OBSTACLE_SLOW:
            return 'drive_slow'

        return 'drive'

    def execute_action(self, action):
        """Execute the decided action

        Args:
            action: Action string from decide_action()
        """
        if action == 'drive':
            self.robot.drive(DRIVE_SPEED)
            self.robot.underglow('all', 0, 50, 0)  # Green
            display.show(ARROW_FORWARD)
            self.state = "driving"

        elif action == 'drive_slow':
            self.robot.drive(DRIVE_SPEED // 2)
            self.robot.underglow('all', 50, 50, 0)  # Yellow
            display.show(ARROW_FORWARD)
            self.state = "driving"

        elif action == 'reverse':
            self.robot.drive(-REVERSE_SPEED)
            self.robot.underglow('all', 50, 0, 0)  # Red
            display.show(ARROW_BACK)
            self.state = "reversing"
            sleep(REVERSE_TIME)

        elif action == 'turn_left':
            self.robot.stop()
            sleep(100)
            self.robot.turn(-TURN_SPEED)
            self.robot.underglow('all', 0, 0, 50)  # Blue
            display.show(ARROW_LEFT)
            self.state = "turning"
            sleep(TURN_TIME)

        elif action == 'turn_right':
            self.robot.stop()
            sleep(100)
            self.robot.turn(TURN_SPEED)
            self.robot.underglow('all', 0, 0, 50)  # Blue
            display.show(ARROW_RIGHT)
            self.state = "turning"
            sleep(TURN_TIME)

        elif action == 'drift_left':
            self.robot.motors(DRIVE_SPEED // 2, DRIVE_SPEED)
            self.robot.underglow('all', 0, 30, 30)  # Cyan
            display.show(ARROW_LEFT)
            self.state = "driving"

        elif action == 'drift_right':
            self.robot.motors(DRIVE_SPEED, DRIVE_SPEED // 2)
            self.robot.underglow('all', 0, 30, 30)  # Cyan
            display.show(ARROW_RIGHT)
            self.state = "driving"

        else:  # stop
            self.robot.stop()
            self.robot.underglow('all', 50, 0, 0)  # Red
            display.show(EXCLAMATION)
            self.state = "stopped"

    def run(self):
        """Main control loop"""
        print("\nObstacle Avoidance Active")
        print("Button A: Start/Stop")
        print("Button B: Emergency Stop")
        print("-" * 40)

        while True:
            # Check buttons
            self.check_buttons()

            if not self.running:
                sleep(100)
                continue

            # Rate limiting (~15Hz)
            now = running_time()
            if now - self.last_update < UPDATE_INTERVAL:
                sleep(10)
                continue

            self.last_update = now

            # Read sensors
            zones = self.read_sensors()

            # Decide action
            action = self.decide_action(zones)

            # Execute action
            self.execute_action(action)


# ========== Main Entry Point ==========
def main():
    """Main program"""
    try:
        controller = ObstacleAvoidance()
        controller.run()

    except KeyboardInterrupt:
        print("\nStopped by user")
        display.show(Image.HAPPY)

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
