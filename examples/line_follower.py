"""Maqueen Plus V3 - Line Follower Example
Uses 5 IR line sensors to follow a black line on white surface

Controls:
- Button A: Start/Stop
- Button B: Emergency stop

Setup:
1. Calibrate sensors using "Calc-Key" button on robot
2. Place robot on line
3. Press button A to start
"""
from microbit import display, button_a, button_b, sleep, Image
from maqueen_plus_v3 import MaqueenPlusV3


# ========== Configuration ==========
BASE_SPEED = 120           # Base driving speed
TURN_FACTOR = 0.6          # Turn speed multiplier (0-1)

# Line sensor threshold (adjust after calibration)
LINE_THRESHOLD = 512       # 0=black, 1023=white


class LineFollower:
    """PID-based line follower"""

    def __init__(self):
        """Initialize robot"""
        print("Initializing Line Follower...")
        display.show(Image.HEART)

        self.robot = MaqueenPlusV3(laser_enabled=False)
        self.running = False

        # PID constants (tune these for your setup)
        self.kp = 0.5  # Proportional
        self.ki = 0.0  # Integral (usually not needed)
        self.kd = 0.2  # Derivative

        self.last_error = 0
        self.integral = 0

        print("Ready! Press button A to start")
        display.show(Image.HAPPY)

    def read_position(self):
        """Read line position from sensors

        Returns:
            Position error (-2 to +2, 0 = centered)
            -2 = far left, +2 = far right
            None = line lost
        """
        # Read digital sensors [L2, L1, M, R1, R2]
        sensors = self.robot.line_digital(threshold=LINE_THRESHOLD)

        # Show sensor state on LEDs
        led_colors = ['red' if s else 'off' for s in sensors]
        if any(sensors):
            self.robot.headlights(led_colors[0], led_colors[4])
        else:
            self.robot.headlights('blue', 'blue')

        # Calculate weighted position
        # Weights: [-2, -1, 0, 1, 2]
        if sensors[2]:  # Center sensor
            return 0

        if sensors[1]:  # Left-center
            return -1 if not sensors[3] else 0

        if sensors[3]:  # Right-center
            return 1 if not sensors[1] else 0

        if sensors[0]:  # Far left
            return -2

        if sensors[4]:  # Far right
            return 2

        # No sensors detecting line
        return None

    def calculate_steering(self, error):
        """Calculate steering correction using PID

        Args:
            error: Position error from read_position()

        Returns:
            Steering value (-1 to 1)
        """
        if error is None:
            return 0

        # PID calculation
        self.integral += error
        derivative = error - self.last_error

        steering = (self.kp * error +
                   self.ki * self.integral +
                   self.kd * derivative)

        self.last_error = error

        # Clamp to -1 to 1
        return max(-1, min(1, steering))

    def drive(self, steering):
        """Drive robot with steering correction

        Args:
            steering: -1 (left) to 1 (right)
        """
        if steering < 0:  # Turn left
            left_speed = int(BASE_SPEED * (1 + steering * TURN_FACTOR))
            right_speed = BASE_SPEED
        else:  # Turn right
            left_speed = BASE_SPEED
            right_speed = int(BASE_SPEED * (1 - steering * TURN_FACTOR))

        self.robot.motors(left_speed, right_speed)

        # Visual feedback
        if abs(steering) < 0.2:
            display.show(Image.ARROW_N)  # Straight
        elif steering < 0:
            display.show(Image.ARROW_W)  # Left
        else:
            display.show(Image.ARROW_E)  # Right

    def run(self):
        """Main control loop"""
        print("\nLine Follower Active")
        print("Button A: Start/Stop")
        print("Button B: Emergency Stop")
        print("-" * 40)

        line_lost_count = 0

        while True:
            # Check buttons
            if button_a.was_pressed():
                self.running = not self.running
                if self.running:
                    print("Following line...")
                    self.integral = 0
                    self.last_error = 0
                else:
                    print("Stopped")
                    self.robot.stop()
                    display.show(Image.HAPPY)

            if button_b.was_pressed():
                print("Emergency stop!")
                self.running = False
                self.robot.stop()
                display.show(Image.NO)

            if not self.running:
                sleep(100)
                continue

            # Read line position
            error = self.read_position()

            if error is None:
                # Line lost - continue straight briefly
                line_lost_count += 1
                if line_lost_count > 10:
                    print("Line lost!")
                    self.robot.stop()
                    display.show(Image.CONFUSED)
                    self.running = False
                    line_lost_count = 0
                else:
                    # Continue with last steering
                    pass
            else:
                line_lost_count = 0
                # Calculate steering
                steering = self.calculate_steering(error)
                # Drive
                self.drive(steering)

            sleep(20)  # ~50Hz update rate


def main():
    """Main program"""
    try:
        follower = LineFollower()
        follower.run()

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
        except:
            pass


if __name__ == "__main__":
    main()
