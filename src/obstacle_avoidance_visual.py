"""Obstacle Avoidance with LIDAR Visualization

Features:
- White headlights (front), red taillights (rear)
- 5x5 LED matrix shows LIDAR data (brightness = proximity)
- Adaptive scanning: fast when moving, detailed when stopped
- 5-zone navigation algorithm for smooth obstacle avoidance
"""
from microbit import display, Image, button_a, button_b, sleep, running_time
from maqueen_plus_v3 import MaqueenPlusV3
from laser_matrix import LaserMatrix

# ============== Configuration ==============

# Distance thresholds (mm)
CRITICAL_DISTANCE = 150   # Emergency reverse
STOP_DISTANCE = 250       # Stop and turn
SLOW_DISTANCE = 400       # Slow down
SAFE_DISTANCE = 600       # Full speed

# Speed settings
DRIVE_SPEED = 100
SLOW_SPEED = 50
TURN_SPEED = 80
REVERSE_SPEED = 60

# Timing
REVERSE_TIME = 400        # ms
TURN_TIME = 350           # ms

# Debug output
DEBUG = True

# ============== Strategic LIDAR Points ==============
# 20 points for fast scanning (instead of 64)
# Format: (x, y) coordinates in 8x8 grid
STRATEGIC_POINTS = [
    # Row 0 (top/far)
    (0, 0), (2, 0), (5, 0), (7, 0),
    # Row 2
    (0, 2), (2, 2), (5, 2), (7, 2),
    # Row 3-4 (center - more detail)
    (3, 3), (4, 3), (3, 4), (4, 4),
    # Row 5
    (0, 5), (2, 5), (5, 5), (7, 5),
    # Row 7 (bottom/near)
    (0, 7), (2, 7), (5, 7), (7, 7),
]

# ============== Helper Functions ==============

def distance_to_brightness(dist_mm):
    """Convert distance to LED brightness (0-9)
    Closer = brighter
    """
    if dist_mm < 200:
        return 9
    elif dist_mm < 400:
        return 7
    elif dist_mm < 600:
        return 5
    elif dist_mm < 1000:
        return 3
    elif dist_mm < 1500:
        return 1
    else:
        return 0


def pad(s, width):
    """Pad string to width (MicroPython compatible)"""
    s = str(s)
    while len(s) < width:
        s = " " + s
    return s


# ============== Main Class ==============

class VisualObstacleAvoidance:
    """Obstacle avoidance with LIDAR visualization"""

    def __init__(self):
        """Initialize robot and sensors"""
        print("\n" + "="*40)
        print("Visual Obstacle Avoidance")
        print("="*40)

        display.show(Image.HEART)

        # Initialize robot
        print("Init robot...")
        self.robot = MaqueenPlusV3()

        # Initialize LIDAR
        print("Init LIDAR at 0x33...")
        self.laser = LaserMatrix(0x33)

        # Set 8x8 mode (includes 5s delay)
        print("Setting 8x8 mode...")
        self.laser.set_mode(LaserMatrix.MODE_8x8)

        # Initialize LEDs
        self._init_leds()

        # State
        self.running = False
        self.is_moving = False
        self.last_action = "stopped"

        display.show(Image.HAPPY)
        print("Ready! Press A to start")
        print("="*40 + "\n")

    def _init_leds(self):
        """Initialize LEDs: white front, red rear"""
        # Headlights: white
        self.robot.headlights('white', 'white')

        # Underglow: front (0,1) white dim, rear (2,3) red
        self.robot.underglow(0, 50, 50, 50)   # Front left - white dim
        self.robot.underglow(1, 50, 50, 50)   # Front right - white dim
        self.robot.underglow(2, 255, 0, 0)    # Rear left - red
        self.robot.underglow(3, 255, 0, 0)    # Rear right - red

        print("LEDs: white front, red rear")

    def read_strategic_points(self):
        """Read only strategic LIDAR points for fast scanning
        Returns dict with point data
        """
        points = {}
        for x, y in STRATEGIC_POINTS:
            dist = self.laser.read_point(x, y)
            if dist < 0:
                dist = 4000
            points[(x, y)] = dist
        return points

    def read_full_matrix(self):
        """Read full 8x8 LIDAR matrix
        Returns dict with all 64 points
        """
        matrix = self.laser.read_matrix()
        points = {}
        if matrix:
            for y in range(8):
                for x in range(8):
                    idx = y * 8 + x
                    points[(x, y)] = matrix[idx]
        return points

    def calculate_zones(self, points):
        """Calculate 5 navigation zones from LIDAR points
        Returns dict with zone distances
        """
        zones = {
            'far_left': [],
            'left': [],
            'center': [],
            'right': [],
            'far_right': []
        }

        for (x, y), dist in points.items():
            if dist >= 4000:
                continue  # Skip invalid readings

            # Categorize by x coordinate
            if x <= 1:
                zones['far_left'].append(dist)
            elif x <= 2:
                zones['left'].append(dist)
            elif x <= 4:
                zones['center'].append(dist)
            elif x <= 5:
                zones['right'].append(dist)
            else:
                zones['far_right'].append(dist)

        # Calculate averages
        result = {}
        for zone, dists in zones.items():
            if dists:
                result[zone] = sum(dists) // len(dists)
            else:
                result[zone] = 4000

        return result

    def build_display_image(self, points):
        """Build 5x5 Image from LIDAR points
        Maps 8x8 LIDAR grid to 5x5 display
        """
        # Mapping: 5x5 display pixel -> 8x8 LIDAR region
        # Each display pixel averages nearby LIDAR points
        rows = []

        for disp_y in range(5):
            row = ""
            for disp_x in range(5):
                # Map display coords to LIDAR coords
                # Display 5x5 -> LIDAR 8x8 (scale factor ~1.6)
                lidar_x = int(disp_x * 1.6)
                lidar_y = int(disp_y * 1.6)

                # Get distance (check multiple nearby points)
                dists = []
                for dx in range(2):
                    for dy in range(2):
                        lx = min(lidar_x + dx, 7)
                        ly = min(lidar_y + dy, 7)
                        if (lx, ly) in points:
                            dists.append(points[(lx, ly)])

                if dists:
                    avg_dist = sum(dists) // len(dists)
                else:
                    avg_dist = 4000

                brightness = distance_to_brightness(avg_dist)
                row += str(brightness)

            rows.append(row)

        # Create Image from rows
        return Image(":".join(rows))

    def update_display(self, points):
        """Update 5x5 LED matrix with LIDAR visualization"""
        img = self.build_display_image(points)
        display.show(img)

    def find_best_direction(self, zones):
        """Find best direction based on zone distances
        Returns (direction, confidence)
        direction: -1.0 (left) to +1.0 (right), 0 = straight
        confidence: 0.0 (blocked) to 1.0 (clear)
        """
        fl = zones['far_left']
        l = zones['left']
        c = zones['center']
        r = zones['right']
        fr = zones['far_right']

        # Calculate side scores
        left_score = fl * 0.3 + l * 0.7
        right_score = fr * 0.3 + r * 0.7

        # Determine direction and confidence
        if c > SAFE_DISTANCE:
            # Center clear - go straight with minor adjustments
            diff = right_score - left_score
            direction = diff / 3000.0  # Subtle steering
            direction = max(-0.3, min(0.3, direction))
            confidence = min(c / 1000.0, 1.0)
        elif c > STOP_DISTANCE:
            # Approaching obstacle - favor clearer side
            if left_score > right_score:
                direction = -0.5
            else:
                direction = 0.5
            confidence = max(left_score, right_score) / 1000.0
        else:
            # Obstacle close - hard turn
            if left_score > right_score:
                direction = -1.0
            else:
                direction = 1.0
            confidence = max(left_score, right_score) / 1500.0

        confidence = max(0.0, min(1.0, confidence))
        return (direction, confidence)

    def update_status_leds(self, confidence, action):
        """Update underglow LEDs based on confidence/action"""
        if action == "reverse":
            # Red pulsing
            self.robot.underglow(0, 255, 0, 0)
            self.robot.underglow(1, 255, 0, 0)
        elif action == "turn":
            # Blue
            self.robot.underglow(0, 0, 0, 255)
            self.robot.underglow(1, 0, 0, 255)
        elif confidence < 0.3:
            # Low confidence - yellow
            self.robot.underglow(0, 255, 100, 0)
            self.robot.underglow(1, 255, 100, 0)
        elif confidence < 0.6:
            # Medium - cyan
            self.robot.underglow(0, 0, 150, 150)
            self.robot.underglow(1, 0, 150, 150)
        else:
            # High confidence - green
            self.robot.underglow(0, 0, 255, 0)
            self.robot.underglow(1, 0, 255, 0)

    def execute_movement(self, direction, confidence, zones):
        """Execute movement based on direction and zones
        Returns True if robot is moving
        """
        center = zones['center']

        # Emergency: very close obstacle
        if center < CRITICAL_DISTANCE:
            if DEBUG:
                print("EMERGENCY REVERSE!")
            self.robot.drive(-REVERSE_SPEED)
            self.update_status_leds(0, "reverse")
            sleep(REVERSE_TIME)
            # Turn away
            if direction < 0:
                self.robot.turn(-TURN_SPEED)
            else:
                self.robot.turn(TURN_SPEED)
            sleep(TURN_TIME)
            self.last_action = "reverse"
            return True

        # Close obstacle - stop and turn
        if center < STOP_DISTANCE:
            if DEBUG:
                print("TURN " + ("LEFT" if direction < 0 else "RIGHT"))
            if direction < 0:
                self.robot.turn(-TURN_SPEED)
            else:
                self.robot.turn(TURN_SPEED)
            self.update_status_leds(confidence, "turn")
            self.last_action = "turn"
            return True

        # Approaching - slow down
        if center < SLOW_DISTANCE:
            speed = SLOW_SPEED
        else:
            speed = DRIVE_SPEED

        # Apply steering
        if abs(direction) > 0.3:
            # Differential steering
            if direction < 0:
                left = int(speed * (1 + direction))
                right = speed
            else:
                left = speed
                right = int(speed * (1 - direction))
            self.robot.motors(left, right)
            self.last_action = "steer"
        else:
            # Straight
            self.robot.drive(speed)
            self.last_action = "drive"

        self.update_status_leds(confidence, self.last_action)
        return True

    def check_buttons(self):
        """Check button presses"""
        if button_a.was_pressed():
            self.running = not self.running
            if self.running:
                print("STARTED")
                display.show(Image.YES)
                sleep(300)
            else:
                print("STOPPED")
                self.robot.stop()
                self._init_leds()  # Reset LEDs
                display.show(Image.NO)
                sleep(300)

        if button_b.was_pressed():
            print("EMERGENCY STOP")
            self.running = False
            self.robot.stop()
            self._init_leds()
            display.show(Image.SKULL)
            sleep(500)

    def run(self):
        """Main loop"""
        print("Main loop started")
        print("A=Start/Stop, B=Emergency")

        while True:
            # Check buttons first
            self.check_buttons()

            if not self.running:
                # Stopped - detailed scan for visualization
                points = self.read_full_matrix()
                if points:
                    self.update_display(points)
                sleep(100)
                continue

            # Running - adaptive scanning
            if self.is_moving and self.last_action not in ["reverse", "turn"]:
                # Fast scan when moving smoothly
                points = self.read_strategic_points()
            else:
                # Full scan when stopped or maneuvering
                points = self.read_full_matrix()

            if not points:
                print("LIDAR read error")
                self.robot.stop()
                continue

            # Update display
            self.update_display(points)

            # Calculate zones
            zones = self.calculate_zones(points)

            if DEBUG:
                msg = "FL:" + pad(zones['far_left'], 4)
                msg += " L:" + pad(zones['left'], 4)
                msg += " C:" + pad(zones['center'], 4)
                msg += " R:" + pad(zones['right'], 4)
                msg += " FR:" + pad(zones['far_right'], 4)
                print(msg)

            # Find best direction
            direction, confidence = self.find_best_direction(zones)

            # Execute movement
            self.is_moving = self.execute_movement(direction, confidence, zones)


# ============== Entry Point ==============

def main():
    """Entry point"""
    try:
        controller = VisualObstacleAvoidance()
        controller.run()
    except Exception as e:
        print("ERROR: " + str(e))
        display.show(Image.SAD)
        import sys
        sys.print_exception(e)


if __name__ == "__main__":
    main()
