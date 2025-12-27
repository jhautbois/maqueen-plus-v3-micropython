"""Obstacle Avoidance with Polar Map Navigation"""
from microbit import display, Image, button_a, button_b, sleep, running_time
import music
from maqueen_plus_v3 import MaqueenPlusV3
from laser_matrix import LaserMatrix

# Thresholds (mm)
CRIT=120; STOP=200; SLOW=350; SAFE=500

# Speed settings
SPD_MIN=80; SPD_MAX=160; TSPD=90; RSPD=60

# Polar map: 8 sectors, each ~7.5° (total FOV 60°)
# Sector angles: -26, -19, -11, -4, 4, 11, 19, 26 degrees
SECT_ANG = [-26, -19, -11, -4, 4, 11, 19, 26]

def d2b(d):
    """Distance to brightness for display"""
    if d<200: return 9
    if d<400: return 7
    if d<600: return 5
    if d<1000: return 3
    if d<1500: return 1
    return 0

class App:
    def __init__(self):
        display.show(Image.HEART)
        self.r = MaqueenPlusV3()
        self.l = LaserMatrix(0x33)
        self.l.set_mode(8)
        # LED test
        print("Polar Nav v1.0")
        self.r.headlights('red','red'); sleep(300)
        self.r.headlights('green','green'); sleep(300)
        self.r.headlights('white','white')
        self.r.underglow('all',0,255,0)
        # State
        self.on = False
        display.show(Image.HAPPY)
        print("Ready A=go")

    def read_matrix(self):
        """Read full 8x8 LIDAR matrix"""
        m = self.l.read_matrix()
        p = {}
        if m:
            for y in range(8):
                for x in range(8):
                    d = m[y*8+x]
                    p[(x,y)] = d if 0 < d < 4000 else 4000
        return p

    def polar(self, p):
        """Convert 8x8 matrix to 8 polar sectors (min distance per sector)"""
        # Each column maps to a sector (0-7)
        # For each sector, find minimum distance (closest obstacle)
        sectors = [4000] * 8
        for x in range(8):
            col_min = 4000
            for y in range(8):
                d = p.get((x, y), 4000)
                if d < col_min:
                    col_min = d
            sectors[x] = col_min
        return sectors

    def find_gap(self, sectors):
        """Find best gap (direction with most space)
        Returns: (sector_index, distance, gap_width)
        """
        # Find the sector with maximum distance
        best_idx = 3  # Default: center-left
        best_dist = 0

        # Check each sector and its neighbors for a "gap"
        for i in range(8):
            # Average with neighbors for smoother detection
            d = sectors[i]
            if i > 0:
                d = (d + sectors[i-1]) // 2
            if i < 7:
                d = (d + sectors[i+1]) // 2

            if d > best_dist:
                best_dist = d
                best_idx = i

        # Calculate gap width (how many adjacent sectors are also clear)
        width = 1
        thresh = best_dist * 0.7  # 70% of best distance
        # Check left
        for i in range(best_idx - 1, -1, -1):
            if sectors[i] >= thresh:
                width += 1
            else:
                break
        # Check right
        for i in range(best_idx + 1, 8):
            if sectors[i] >= thresh:
                width += 1
            else:
                break

        return (best_idx, best_dist, width)

    def sector_to_angle(self, idx):
        """Convert sector index to angle in degrees"""
        # Sector 0 = left (-26°), Sector 7 = right (+26°)
        # Center is between 3 and 4 (around 0°)
        return SECT_ANG[idx] if idx < 8 else 0

    def disp(self, p):
        """Display LIDAR on 5x5 matrix"""
        rows = []
        for dy in range(5):
            row = ""
            for dx in range(5):
                lx, ly = int((4-dx)*1.6), int((4-dy)*1.6)
                ds = []
                for i in range(2):
                    for j in range(2):
                        k = (min(lx+i,7), min(ly+j,7))
                        if k in p:
                            ds.append(p[k])
                row += str(d2b(sum(ds)//len(ds) if ds else 4000))
            rows.append(row)
        display.show(Image(":".join(rows)))

    def leds(self, state):
        """Update LEDs based on state"""
        if state == "rev":
            self.r.underglow('all', 255, 0, 0)
        elif state == "turn":
            self.r.underglow('all', 0, 0, 255)
        elif state == "slow":
            self.r.underglow('all', 255, 150, 0)
        else:  # drive
            self.r.underglow('all', 0, 255, 0)

    def nav(self, sectors):
        """Navigate using polar map"""
        # Get center distance (average of sectors 3,4)
        center = (sectors[3] + sectors[4]) // 2

        # Find best gap
        gap_idx, gap_dist, gap_width = self.find_gap(sectors)
        gap_angle = self.sector_to_angle(gap_idx)

        # Debug
        # print("C:" + str(center) + " Gap:" + str(gap_idx) + " A:" + str(gap_angle) + " D:" + str(gap_dist))

        # Critical: reverse if too close
        if center < CRIT:
            music.pitch(800, 50)
            self.r.drive(-RSPD)
            self.leds("rev")
            sleep(300)
            # Turn toward gap
            if gap_angle < 0:
                self.r.turn(-TSPD)
            else:
                self.r.turn(TSPD)
            sleep(250)
            return

        # Calculate turn intensity based on gap angle
        # angle: -26 to +26, map to turn factor -1 to +1
        turn_factor = gap_angle / 30.0  # Normalize to -0.87 to +0.87

        # If center is blocked, need stronger turn
        if center < STOP:
            # Pure rotation toward gap
            if abs(turn_factor) < 0.2:
                # Gap is roughly ahead but center blocked
                # Check which side is better
                left_avg = (sectors[0] + sectors[1] + sectors[2]) // 3
                right_avg = (sectors[5] + sectors[6] + sectors[7]) // 3
                turn_factor = -0.7 if left_avg > right_avg else 0.7

            # Rotate in place
            turn_speed = int(TSPD * (0.5 + abs(turn_factor) * 0.5))
            if turn_factor < 0:
                self.r.turn(-turn_speed)
            else:
                self.r.turn(turn_speed)
            self.leds("turn")
            return

        # Calculate speed based on distance
        if center > SAFE:
            spd = SPD_MAX
        elif center > SLOW:
            spd = SPD_MIN + int((SPD_MAX - SPD_MIN) * (center - SLOW) / (SAFE - SLOW))
        else:
            spd = SPD_MIN

        # Apply differential steering based on gap angle
        if abs(turn_factor) > 0.15:
            # Steer toward gap
            if turn_factor < 0:
                # Turn left: slow left wheel
                left_spd = int(spd * (1 + turn_factor))  # turn_factor is negative
                right_spd = spd
            else:
                # Turn right: slow right wheel
                left_spd = spd
                right_spd = int(spd * (1 - turn_factor))
            self.r.motors(left_spd, right_spd)
            self.leds("slow" if center < SLOW else "drive")
        else:
            # Go straight
            self.r.drive(spd)
            self.leds("drive")

    def btn(self):
        """Handle buttons (A=start/stop, B=emergency stop)"""
        if button_a.was_pressed():
            self.on = not self.on
            if self.on:
                display.clear()
            else:
                self.r.stop()
                display.show(Image.NO)
            sleep(300)

        if button_b.was_pressed():
            self.on = False
            self.r.stop()
            display.show(Image.SKULL)
            sleep(500)

    def run(self):
        """Main loop"""
        while True:
            self.btn()

            if not self.on:
                # Stopped: show LIDAR visualization
                p = self.read_matrix()
                if p:
                    self.disp(p)
                sleep(100)
                continue

            # Running: read LIDAR and navigate (no display update)
            p = self.read_matrix()
            if not p:
                self.r.stop()
                continue

            # Convert to polar and navigate
            sectors = self.polar(p)
            self.nav(sectors)

try:
    App().run()
except Exception as e:
    print("ERR:" + str(e))
    display.show(Image.SAD)
