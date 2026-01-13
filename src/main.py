"""Navigation V2 - Anti-oscillation et echappement de coins"""
from microbit import display, Image, button_a, button_b, sleep, running_time, accelerometer
from micropython import const, mem_info
import gc
import music
from maqueen_plus_v3 import MaqueenPlusV3
from laser_matrix import LaserMatrix
from gap_finder import GapFinder
from fall_detector import FallDetector
from micro_slam import MicroSLAM

# Seuils (mm)
_CRIT = const(120)
_STOP = const(200)
_SLOW = const(350)
_SAFE = const(500)

# Vitesses
_SPD_MIN = const(80)
_SPD_MAX = const(160)
_TSPD = const(90)
_ESCAPE_SPD = const(80)

# Timing
_CYCLE_MS = const(80)
_ESCAPE_COOLDOWN = const(2000)


class NavV2:
    def __init__(self):
        display.show(Image.HEART)
        self.r = MaqueenPlusV3()
        self.l = LaserMatrix(0x33)
        self.l.set_mode(8)

        self.gap = GapFinder()

        self.fall = FallDetector(enable_cliff=True)
        self.danger_lockout = 0

        # SLAM (odometrie + grille 8x8)
        self.slam = MicroSLAM()

        # Test LEDs
        print("Nav V2")
        self.r.headlights('red', 'red')
        sleep(300)
        self.r.headlights('green', 'green')
        sleep(300)
        self.r.headlights('white', 'white')
        self.r.underglow('all', 0, 255, 0)

        # Etat
        self.on = False
        self.last_escape = 0
        self.escape_count = 0
        self.ml = 0  # Derniere commande moteur gauche
        self.mr = 0  # Derniere commande moteur droite

        display.show(Image.HAPPY)
        # Diagnostic memoire
        gc.collect()
        free = gc.mem_free()
        alloc = gc.mem_alloc()
        print("RAM: " + str(alloc) + "/" + str(alloc + free) + " (" + str(free) + " free)")
        print("Ready A=go")

    def read_lidar_cols(self):
        """Lire LIDAR et retourner 8 colonnes (min par colonne)"""
        m = self.l.read_matrix()
        if not m:
            return None

        cols = []
        for x in range(8):
            col_min = 4000
            for y in range(8):
                d = m[y * 8 + x]
                if 0 < d < col_min:
                    col_min = d
            cols.append(col_min)
        return cols

    def mot(self, l, r):
        """Wrapper moteur avec tracking"""
        self.ml = l
        self.mr = r
        self.r.motors(l, r)

    def leds(self, state):
        """Mettre a jour LEDs selon etat"""
        if state == "danger":
            self.r.underglow('all', 255, 0, 255)  # Magenta = danger
            self.r.headlights('purple', 'purple')
        elif state == "escape":
            self.r.underglow('all', 255, 0, 0)
            self.r.headlights('red', 'red')
        elif state == "turn":
            self.r.underglow('all', 0, 0, 255)
            self.r.headlights('blue', 'blue')
        elif state == "slow":
            self.r.underglow('all', 255, 150, 0)
            self.r.headlights('yellow', 'yellow')
        else:  # drive
            self.r.underglow('all', 0, 255, 0)
            self.r.headlights('white', 'white')

    def escape_corner(self, gap_angle):
        """Echappement de coin avec detection obstacles"""
        self.last_escape = running_time()
        self.escape_count += 1
        self.leds("escape")
        music.pitch(600, 100)

        # 1. Reculer brievement (pas de LIDAR arriere)
        self.mot(-_ESCAPE_SPD, -_ESCAPE_SPD)
        sleep(400)
        self.mot(0, 0)
        sleep(50)

        # 2. Tourner vers le passage
        if abs(gap_angle) < 15:
            angle = 60 if self.escape_count % 2 == 0 else -60
        else:
            angle = gap_angle * 1.5
            angle = max(-90, min(90, angle))

        if angle > 0:
            self.mot(-_TSPD, _TSPD)
        else:
            self.mot(_TSPD, -_TSPD)
        sleep(int(abs(angle) * 5))
        self.mot(0, 0)
        sleep(50)

        # 3. Avancer avec detection d'obstacles
        self.mot(_SPD_MIN, _SPD_MIN)
        for _ in range(6):  # 6 x 50ms = 300ms max
            sleep(50)
            cols = self.read_lidar_cols()
            if cols:
                center = (cols[3] + cols[4]) // 2
                if center < _CRIT:
                    break
        self.mot(0, 0)
        self.gap.reset()

    def nav(self, cols):
        """Navigation principale"""
        # Utiliser colonnes LIDAR directement (8 colonnes = 8 secteurs)
        # cols[0]=gauche, cols[7]=droite, cols[3-4]=centre

        # Trouver meilleur passage avec les colonnes LIDAR
        gap_offset, gap_dist, gap_angle = self.gap.find_gap(cols)

        # Distance centre (moyenne colonnes 3,4)
        center = (cols[3] + cols[4]) // 2

        # Debug occasionnel
        # if self.cycle_count == 0:
        #     print("C:" + str(center) + " G:" + str(gap_angle))

        # Critique: echappement si tres proche
        if center < _CRIT:
            # Verifier cooldown
            if running_time() - self.last_escape > _ESCAPE_COOLDOWN:
                self.escape_corner(gap_angle)
            else:
                # Juste reculer
                self.mot(-_ESCAPE_SPD, -_ESCAPE_SPD)
                self.leds("escape")
            return

        # Bloque: tourner sur place
        if center < _STOP:
            # Comparer gauche vs droite pour decider
            left_avg = (cols[0] + cols[1] + cols[2]) // 3
            right_avg = (cols[5] + cols[6] + cols[7]) // 3

            if abs(gap_angle) < 10:
                # Passage devant mais bloque, choisir un cote
                gap_angle = -30 if left_avg > right_avg else 30

            if gap_angle < 0:
                self.mot(_TSPD, -_TSPD)
            else:
                self.mot(-_TSPD, _TSPD)
            self.leds("turn")
            return

        # Calculer vitesse selon distance
        if center > _SAFE:
            spd = _SPD_MAX
        elif center > _SLOW:
            ratio = (center - _SLOW) / (_SAFE - _SLOW)
            spd = _SPD_MIN + int((_SPD_MAX - _SPD_MIN) * ratio)
        else:
            spd = _SPD_MIN

        # Preferer aller droit si le centre est suffisamment libre
        if center > _SLOW and abs(gap_angle) < 15:
            # Centre OK et passage presque droit -> aller tout droit
            self.mot(spd, spd)
            self.leds("drive")
            return

        # Appliquer direction differentielle
        turn_factor = gap_angle / 30.0  # Normaliser (30 deg = max)
        turn_factor = max(-0.7, min(0.7, turn_factor))  # Limiter

        if abs(turn_factor) > 0.1:
            if turn_factor < 0:
                # Tourner gauche
                left_spd = int(spd * (1 + turn_factor))
                right_spd = spd
            else:
                # Tourner droite
                left_spd = spd
                right_spd = int(spd * (1 - turn_factor))
            self.mot(left_spd, right_spd)
            self.leds("slow" if center < _SLOW else "drive")
        else:
            # Tout droit
            self.mot(spd, spd)
            self.leds("drive")

    def check_danger(self):
        """Check for tilt/freefall/cliff danger. Returns True if danger detected."""
        # Use fast check (accelerometer only) to save I2C bandwidth
        # Full check with cliff detection every 4th call
        danger, reason = self.fall.check_fast()

        if danger:
            self.mot(0, 0)
            self.leds("danger")

            # Show reason on display
            if reason == "freefall":
                display.show(Image.ARROW_S)
            elif reason == "pickup":
                display.show(Image.SURPRISED)
            else:
                display.show(Image.CONFUSED)

            print("DANGER: " + reason)
            self.on = False
            self.danger_lockout = running_time() + 2000  # 2s lockout
            return True

        return False

    def btn(self):
        """Gerer boutons"""
        if button_a.was_pressed():
            # Check lockout after danger
            if running_time() < self.danger_lockout:
                music.pitch(200, 100)  # Low beep = still locked out
                return

            self.on = not self.on
            if self.on:
                display.clear()
                self.escape_count = 0
                self.slam.reset()
            else:
                self.mot(0, 0)
                display.show(Image.NO)
            sleep(300)

        if button_b.was_pressed():
            self.on = False
            self.mot(0, 0)
            display.show(Image.SKULL)
            sleep(500)

    def run(self):
        """Boucle principale"""
        while True:
            self.btn()

            # Safety check FIRST - before any movement
            if self.on and self.check_danger():
                continue

            cols = self.read_lidar_cols()

            if not self.on:
                # Diagnostic SLAM
                px, py, ph = self.slam.pose()
                print("P:" + str(px) + "," + str(py) + " H:" + str(ph))
                sleep(2000)
                continue

            if not cols:
                self.mot(0, 0)
                continue

            # Mise a jour SLAM avec commandes moteur
            self.slam.update(self.ml, self.mr)
            self.slam.lidar(cols)

            # Naviguer
            self.nav(cols)
            sleep(_CYCLE_MS)


try:
    NavV2().run()
except Exception as e:
    print("ERR:" + str(e))
    display.show(Image.SAD)
