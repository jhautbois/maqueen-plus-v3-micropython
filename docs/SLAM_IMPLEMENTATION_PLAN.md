# Plan d'implementation SLAM leger pour micro:bit V2

## Vue d'ensemble

Objectif : Ajouter une memoire spatiale au robot pour ameliorer la navigation.

### Architecture cible

```
┌─────────────────────────────────────────────────────────────┐
│                        main.py                               │
│  NavV2 - Boucle principale                                   │
└─────────────────┬───────────────────────────────────────────┘
                  │
    ┌─────────────┼─────────────┬─────────────────┐
    ▼             ▼             ▼                 ▼
┌────────┐  ┌──────────┐  ┌───────────┐  ┌──────────────┐
│ LIDAR  │  │ Odometry │  │ MiniGrid  │  │ GapFinder    │
│ 8x8    │  │ encoders │  │ 32x32     │  │ anti-oscill  │
└────────┘  └──────────┘  └───────────┘  └──────────────┘
    │             │             │
    └─────────────┴─────────────┘
                  │
                  ▼
         ┌───────────────┐
         │ DistanceMap   │
         │ 16 secteurs   │
         │ (existant)    │
         └───────────────┘
```

## Phase 1 : Odometrie (Priorite haute)

### Fichier : `src/odometry.py`

**Objectif** : Suivre position (x, y) et heading du robot via encodeurs.

**Registres STM8** :
- `0x04` : Encodeur gauche (format a valider)
- `0x06` : Encodeur droite

**Specifications Maqueen Plus V3** :
- Roues : 42mm diametre → 132mm circonference
- Encodeurs : 4 ticks/revolution magnetique
- Ecart roues : ~80mm
- Resolution : ~33mm par tick

**Implementation** :

```python
"""Odometrie simple pour Maqueen Plus V3"""
from microbit import i2c
from micropython import const

_STM8 = const(0x10)
_REG_ENC_L = const(0x04)
_REG_ENC_R = const(0x06)
_TICKS_TO_MM = const(33)      # mm par tick
_WHEEL_BASE = const(80)       # mm entre roues

# Directions (8 points cardinaux) - dx, dy multiplies par 181
_DIR_X = (0, 128, 181, 128, 0, -128, -181, -128)
_DIR_Y = (181, 128, 0, -128, -181, -128, 0, 128)

class Odometry:
    def __init__(self):
        self._last_l = 0
        self._last_r = 0
        self._x = 0           # Position en mm
        self._y = 0
        self._heading = 0     # 0-7 (N, NE, E, SE, S, SW, W, NW)
        self._heading_deg = 0 # 0-359 degres
        self._initialized = False

    def _read_encoder(self, reg):
        """Lire valeur encodeur 16-bit"""
        try:
            i2c.write(_STM8, bytes([reg]))
            d = i2c.read(_STM8, 2)
            return d[0] | (d[1] << 8)
        except:
            return 0

    def init(self):
        """Initialiser avec valeurs actuelles"""
        self._last_l = self._read_encoder(_REG_ENC_L)
        self._last_r = self._read_encoder(_REG_ENC_R)
        self._initialized = True

    def update(self):
        """Mettre a jour position. Appeler chaque cycle."""
        if not self._initialized:
            self.init()
            return 0, 0

        # Lire encodeurs
        enc_l = self._read_encoder(_REG_ENC_L)
        enc_r = self._read_encoder(_REG_ENC_R)

        # Calculer deltas (gerer wraparound 16-bit)
        dl = enc_l - self._last_l
        dr = enc_r - self._last_r
        if dl > 32768: dl -= 65536
        if dl < -32768: dl += 65536
        if dr > 32768: dr -= 65536
        if dr < -32768: dr += 65536

        self._last_l = enc_l
        self._last_r = enc_r

        # Convertir en mm
        mm_l = dl * _TICKS_TO_MM
        mm_r = dr * _TICKS_TO_MM

        # Deplacement lineaire et angulaire
        linear = (mm_l + mm_r) // 2
        # angular en degres : (dr - dl) * 57.3 / wheel_base
        angular = (mm_r - mm_l) * 57 // _WHEEL_BASE

        # Mettre a jour heading
        self._heading_deg = (self._heading_deg + angular) % 360
        self._heading = self._heading_deg // 45  # 0-7

        # Mettre a jour position
        if abs(linear) > 5:  # Seuil bruit
            dx = linear * _DIR_X[self._heading] // 181
            dy = linear * _DIR_Y[self._heading] // 181
            self._x += dx
            self._y += dy

        return linear, angular

    def get_pose(self):
        """Retourne (x_mm, y_mm, heading_deg)"""
        return (self._x, self._y, self._heading_deg)

    def get_heading_index(self):
        """Retourne heading 0-7 pour rotation carte"""
        return self._heading

    def reset(self):
        """Reset position a origine"""
        self._x = 0
        self._y = 0
        self._heading = 0
        self._heading_deg = 0
```

**Tests requis** :
1. Verifier format registres 0x04/0x06 (little-endian 16-bit?)
2. Valider direction des deltas (positif = avant?)
3. Calibrer TICKS_TO_MM avec mesure reelle

---

## Phase 2 : Grille d'occupation (Priorite moyenne)

### Fichier : `src/mini_grid.py`

**Objectif** : Memoriser obstacles pour eviter de re-explorer.

**Specifications** :
- Taille : 32x32 cellules = 1024 cellules
- Stockage : 2 bits/cellule = 256 bytes
- Resolution : 12cm par cellule
- Couverture : 3.84m x 3.84m
- Etats : 0=inconnu, 1=libre, 2=occupe, 3=reserve

**Implementation** :

```python
"""Grille d'occupation compacte"""
from micropython import const

_SIZE = const(32)
_CELL_MM = const(120)  # 12cm
_UNKNOWN = const(0)
_FREE = const(1)
_OCCUPIED = const(2)

# Tables sin/cos * 256 pour 8 colonnes LIDAR
# Angles: -26.25 a +26.25 degres
_SIN = (-113, -82, -50, -17, 17, 50, 82, 113)
_COS = (229, 243, 251, 256, 256, 251, 243, 229)

class MiniGrid:
    def __init__(self):
        self._data = bytearray(256)  # 1024 cells * 2 bits
        self._ox = _SIZE // 2  # Origine = centre grille
        self._oy = _SIZE // 2

    def _pack_idx(self, x, y):
        """Retourne (byte_index, bit_offset)"""
        cell = y * _SIZE + x
        return cell >> 2, (cell & 3) << 1

    def get(self, x, y):
        """Lire cellule (coordonnees grille)"""
        if x < 0 or x >= _SIZE or y < 0 or y >= _SIZE:
            return _UNKNOWN
        bi, bo = self._pack_idx(x, y)
        return (self._data[bi] >> bo) & 3

    def set(self, x, y, val):
        """Ecrire cellule"""
        if x < 0 or x >= _SIZE or y < 0 or y >= _SIZE:
            return
        bi, bo = self._pack_idx(x, y)
        self._data[bi] = (self._data[bi] & ~(3 << bo)) | ((val & 3) << bo)

    def world_to_grid(self, x_mm, y_mm):
        """Convertir position monde en coordonnees grille"""
        gx = self._ox + x_mm // _CELL_MM
        gy = self._oy + y_mm // _CELL_MM
        return (gx, gy)

    def update_from_lidar(self, cols, robot_x, robot_y, heading_idx):
        """Mettre a jour grille depuis LIDAR 8 colonnes"""
        rx, ry = self.world_to_grid(robot_x, robot_y)

        for i, d in enumerate(cols):
            if d <= 50 or d >= 3000:
                continue

            # Distance en cellules
            cells = d // _CELL_MM
            if cells < 1 or cells > 15:
                continue

            # Direction avec rotation heading
            # Simplifie: on ignore heading pour l'instant
            sin_a = _SIN[i]
            cos_a = _COS[i]

            # Marquer obstacle
            ox = rx + (cells * sin_a) // 256
            oy = ry + (cells * cos_a) // 256
            self.set(ox, oy, _OCCUPIED)

            # Optionnel: marquer libre le long du rayon
            # for r in range(1, cells):
            #     fx = rx + (r * sin_a) // 256
            #     fy = ry + (r * cos_a) // 256
            #     if self.get(fx, fy) == _UNKNOWN:
            #         self.set(fx, fy, _FREE)

    def is_blocked(self, robot_x, robot_y, dx, dy):
        """Verifier si direction relative est bloquee"""
        rx, ry = self.world_to_grid(robot_x, robot_y)
        return self.get(rx + dx, ry + dy) == _OCCUPIED

    def count_obstacles(self):
        """Compter cellules occupees (debug)"""
        count = 0
        for b in self._data:
            for i in range(4):
                if ((b >> (i * 2)) & 3) == _OCCUPIED:
                    count += 1
        return count

    def clear(self):
        """Effacer grille"""
        for i in range(len(self._data)):
            self._data[i] = 0
```

---

## Phase 3 : Integration dans NavV2

### Modifications `main.py`

```python
# Imports additionnels
from odometry import Odometry
from mini_grid import MiniGrid

class NavV2:
    def __init__(self):
        # ... code existant ...

        # Nouveaux modules
        self.odo = Odometry()
        self.grid = MiniGrid()

        # Compteur pour updates periodiques
        self._grid_update_count = 0

    def run(self):
        while True:
            self.btn()

            if self.on and self.check_danger():
                continue

            cols = self.read_lidar_cols()

            if not self.on:
                x, y, z = accelerometer.get_values()
                print("XYZ: " + str(x) + " " + str(y) + " " + str(z))
                sleep(1000)
                continue

            if not cols:
                self.r.stop()
                continue

            # === NOUVEAU : Mise a jour odometrie ===
            linear, angular = self.odo.update()

            # Alimenter heading a distance_map si utilise
            # self.dist_map.rotate(angular)

            # === NOUVEAU : Mise a jour grille periodique ===
            self._grid_update_count += 1
            if self._grid_update_count >= 5:  # Toutes les 5 iterations
                self._grid_update_count = 0
                x, y, _ = self.odo.get_pose()
                self.grid.update_from_lidar(cols, x, y, self.odo.get_heading_index())

            # Navigation existante
            self.nav(cols)
            sleep(_CYCLE_MS)
```

### Utilisation de la grille pour navigation

```python
def nav(self, cols):
    # ... code existant ...

    # Avant de decider direction, verifier grille
    x, y, _ = self.odo.get_pose()

    # Si gap_finder suggere gauche mais grille dit obstacle
    if gap_angle < -15:
        if self.grid.is_blocked(x, y, -1, 1):
            # Forcer autre direction
            gap_angle = 15

    # ... reste du code ...
```

---

## Phase 4 : Tests et validation

### Etape 1 : Valider encodeurs

```python
# Script de test encodeurs
from microbit import i2c, sleep, display, Image

def test_encoders():
    display.show(Image.ARROW_N)
    last_l = 0
    last_r = 0

    while True:
        i2c.write(0x10, bytes([0x04]))
        dl = i2c.read(0x10, 2)
        enc_l = dl[0] | (dl[1] << 8)

        i2c.write(0x10, bytes([0x06]))
        dr = i2c.read(0x10, 2)
        enc_r = dr[0] | (dr[1] << 8)

        if enc_l != last_l or enc_r != last_r:
            print("L:" + str(enc_l) + " R:" + str(enc_r))
            last_l = enc_l
            last_r = enc_r

        sleep(100)

test_encoders()
```

**Procedure** :
1. Flasher script test
2. Pousser robot manuellement vers l'avant
3. Verifier que L et R augmentent
4. Pousser vers l'arriere → doivent diminuer
5. Tourner sur place → L et R opposes

### Etape 2 : Calibrer odometrie

1. Marquer position depart
2. Faire avancer robot 1 metre (mesure)
3. Comparer avec `odo.get_pose()[1]` (y en mm)
4. Ajuster `_TICKS_TO_MM` si necessaire

### Etape 3 : Valider grille

1. Demarrer robot dans piece vide
2. Laisser explorer 30 secondes
3. Afficher `grid.count_obstacles()`
4. Devrait correspondre approximativement aux murs detectes

---

## Budget memoire final

| Module | Code | Donnees | Total |
|--------|------|---------|-------|
| odometry.py | ~400 bytes | ~20 bytes | ~420 bytes |
| mini_grid.py | ~500 bytes | 256 bytes | ~756 bytes |
| Integration main.py | ~200 bytes | - | ~200 bytes |
| **Total** | | | **~1.4 KB** |

Reste largement dans le budget de 50KB disponible.

---

## Ordre d'implementation

1. **Phase 1A** : Script test encodeurs (valider registres)
2. **Phase 1B** : Module odometry.py
3. **Phase 1C** : Integration basique (juste logging position)
4. **Phase 2A** : Module mini_grid.py
5. **Phase 2B** : Integration grille avec LIDAR
6. **Phase 3** : Utiliser grille pour navigation
7. **Phase 4** : Optimisations (rotation heading, ray casting)

---

## Risques et mitigations

| Risque | Impact | Mitigation |
|--------|--------|------------|
| Format encodeurs incorrect | Bloquant | Script test d'abord |
| Derive odometrie excessive | Degrade qualite carte | Reset periodique, utiliser LIDAR comme reference principale |
| Memoire insuffisante | Crash | Surveiller gc.mem_free() |
| Performance (cycle trop long) | Navigation saccadee | Mise a jour grille moins frequente |

---

## Extensions futures (post-implementation)

1. **Rotation grille avec heading** : Plus precis mais plus complexe
2. **Expiration cellules** : Oublier vieilles donnees
3. **Landmarks** : Detecter coins pour meilleure localisation
4. **Sauvegarde grille** : Persister entre sessions (filesystem)
