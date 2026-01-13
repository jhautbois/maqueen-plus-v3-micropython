# Cartographie 2D / SLAM sur micro:bit V2

## Resume

Implementer un SLAM complet sur micro:bit V2 avec ~50KB RAM est extremement difficile mais des approches simplifiees sont possibles.

## Budget Memoire

| Composant | Memoire |
|-----------|---------|
| MicroPython core | ~60KB (fixe) |
| Code navigation actuel | ~15KB |
| **Disponible pour cartographie** | **~50KB** |

## Ce qui est realisable

### 1. Carte polaire (DEJA IMPLEMENTE)
- `distance_map.py` : 16 secteurs, 64 bytes
- Stocke distances 360 degres autour du robot
- Vieillit les donnees, tourne avec le robot

### 2. Odometrie simple (~300 bytes)
- Registres encodeurs : `0x04` (gauche), `0x06` (droite)
- Precision : ~10-20% de derive par metre
- Utile pour estimer le heading

```python
from microbit import i2c
from micropython import const

_STM8 = const(0x10)
_ENC_L = const(0x04)
_ENC_R = const(0x06)
_MM_PER_TICK = const(33)  # 42mm roue, 4 ticks/tour

class SimpleOdometry:
    def __init__(self):
        self._enc_l = 0
        self._enc_r = 0
        self._x = 0  # mm
        self._y = 0
        self._heading = 0  # degres

    def _read_enc(self, reg):
        try:
            i2c.write(_STM8, bytes([reg]))
            d = i2c.read(_STM8, 2)
            return d[0] | (d[1] << 8)
        except:
            return 0

    def update(self):
        el = self._read_enc(_ENC_L)
        er = self._read_enc(_ENC_R)

        dl = (el - self._enc_l) & 0xFFFF
        dr = (er - self._enc_r) & 0xFFFF
        if dl > 32768: dl -= 65536
        if dr > 32768: dr -= 65536

        self._enc_l = el
        self._enc_r = er

        mm_l = dl * _MM_PER_TICK
        mm_r = dr * _MM_PER_TICK

        linear = (mm_l + mm_r) // 2
        angular = (mm_r - mm_l) * 57 // 80  # 80mm entre roues

        self._heading = (self._heading + angular) % 360
        return linear, angular
```

### 3. Mini grille d'occupation (~256 bytes)
- 32x32 cellules, 2 bits par cellule
- Resolution 12cm, couverture 3.8m x 3.8m
- Etats : inconnu (0), libre (1), occupe (2)

```python
from micropython import const

_GRID_SIZE = const(32)
_CELL_CM = const(12)

class MiniGrid:
    def __init__(self):
        # 1024 cellules * 2 bits = 256 bytes
        self._data = bytearray(256)
        self._rx = 16  # Position robot
        self._ry = 16

    def _idx(self, x, y):
        cell = y * _GRID_SIZE + x
        return cell >> 2, (cell & 3) << 1

    def get(self, x, y):
        if x < 0 or x >= _GRID_SIZE or y < 0 or y >= _GRID_SIZE:
            return 0
        byte_i, bit_off = self._idx(x, y)
        return (self._data[byte_i] >> bit_off) & 3

    def set(self, x, y, val):
        if x < 0 or x >= _GRID_SIZE or y < 0 or y >= _GRID_SIZE:
            return
        byte_i, bit_off = self._idx(x, y)
        mask = ~(3 << bit_off) & 0xFF
        self._data[byte_i] = (self._data[byte_i] & mask) | ((val & 3) << bit_off)
```

### 4. Detection de landmarks (~164 bytes)
- 16 landmarks max (coins, passages)
- Coordonnees polaires (distance, angle)
- Age pour expiration

## Ce qui N'EST PAS realisable

| Fonctionnalite | Raison |
|----------------|--------|
| SLAM complet avec fermeture de boucle | Necessite optimisation de graphe, >10MB RAM |
| SLAM visuel | Pas de camera, necessite GPU |
| Cartes haute resolution (>64x64) | Contraintes memoire |
| SLAM par filtre particulaire | Chaque particule necessite sa propre carte |
| Scan matching ICP | Necessite operations matricielles |

## Architecture recommandee

```
Priorite 1 : Carte polaire (existe)     - 64 bytes
Priorite 2 : Odometrie simple           - ~300 bytes
Priorite 3 : Mini grille 32x32          - 256 bytes
Priorite 4 : Landmarks                  - 64 bytes
                                        -----------
                              Total :   ~684 bytes
```

## Tables trigonometriques pre-calculees

Pour eviter math.sin/cos (couteux), utiliser des tables :

```python
# sin*256 et cos*256 pour les 8 colonnes LIDAR
# Angles : -26.25, -18.75, -11.25, -3.75, 3.75, 11.25, 18.75, 26.25 deg
_SIN8 = (-113, -82, -50, -17, 17, 50, 82, 113)
_COS8 = (229, 243, 251, 256, 256, 251, 243, 229)
```

## Limites de l'odometrie

- Glissement des roues sur surfaces lisses
- Resolution encodeur faible (4 ticks/tour = 33mm/tick)
- Derive cumulative rapide
- Utilisable uniquement pour :
  - Estimation grossiere du heading
  - Suivi court terme dans une session
  - PAS pour localisation long terme

## Prochaines etapes

1. **Tester les registres encodeurs** (0x04, 0x06) - format non valide
2. **Implementer odometrie** pour ameliorer rotation carte polaire
3. **Optionnel** : Mini grille pour memoire obstacles moyen terme

## Sources

- [MicroPython micro:bit](https://microbit-micropython.readthedocs.io/)
- [NanoSLAM](https://arxiv.org/html/2309.09008v2) - SLAM sur micro-robots
- [Ultra-Lightweight C-SLAM](https://arxiv.org/html/2407.03136v2) - SLAM sur MCU 1.5MB RAM
- [DFRobot Maqueen Plus V3](https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3)
- [VL53L5CX Datasheet](https://www.st.com/resource/en/datasheet/vl53l5cx.pdf)
