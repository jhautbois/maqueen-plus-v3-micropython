# Navigation V2 - Spécification de Haut Niveau

## Résumé Exécutif

Ce document décrit l'architecture d'un système de navigation amélioré pour le robot Maqueen Plus V3, résolvant les problèmes de mouvement en serpentin et d'échappement de coins.

**Problèmes identifiés :**
1. Mouvement en serpentin (oscillation gauche/droite)
2. Incapacité à sortir des coins
3. Pas de mémoire des obstacles hors du champ de vision
4. Pas d'utilisation du PID pour les mouvements précis

**Solution proposée :**
Architecture modulaire avec carte de distance polaire, contrôle PID, et odométrie.

---

## Architecture Modulaire

```
┌─────────────────────────────────────────────────────────────────┐
│                      NAVIGATION V2                               │
├─────────────────────────────────────────────────────────────────┤
│                                                                  │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐           │
│  │   LIDAR      │  │   ENCODERS   │  │   MOTEURS    │           │
│  │   8x8 ToF    │  │   Magnétiques│  │   N20 + PID  │           │
│  └──────┬───────┘  └──────┬───────┘  └──────▲───────┘           │
│         │                 │                  │                   │
│         ▼                 ▼                  │                   │
│  ┌──────────────────────────────────────────┴───────┐           │
│  │              DISTANCE MAP (16 secteurs)           │           │
│  │  - Carte polaire 360° autour du robot            │           │
│  │  - Vieillissement des données                    │           │
│  │  - Rotation quand le robot tourne                │           │
│  └──────────────────────────┬───────────────────────┘           │
│                             │                                    │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────┐           │
│  │              GAP FINDER (Anti-oscillation)        │           │
│  │  - Lissage médian des secteurs                   │           │
│  │  - Hystérésis de direction                       │           │
│  │  - Détection d'oscillation                       │           │
│  └──────────────────────────┬───────────────────────┘           │
│                             │                                    │
│                             ▼                                    │
│  ┌──────────────────────────────────────────────────┐           │
│  │              MOTION CONTROLLER                    │           │
│  │  - PID pour angles précis (±5°)                  │           │
│  │  - PID pour distances précises (±2cm)            │           │
│  │  - Échappement progressif des coins              │           │
│  └──────────────────────────────────────────────────┘           │
│                                                                  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Module 1: Distance Map (Carte Polaire)

### Objectif
Maintenir une représentation 360° des obstacles autour du robot, même ceux hors du champ de vision du LIDAR (60°).

### Spécifications

| Paramètre | Valeur | Justification |
|-----------|--------|---------------|
| Nombre de secteurs | 16 | 22.5° par secteur, bon compromis mémoire/précision |
| Stockage par secteur | 1 entier | Distance (10 bits) + âge (4 bits) = 14 bits |
| Mémoire totale | ~300 octets | Compatible micro:bit |
| Âge maximum | 15 ticks | Données expirées après ~1.5s |

### Structure de données

```python
# Carte polaire: 16 secteurs de 22.5° chacun
#
#            0 (avant)
#        15     1
#      14         2
#    13             3
#   12 (gauche)     4 (droite)
#    11             5
#      10         6
#        9       7
#            8 (arrière)

NUM_SECTORS = 16
polar_map = [0] * NUM_SECTORS  # 64 octets

# Packing: (age << 10) | (distance >> 2)
# - distance: 0-4000mm stocké en 0-1000 (10 bits)
# - age: 0-15 ticks (4 bits)
```

### Algorithmes clés

1. **Mise à jour depuis LIDAR** : Les 8 colonnes LIDAR → 3 secteurs avant (15, 0, 1)
2. **Rotation de la carte** : Quand le robot tourne de 22.5°, décaler les indices
3. **Vieillissement** : Incrémenter l'âge chaque 5 cycles, expirer à MAX_AGE
4. **Recherche du meilleur passage** : Score = distance × fraîcheur + bonus voisins

---

## Module 2: Gap Finder (Anti-oscillation)

### Problèmes résolus

| Problème | Cause racine | Solution |
|----------|--------------|----------|
| Serpentin | Pas de mémoire de direction | Hystérésis + compteur de stabilité |
| Oscillation rapide | Seuil 70% trop sensible | Seuil hybride absolu/relatif |
| Bruit capteur | `min()` trop sensible | Médiane au lieu de minimum |
| Biais gauche | Moyenne en cascade | Lissage symétrique |

### Hystérésis de direction

```python
class GapFinder:
    def __init__(self):
        self.last_gap_idx = 3      # Dernière direction choisie
        self.stability = 0          # Compteur de stabilité (0-3)
        self.turn_history = [0]*4   # Historique des virages

    def find_gap_with_hysteresis(self, sectors):
        # 1. Lissage médian
        smoothed = self.median_smooth(sectors)

        # 2. Trouver meilleur secteur
        best_idx, best_dist = self.find_best(smoothed)

        # 3. Hystérésis: préférer direction précédente si similaire
        if abs(best_idx - self.last_gap_idx) <= 1:
            self.stability += 1
        else:
            self.stability = 0

        # 4. Exiger 3 frames consécutives avant changement
        if self.stability < 3:
            best_idx = self.last_gap_idx

        self.last_gap_idx = best_idx
        return best_idx
```

### Détection d'oscillation

```python
def is_oscillating(self):
    """Détecter alternance gauche-droite"""
    alternations = sum(1 for i in range(3)
                       if self.turn_history[i] * self.turn_history[i+1] < 0)
    return alternations >= 2

def commit_direction(self, polar_map):
    """Quand oscillation détectée, s'engager sur un côté"""
    left_sum = sum(get_dist(polar_map[16-i]) for i in range(1, 8))
    right_sum = sum(get_dist(polar_map[i]) for i in range(1, 8))
    return 14 if left_sum > right_sum else 2  # Secteur cible
```

---

## Module 3: Motion Controller (PID)

### Registres STM8 pour PID

| Registre | Adresse | Fonction |
|----------|---------|----------|
| PID_CONTROL | 0x3C | Enable/disable (0/1) |
| PID_DIST_L/M/H | 0x40-42 | Distance en mm (24 bits) |
| PID_ANGLE_L/H | 0x43-44 | Angle en degrés (16 bits signés) |
| PID_STATUS | 0x57 | Flag de complétion (1=terminé) |
| SPEED_RT | 0x4C | Vitesse en temps réel |

### Utilisation du PID

```python
def pid_angle(self, angle_deg, speed=80):
    """Rotation précise avec PID (±5°)"""
    self._write_reg(0x3C, 1)  # Enable PID
    sleep(10)

    a = int(angle_deg)
    if a < 0:
        a = (1 << 16) + a  # Complément à 2

    self._write_reg(0x43, a & 0xFF)
    self._write_reg(0x44, (a >> 8) & 0xFF)

def pid_distance(self, dist_cm, speed=100):
    """Déplacement précis avec PID (±2cm)"""
    self._write_reg(0x3C, 1)
    sleep(10)

    d = int(dist_cm * 10)  # Convertir en mm
    self._write_reg(0x40, d & 0xFF)
    self._write_reg(0x41, (d >> 8) & 0xFF)
    self._write_reg(0x42, (d >> 16) & 0xFF)

def pid_wait(self, timeout_ms=5000):
    """Attendre fin du mouvement PID"""
    start = running_time()
    while running_time() - start < timeout_ms:
        i2c.write(0x10, bytes([0x57]))
        if i2c.read(0x10, 1)[0] == 1:
            return True
        sleep(50)
    return False
```

### Échappement progressif des coins

```python
def escape_corner(self, gap_angle):
    """Échappement de coin avec PID"""
    # 1. Reculer de 15cm précisément
    self.pid_distance(-15, 80)
    self.pid_wait(3000)

    # 2. Tourner vers le passage avec angle précis
    if abs(gap_angle) < 10:
        # Passage devant mais bloqué → tourner 60°
        angle = 60 if self.last_turn > 0 else -60
    else:
        angle = gap_angle * 1.5  # Amplifier légèrement

    self.pid_angle(angle, 100)
    self.pid_wait(3000)

    # 3. Avancer de 20cm
    self.pid_distance(20, 100)
    self.pid_wait(4000)
```

---

## Module 4: Odométrie (Optionnel)

### Objectif
Estimer la rotation du robot pour mettre à jour la carte polaire.

### Approche simplifiée (sans encodeurs)

```python
def estimate_rotation(self, left_speed, right_speed, dt_ms):
    """Estimer rotation depuis différentiel de vitesse"""
    # Approximation: différence de vitesse → taux de rotation
    # Calibration nécessaire pour chaque robot
    diff = right_speed - left_speed
    return diff * dt_ms / 1000 * ROTATION_FACTOR  # degrés
```

### Approche avec encodeurs (si implémentée)

```python
def update_odometry(self, left_ticks, right_ticks):
    """Mise à jour depuis encodeurs magnétiques"""
    MM_PER_TICK = 33  # 132mm circonférence / 4 ticks
    WHEEL_BASE = 80   # mm

    dl = (left_ticks - self.last_left) * MM_PER_TICK
    dr = (right_ticks - self.last_right) * MM_PER_TICK

    delta_heading = (dr - dl) / WHEEL_BASE * 57.3  # degrés
    self.heading += delta_heading

    # Mettre à jour la carte polaire
    self.distance_map.rotate(delta_heading)
```

---

## Plan d'Implémentation

### Phase 1: Distance Map (Priorité haute)
- [ ] Créer module `distance_map.py` (~300 octets)
- [ ] Implémenter packing/unpacking distance+âge
- [ ] Implémenter mise à jour depuis LIDAR
- [ ] Implémenter rotation de la carte
- [ ] Implémenter vieillissement
- [ ] Tester en isolation

### Phase 2: Gap Finder (Priorité haute)
- [ ] Remplacer `min()` par médiane dans `polar()`
- [ ] Corriger le lissage dans `find_gap()`
- [ ] Ajouter hystérésis de direction
- [ ] Ajouter détection d'oscillation
- [ ] Ajouter logique de "commit" sur un côté

### Phase 3: PID Integration (Priorité moyenne)
- [ ] Ajouter méthodes PID à `MaqueenPlusV3`
- [ ] Implémenter `pid_angle()`, `pid_distance()`, `pid_wait()`
- [ ] Modifier échappement de coin pour utiliser PID
- [ ] Tester précision des mouvements

### Phase 4: Odométrie (Priorité basse)
- [ ] Implémenter estimation de rotation simple
- [ ] Calibrer facteur de rotation
- [ ] Optionnel: lire encodeurs STM8 si registres disponibles

---

## Contraintes Techniques

### Mémoire
| Module | Estimation | Budget |
|--------|------------|--------|
| distance_map.py | 300 octets | ✓ |
| gap_finder additions | 200 octets | ✓ |
| pid methods | 300 octets | ✓ |
| odometry | 200 octets | ✓ |
| **Total** | **1000 octets** | < 2KB ✓ |

### MicroPython Limitations
- Pas de f-strings → utiliser `+` ou `.format()`
- Pas de `deque` → liste circulaire manuelle
- Pas de `math.sin/cos` efficace → approximation ou lookup
- Minification obligatoire pour déploiement

---

## Métriques de Succès

| Critère | Avant | Objectif |
|---------|-------|----------|
| Oscillation gauche/droite | Fréquente | < 1 par minute |
| Échappement de coin | Échec ~50% | > 90% succès |
| Temps pour sortir d'un coin | > 10s | < 3s |
| Collision avec obstacles | Occasionnelle | Rare |
| Mouvement fluide | Saccadé | Lisse |

---

## Fichiers à Modifier

```
src/
├── main.py              # Intégration navigation V2
├── maqueen_plus_v3.py   # Ajouter méthodes PID
├── laser_matrix.py      # Inchangé
├── distance_map.py      # NOUVEAU: carte polaire
└── gap_finder.py        # NOUVEAU: anti-oscillation (ou intégré dans main)
```

---

## Références

- [Wiki DFRobot Maqueen Plus V3](https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3)
- [GitHub MakeCode Extension](https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20)
- Skill local: `~/.claude/skills/maqueen-plus-v3/SKILL.md`
