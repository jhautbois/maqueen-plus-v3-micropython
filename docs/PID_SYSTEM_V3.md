# Maqueen Plus V3 PID System Documentation

## Découverte Majeure

Le Maqueen Plus V3 possède un **système PID matériel complet** dans le contrôleur STM8, mais il fonctionne différemment des versions précédentes et de la bibliothèque Arduino.

## Tests de Calibration

| Test | Commande | Mesuré | Précision |
|------|----------|--------|-----------|
| 1 | 25 unités | 25 cm | ✓ Parfait |
| 2 | 50 unités | 50 cm | ✓ Parfait |
| 3 | 12 unités | 12 cm | ✓ Parfait |
| 4 | 100 unités | 100 cm | ✓ Parfait |
| 5 | 90° rotation | ~90° | ✓ Excellent |
| 6 | 180° rotation | ~180° | ✓ Excellent |

**Conclusion**:
- **1 unité = 1 centimètre exactement**
- Précision distance: ±1 cm
- Précision rotation: ±5°
- Le PID est **pré-calibré en usine**

## Registres I2C (STM8 @ 0x10)

### Distance Control

| Registre | Nom | Valeurs | Description |
|----------|-----|---------|-------------|
| 0x40 | DIR_DISTANCE | 1=avant, 2=arrière | Direction du mouvement |
| 0x41 | DISTANCE_HIGH | 0-255 | Distance haute (byte) |
| 0x42 | DISTANCE_LOW | 0-255 | Distance basse (byte) |
| 0x55 | SPEED_DISTANCE | 1-10 (défaut=2) | Vitesse de déplacement |

### Rotation Control

| Registre | Nom | Valeurs | Description |
|----------|-----|---------|-------------|
| 0x43 | DIR_ANGLE | 1=CW, 2=CCW | Direction rotation |
| 0x44 | ANGLE | 0-255 | Angle en degrés |
| 0x56 | SPEED_ANGLE | 1-10 (défaut=2) | Vitesse de rotation |

### Mode Control

| Registre | Nom | Valeurs | Description |
|----------|-----|---------|-------------|
| 0x3C | MODE_CONTROL | 0x06=start, 0x10=stop | Commande PID |
| 0x57 | STATUS | 0x01=done | État de complétion |

### Registres NON fonctionnels sur V3

| Registre | Nom | État | Raison |
|----------|-----|------|--------|
| 0x0A | PID_SWITCH | ❌ Inactif | Interface Arduino non supportée |
| 0x04/0x06 | ENCODEURS | ❌ Toujours 0 | Non accessibles via I2C sur V3 |

## Séquence de Commande PID

### Distance (exemple: avancer 50cm)

```c
// 1. Direction
i2c_write(0x10, bytes([0x40, 1]));  // Avant

// 2. Vitesse
i2c_write(0x10, bytes([0x55, 2]));  // Speed = 2

// 3. Distance (50cm = 0x0032)
i2c_write(0x10, bytes([0x41, 0x00]));  // High byte
i2c_write(0x10, bytes([0x42, 0x32]));  // Low byte = 50

// 4. Start PID
i2c_write(0x10, bytes([0x3C, 0x06]));

// 5. Wait for completion
while (read_reg(0x57) != 0x01) {
    sleep(50);
}
```

### Rotation (exemple: tourner 90° CW)

```c
// 1. Direction
i2c_write(0x10, bytes([0x43, 1]));  // Clockwise

// 2. Vitesse
i2c_write(0x10, bytes([0x56, 2]));  // Speed = 2

// 3. Angle
i2c_write(0x10, bytes([0x44, 90]));  // 90 degrés

// 4. Start PID
i2c_write(0x10, bytes([0x3C, 0x06]));

// 5. Wait for completion
while (read_reg(0x57) != 0x01) {
    sleep(50);
}
```

## API Zephyr

### Fonctions principales (pid_control.h)

```c
// Initialisation
int pid_init(void);

// Déplacements
int pid_move_distance(uint16_t distance_cm, enum pid_direction direction, uint8_t speed);
int pid_rotate_angle(uint8_t angle_deg, enum pid_rotation direction, uint8_t speed);

// Convenience functions
int pid_forward(uint16_t cm);      // Avancer
int pid_backward(uint16_t cm);     // Reculer
int pid_rotate_cw(uint8_t degrees); // Tourner à droite
int pid_rotate_ccw(uint8_t degrees);// Tourner à gauche

// Status
bool pid_is_done(void);            // Vérifie si terminé
int pid_wait_done(uint32_t timeout_ms);  // Attend la fin
int pid_stop(void);                // Arrêt d'urgence
```

### Mise à jour de l'odométrie

```c
// Après un mouvement PID, mettre à jour l'odométrie:

// Distance
pid_forward(50);  // Avance 50cm
odometry_update_pid_distance(&odom, 500);  // 500mm

// Rotation
pid_rotate_cw(90);  // Tourne 90°
odometry_update_pid_rotation(&odom, 90000);  // 90000 millidegrés
```

## Exemple d'utilisation

```c
#include "pid_control.h"
#include "odometry.h"

int main(void)
{
    struct odometry odom;

    // Init
    pid_init();
    odometry_init(&odom, i2c_dev);
    odometry_reset(&odom);

    // Carré de 1m x 1m
    for (int i = 0; i < 4; i++) {
        // Avancer 100cm
        pid_forward(100);
        odometry_update_pid_distance(&odom, 1000);

        // Tourner 90°
        pid_rotate_cw(90);
        odometry_update_pid_rotation(&odom, 90000);

        k_sleep(K_MSEC(500));
    }

    printk("Final position: (%d, %d) mm\n", odom.x, odom.y);
    printk("Final heading: %d°\n", odometry_get_heading_deg(&odom));

    return 0;
}
```

## Avantages du PID V3

1. **Pas de calibration nécessaire** - Pré-calibré en usine
2. **Précision excellente** - ±1cm sur distance, ±5° sur rotation
3. **Simple à utiliser** - Commandes haut niveau (cm, degrés)
4. **Bloquant** - Attend automatiquement la fin du mouvement
5. **Feedback via status** - Registre 0x57 indique la complétion

## Différences avec V2.0

| Feature | V2.0 (Arduino) | V3 (Actuel) |
|---------|----------------|-------------|
| PID Switch (0x0A) | ✓ Fonctionnel | ❌ Non supporté |
| Encodeurs (0x04/0x06) | ✓ Lisibles | ❌ Non accessibles |
| Commandes distance/angle | ✓ Via 0x40-0x44 | ✓ Via 0x40-0x44 |
| Registre status (0x57) | ✓ Présent | ✓ Présent |
| Vitesses réelles (0x4C/0x4D) | ? | ✓ Fonctionnel |

## Stratégie de Navigation

### Ancien système (vitesses continues)
```c
// Contrôle moteur direct + calcul odométrie manuel
maqueen_motor_set(left, FORWARD, 100);
maqueen_motor_set(right, FORWARD, 100);
while (running) {
    odometry_update(&odom);  // Intégration vitesses
    // Navigation continue
}
```

### Nouveau système (PID discret)
```c
// Mouvements précis par incréments
while (exploring) {
    scan_environment();
    decide_direction();

    // Mouvement précis
    pid_forward(20);  // 20cm exact
    odometry_update_pid_distance(&odom, 200);  // Mise à jour exacte

    // Ou rotation
    pid_rotate_cw(45);  // 45° exact
    odometry_update_pid_rotation(&odom, 45000);
}
```

**Avantages navigation PID:**
- Pas de drift d'odométrie (mouvements exacts)
- Simplification du code (pas de contrôle moteur manuel)
- Meilleure précision de mapping
- Le robot s'arrête précisément aux positions cibles

## Limitations

1. **Distance maximale**: 65535 cm (655m) - largement suffisant
2. **Angle maximal**: 255° par commande (faire 2 commandes pour 360°)
3. **Vitesse limitée**: 1-10 (vitesse 10 non testée)
4. **Bloquant**: Impossible d'interrompre un mouvement (sauf pid_stop())

## Tests Python

Fichiers de test disponibles:
- `test_pid_status.py` - Test rotation et distance avec monitoring status
- `test_pid_simple.py` - 4 tests de distances calibrées
- `test_pid_calib.py` - Suite complète de calibration

## Fichiers Zephyr

- `zephyr-app/src/pid_control.h/c` - Module PID
- `zephyr-app/src/odometry.h/c` - Fonctions de mise à jour PID
- `zephyr-app/apps/mapper/src/main_pid.c.example` - Exemple d'intégration

## Conclusion

Le système PID du Maqueen Plus V3 est **robuste, précis et pré-calibré**. Il simplifie grandement la navigation en offrant des mouvements discrets de haute précision, éliminant le besoin de calibration manuelle complexe et réduisant les erreurs d'odométrie.

**Recommandation**: Utiliser le PID pour tous les mouvements du mapper au lieu du contrôle moteur manuel.
