# Plan : Système de Cartographie Zephyr pour Maqueen Plus V3

**Version améliorée** - Profite du budget mémoire disponible pour meilleure précision.

## Objectif

Créer un système de cartographie autonome **en Zephyr RTOS (C)** où le robot :
1. Scanne son environnement à 360° au démarrage
2. Navigue vers les zones inexplorées (plus grande distance libre)
3. Met à jour la carte en continu avec probabilités
4. Sauvegarde la carte en RAM et Flash (NVS)
5. Exporte la carte vers un PC via shell Zephyr

---

## Infrastructure Zephyr Existante

### Drivers déjà implémentés

| Module | Fichiers | Fonctionnalités |
|--------|----------|-----------------|
| Odométrie | `src/odometry.c/h` | Position x,y (mm), heading (millidegrés), lookup sin/cos |
| Moteurs | `src/maqueen.c/h` | Contrôle vitesse, lecture vitesse temps réel (0x4C) |
| LIDAR | `drivers/sen0628/` | Matrix 8×8, colonnes min/avg, timestamps |

### Structures existantes

```c
// odometry.h
struct odometry {
    int32_t x, y;           // Position en mm
    int32_t heading_mdeg;   // Cap 0-360000 millidegrés
    int32_t dist_left, dist_right;
    int8_t dir_left, dir_right;
    const struct device *i2c_dev;
};

// sen0628.h
struct sen0628_scan {
    uint16_t distances[64]; // Matrice 8×8
    uint32_t timestamp_ms;
    uint8_t mode;
    uint16_t min_distance;
    uint8_t min_x, min_y;
};

struct sen0628_columns {
    uint16_t min[8];        // Distance min par colonne
    uint16_t avg[8];
    uint8_t valid_count[8];
};
```

---

## Architecture Améliorée

### Grille d'occupation : 48×48 cellules avec log-odds

| Paramètre | Valeur | Justification |
|-----------|--------|---------------|
| Taille | 48×48 = 2304 cellules | Couverture **4.8m × 4.8m** |
| Résolution | **100mm/cellule** | Précision doublée vs 150mm |
| Stockage | **8 bits/cellule = 2304 octets** | Log-odds probabiliste |
| Plage log-odds | -128 à +127 | 0 = incertain, >0 = occupé, <0 = libre |

**Avantage log-odds** : Les mesures s'accumulent de façon probabiliste, filtrant le bruit.

```
Log-odds update: l_new = l_old + l_sensor - l_prior
Probabilité:     p = 1 / (1 + exp(-l))

Valeurs typiques:
  l = -128 : p ≈ 0% (définitivement libre)
  l = -40  : p ≈ 2% (probablement libre)
  l = 0    : p = 50% (incertain)
  l = +40  : p ≈ 98% (probablement occupé)
  l = +127 : p ≈ 100% (définitivement occupé)
```

### Scan 360° avec capteur 60° FOV

```
Position 0: 0°   (avant)     ← lecture LIDAR (~480ms en 4×4)
Position 1: 60°  (droite)    ← rotation + lecture
Position 2: 120°             ← rotation + lecture
Position 3: 180° (arrière)   ← rotation + lecture
Position 4: 240°             ← rotation + lecture
Position 5: 300° (gauche)    ← rotation + lecture
Retour: 0°

Total: ~5-6 secondes (mode 4×4 pour la vitesse)
```

### Ray Casting Bresenham

Pour chaque mesure LIDAR :
1. Tracer une ligne du robot vers le point détecté
2. Marquer **libre** tous les pixels traversés (log-odds -= 15)
3. Marquer **occupé** le pixel final (log-odds += 30)
4. Saturer aux bornes [-128, +127]

---

## Nouveaux Modules à Créer

### 1. `src/occupancy_grid.c/h` - Grille log-odds

```c
// occupancy_grid.h
#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>
#include <stdbool.h>

#define GRID_SIZE 48
#define CELL_MM 100
#define GRID_BYTES (GRID_SIZE * GRID_SIZE)  // 2304 octets

// Constantes log-odds
#define LOG_ODDS_PRIOR      0     // Valeur initiale (incertain)
#define LOG_ODDS_FREE      -15    // Incrément "libre"
#define LOG_ODDS_OCCUPIED   30    // Incrément "occupé"
#define LOG_ODDS_MIN      -128    // Borne min (définitivement libre)
#define LOG_ODDS_MAX       127    // Borne max (définitivement occupé)

// Seuils pour classification
#define THRESHOLD_FREE     -20    // < -20 = libre
#define THRESHOLD_OCCUPIED  20    // > +20 = occupé

struct occupancy_grid {
    int8_t data[GRID_BYTES];      // Grille log-odds
    int32_t origin_x, origin_y;   // Origine en mm (centre grille = position initiale robot)
    uint32_t update_count;        // Compteur mises à jour
};

// === Initialisation ===
void grid_init(struct occupancy_grid *grid);
void grid_clear(struct occupancy_grid *grid);

// === Accès cellules ===
int8_t grid_get_logodds(const struct occupancy_grid *grid, int cx, int cy);
void grid_set_logodds(struct occupancy_grid *grid, int cx, int cy, int8_t value);

// Accès par état (pour navigation)
typedef enum {
    CELL_UNKNOWN = 0,
    CELL_FREE = 1,
    CELL_OCCUPIED = 2
} cell_state_t;

cell_state_t grid_get_state(const struct occupancy_grid *grid, int cx, int cy);

// === Conversion coordonnées ===
// Monde (mm) -> Cellule
void grid_world_to_cell(const struct occupancy_grid *grid,
                        int32_t world_x, int32_t world_y,
                        int *cell_x, int *cell_y);

// Cellule -> Monde (centre de la cellule)
void grid_cell_to_world(const struct occupancy_grid *grid,
                        int cell_x, int cell_y,
                        int32_t *world_x, int32_t *world_y);

// === Mise à jour depuis capteur ===
// Marque un obstacle et le chemin libre (Bresenham ray casting)
void grid_update_ray(struct occupancy_grid *grid,
                     int32_t robot_x, int32_t robot_y,
                     int32_t target_x, int32_t target_y,
                     bool hit_obstacle);

// Mise à jour depuis colonnes LIDAR
struct sen0628_columns;
struct odometry;
void grid_update_from_lidar(struct occupancy_grid *grid,
                            const struct sen0628_columns *cols,
                            const struct odometry *odom);

// === Statistiques ===
struct grid_stats {
    int unknown_count;
    int free_count;
    int occupied_count;
    int total_updates;
};

void grid_get_stats(const struct occupancy_grid *grid, struct grid_stats *stats);

// === Sérialisation ===
// Pour export: convertit log-odds en 2-bit compact (unknown/free/occupied)
// Réduit 2304 -> 576 octets pour transfert
int grid_export_compact(const struct occupancy_grid *grid, uint8_t *out, size_t out_size);

// Pour affichage: convertit en hex string
int grid_to_hex(const struct occupancy_grid *grid, char *out, size_t out_size);

#endif // OCCUPANCY_GRID_H
```

**Implémentation ray casting Bresenham :**

```c
// occupancy_grid.c (extrait)

// Bresenham line algorithm avec mise à jour log-odds
void grid_update_ray(struct occupancy_grid *grid,
                     int32_t robot_x, int32_t robot_y,
                     int32_t target_x, int32_t target_y,
                     bool hit_obstacle)
{
    int x0, y0, x1, y1;
    grid_world_to_cell(grid, robot_x, robot_y, &x0, &y0);
    grid_world_to_cell(grid, target_x, target_y, &x1, &y1);

    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        // Dernière cellule = obstacle (si hit)
        bool is_endpoint = (x0 == x1 && y0 == y1);

        if (is_endpoint && hit_obstacle) {
            // Marquer occupé
            int8_t val = grid_get_logodds(grid, x0, y0);
            val = CLAMP(val + LOG_ODDS_OCCUPIED, LOG_ODDS_MIN, LOG_ODDS_MAX);
            grid_set_logodds(grid, x0, y0, val);
        } else if (!is_endpoint) {
            // Marquer libre (cellules traversées)
            int8_t val = grid_get_logodds(grid, x0, y0);
            val = CLAMP(val + LOG_ODDS_FREE, LOG_ODDS_MIN, LOG_ODDS_MAX);
            grid_set_logodds(grid, x0, y0, val);
        }

        if (is_endpoint) break;

        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 < dx)  { err += dx; y0 += sy; }
    }

    grid->update_count++;
}
```

### 2. `src/scanner_360.c/h` - Scanner rotatif amélioré

```c
// scanner_360.h
#ifndef SCANNER_360_H
#define SCANNER_360_H

#include <stdbool.h>
#include <zephyr/kernel.h>

#define SCAN_POSITIONS 6
#define SCAN_ANGLE_DEG 60

struct scanner_360 {
    const struct device *i2c_dev;
    struct occupancy_grid *grid;
    struct odometry *odom;

    // État
    int current_position;
    bool scan_complete;
    int32_t initial_heading;  // Pour retour précis

    // Calibration rotation (ajustable)
    int turn_speed;           // Vitesse rotation (défaut: 100)
    int ms_per_degree;        // Durée par degré (calibré)
};

// Callback progression
typedef void (*scan_progress_cb)(int position, int total, void *user_data);

// API
int scanner_init(struct scanner_360 *scanner,
                 const struct device *i2c,
                 struct occupancy_grid *grid,
                 struct odometry *odom);

// Calibration: mesure ms/degré avec odométrie
int scanner_calibrate_rotation(struct scanner_360 *scanner);

// Scan 360° complet (bloquant)
int scanner_full_scan(struct scanner_360 *scanner,
                      scan_progress_cb cb,
                      void *user_data);

// Scan single position (non-bloquant, pour contrôle fin)
int scanner_scan_position(struct scanner_360 *scanner, int position);

#endif // SCANNER_360_H
```

### 3. `src/frontier_nav.c/h` - Navigation exploratoire avancée

```c
// frontier_nav.h
#ifndef FRONTIER_NAV_H
#define FRONTIER_NAV_H

#include <stdbool.h>

// Directions: 0=N, 1=NE, 2=E, 3=SE, 4=S, 5=SW, 6=W, 7=NW
#define NUM_DIRECTIONS 8

struct frontier_result {
    int direction;            // Direction optimale (0-7)
    int score;                // Score (plus = meilleur)
    int distance_cells;       // Distance à la frontière
    int unknown_count;        // Cellules inconnues dans cette direction
    bool exploration_complete;
};

// Direction vectors (cosine × 1000, sine × 1000)
extern const int16_t DIR_COS[8];
extern const int16_t DIR_SIN[8];

// === API principale ===

// Trouve la meilleure direction vers zone inexplorée
// Utilise ray casting dans 8 directions + bonus pour directions avec plus d'inconnus
int frontier_find_best(const struct occupancy_grid *grid,
                       int robot_cell_x, int robot_cell_y,
                       int robot_heading_deg,
                       struct frontier_result *result);

// Vérifie si exploration terminée
bool frontier_is_complete(const struct occupancy_grid *grid, int threshold);

// === Utilitaires ===

// Convertit direction (0-7) en angle (0, 45, 90, ..., 315)
int frontier_dir_to_angle(int direction);

// Trouve direction la plus proche d'un angle donné
int frontier_angle_to_dir(int angle_deg);

// Calcule différence angulaire signée entre deux directions
// Retourne -4 à +3 (négatif = tourner gauche, positif = tourner droite)
int frontier_dir_diff(int from_dir, int to_dir);

#endif // FRONTIER_NAV_H
```

**Algorithme frontier amélioré :**

```c
// frontier_nav.c (extrait)

int frontier_find_best(const struct occupancy_grid *grid,
                       int robot_cx, int robot_cy,
                       int robot_heading_deg,
                       struct frontier_result *result)
{
    int best_dir = 0;
    int best_score = -1;
    int best_dist = GRID_SIZE;
    int best_unknown = 0;

    int robot_dir = frontier_angle_to_dir(robot_heading_deg);

    for (int d = 0; d < NUM_DIRECTIONS; d++) {
        int unknown_count = 0;
        int first_unknown = 0;
        bool path_blocked = false;

        // Ray cast dans direction d (jusqu'à 20 cellules = 2m)
        for (int dist = 1; dist <= 20; dist++) {
            int cx = robot_cx + (dist * DIR_COS[d]) / 1000;
            int cy = robot_cy + (dist * DIR_SIN[d]) / 1000;

            cell_state_t state = grid_get_state(grid, cx, cy);

            if (state == CELL_OCCUPIED) {
                path_blocked = true;
                break;
            }

            if (state == CELL_UNKNOWN) {
                unknown_count++;
                if (first_unknown == 0) {
                    first_unknown = dist;
                }
            }
        }

        if (!path_blocked && unknown_count > 0) {
            // Score: favorise plus d'inconnus + frontière proche
            // Bonus pour direction actuelle (évite oscillations)
            int dir_penalty = abs(frontier_dir_diff(robot_dir, d));
            int score = unknown_count * 10
                      + (20 - first_unknown) * 2
                      - dir_penalty * 3;

            if (score > best_score) {
                best_score = score;
                best_dir = d;
                best_dist = first_unknown;
                best_unknown = unknown_count;
            }
        }
    }

    result->direction = best_dir;
    result->score = best_score;
    result->distance_cells = best_dist;
    result->unknown_count = best_unknown;
    result->exploration_complete = (best_score < 0);

    return (best_score >= 0) ? 0 : -1;
}
```

### 4. `src/map_storage.c/h` - Persistance NVS améliorée

```c
// map_storage.h
#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <stdint.h>
#include <stdbool.h>

#define MAP_MAGIC 0x4D415032  // 'MAP2'
#define MAP_VERSION 2

struct map_header {
    uint32_t magic;
    uint16_t version;
    uint16_t grid_size;       // 48
    uint16_t cell_mm;         // 100
    uint16_t reserved;
    int32_t origin_x;         // Origine grille
    int32_t origin_y;
    int32_t robot_x;          // Position robot sauvegardée
    int32_t robot_y;
    int32_t robot_heading;
    uint32_t update_count;
    uint32_t timestamp;       // k_uptime_get_32()
    uint32_t checksum;        // CRC32 des données
};

// API
int map_storage_init(void);

// Sauvegarde complète (header + grille log-odds)
int map_storage_save(const struct occupancy_grid *grid,
                     const struct odometry *odom);

// Chargement avec vérification checksum
int map_storage_load(struct occupancy_grid *grid,
                     struct odometry *odom);

// Efface la carte stockée
int map_storage_clear(void);

// Vérifie si une carte existe en NVS
bool map_storage_exists(void);

// Retourne info sur carte stockée (sans charger)
int map_storage_info(struct map_header *header);

#endif // MAP_STORAGE_H
```

### 5. `src/map_shell.c` - Commandes shell enrichies

```c
// map_shell.c - Commandes shell Zephyr

/*
 * Commandes disponibles:
 *
 * map show [scale]    - Affiche carte ASCII (scale: 1, 2, 4)
 * map export          - Export format parsable pour PC
 * map save            - Sauvegarde en NVS
 * map load            - Charge depuis NVS
 * map clear           - Efface carte RAM
 * map erase           - Efface carte NVS
 * map stats           - Statistiques détaillées
 * map info            - Info carte NVS
 * map cell <x> <y>    - Affiche état d'une cellule
 * map pose            - Affiche position robot
 */

// Format export amélioré:
// ---MAP-START---
// VERSION:2
// SIZE:48
// CELL:100
// ORIGIN:0,0
// POSE:1200,-450,135000
// UPDATES:1523
// DATA:7F7F7F7F7F7F... (4608 hex chars = 2304 bytes)
// STATS:1800,400,104
// CRC:A1B2C3D4
// ---MAP-END---
```

---

## Application Principale

### `apps/mapper/src/main.c`

```c
/*
 * Mapper Application - Exploration autonome avec cartographie
 *
 * Bouton A : Démarrer/Pause exploration
 * Bouton B : Sauvegarder et exporter carte
 * LED Display : Animation selon état
 * Shell : Commandes map *
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "occupancy_grid.h"
#include "scanner_360.h"
#include "frontier_nav.h"
#include "map_storage.h"
#include "odometry.h"
#include "maqueen.h"
#include "sen0628.h"

LOG_MODULE_REGISTER(mapper, LOG_LEVEL_INF);

// États
enum mapper_state {
    STATE_INIT,
    STATE_IDLE,
    STATE_SCANNING,
    STATE_EXPLORING,
    STATE_PAUSED,
    STATE_ERROR
};

// Contexte global
static struct {
    enum mapper_state state;
    struct occupancy_grid grid;
    struct odometry odom;
    struct scanner_360 scanner;
    const struct device *i2c_dev;
    uint32_t loop_count;
    int explore_speed;
} ctx;

// === Exploration Loop ===

static void explore_step(void)
{
    struct sen0628_columns cols;
    struct frontier_result frontier;

    // 1. Update odometry
    odometry_update(&ctx.odom);

    // 2. Read LIDAR columns
    int ret = sen0628_get_columns(ctx.i2c_dev, &cols);
    if (ret < 0) {
        LOG_WRN("LIDAR read failed");
        return;
    }

    // 3. Update grid
    grid_update_from_lidar(&ctx.grid, &cols, &ctx.odom);

    // 4. Get robot position in grid
    int rx, ry;
    grid_world_to_cell(&ctx.grid, ctx.odom.x, ctx.odom.y, &rx, &ry);
    int heading_deg = ctx.odom.heading_mdeg / 1000;

    // 5. Find best frontier direction
    ret = frontier_find_best(&ctx.grid, rx, ry, heading_deg, &frontier);

    if (frontier.exploration_complete) {
        LOG_INF("Exploration complete!");
        map_storage_save(&ctx.grid, &ctx.odom);
        maqueen_stop_all(ctx.i2c_dev);
        ctx.state = STATE_IDLE;
        return;
    }

    // 6. Navigate
    int center_dist = (cols.min[3] + cols.min[4]) / 2;
    int current_dir = frontier_angle_to_dir(heading_deg);
    int turn_diff = frontier_dir_diff(current_dir, frontier.direction);

    // Évitement d'obstacle prioritaire
    if (center_dist < 150) {
        // Trop proche, reculer et tourner
        maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_BACKWARD, 80);
        maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_BACKWARD, 80);
        k_sleep(K_MSEC(300));

        // Tourner vers direction libre
        if (turn_diff >= 0) {
            maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_BACKWARD, 80);
            maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_FORWARD, 80);
        } else {
            maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_FORWARD, 80);
            maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_BACKWARD, 80);
        }
        k_sleep(K_MSEC(200));
    }
    else if (abs(turn_diff) > 1) {
        // Besoin de tourner significativement
        int speed_fast = ctx.explore_speed;
        int speed_slow = ctx.explore_speed * 60 / 100;

        if (turn_diff > 0) {
            maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_FORWARD, speed_fast);
            maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_FORWARD, speed_slow);
        } else {
            maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_FORWARD, speed_slow);
            maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_FORWARD, speed_fast);
        }
    }
    else {
        // Aller tout droit vers frontière
        int speed = (center_dist > 500) ? ctx.explore_speed : ctx.explore_speed * 70 / 100;
        maqueen_motor_set(ctx.i2c_dev, MOTOR_LEFT, DIR_FORWARD, speed);
        maqueen_motor_set(ctx.i2c_dev, MOTOR_RIGHT, DIR_FORWARD, speed);
    }

    ctx.loop_count++;
}

// === Main ===

int main(void)
{
    LOG_INF("Mapper starting...");

    // Init hardware
    ctx.i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
    if (!device_is_ready(ctx.i2c_dev)) {
        LOG_ERR("I2C not ready");
        return -1;
    }

    maqueen_init(ctx.i2c_dev);
    sen0628_init(ctx.i2c_dev);
    sen0628_set_mode(ctx.i2c_dev, 4);  // 4x4 pour vitesse

    // Init mapping
    grid_init(&ctx.grid);
    odometry_init(&ctx.odom, ctx.i2c_dev);
    scanner_init(&ctx.scanner, ctx.i2c_dev, &ctx.grid, &ctx.odom);
    map_storage_init();

    // Tenter de charger carte existante
    if (map_storage_exists()) {
        if (map_storage_load(&ctx.grid, &ctx.odom) == 0) {
            LOG_INF("Loaded saved map");
        }
    }

    ctx.state = STATE_IDLE;
    ctx.explore_speed = 100;

    LOG_INF("Ready - Press A to start");

    // Main loop
    while (true) {
        // Check buttons (simplified)
        // ... button handling ...

        switch (ctx.state) {
        case STATE_SCANNING:
            scanner_full_scan(&ctx.scanner, NULL, NULL);
            ctx.state = STATE_EXPLORING;
            LOG_INF("Scan complete, exploring...");
            break;

        case STATE_EXPLORING:
            explore_step();
            k_sleep(K_MSEC(100));  // ~10 Hz
            break;

        case STATE_IDLE:
        case STATE_PAUSED:
            k_sleep(K_MSEC(100));
            break;

        default:
            k_sleep(K_MSEC(1000));
        }
    }

    return 0;
}
```

---

## Configuration Zephyr

### `apps/mapper/prj.conf`

```ini
# Core
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_MODE_IMMEDIATE=y

# Console/Shell pour export
CONFIG_SHELL=y
CONFIG_SHELL_BACKEND_SERIAL=y
CONFIG_SHELL_LOG_LEVEL_INF=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y

# NVS pour persistance
CONFIG_NVS=y
CONFIG_FLASH=y
CONFIG_FLASH_MAP=y
CONFIG_FLASH_PAGE_LAYOUT=y

# Math (pour CRC si nécessaire)
CONFIG_CRC=y

# Timers
CONFIG_SYS_CLOCK_TICKS_PER_SEC=1000
```

### Device Tree Overlay (`apps/mapper/boards/bbc_microbit_v2.overlay`)

```dts
/ {
    chosen {
        zephyr,flash = &flash0;
    };
};

&flash0 {
    partitions {
        compatible = "fixed-partitions";
        #address-cells = <1>;
        #size-cells = <1>;

        /* Partition NVS pour stockage carte */
        /* 32KB à la fin du flash utilisateur */
        storage_partition: partition@70000 {
            label = "storage";
            reg = <0x70000 0x8000>;
        };
    };
};
```

---

## Budget Mémoire (Version Améliorée)

| Composant | Flash | RAM |
|-----------|-------|-----|
| Zephyr kernel + drivers | ~40 KB | ~20 KB |
| SEN0628 driver | ~3 KB | 200 B |
| odometry.c | ~1 KB | 32 B |
| maqueen.c | ~1 KB | 16 B |
| **occupancy_grid.c** | ~2 KB | **2304 B** (grille 48×48) |
| scanner_360.c | ~800 B | 64 B |
| frontier_nav.c | ~1 KB | 32 B |
| map_storage.c | ~800 B | 64 B |
| map_shell.c | ~1.5 KB | 128 B |
| main.c (mapper) | ~2 KB | 256 B |
| Shell buffers | ~2 KB | ~2 KB |
| **Total** | **~55 KB** | **~25 KB** |

**Disponible** : 512 KB Flash, 128 KB RAM
**Marge** : 457 KB Flash, 103 KB RAM → **Très confortable**

---

## Séquence d'Implémentation

### Phase 1 : Grille log-odds (priorité haute)
1. Créer `src/occupancy_grid.c/h`
2. Implémenter get/set log-odds
3. Implémenter `grid_update_ray()` Bresenham
4. Implémenter `grid_update_from_lidar()`
5. Test unitaire avec données simulées

### Phase 2 : Scanner 360° amélioré
1. Créer `src/scanner_360.c/h`
2. Implémenter calibration rotation avec odométrie
3. Intégrer avec occupancy_grid
4. Tester retour au cap initial

### Phase 3 : Stockage NVS robuste
1. Créer `src/map_storage.c/h`
2. Configurer partition flash
3. Implémenter save/load avec checksum CRC32
4. Tester persistance après reboot

### Phase 4 : Navigation frontier avancée
1. Créer `src/frontier_nav.c/h`
2. Implémenter scoring avec pénalité direction
3. Ajouter détection exploration complète
4. Test avec carte connue

### Phase 5 : Shell et export
1. Créer `src/map_shell.c`
2. Implémenter toutes les commandes
3. Format export compatible PC
4. Tester round-trip

### Phase 6 : Application mapper
1. Créer `apps/mapper/`
2. Intégrer tous les modules
3. Machine d'états complète
4. Test exploration réelle

---

## Fichiers à Créer

```
zephyr-app/
├── src/
│   ├── occupancy_grid.c    # NOUVEAU - 48×48 log-odds
│   ├── occupancy_grid.h    # NOUVEAU
│   ├── scanner_360.c       # NOUVEAU - Scan rotatif calibré
│   ├── scanner_360.h       # NOUVEAU
│   ├── frontier_nav.c      # NOUVEAU - Navigation frontier
│   ├── frontier_nav.h      # NOUVEAU
│   ├── map_storage.c       # NOUVEAU - NVS avec CRC
│   ├── map_storage.h       # NOUVEAU
│   └── map_shell.c         # NOUVEAU - Commandes shell
├── apps/
│   └── mapper/             # NOUVEAU
│       ├── CMakeLists.txt
│       ├── prj.conf
│       ├── Kconfig
│       ├── boards/
│       │   └── bbc_microbit_v2.overlay
│       └── src/
│           └── main.c
└── tools/
    └── map_viewer.py       # NOUVEAU - Visualisation PC
```

---

## Vérification

### Tests unitaires
- [ ] occupancy_grid : log-odds update correct
- [ ] occupancy_grid : Bresenham ray casting
- [ ] occupancy_grid : conversion world <-> cell
- [ ] scanner_360 : calibration rotation
- [ ] scanner_360 : retour cap initial
- [ ] frontier_nav : trouve direction optimale
- [ ] frontier_nav : détecte exploration complète
- [ ] map_storage : save/load avec CRC OK

### Tests système
- [ ] Scan 360° dans pièce avec obstacles
- [ ] Carte visible via `map show`
- [ ] Export shell parsable par `map_viewer.py`
- [ ] Exploration complète zone ~3m × 3m
- [ ] Carte persiste après reboot
- [ ] Log-odds convergent (zones visitées souvent → valeurs saturées)

### Commandes de test
```bash
# Activer virtualenv Zephyr
source ~/zephyrproject/.venv/bin/activate
export ZEPHYR_BASE=~/zephyrproject/zephyr

# Build
cd ~/Projects/Léandre/micro_bit/zephyr-app/apps/mapper
west build -b bbc_microbit_v2 . -- -DBOARD_ROOT="$(pwd)/../.."

# Flash
west flash

# Monitor shell
screen /dev/ttyACM0 115200

# Commandes shell:
uart:~$ map stats
uart:~$ map show 2
uart:~$ map export
uart:~$ map save
uart:~$ map cell 24 24
```

---

## Script PC pour Visualisation

```python
#!/usr/bin/env python3
"""
tools/map_viewer.py - Visualisation carte depuis shell Zephyr

Usage:
    python map_viewer.py                    # Lecture série
    python map_viewer.py --file map.txt     # Depuis fichier
    python map_viewer.py --plot             # Graphique matplotlib
"""
import serial
import sys
import argparse

def read_map_serial(port='/dev/ttyACM0', timeout=5):
    """Lit la carte depuis le port série"""
    ser = serial.Serial(port, 115200, timeout=timeout)
    ser.write(b'\r\nmap export\r\n')

    data = {}
    in_map = False

    for _ in range(100):
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if '---MAP-START---' in line:
            in_map = True
            continue
        if '---MAP-END---' in line:
            break
        if in_map and ':' in line:
            key, val = line.split(':', 1)
            data[key] = val

    ser.close()
    return data

def render_ascii(data, scale=1):
    """Affiche la carte en ASCII"""
    size = int(data.get('SIZE', 48))
    hex_str = data.get('DATA', '')

    # Convertir hex -> bytes (log-odds 8-bit ou compact 2-bit)
    try:
        grid = bytes.fromhex(hex_str)
    except:
        print("Invalid hex data")
        return

    # Déterminer format (log-odds = size², compact = size²/4)
    is_logodds = len(grid) == size * size

    chars = {
        'unknown': '.',
        'free': ' ',
        'occupied': '#',
        'robot': '@'
    }

    # Parse position robot
    pose = data.get('POSE', '0,0,0').split(',')
    robot_x = int(pose[0]) if len(pose) > 0 else 0
    robot_y = int(pose[1]) if len(pose) > 1 else 0
    cell_mm = int(data.get('CELL', 100))
    robot_cx = size // 2 + robot_x // cell_mm
    robot_cy = size // 2 + robot_y // cell_mm

    print(f"+{'-' * (size // scale)}+")

    for y in range(0, size, scale):
        row = '|'
        for x in range(0, size, scale):
            # Position robot
            if x <= robot_cx < x + scale and y <= robot_cy < y + scale:
                row += chars['robot']
                continue

            # Valeur cellule
            if is_logodds:
                idx = y * size + x
                val = grid[idx] if idx < len(grid) else 0
                # Convertir log-odds signé
                if val > 127:
                    val = val - 256

                if val < -20:
                    row += chars['free']
                elif val > 20:
                    row += chars['occupied']
                else:
                    row += chars['unknown']
            else:
                # Format compact 2-bit
                cell = y * size + x
                bi = cell // 4
                bo = (cell % 4) * 2
                val = (grid[bi] >> bo) & 3 if bi < len(grid) else 0
                states = [chars['unknown'], chars['free'], chars['occupied'], '?']
                row += states[val]

        print(row + '|')

    print(f"+{'-' * (size // scale)}+")

    # Stats
    if 'STATS' in data:
        stats = data['STATS'].split(',')
        print(f"Unknown: {stats[0]}, Free: {stats[1]}, Occupied: {stats[2]}")

    print(f"Pose: x={robot_x}mm, y={robot_y}mm, heading={pose[2] if len(pose) > 2 else 0}mdeg")

def render_plot(data):
    """Affiche avec matplotlib"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("matplotlib required: pip install matplotlib")
        return

    size = int(data.get('SIZE', 48))
    hex_str = data.get('DATA', '')
    grid = bytes.fromhex(hex_str)

    # Construire array numpy
    arr = np.zeros((size, size))
    is_logodds = len(grid) == size * size

    for y in range(size):
        for x in range(size):
            if is_logodds:
                val = grid[y * size + x]
                if val > 127:
                    val = val - 256
                arr[y, x] = val
            else:
                cell = y * size + x
                bi = cell // 4
                bo = (cell % 4) * 2
                val = (grid[bi] >> bo) & 3 if bi < len(grid) else 0
                arr[y, x] = [-50, -100, 100, 0][val]  # Map to log-odds-like

    plt.figure(figsize=(10, 10))
    plt.imshow(arr, cmap='RdYlGn_r', vmin=-128, vmax=127, origin='lower')
    plt.colorbar(label='Log-odds (- = free, + = occupied)')
    plt.title('Occupancy Grid')
    plt.xlabel('X (cells)')
    plt.ylabel('Y (cells)')
    plt.grid(True, alpha=0.3)
    plt.show()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Map viewer for Zephyr mapper')
    parser.add_argument('--port', default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--file', help='Read from file instead of serial')
    parser.add_argument('--plot', action='store_true', help='Show matplotlib plot')
    parser.add_argument('--scale', type=int, default=1, help='ASCII scale (1, 2, 4)')
    args = parser.parse_args()

    if args.file:
        # Parse from file
        data = {}
        with open(args.file) as f:
            for line in f:
                if ':' in line:
                    k, v = line.strip().split(':', 1)
                    data[k] = v
    else:
        print(f"Reading map from {args.port}...")
        data = read_map_serial(args.port)

    if not data:
        print("No data received")
        sys.exit(1)

    if args.plot:
        render_plot(data)
    else:
        render_ascii(data, args.scale)
```

---

## Résumé des Améliorations

| Aspect | Version Simple | Version Améliorée |
|--------|----------------|-------------------|
| Grille | 32×32, 2-bit (256B) | **48×48, 8-bit log-odds (2304B)** |
| Résolution | 150mm/cellule | **100mm/cellule** |
| Couverture | 4.8m × 4.8m | 4.8m × 4.8m |
| Probabilités | Non (états discrets) | **Oui (log-odds accumulatifs)** |
| Ray casting | Simple | **Bresenham complet** |
| Calibration rotation | Fixe | **Automatique avec odométrie** |
| Checksum NVS | Non | **CRC32** |
| Export | Basique | **Multi-format + viewer Python** |
| Mémoire RAM | ~21 KB | **~25 KB** |

Ce plan tire parti du budget mémoire disponible pour une **cartographie plus précise et robuste**.
