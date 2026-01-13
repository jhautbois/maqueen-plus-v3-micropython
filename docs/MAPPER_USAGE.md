# Mapper Application - User Guide

## Overview

The mapper application allows the Maqueen Plus V3 robot to autonomously explore
and map its environment using the SEN0628 LIDAR sensor.

## Hardware Requirements

- micro:bit V2 on Maqueen Plus V3 robot
- SEN0628 LIDAR sensor connected to edge connector (P19/P20 = I2C1)
- Fully charged batteries

## Usage Sequence

```
┌─────────────────────────────────────────────────────────────┐
│  1. STARTUP                                                 │
│     - Place robot on the floor                              │
│     - Power on micro:bit                                    │
│     - Log shows "Ready - Press A"                           │
│     - State: IDLE                                           │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼ Button A
┌─────────────────────────────────────────────────────────────┐
│  2. 360° SCAN                                               │
│     - Robot rotates in place (6 positions × 60°)            │
│     - At each position: reads LIDAR 8×8 matrix              │
│     - Duration: ~5-6 seconds                                │
│     - Builds initial map around robot                       │
│     - State: SCANNING                                       │
└─────────────────────────────────────────────────────────────┘
                            │
                            ▼ Scan complete
┌─────────────────────────────────────────────────────────────┐
│  3. AUTONOMOUS EXPLORATION                                  │
│     - Loop ~10× per second:                                 │
│       1. Read LIDAR (obstacles ahead)                       │
│       2. Update odometry (estimated position)               │
│       3. Update grid (Bresenham ray casting)                │
│       4. Find "frontier" (nearest unknown area)             │
│       5. Navigate toward frontier                           │
│     - Real-time obstacle avoidance                          │
│     - State: EXPLORING                                      │
└─────────────────────────────────────────────────────────────┘
                            │
            ┌───────────────┼───────────────┐
            ▼               ▼               ▼
      Button A         Button B        Auto-stop
      (Pause)          (Save map)      (map complete)
```

## Button Controls

| Button | In IDLE | In EXPLORING | In PAUSED |
|--------|---------|--------------|-----------|
| A | Start scan | Pause | Resume |
| B | Save & export | Save progress | Save map |

## How the Map is Built

```
    LIDAR sees this:            Robot updates the grid:

         ┌───┐                    . . . . . . . .
         │WALL│                   . . # # # . . .  ← obstacle detected
         └───┘                    .           . .
           ▲                      .           . .
           │ 800mm                .     ↑     . .  ← robot here
           │                      .           . .
         ┌─┴─┐                    .           . .
         │ ↑ │                    . . . . . . . .
         └───┘
        Robot                  Bresenham traces a ray:
                               - cells traversed = FREE (space)
                               - final cell = OCCUPIED (wall)
```

## Frontier-Based Navigation

The robot always seeks the nearest **frontier** - the boundary between known and unknown areas:

```
    # # # # # # . . .
    #         # . . .     "." = unknown (not yet explored)
    #    ↑    # . . .     " " = free (explored, empty)
    #    R    →→→→→ ?     "#" = occupied (wall/obstacle)
    #         # . . .
    # # # # # # . . .     Robot moves toward "?" to explore
```

## Shell Commands

Connect via serial terminal: `screen /dev/ttyACM0 115200`

| Command | Description |
|---------|-------------|
| `map show` | Display ASCII map |
| `map show 2` | Display scaled map (1/2 size) |
| `map stats` | Show statistics (% explored) |
| `map info` | Grid parameters |
| `map cell <x> <y>` | Query specific cell |
| `map pose` | Robot position |
| `map save` | Save to flash (NVS) |
| `map load` | Load from flash |
| `map clear` | Clear map (keep in RAM) |
| `map erase` | Erase from flash |
| `map export` | Machine-readable export for PC |

## PC Visualization

Export the map and view on PC:

```bash
# From serial (robot connected)
cd tools
python3 map_viewer.py

# From saved file
python3 map_viewer.py --file map.txt

# Matplotlib visualization
python3 map_viewer.py --plot

# Save export to file
python3 map_viewer.py --save my_map.txt
```

## Exploration Stop Criteria

Exploration stops automatically when:
- Less than 5% of cells remain "unknown"
- Or no accessible frontier found

## Grid Specifications

| Parameter | Value |
|-----------|-------|
| Grid size | 48 × 48 cells |
| Cell size | 100 mm |
| Coverage | 4.8 m × 4.8 m |
| Storage | 2304 bytes (log-odds) |
| Origin | Center of grid |

## Log-Odds Representation

Each cell stores a probability value:
- **-128 to -21**: Free space (white in visualization)
- **-20 to +20**: Unknown (gray)
- **+21 to +127**: Occupied (black)

Updates use Bayesian inference:
- Ray passes through cell → decrease value (more likely free)
- Ray ends at cell → increase value (more likely occupied)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Robot doesn't start | Check I2C connections, verify LIDAR responds |
| Map looks wrong | Calibrate rotation timing (`scanner.ms_per_degree`) |
| Robot hits walls | Reduce exploration speed, check LIDAR alignment |
| Map not saved | Verify NVS partition in device tree overlay |
