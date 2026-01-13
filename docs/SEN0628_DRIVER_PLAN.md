# SEN0628 Zephyr Driver - Detailed Implementation Plan

## Overview

This document provides a comprehensive implementation plan for the SEN0628 8x8 ToF LIDAR driver, with specific focus on SLAM (Simultaneous Localization and Mapping) requirements.

## SLAM Requirements Analysis

### What SLAM Needs from the LIDAR

1. **Full Matrix Data** - Complete 8x8 (64 point) distance readings
2. **Consistent Timing** - Known/predictable scan intervals for motion correlation
3. **High Frequency** - Faster reads = better motion estimation
4. **Confidence/Quality** - Distinguish valid from invalid readings
5. **Coordinate System** - Clear understanding of sensor orientation
6. **Multiple Read Modes** - Trade-off between speed and resolution

### Data Flow for SLAM

```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   SEN0628   │───►│   Driver    │───►│  SLAM Core  │───►│  Occupancy  │
│   Sensor    │    │  (Zephyr)   │    │  Algorithm  │    │    Grid     │
└─────────────┘    └─────────────┘    └─────────────┘    └─────────────┘
                          │
                   ┌──────┴──────┐
                   │             │
              ┌────▼────┐  ┌─────▼─────┐
              │ Odometry │  │ Gap Finder│
              │  Fusion  │  │ (NavV2)   │
              └─────────┘  └───────────┘
```

---

## Sensor Modes & Trade-offs

### Mode Comparison

| Mode | Points | FOV per Zone | Read Time* | Use Case |
|------|--------|--------------|------------|----------|
| 8x8 | 64 | 7.5° × 7.5° | ~2-3s | High-resolution mapping |
| 4x4 | 16 | 15° × 15° | ~0.5-1s | Fast obstacle avoidance |

*Using point-by-point reading with 30ms delay each

### Recommended Modes for SLAM

1. **Exploration Mode (8x8)** - Full resolution for map building
2. **Navigation Mode (4x4)** - Fast updates for real-time avoidance
3. **Hybrid Mode** - Alternate between modes based on robot state

---

## Driver Architecture

### Layer Structure

```
┌─────────────────────────────────────────────────────────┐
│                    Application Layer                     │
│  (SLAM, Navigation, Test App)                           │
├─────────────────────────────────────────────────────────┤
│                    Public API Layer                      │
│  sen0628_read_matrix(), sen0628_read_point(), etc.      │
├─────────────────────────────────────────────────────────┤
│                  Protocol Layer                          │
│  _send_packet(), _recv_packet(), packet parsing         │
├─────────────────────────────────────────────────────────┤
│                    I2C Layer                             │
│  Chunked transfers, timing, error handling              │
├─────────────────────────────────────────────────────────┤
│                 Zephyr I2C Driver                        │
│  i2c_write(), i2c_read(), i2c_transfer()               │
└─────────────────────────────────────────────────────────┘
```

### Data Structures

```c
/* Scan result with metadata for SLAM */
struct sen0628_scan {
    uint16_t distances[64];     /* Distance matrix (row-major) */
    uint32_t timestamp_ms;      /* Kernel uptime at scan start */
    uint32_t duration_ms;       /* Time to complete scan */
    uint8_t mode;               /* 4 or 8 */
    uint8_t valid_count;        /* Number of valid readings */
    uint8_t min_distance_idx;   /* Index of closest point */
    uint16_t min_distance;      /* Closest distance in scan */
};

/* Column statistics for obstacle avoidance */
struct sen0628_columns {
    uint16_t min[8];            /* Minimum distance per column */
    uint16_t avg[8];            /* Average distance per column */
    uint8_t valid_count[8];     /* Valid readings per column */
};

/* Driver runtime data */
struct sen0628_data {
    uint8_t mode;               /* Current mode (4 or 8) */
    uint8_t addr;               /* I2C address */
    struct sen0628_scan last_scan;  /* Most recent scan */
    bool initialized;
};
```

---

## API Design (Extended for SLAM)

### Core Functions

```c
/* Initialization */
int sen0628_init(const struct device *dev);

/* Mode control */
int sen0628_set_mode(const struct device *dev, uint8_t mode);
uint8_t sen0628_get_mode(const struct device *dev);

/* Single point read */
int sen0628_read_point(const struct device *dev,
                       uint8_t x, uint8_t y,
                       uint16_t *distance);

/* Full matrix read */
int sen0628_read_matrix(const struct device *dev,
                        uint16_t *distances);

/* Matrix read with metadata (for SLAM) */
int sen0628_read_scan(const struct device *dev,
                      struct sen0628_scan *scan);
```

### SLAM-Specific Functions

```c
/* Get column statistics for obstacle avoidance */
int sen0628_get_columns(const struct device *dev,
                        struct sen0628_columns *cols);

/* Get polar coordinates (angle, distance) for each point */
int sen0628_get_polar(const struct device *dev,
                      struct sen0628_polar *polar,
                      uint8_t count);

/* Check if specific sector is clear */
bool sen0628_sector_clear(const struct device *dev,
                          uint8_t sector,  /* 0-7 for 8 directions */
                          uint16_t min_distance);

/* Get minimum distance in field of view */
uint16_t sen0628_get_min_distance(const struct device *dev);
```

### Async/Callback Functions (Future)

```c
/* Start async scan (uses work queue) */
int sen0628_start_scan_async(const struct device *dev,
                             sen0628_scan_cb_t callback,
                             void *user_data);

/* Check if scan in progress */
bool sen0628_scan_busy(const struct device *dev);

/* Cancel ongoing scan */
int sen0628_cancel_scan(const struct device *dev);
```

---

## Coordinate System

### Sensor Orientation (Mounted on Maqueen)

```
        FRONT (Robot direction)
              ▲
              │
    ┌─────────┼─────────┐
    │ [0,0]   │   [7,0] │  ← Row 0 (top)
    │    ┌────┴────┐    │
    │    │ SENSOR  │    │
    │    │  FOV    │    │
    │    │  60°    │    │
    │    └────┬────┘    │
    │ [0,7]   │   [7,7] │  ← Row 7 (bottom)
    └─────────┼─────────┘
              │
        GROUND/FLOOR
```

### Column to Angle Mapping (8x8 mode)

| Column | Angle from Center | Direction |
|--------|-------------------|-----------|
| 0 | -26.25° | Far Left |
| 1 | -18.75° | Left |
| 2 | -11.25° | Slight Left |
| 3 | -3.75° | Center-Left |
| 4 | +3.75° | Center-Right |
| 5 | +11.25° | Slight Right |
| 6 | +18.75° | Right |
| 7 | +26.25° | Far Right |

### Row to Vertical Angle (8x8 mode)

| Row | Vertical Angle | What it sees |
|-----|----------------|--------------|
| 0 | +26.25° | Above horizon (ceiling/tall objects) |
| 1-2 | +11-19° | Upper obstacles |
| 3-4 | ±4° | Horizon (main obstacles) |
| 5-6 | -11-19° | Lower obstacles |
| 7 | -26.25° | Ground/floor |

**For SLAM/Navigation**: Rows 2-5 are most relevant for obstacle detection.

---

## Implementation Phases

### Phase 1: Driver Skeleton (Files & Build)

**Files to create:**
```
zephyr-app/
├── drivers/
│   └── sen0628/
│       ├── Kconfig
│       ├── CMakeLists.txt
│       ├── sen0628.c
│       ├── sen0628.h
│       └── sen0628_proto.h
├── dts/bindings/
│   └── dfrobot,sen0628.yaml
└── include/
    └── sen0628.h          (public API)
```

**Tasks:**
- [ ] 1.1 Create Kconfig with options
- [ ] 1.2 Create CMakeLists.txt
- [ ] 1.3 Create device tree binding
- [ ] 1.4 Create header files with API and constants
- [ ] 1.5 Create driver skeleton (init, device macros)
- [ ] 1.6 Verify build compiles

### Phase 2: I2C Protocol Layer

**Tasks:**
- [ ] 2.1 Implement packet building (`_build_packet`)
- [ ] 2.2 Implement packet sending with chunking (`_send_packet`)
- [ ] 2.3 Implement response receiving (`_recv_packet`)
- [ ] 2.4 Implement status parsing and error handling
- [ ] 2.5 Add retry logic with timeouts
- [ ] 2.6 Test basic I2C communication

### Phase 3: Core Sensor Functions

**Tasks:**
- [ ] 3.1 Implement `sen0628_set_mode()` with 5s delay
- [ ] 3.2 Implement `sen0628_read_point()`
- [ ] 3.3 Implement `sen0628_read_matrix()` (point-by-point)
- [ ] 3.4 Try `CMD_ALLDATA` bulk read (test if it works)
- [ ] 3.5 Implement `sen0628_read_scan()` with metadata
- [ ] 3.6 Add timing measurements

### Phase 4: SLAM Support Functions

**Tasks:**
- [ ] 4.1 Implement `sen0628_get_columns()` for obstacle avoidance
- [ ] 4.2 Implement polar coordinate conversion
- [ ] 4.3 Implement sector clearance check
- [ ] 4.4 Add scan statistics (min, max, avg)
- [ ] 4.5 Implement row filtering (ground rejection)

### Phase 5: Optimization

**Tasks:**
- [ ] 5.1 Measure actual read times
- [ ] 5.2 Test reduced delays (20ms, 15ms, 10ms)
- [ ] 5.3 Implement partial scan (selected columns only)
- [ ] 5.4 Add caching for recent readings
- [ ] 5.5 Profile memory usage

### Phase 6: Testing & Validation

**Tasks:**
- [ ] 6.1 Create comprehensive test application
- [ ] 6.2 Test all modes (4x4, 8x8)
- [ ] 6.3 Validate distance accuracy
- [ ] 6.4 Test at different I2C addresses
- [ ] 6.5 Long-duration stability test
- [ ] 6.6 Integration test with SLAM code

---

## Test Application Features

### Basic Tests
1. **Init Test** - Verify sensor detection and initialization
2. **Mode Test** - Switch between 4x4 and 8x8 modes
3. **Point Read Test** - Read individual points
4. **Matrix Read Test** - Full matrix scan with timing
5. **Continuous Scan Test** - Multiple scans with statistics

### Visualization
1. **ASCII Matrix** - Print distance matrix to console
2. **Column Bars** - Show column minimums as bar chart
3. **Obstacle Indicator** - Simple clear/blocked display

### Performance Tests
1. **Scan Timing** - Measure time for different operations
2. **Delay Optimization** - Find minimum stable delays
3. **Memory Usage** - Report RAM consumption

---

## Known Constraints

### Hardware Limitations

| Constraint | Value | Impact |
|------------|-------|--------|
| I2C chunk size | 32 bytes | Must chunk large transfers |
| Mode change delay | 5000ms | Avoid frequent mode changes |
| Point read delay | ~30ms | Limits scan rate |
| Max distance | 4000mm | Invalid readings = 4000 |
| Min distance | 20mm | Readings below are unreliable |

### Calculated Performance

| Mode | Points | Min Time | Max Rate |
|------|--------|----------|----------|
| 8x8 | 64 | 64 × 30ms = 1.92s | 0.5 Hz |
| 4x4 | 16 | 16 × 30ms = 0.48s | 2 Hz |

**Note**: Actual times may vary. Need to measure on hardware.

---

## Memory Budget

| Component | Size | Notes |
|-----------|------|-------|
| Distance buffer | 128 bytes | 64 × 16-bit |
| Scan metadata | 16 bytes | Timestamps, stats |
| Protocol buffers | 64 bytes | TX/RX buffers |
| Driver state | 32 bytes | Mode, address, etc |
| **Total** | ~256 bytes | Plus stack |

---

## Future Enhancements

### Short Term
- Async scanning with work queue
- Configurable delays via device tree
- Power management (standby mode)

### Medium Term
- Zephyr sensor API integration
- Multi-sensor support (cascaded sensors)
- DMA-based I2C transfers

### Long Term
- Direct VL53L7CX access (bypass RP2040)
- Custom RP2040 firmware for faster bulk reads
- Hardware trigger synchronization

---

## References

- [MicroPython driver](../src/laser_matrix.py)
- [DFRobot Arduino Library](https://github.com/DFRobot/DFRobot_MatrixLidar)
- [DFRobot Wiki](https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor)
- [VL53L7CX Datasheet](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf)
- [Existing task document](./SEN0628_DRIVER_TASK.md)

---

*Plan created: 2026-01-11*
*Target: BBC micro:bit V2 + DFRobot Maqueen Plus V3*
