# Zephyr RTOS Roadmap for DFRobot Maqueen Plus V3

This document outlines the plan to port the Maqueen Plus V3 robot control to Zephyr RTOS, running on micro:bit V2.

## Executive Summary

Zephyr RTOS provides a production-grade alternative to MicroPython for the Maqueen Plus V3. While some drivers exist, **several critical components need development**.

### Driver Availability Matrix

| Component | I2C Address | Zephyr Status | Priority |
|-----------|-------------|---------------|----------|
| STM8 Motor Controller | 0x10 | **Partial** - Basic sample exists | High |
| SEN0628 8x8 ToF LIDAR | 0x33 | **Missing** - Needs new driver | Critical |
| WS2812 Underglow LEDs | P1 (GPIO) | **Available** - ws2812_gpio driver | Medium |
| LSM303AGR Accel/Mag | Built-in | **Available** - Full driver exists | Low |
| Encoders (via STM8) | 0x10 | **Missing** - Needs extension | High |
| Headlights (via STM8) | 0x10 | **Missing** - Needs extension | Medium |

---

## Phase 1: Foundation (Immediate)

### 1.1 Project Setup

Create a Zephyr application structure:

```
maqueen-zephyr/
├── CMakeLists.txt
├── prj.conf
├── boards/
│   └── bbc_microbit_v2.overlay
├── dts/bindings/
│   ├── dfrobot,maqueen-plus-v3.yaml
│   └── dfrobot,sen0628.yaml
├── drivers/
│   ├── maqueen_plus_v3/
│   └── sen0628/
├── include/
│   └── maqueen.h
└── src/
    └── main.c
```

### 1.2 Extend Existing Motor Sample

The existing sample at `/home/jm/zephyrproject/zephyr/samples/boards/bbc/microbit/line_follower_robot/` targets micro:bit V1. We need:

1. Create `bbc_microbit_v2.overlay`:
```dts
/ {
    zephyr,user {
        left-gpios = <&edge_connector 13 GPIO_ACTIVE_HIGH>;
        right-gpios = <&edge_connector 14 GPIO_ACTIVE_HIGH>;
    };
};

&i2c0 {
    clock-frequency = <I2C_BITRATE_FAST>;

    motorctl: maqueen-plus-v3@10 {
        compatible = "dfrobot,maqueen-plus-v3";
        reg = <0x10>;
        label = "MAQUEEN";
    };
};
```

2. Test basic motor control works on V2 hardware

---

## Phase 2: Full Motor Controller Driver

### 2.1 Maqueen Plus V3 STM8 Protocol

The STM8 at address 0x10 handles:
- **Motors**: Registers 0x00 (left) and 0x02 (right)
- **Headlights**: Register 0x0B (color indices 0-6)
- **Encoders**: Registers 0x04-0x07 (16-bit counters)
- **Version**: Register 0x32

### 2.2 Driver API Design

```c
/* include/maqueen.h */
#ifndef MAQUEEN_H
#define MAQUEEN_H

#include <zephyr/device.h>

enum maqueen_motor {
    MAQUEEN_MOTOR_LEFT = 0,
    MAQUEEN_MOTOR_RIGHT = 1,
};

enum maqueen_direction {
    MAQUEEN_DIR_FORWARD = 0,
    MAQUEEN_DIR_REVERSE = 1,
};

enum maqueen_headlight {
    MAQUEEN_HEADLIGHT_LEFT = 0,
    MAQUEEN_HEADLIGHT_RIGHT = 1,
};

enum maqueen_headlight_color {
    MAQUEEN_COLOR_OFF = 0,
    MAQUEEN_COLOR_RED = 1,
    MAQUEEN_COLOR_GREEN = 2,
    MAQUEEN_COLOR_YELLOW = 3,
    MAQUEEN_COLOR_BLUE = 4,
    MAQUEEN_COLOR_PURPLE = 5,
    MAQUEEN_COLOR_CYAN = 6,
    MAQUEEN_COLOR_WHITE = 7,
};

/* Motor control */
int maqueen_motor_set(const struct device *dev,
                      enum maqueen_motor motor,
                      enum maqueen_direction dir,
                      uint8_t speed);

int maqueen_motor_stop(const struct device *dev,
                       enum maqueen_motor motor);

int maqueen_motor_stop_all(const struct device *dev);

/* Encoders */
int maqueen_encoder_get(const struct device *dev,
                        enum maqueen_motor motor,
                        int16_t *count);

int maqueen_encoder_clear(const struct device *dev,
                          enum maqueen_motor motor);

/* Headlights */
int maqueen_headlight_set(const struct device *dev,
                          enum maqueen_headlight light,
                          enum maqueen_headlight_color color);

/* Version */
int maqueen_get_version(const struct device *dev, uint8_t *version);

#endif /* MAQUEEN_H */
```

### 2.3 Device Tree Binding

```yaml
# dts/bindings/dfrobot,maqueen-plus-v3.yaml
description: DFRobot Maqueen Plus V3 robot motor controller

compatible: "dfrobot,maqueen-plus-v3"

include: i2c-device.yaml

properties:
  underglow-gpios:
    type: phandle-array
    required: false
    description: GPIO pin for WS2812 underglow LEDs
```

---

## Phase 3: SEN0628 ToF LIDAR Driver (Critical)

### 3.1 Sensor Overview

The DFRobot SEN0628 is an 8x8 ToF matrix sensor based on VL53L5CX or similar. It provides 64-point depth data.

**I2C Protocol** (based on MicroPython driver analysis):
- Address: 0x33 (configurable 0x30-0x33)
- Mode command: 0x06 (set ranging mode)
- Data read: Individual point reads via CMD_FIXED_POINT

### 3.2 Driver Implementation Plan

```c
/* drivers/sen0628/sen0628.h */
#define SEN0628_MATRIX_SIZE 8
#define SEN0628_TOTAL_POINTS 64

struct sen0628_config {
    struct i2c_dt_spec i2c;
    uint8_t ranging_mode;
};

struct sen0628_data {
    uint16_t distances[SEN0628_MATRIX_SIZE][SEN0628_MATRIX_SIZE];
};
```

### 3.3 Sensor API Options

**Option A: Custom Driver API**
```c
int sen0628_read_matrix(const struct device *dev,
                        uint16_t distances[8][8]);

int sen0628_read_column(const struct device *dev,
                        uint8_t col,
                        uint16_t distances[8]);

int sen0628_set_mode(const struct device *dev, uint8_t mode);
```

**Option B: Zephyr Sensor API Extension**
```c
/* Use SENSOR_CHAN_PROX or custom channel */
sensor_sample_fetch(dev);
sensor_channel_get(dev, SENSOR_CHAN_DISTANCE, &val);
```

### 3.4 Device Tree Binding

```yaml
# dts/bindings/dfrobot,sen0628.yaml
description: DFRobot SEN0628 8x8 ToF LIDAR Matrix Sensor

compatible: "dfrobot,sen0628"

include: i2c-device.yaml

properties:
  ranging-mode:
    type: int
    required: false
    default: 1
    description: |
      Ranging mode (affects speed vs accuracy):
      0 = Short range
      1 = Medium range (default)
      2 = Long range
```

---

## Phase 4: WS2812 Underglow Integration

### 4.1 Existing Driver

Zephyr has multiple WS2812 drivers:
- `ws2812_gpio.c` - Bit-bang via GPIO (most compatible)
- `ws2812_spi.c` - SPI-based (faster, requires SPI)
- `ws2812_i2s.c` - I2S-based (nRF52 optimized)

### 4.2 Device Tree Configuration

```dts
/ {
    underglow: ws2812 {
        compatible = "worldsemi,ws2812-gpio";
        chain-length = <4>;  /* Maqueen has 4 underglow LEDs */
        color-mapping = <LED_COLOR_ID_GREEN
                         LED_COLOR_ID_RED
                         LED_COLOR_ID_BLUE>;
        in-gpios = <&edge_connector 1 0>;  /* P1 */
    };
};
```

### 4.3 Usage

```c
#include <zephyr/drivers/led_strip.h>

const struct device *strip = DEVICE_DT_GET(DT_NODELABEL(underglow));

struct led_rgb pixels[4] = {
    { .r = 255, .g = 0, .b = 0 },
    { .r = 0, .g = 255, .b = 0 },
    { .r = 0, .g = 0, .b = 255 },
    { .r = 255, .g = 255, .b = 255 },
};

led_strip_update_rgb(strip, pixels, ARRAY_SIZE(pixels));
```

---

## Phase 5: Obstacle Avoidance Application

### 5.1 Architecture

```
┌─────────────────────────────────────────────────┐
│                  Application                     │
│  ┌───────────┐ ┌───────────┐ ┌───────────────┐  │
│  │ Navigator │ │Gap Finder │ │Fall Detector  │  │
│  └─────┬─────┘ └─────┬─────┘ └───────┬───────┘  │
│        │             │               │          │
├────────┼─────────────┼───────────────┼──────────┤
│        ▼             ▼               ▼          │
│  ┌───────────┐ ┌───────────┐ ┌───────────────┐  │
│  │ Motor Drv │ │ ToF Drv   │ │ Sensor API    │  │
│  │ (maqueen) │ │ (sen0628) │ │ (lsm303agr)   │  │
│  └───────────┘ └───────────┘ └───────────────┘  │
│                    Drivers                       │
├─────────────────────────────────────────────────┤
│                Zephyr Kernel                     │
│  (threads, timers, work queues, I2C, GPIO)      │
└─────────────────────────────────────────────────┘
```

### 5.2 Threading Model

```c
/* Main threads */
K_THREAD_DEFINE(sensor_thread, 1024,
                sensor_thread_entry, NULL, NULL, NULL,
                K_PRIO_COOP(5), 0, 0);

K_THREAD_DEFINE(nav_thread, 1024,
                navigation_thread_entry, NULL, NULL, NULL,
                K_PRIO_COOP(6), 0, 0);

/* Work queue for non-time-critical tasks */
K_WORK_DEFINE(led_update_work, led_update_handler);
```

### 5.3 Sample Application Structure

```c
#include <zephyr/kernel.h>
#include <maqueen.h>
#include <sen0628.h>
#include <zephyr/drivers/sensor.h>

const struct device *maqueen = DEVICE_DT_GET(DT_NODELABEL(motorctl));
const struct device *lidar = DEVICE_DT_GET(DT_NODELABEL(lidar));
const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));

void main(void)
{
    if (!device_is_ready(maqueen) ||
        !device_is_ready(lidar) ||
        !device_is_ready(accel)) {
        printk("Devices not ready\n");
        return;
    }

    while (1) {
        uint16_t distances[8][8];
        sen0628_read_matrix(lidar, distances);

        /* Find best gap */
        int best_col = find_best_gap(distances);

        /* Navigate toward gap */
        navigate_to_gap(maqueen, best_col);

        k_msleep(50);  /* 20 Hz loop */
    }
}
```

---

## Development Phases & Dependencies

```
Phase 1: Foundation
    │
    ├── Task 1.1: Create project structure
    ├── Task 1.2: Test V2 build with existing sample
    └── Task 1.3: Create V2 overlay
          │
          ▼
Phase 2: Motor Driver
    │
    ├── Task 2.1: Create DT binding
    ├── Task 2.2: Implement driver skeleton
    ├── Task 2.3: Add motor control
    ├── Task 2.4: Add encoder support
    └── Task 2.5: Add headlight control
          │
          ▼
Phase 3: ToF LIDAR Driver  ◄── CRITICAL PATH
    │
    ├── Task 3.1: Reverse engineer I2C protocol
    ├── Task 3.2: Create DT binding
    ├── Task 3.3: Implement basic driver
    ├── Task 3.4: Add matrix read support
    └── Task 3.5: Optimize read performance
          │
          ▼
Phase 4: LED Integration
    │
    ├── Task 4.1: Configure WS2812 overlay
    └── Task 4.2: Test underglow control
          │
          ▼
Phase 5: Application
    │
    ├── Task 5.1: Port gap finder algorithm
    ├── Task 5.2: Port fall detector
    ├── Task 5.3: Implement navigation
    └── Task 5.4: Test complete system
```

---

## Known Challenges

### 1. SEN0628 Protocol Documentation

The MicroPython driver shows the sensor requires:
- 5-second delay after mode changes
- Individual point reads (bulk read unreliable)
- Careful timing between reads

We need to verify this works similarly in Zephyr or find optimizations.

### 2. Real-Time Constraints

MicroPython had garbage collection pauses. Zephyr eliminates this, but we need to:
- Profile I2C transaction times
- Ensure sensor reads complete within timing budget
- Balance thread priorities

### 3. Memory Budget

micro:bit V2 has 128 KB RAM. Budget estimate:
- Zephyr kernel: ~20-30 KB
- Drivers: ~5-10 KB
- Application: ~10-20 KB
- Stack space: ~10 KB
- Distance matrix buffer: 128 bytes
- **Available**: ~60-80 KB margin

---

## References

### Existing Zephyr Resources

- **Line follower sample**: `/home/jm/zephyrproject/zephyr/samples/boards/bbc/microbit/line_follower_robot/`
- **micro:bit V2 board**: `/home/jm/zephyrproject/zephyr/boards/bbc/microbit_v2/`
- **WS2812 driver**: `/home/jm/zephyrproject/zephyr/drivers/led_strip/ws2812_gpio.c`
- **VL53L0X driver** (similar ToF): `/home/jm/zephyrproject/zephyr/drivers/sensor/st/vl53l0x/`
- **LSM303AGR driver**: `/home/jm/zephyrproject/zephyr/drivers/sensor/st/lsm303agr/`

### Hardware Documentation

- [DFRobot Maqueen Plus V3 Wiki](https://wiki.dfrobot.com/SKU_MBT0021-EN_Maqueen_Plus_V3)
- [DFRobot SEN0628 Wiki](https://wiki.dfrobot.com/SKU_SEN0628_8x8_ToF_Matrix_Sensor)
- [micro:bit V2 Pinout](https://tech.microbit.org/hardware/schematic/)

---

## Next Steps

1. **Immediate**: Set up Zephyr build environment and test existing line follower on V2
2. **Short-term**: Develop Maqueen Plus V3 full driver with encoders/headlights
3. **Medium-term**: Develop SEN0628 ToF matrix driver (critical path)
4. **Long-term**: Port complete obstacle avoidance application

---

*Document created: 2026-01-11*
*Zephyr version: 4.3.99*
*Target: BBC micro:bit V2 + DFRobot Maqueen Plus V3*
