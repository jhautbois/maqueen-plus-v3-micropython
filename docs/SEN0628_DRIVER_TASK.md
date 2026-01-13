# SEN0628 Zephyr Driver Development Task

## Executive Summary

Develop a Zephyr RTOS driver for the DFRobot SEN0628 8x8 Matrix ToF sensor, which uses the **RP2040 + VL53L7CX** architecture. The driver will communicate via I2C with the RP2040 firmware, not directly with the VL53L7CX chip.

**Critical Insight**: The SEN0628 has an onboard RP2040 that handles all VL53L7CX communication internally. We communicate with the RP2040's custom protocol, NOT the raw VL53L7CX registers.

---

## Hardware Architecture

```
┌─────────────────────────────────────────────────────┐
│              DFRobot SEN0628 Module                 │
│                                                     │
│  ┌───────────┐          ┌─────────────────────┐    │
│  │ VL53L7CX  │◄────────►│      RP2040         │    │
│  │  ToF Chip │  (SPI?)  │   (Firmware V1.3)   │    │
│  └───────────┘          └──────────┬──────────┘    │
│                                    │               │
│                              ┌─────┴─────┐         │
│                              │ I2C Slave │         │
│                              │ 0x30-0x33 │         │
│                              └─────┬─────┘         │
└────────────────────────────────────┼───────────────┘
                                     │
                               I2C Bus
                                     │
                              ┌──────┴──────┐
                              │ micro:bit V2│
                              │  (nRF52833) │
                              └─────────────┘
```

---

## Specifications

### Sensor Module (SEN0628)

| Parameter | Value |
|-----------|-------|
| Supply Voltage | 3.3V - 5V |
| I2C Address | 0x30, 0x31, 0x32, **0x33** (default) |
| Detection Range | 20mm - 4000mm |
| Matrix Modes | 4x4 (16 zones) or 8x8 (64 zones) |
| Field of View | 60° x 60° (90° diagonal) |
| Max Sample Rate | 60 Hz |
| Accuracy (20-200mm) | ±11mm (white), ±12mm (gray) |
| Accuracy (200-4000mm) | ±5% (white), ±6% (gray) |
| Invalid Value | 4000 (returned when no target) |
| Firmware Version | V1.3 (Sept 2025) |
| Host Processor | RP2040 |

### Underlying ToF Chip (VL53L7CX)

| Parameter | Value |
|-----------|-------|
| Manufacturer | STMicroelectronics |
| Technology | Time-of-Flight (ToF) SPAD array |
| Zones | 8x8 (64 zones) configurable |
| Max Range | 350cm |
| FOV | 60° x 60° (90° diagonal) |
| Frame Rate | Up to 60 Hz |

**Datasheet**: [VL53L7CX Datasheet (ST)](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf)

---

## I2C Protocol (RP2040 Firmware)

### Packet Format

**Command Packet (Host → Sensor)**:
```
┌──────┬───────────┬───────────┬─────┬────────────┐
│ Head │ ArgsNumH  │ ArgsNumL  │ Cmd │ Args[...]  │
│ 0x55 │  (MSB)    │  (LSB)    │     │            │
└──────┴───────────┴───────────┴─────┴────────────┘
```

**Response Packet (Sensor → Host)**:
```
┌────────┬─────┬────────┬────────┬─────────────┐
│ Status │ Cmd │ LenL   │ LenH   │ Data[...]   │
│        │     │ (LSB)  │ (MSB)  │             │
└────────┴─────┴────────┴────────┴─────────────┘
```

### Status Codes

| Code | Meaning |
|------|---------|
| 0x53 | Success |
| 0x63 | Failure |

### Commands

| Command | ID | Args | Response Data |
|---------|-----|------|---------------|
| `CMD_SETMODE` | 0x01 | `[mode]` (4 or 8) | Status only |
| `CMD_ALLDATA` | 0x02 | None | Matrix data (32 or 128 bytes) |
| `CMD_FIXED_POINT` | 0x03 | `[x, y]` | `[distL, distH]` (16-bit LE) |

### Timing Constraints

| Operation | Delay | Notes |
|-----------|-------|-------|
| After mode change | **5000ms** | Critical settling time |
| After CMD_FIXED_POINT | 30ms | Before reading response |
| I2C chunk read | 5ms | Between 32-byte chunks |
| I2C max transfer | 32 bytes | Chunk larger transfers |
| Receive timeout | 8000ms | Max wait for response |
| Inter-poll interval | 17ms | Retry delay |

### Data Format

**Matrix Data (8x8 mode = 128 bytes)**:
```
Index 0:  [y=0, x=0] distL, [y=0, x=0] distH
Index 2:  [y=0, x=1] distL, [y=0, x=1] distH
...
Index 126: [y=7, x=7] distL, [y=7, x=7] distH
```

Distance values are **16-bit little-endian** in millimeters.
Range: 0-4000mm (4000 = invalid/no target)

---

## Driver Design

### Option A: Custom SEN0628 Driver (Recommended)

This approach communicates directly with the RP2040 firmware protocol.

**Pros**:
- Simple, matches MicroPython driver
- No need to port ST's complex VL53L7CX ULD
- Works with existing firmware

**Cons**:
- Sensor-specific, not reusable
- No access to advanced VL53L7CX features

### Option B: VL53L5CX/L7CX Direct Driver

Port ST's ULD driver to bypass RP2040 and talk directly to VL53L7CX.

**Pros**:
- Access to full sensor capabilities
- Reusable for other VL53Lx sensors
- Could be upstreamed to Zephyr

**Cons**:
- Much more complex (large firmware blob upload)
- May not work without RP2040 firmware modifications
- Endianness issues reported on nRF platforms

**Recommendation**: Start with **Option A** for the Maqueen project. Option B is a separate long-term effort.

---

## File Structure

```
drivers/sensor/dfrobot/sen0628/
├── Kconfig
├── CMakeLists.txt
├── sen0628.c           # Main driver implementation
├── sen0628.h           # Internal header
└── sen0628_regs.h      # Protocol definitions

dts/bindings/sensor/
└── dfrobot,sen0628.yaml

include/zephyr/drivers/sensor/
└── sen0628.h           # Public API (optional)
```

---

## Implementation Tasks

### Phase 1: Skeleton & Build System

- [ ] **Task 1.1**: Create `Kconfig` with sensor options
- [ ] **Task 1.2**: Create `CMakeLists.txt`
- [ ] **Task 1.3**: Create device tree binding `dfrobot,sen0628.yaml`
- [ ] **Task 1.4**: Create basic driver skeleton with init function
- [ ] **Task 1.5**: Test build compiles for micro:bit V2

### Phase 2: I2C Communication Layer

- [ ] **Task 2.1**: Implement `_send_packet()` function
  - Build packet with header, length, command, args
  - Handle chunked writes for >32 bytes
  - Add inter-chunk delays

- [ ] **Task 2.2**: Implement `_recv_packet()` function
  - Read 4-byte header (status, cmd, lenL, lenH)
  - Read data in 32-byte chunks
  - Parse status and validate response

- [ ] **Task 2.3**: Implement timeout and retry logic
  - 8-second receive timeout
  - 17ms poll interval
  - Error handling

### Phase 3: Core Functionality

- [ ] **Task 3.1**: Implement `sen0628_set_mode()`
  - Send CMD_SETMODE with mode parameter
  - Wait 5 seconds after mode change
  - Store current mode in driver data

- [ ] **Task 3.2**: Implement `sen0628_read_point(x, y)`
  - Send CMD_FIXED_POINT with coordinates
  - Wait 30ms
  - Parse 16-bit LE distance

- [ ] **Task 3.3**: Implement `sen0628_read_matrix()`
  - Either read all 64 points individually
  - Or try CMD_ALLDATA (may not work reliably)
  - Store results in provided buffer

### Phase 4: Zephyr Sensor API Integration (Optional)

- [ ] **Task 4.1**: Implement `sensor_sample_fetch()`
  - Read entire matrix into driver data buffer

- [ ] **Task 4.2**: Implement `sensor_channel_get()`
  - Return distance values via sensor API
  - Define custom channel for matrix data

### Phase 5: Testing & Optimization

- [ ] **Task 5.1**: Create test application
- [ ] **Task 5.2**: Test on micro:bit V2 hardware
- [ ] **Task 5.3**: Measure and optimize read timing
- [ ] **Task 5.4**: Test 4x4 vs 8x8 modes
- [ ] **Task 5.5**: Test address switching (0x30-0x33)

---

## API Design

### Public Header

```c
/* include/zephyr/drivers/sensor/sen0628.h */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0628_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0628_H_

#include <zephyr/device.h>
#include <stdint.h>

#define SEN0628_MATRIX_4X4  4
#define SEN0628_MATRIX_8X8  8

#define SEN0628_MAX_DISTANCE    4000
#define SEN0628_INVALID_VALUE   4000

/**
 * @brief Set the matrix ranging mode
 *
 * @param dev Device handle
 * @param mode SEN0628_MATRIX_4X4 or SEN0628_MATRIX_8X8
 * @return 0 on success, negative errno on failure
 *
 * @note This function blocks for 5 seconds after mode change
 */
int sen0628_set_mode(const struct device *dev, uint8_t mode);

/**
 * @brief Get current matrix mode
 *
 * @param dev Device handle
 * @return Current mode (4 or 8)
 */
uint8_t sen0628_get_mode(const struct device *dev);

/**
 * @brief Read distance at a specific point
 *
 * @param dev Device handle
 * @param x X coordinate (0 to mode-1)
 * @param y Y coordinate (0 to mode-1)
 * @param distance Pointer to store distance in mm
 * @return 0 on success, negative errno on failure
 */
int sen0628_read_point(const struct device *dev,
                       uint8_t x, uint8_t y,
                       uint16_t *distance);

/**
 * @brief Read entire distance matrix
 *
 * @param dev Device handle
 * @param distances Buffer for distances (must be mode*mode elements)
 * @return 0 on success, negative errno on failure
 *
 * @note In 8x8 mode, this reads 64 values (takes ~2-3 seconds)
 */
int sen0628_read_matrix(const struct device *dev,
                        uint16_t *distances);

/**
 * @brief Read minimum distance per column
 *
 * @param dev Device handle
 * @param min_per_col Buffer for column minimums (mode elements)
 * @return 0 on success, negative errno on failure
 *
 * @note Useful for obstacle avoidance (8 columns = 8 directions)
 */
int sen0628_read_columns_min(const struct device *dev,
                             uint16_t *min_per_col);

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_SEN0628_H_ */
```

### Internal Protocol Definitions

```c
/* drivers/sensor/dfrobot/sen0628/sen0628_regs.h */

#ifndef SEN0628_REGS_H_
#define SEN0628_REGS_H_

/* Default I2C address */
#define SEN0628_DEFAULT_ADDR    0x33

/* Packet header */
#define SEN0628_PACKET_HEAD     0x55

/* Commands */
#define SEN0628_CMD_SETMODE     0x01
#define SEN0628_CMD_ALLDATA     0x02
#define SEN0628_CMD_FIXED_POINT 0x03

/* Status codes */
#define SEN0628_STATUS_SUCCESS  0x53
#define SEN0628_STATUS_FAILURE  0x63

/* Timing (milliseconds) */
#define SEN0628_MODE_SETTLE_MS  5000
#define SEN0628_READ_DELAY_MS   30
#define SEN0628_CHUNK_DELAY_MS  5
#define SEN0628_RECV_TIMEOUT_MS 8000
#define SEN0628_POLL_INTERVAL_MS 17

/* I2C constraints */
#define SEN0628_I2C_MAX_CHUNK   32

/* Matrix modes */
#define SEN0628_MODE_4X4        4
#define SEN0628_MODE_8X8        8

/* Distance limits */
#define SEN0628_DIST_MIN        20
#define SEN0628_DIST_MAX        4000
#define SEN0628_DIST_INVALID    4000

#endif /* SEN0628_REGS_H_ */
```

---

## Device Tree Binding

```yaml
# dts/bindings/sensor/dfrobot,sen0628.yaml

description: DFRobot SEN0628 8x8 Matrix ToF LIDAR Sensor

compatible: "dfrobot,sen0628"

include: [sensor-device.yaml, i2c-device.yaml]

properties:
  reg:
    required: true
    description: |
      I2C address. Default is 0x33.
      Configurable addresses: 0x30, 0x31, 0x32, 0x33

  initial-mode:
    type: int
    default: 8
    description: |
      Initial matrix mode.
      4 = 4x4 matrix (16 zones)
      8 = 8x8 matrix (64 zones, default)
    enum:
      - 4
      - 8
```

### Device Tree Example

```dts
&i2c0 {
    status = "okay";
    clock-frequency = <I2C_BITRATE_FAST>;

    sen0628: sen0628@33 {
        compatible = "dfrobot,sen0628";
        reg = <0x33>;
        initial-mode = <8>;
        label = "LIDAR";
    };
};
```

---

## Known Issues & Workarounds

### 1. CMD_ALLDATA Unreliable

The MicroPython driver shows that `CMD_ALLDATA` (bulk read) doesn't work reliably:

```python
def read_matrix(self):
    # Read all points individually (CMD_ALLDATA doesn't work)
    for y in range(self.mode):
        for x in range(self.mode):
            d = self.read_point(x, y)
```

**Workaround**: Use point-by-point reads with `CMD_FIXED_POINT`

### 2. 5-Second Mode Change Delay

After changing modes, the sensor requires a 5-second settling time:

```python
sleep(5000)  # Critical: 5 second delay after mode change
```

**Workaround**: Document this behavior, possibly use async/work queue

### 3. Slow Matrix Read (~2-3 seconds for 8x8)

Reading 64 points individually with 30ms delay each = ~2 seconds minimum

**Potential Optimizations**:
- Reduce delay if stable (test with 10-20ms)
- Try CMD_ALLDATA periodically (may work with newer firmware)
- Cache recent readings for interpolation

### 4. I2C 32-Byte Chunk Limit

The micro:bit/nRF I2C has buffer limitations:

```python
I2C_MAX = 32  # Maximum transferred data via I2C
```

**Workaround**: Chunk large transfers, add inter-chunk delays

---

## Testing Checklist

- [ ] Driver compiles without errors
- [ ] Device detected on I2C bus at 0x33
- [ ] `sen0628_set_mode(8)` succeeds
- [ ] `sen0628_read_point(0, 0)` returns valid distance
- [ ] `sen0628_read_matrix()` returns 64 values
- [ ] Distance readings match physical reality
- [ ] 4x4 mode works
- [ ] Alternative I2C addresses (0x30-0x32) work
- [ ] No I2C errors after extended operation
- [ ] Memory usage acceptable (~200 bytes data + stack)

---

## References

### Documentation

- [DFRobot SEN0628 Wiki](https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor)
- [DFRobot Product Page](https://www.dfrobot.com/product-2999.html)
- [VL53L7CX Product Page (ST)](https://www.st.com/en/imaging-and-photonics-solutions/vl53l7cx.html)
- [VL53L7CX Datasheet](https://www.st.com/resource/en/datasheet/vl53l7cx.pdf)

### Code References

- [DFRobot Arduino Library](https://github.com/DFRobot/DFRobot_MatrixLidar)
- [ST VL53L5CX Driver](https://github.com/STMicroelectronics/stm32-vl53l5cx)
- [X-CUBE-TOF1 (ST)](https://github.com/STMicroelectronics/x-cube-tof1)
- [VL53L5CX Zephyr Discussion](https://community.st.com/t5/imaging-sensors/vl53l5cx-with-zephyr-on-nrf-board/td-p/820955)

### Local Code

- MicroPython driver: `/home/jm/Projects/Léandre/micro_bit/src/laser_matrix.py`
- Zephyr installation: `/home/jm/zephyrproject`

---

## Estimated Effort

| Phase | Tasks | Complexity |
|-------|-------|------------|
| Phase 1: Skeleton | 5 tasks | Low |
| Phase 2: I2C Layer | 3 tasks | Medium |
| Phase 3: Core Functions | 3 tasks | Medium |
| Phase 4: Sensor API | 2 tasks | Low |
| Phase 5: Testing | 5 tasks | Medium |

**Dependencies**:
- Zephyr build environment configured
- micro:bit V2 hardware available
- SEN0628 sensor module

---

*Document created: 2026-01-11*
*Based on: DFRobot Arduino library analysis, MicroPython driver reverse engineering, ST VL53L7CX documentation*
