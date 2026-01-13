# Missing Zephyr Drivers for Maqueen Plus V3

Analysis of driver gaps between current Zephyr RTOS (v4.3.99) and Maqueen Plus V3 hardware requirements.

## Summary

| Priority | Driver | Status | Effort Estimate |
|----------|--------|--------|-----------------|
| **CRITICAL** | SEN0628 ToF Matrix | Not in Zephyr | New driver |
| **HIGH** | Maqueen Plus V3 Full | Partial (motors only) | Extension |
| **MEDIUM** | WS2812 GPIO timing | May need tuning | Configuration |
| **LOW** | LSM303AGR | Already available | None |

---

## 1. SEN0628 8x8 ToF LIDAR Matrix (CRITICAL)

### Status: **NOT AVAILABLE**

No driver exists in Zephyr for the DFRobot SEN0628 8x8 ToF matrix sensor.

### Similar Drivers in Zephyr

- **VL53L0X**: `/home/jm/zephyrproject/zephyr/drivers/sensor/st/vl53l0x/` - Single-point ToF
- **VL53L1X**: `/home/jm/zephyrproject/zephyr/drivers/sensor/st/vl53l1x/` - Single-point ToF, longer range

These are single-point sensors, not 8x8 matrix sensors.

### What Needs to Be Built

```
drivers/sensor/dfrobot/sen0628/
├── Kconfig
├── CMakeLists.txt
├── sen0628.c          # Driver implementation
├── sen0628.h          # Internal header
└── sen0628_regs.h     # Register definitions
```

### I2C Protocol (from MicroPython reverse engineering)

```
Address: 0x33 (default), configurable 0x30-0x33

Registers:
- 0x06: Set ranging mode (requires 5s settling time)
- CMD_FIXED_POINT: Read individual distance point
- Matrix is read point-by-point (bulk read unreliable)

Data format: 16-bit distance in mm per cell
```

### Development Tasks

1. Create device tree binding `dfrobot,sen0628.yaml`
2. Implement basic I2C communication
3. Add matrix read function (64 points)
4. Add mode configuration
5. Optimize read performance (currently slow due to point-by-point reads)
6. Consider using Zephyr sensor API vs custom API

---

## 2. Maqueen Plus V3 Motor Controller (HIGH)

### Status: **PARTIAL**

Basic motor control exists in line follower sample, but missing:
- Encoder reading
- Headlight control
- Proper driver structure

### Existing Code Location

```
/home/jm/zephyrproject/zephyr/samples/boards/bbc/microbit/line_follower_robot/
├── src/main.c                        # Inline motor control
├── dts/bindings/motor-controller.yaml # Generic binding
└── boards/bbc_microbit.overlay       # V1 only!
```

### What's Missing

| Feature | Register | Status |
|---------|----------|--------|
| Motor speed/direction | 0x00, 0x02 | Working |
| Encoder left | 0x04-0x05 | Missing |
| Encoder right | 0x06-0x07 | Missing |
| Headlight left | 0x0B | Missing |
| Headlight right | 0x0C | Missing |
| Clear encoder | 0x08 | Missing |
| Version | 0x32 | Missing |

### Development Tasks

1. Create proper driver (not inline code)
2. Create device tree binding `dfrobot,maqueen-plus-v3.yaml`
3. Implement encoder read/clear
4. Implement headlight control
5. Create micro:bit V2 overlay (only V1 exists)

---

## 3. WS2812 Underglow LEDs (MEDIUM)

### Status: **AVAILABLE** (configuration needed)

Zephyr has multiple WS2812 drivers. For micro:bit V2, best options:

| Driver | Path | Notes |
|--------|------|-------|
| `ws2812_gpio` | `drivers/led_strip/ws2812_gpio.c` | Bit-bang, most compatible |
| `ws2812_i2s` | `drivers/led_strip/ws2812_i2s.c` | nRF optimized |
| `ws2812_spi` | `drivers/led_strip/ws2812_spi.c` | Requires SPI pin |

### Configuration Required

Device tree overlay for micro:bit V2:

```dts
/ {
    underglow: ws2812 {
        compatible = "worldsemi,ws2812-gpio";
        chain-length = <4>;
        color-mapping = <LED_COLOR_ID_GREEN
                         LED_COLOR_ID_RED
                         LED_COLOR_ID_BLUE>;
        in-gpios = <&edge_connector 1 0>;  /* P1 */
    };
};
```

### Potential Issues

- GPIO bit-bang timing on nRF52833 may need tuning
- May conflict with other timing-sensitive operations
- Consider using I2S driver for better performance

---

## 4. LSM303AGR Accelerometer/Magnetometer (LOW)

### Status: **AVAILABLE**

Full driver exists and is already configured in micro:bit V2 board definition.

### Location

```
/home/jm/zephyrproject/zephyr/drivers/sensor/st/lsm303agr/
├── lsm303agr_accel.c
├── lsm303agr_magn.c
└── ...
```

### Device Tree (already in micro:bit V2)

```dts
&i2c0 {
    lsm303agr_accel: lsm303agr-accel@19 {
        compatible = "st,lsm303agr-accel";
        reg = <0x19>;
    };

    lsm303agr_magn: lsm303agr-magn@1e {
        compatible = "st,lsm303agr-magn";
        reg = <0x1e>;
    };
};
```

### Usage

```c
const struct device *accel = DEVICE_DT_GET(DT_NODELABEL(lsm303agr_accel));
struct sensor_value val[3];

sensor_sample_fetch(accel);
sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, val);
```

---

## Driver Development Priority Order

```
1. [CRITICAL] SEN0628 ToF Matrix Driver
   └── Blocks: Obstacle detection, navigation
   └── Effort: New driver from scratch

2. [HIGH] Maqueen Plus V3 Full Driver
   └── Blocks: Encoder-based odometry, headlights
   └── Effort: Extend existing sample code

3. [MEDIUM] WS2812 Configuration
   └── Blocks: Status indication only
   └── Effort: Device tree configuration

4. [LOW] LSM303AGR
   └── Blocks: Nothing (already working)
   └── Effort: None required
```

---

## Contribution Opportunity

The **SEN0628 driver** could be contributed upstream to Zephyr if developed following their coding standards:
- Path: `drivers/sensor/dfrobot/sen0628/`
- Device tree binding: `dts/bindings/sensor/dfrobot,sen0628.yaml`
- Vendor prefix: `dfrobot` (already in `dts/bindings/vendor-prefixes.txt`)

The **Maqueen Plus V3 driver** could enhance the existing sample or become a proper driver.

---

*Document created: 2026-01-11*
