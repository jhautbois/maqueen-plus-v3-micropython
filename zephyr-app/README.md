# Maqueen Plus V3 Zephyr Test Application

Test application for DFRobot Maqueen Plus V3 robot on BBC micro:bit V2 using Zephyr RTOS.

## Features Tested

1. **WS2812 Underglow LEDs** - 4 RGB LEDs on P1
2. **Motor Control** - Left/right DC motors via STM8 controller (I2C 0x10)
3. **Headlights** - RGB headlights via STM8
4. **Encoders** - Wheel encoder reading
5. **Accelerometer** - LSM303AGR on-board sensor
6. **Rainbow Demo** - Animated LED pattern

## Prerequisites

### Zephyr Environment

```bash
# Source Zephyr environment (adjust path as needed)
export ZEPHYR_BASE=/home/jm/zephyrproject/zephyr
source $ZEPHYR_BASE/zephyr-env.sh
```

### Dependencies

- Zephyr SDK installed
- `west` command available
- micro:bit V2 connected via USB

## Building

```bash
# From this directory
west build -b bbc_microbit_v2 .

# Or with explicit board specification
west build -b bbc/microbit_v2 /home/jm/Projects/Léandre/micro_bit/zephyr-app
```

### Clean Build

```bash
west build -t clean
# or remove build directory
rm -rf build/
west build -b bbc_microbit_v2 .
```

## Flashing

```bash
# Flash to connected micro:bit
west flash

# Or copy HEX file manually
cp build/zephyr/zephyr.hex /media/$USER/MICROBIT/
```

## Monitoring Output

```bash
# Serial console (115200 baud)
screen /dev/ttyACM0 115200

# Or with minicom
minicom -D /dev/ttyACM0 -b 115200

# Or with west
west espressif monitor  # (if available)
```

Expected output:
```
========================================
Maqueen Plus V3 Zephyr Test Application
========================================
I2C ready, starting tests...

=== Test 1: Underglow LEDs ===
Cycling through 7 colors on 4 LEDs
Color 0: R=32 G=0 B=0
...

=== Test 2: Motors ===
Maqueen Plus V3 initialized (version: 0x01)
Moving forward (speed 100) for 1 second...
...
```

## Hardware Setup

Connect the Maqueen Plus V3 to micro:bit V2:
- Slot micro:bit into Maqueen's edge connector
- Power via USB or batteries
- Ensure sensor/motor switch is ON

### Pin Mapping

| Function | micro:bit Pin | nRF52833 GPIO |
|----------|---------------|---------------|
| Underglow LEDs | P1 | gpio0.3 |
| Left Line Sensor | P13 | gpio0.17 |
| Right Line Sensor | P14 | gpio0.1 |
| I2C SDA | P20 | gpio1.0 |
| I2C SCL | P19 | gpio0.26 |

### I2C Devices

| Device | Address | Description |
|--------|---------|-------------|
| STM8 Motor Controller | 0x10 | Motors, headlights, encoders |
| LSM303AGR Accel | 0x19 | On-board accelerometer |
| LSM303AGR Mag | 0x1E | On-board magnetometer |
| SEN0628 LIDAR | 0x33 | ToF sensor (if connected) |

## Project Structure

```
zephyr-app/
├── CMakeLists.txt          # Build configuration
├── prj.conf                # Kconfig options
├── README.md               # This file
├── boards/
│   └── bbc_microbit_v2.overlay  # Device tree overlay
├── dts/
│   └── bindings/
│       └── dfrobot,maqueen-motor.yaml  # DT binding
└── src/
    ├── main.c              # Test application
    ├── maqueen.c           # Motor controller driver
    └── maqueen.h           # Driver header
```

## Troubleshooting

### Build Errors

**"Zephyr not found"**
```bash
source /home/jm/zephyrproject/zephyr/zephyr-env.sh
```

**"Board not found"**
```bash
# Use full path
west build -b bbc_microbit_v2 .
# Note: some Zephyr versions use bbc_microbit_v2 (underscore)
```

### Runtime Errors

**"I2C device not ready"**
- Check micro:bit is properly seated in Maqueen
- Verify power to Maqueen

**"LED strip not ready"**
- Check WS2812 GPIO driver is enabled in prj.conf
- Verify P1 connection

**Motors don't move**
- Check Maqueen power switch
- Verify battery level
- Check I2C communication in logs

## Next Steps

1. **Add SEN0628 LIDAR Driver** - See `docs/SEN0628_DRIVER_TASK.md`
2. **Implement Navigation** - Port gap finder algorithm
3. **Add Shell Commands** - Interactive motor/LED control

## License

SPDX-License-Identifier: Apache-2.0
