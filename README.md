# Maqueen Plus V3 MicroPython Library

MicroPython library and programs for the DFRobot Maqueen Plus V3 robot, controlled by micro:bit V2.

## Features

- **Complete API** for all Maqueen Plus V3 hardware
- **8x8 Laser ToF Matrix** sensor support (SEN0628)
- **Obstacle avoidance** using LIDAR
- **Line following** with 5 IR sensors
- **Light seeking** behavior
- **IR remote control** (template)
- **CLI workflow** with Makefile
- **Type stubs** for Zed editor autocomplete

## Quick Start

### 1. Install Tools

```bash
# Python packages
pip install uflash microfs

# Serial terminal (choose one)
sudo apt-get install screen picocom
```

### 2. Clone or Download Project

```bash
cd /path/to/maqueen-v3-project
```

### 3. Deploy Obstacle Avoidance Program

```bash
make all          # Flash main.py + upload libraries
```

### 4. Connect to REPL (optional)

```bash
make repl         # View debug output
```

Press button A on micro:bit to start the obstacle avoidance program.

## Project Structure

```
├── src/
│   ├── main.py              # Obstacle avoidance program (default)
│   ├── maqueen_plus_v3.py   # Main robot library
│   ├── laser_matrix.py      # ToF sensor driver
│   ├── test_laser.py        # Laser sensor diagnostic
│   └── i2c_scanner.py       # I2C diagnostic tool
├── examples/
│   ├── line_follower.py     # Line tracking program
│   ├── light_seeker.py      # Light-following program
│   └── remote_control.py    # IR remote template
├── docs/
│   ├── QUICK_START.md       # Getting started guide
│   └── SOURCES.md           # Reference links
├── stubs/
│   └── microbit.pyi         # Type hints for Zed editor
├── Makefile                 # Build automation
├── build.py                 # Minification build script
├── build/                   # Minified output (generated)
├── flash_microbit.sh        # Auto-mount and flash utility
└── README.md                # This file
```

## Hardware Components

### DFRobot Maqueen Plus V3

- **micro:bit V2** controller (~100KB RAM for MicroPython)
- **STM8 motor controller** (I2C 0x10)
  - 2x N20 metal gear motors with encoders
  - PID control support
- **Laser ToF Matrix Sensor** (I2C 0x33 default)
  - 8x8 grid = 64 distance points
  - Range: 20-4000mm
  - 60° FOV, 15Hz update rate
- **5x IR line sensors** (L2, L1, M, R1, R2)
- **2x ambient light sensors** (left/right)
- **2x RGB headlights** (controlled via STM8)
- **4x WS2812 underglow LEDs** (micro:bit pin 15)
- **IR receiver** for remote control
- **Battery**: 18650 Li-ion or 4xAA

### Pinout Reference

- **Pin 15**: WS2812 underglow LEDs (4 LEDs)
- **Pin 8**: IR receiver (optional)
- **I2C**: STM8 (0x10), Laser sensor (0x33)

## API Reference

### Basic Usage

```python
from maqueen_plus_v3 import MaqueenPlusV3

robot = MaqueenPlusV3()
```

### Motor Control

```python
# Drive forward/backward
robot.drive(150)           # Speed: -255 to 255

# Turn in place
robot.turn(100)            # Speed: -255 (left) to 255 (right)

# Independent motor control
robot.motors(left, right)  # Each: -255 to 255

# Stop
robot.stop()
```

### Laser ToF Sensor

```python
# Get simplified zones (left, center, right)
zones = robot.laser_zones()
print(zones['center'])     # Distance in mm

# Get full 8x8 matrix
matrix = robot.laser_matrix()  # List of 64 distances

# Get center distance only
distance = robot.laser_distance()
```

### Line Sensors

```python
# Analog values (0-1023)
sensors = robot.line_sensors()  # [L2, L1, M, R1, R2]

# Digital (0/1) with threshold
digital = robot.line_digital(threshold=512)
```

### Light Sensors

```python
left, right = robot.light_sensors()  # 0-1023
```

### RGB LEDs

```python
# Headlights (via STM8)
robot.headlights('red', 'green')
robot.headlights('off', 'off')

# Underglow (WS2812)
robot.underglow(0, 255, 0, 0)      # LED 0 = red
robot.underglow('all', 0, 255, 0)  # All = green
robot.underglow_off()
```

### Encoders

```python
left, right = robot.read_encoders()  # Encoder counts
speed = robot.speed()                # Delta since last read
robot.reset_encoders()
```

### Utility

```python
robot.self_test()           # Run diagnostic test
version = robot.get_version()  # STM8 firmware version
```

## Makefile Commands

### Build & Deploy (Minified)

The project includes a build system that minifies Python files before deployment, reducing file size while keeping source code readable.

```bash
make build        # Minify src/ files to build/ directory
make deploy       # Build, flash, and upload minified files
```

**Requirements:** `pip install python-minifier`

The minification typically reduces file sizes by 40-60%, which helps with micro:bit's limited storage.

### Development (Non-minified)

```bash
make all          # Flash main.py + upload libraries (from src/)
make flash        # Flash main.py only
make upload       # Upload library files
make ls           # List files on micro:bit
make clean        # Remove libraries from micro:bit
```

### Debugging

```bash
make repl         # Open serial REPL (Ctrl-A K to exit)
make scan         # Run I2C scanner
make test         # Deploy self-test program
make check        # Check tool installation
```

### Deploy Examples

```bash
make obstacle         # Obstacle avoidance (default)
make line-follower    # Line tracking
make light-seeker     # Light following
make remote-control   # IR remote (template)
```

## Programming Guide

### Obstacle Avoidance

The default `main.py` implements obstacle avoidance:

1. Continuously scans with LIDAR (15 Hz)
2. Divides FOV into 3 zones (left, center, right)
3. Stops if center obstacle < 25cm
4. Reverses if any obstacle < 15cm
5. Turns toward clearest direction
6. LED indicators:
   - **Green underglow**: Path clear
   - **Red underglow**: Obstacle detected
   - **Display arrows**: Current direction

**Controls:**
- Button A: Start/Stop
- Button B: Emergency stop

### Line Following

See `examples/line_follower.py`:

- PID controller for smooth tracking
- Calibrate sensors with "Calc-Key" button first
- Adjust `LINE_THRESHOLD` for your surface
- Tune `kp`, `ki`, `kd` constants for performance

### Light Seeking

See `examples/light_seeker.py`:

- Follows bright light source (flashlight, lamp)
- Differential drive for smooth turning
- Stops when reaching light source

### Custom Programs

```python
from microbit import display, sleep, Image
from maqueen_plus_v3 import MaqueenPlusV3

robot = MaqueenPlusV3()

while True:
    zones = robot.laser_zones()

    if zones['center'] < 200:  # Obstacle close
        robot.stop()
        display.show(Image.SAD)
    else:
        robot.drive(100)
        display.show(Image.HAPPY)

    sleep(100)
```

## MicroPython Constraints

### Important Limitations

- **NO `time` module**: Use `microbit.sleep(ms)` instead
- **I2C**: Use `microbit.i2c`, NOT `machine.I2C`
- **No standard library**: No `struct`, `json`, `re`
- **Memory**: Keep code modular, ~100KB RAM on V2
- **File size**: Split large programs into modules

### Available Modules

- `microbit` - Main hardware API
- `neopixel` - WS2812 LEDs
- `radio` - micro:bit to micro:bit wireless
- `music` - Audio/buzzer
- `speech` - Text-to-speech
- `machine` - Limited low-level access

## Troubleshooting

### Sensor Not Found

```bash
# Run I2C scanner to identify devices
make scan
make repl  # View results
```

Expected addresses:
- **0x10**: STM8 motor controller (required)
- **0x33**: Laser ToF sensor (default, configurable 0x30-0x33)

### Laser Sensor Issues

1. Check physical connection
2. Verify I2C address with scanner
3. Check DIP switches on sensor (address selection)
4. Power cycle robot after address changes
5. Update `laser_matrix.py` with correct protocol if needed

### Motor Not Working

1. Check battery level
2. Verify STM8 detected (I2C scan)
3. Ensure speed values are within -255 to 255
4. Check motor wiring connections

### File Upload Fails

```bash
# Check micro:bit connection
make check

# Clean and retry
make clean
make upload
```

### REPL Connection Issues

```bash
# Find correct port
ls /dev/tty* | grep -E "(ACM|USB)"

# Manually specify port (edit Makefile)
PORT = /dev/ttyACM0
```

## Laser Sensor Notes

The `laser_matrix.py` driver implements communication with the SEN0628 Matrix Laser Ranging Sensor:

- **Protocol**: Command-based packet protocol over I2C
- **Modes**: 8x8 (64 points, ~0.5Hz due to individual reads) and 4x4 (16 points)
- **Timing**: 5-second delay required after mode change

The driver reads points individually as CMD_ALLDATA bulk read was not working reliably. This results in slower update rates but provides reliable data.

See DFRobot's Arduino library for reference:
- https://github.com/DFRobot/DFRobot_MatrixLidar

## Contributing

This is a personal project, but feel free to:

1. Fork and adapt for your own use
2. Report issues with specific hardware/firmware versions
3. Share improvements to sensor drivers
4. Add more example programs

## References

- **Maqueen Plus V3 Wiki**: https://wiki.dfrobot.com/SKU_MBT0050_Maqueen_Plus_V3
- **SEN0628 Laser Sensor**: https://wiki.dfrobot.com/SKU_SEN0628_Matrix%20Laser%20Ranging%20Sensor
- **MakeCode Library**: https://github.com/DFRobot/pxt-DFRobot_MaqueenPlus_v20
- **micro:bit MicroPython**: https://microbit-micropython.readthedocs.io/
- **Community MicroPython Libraries**:
  - https://github.com/jdonwells/micropython-MaqueenPlusV2
  - https://github.com/almasy/micropython-maqueen-plus

## License

This project is provided as-is for educational purposes. DFRobot hardware and documentation are subject to their respective licenses.

---

**Built for micro:bit V2 + DFRobot Maqueen Plus V3**
