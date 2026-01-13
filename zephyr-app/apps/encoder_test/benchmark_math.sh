#!/bin/bash
# Benchmark: Compare math implementations for odometry
# Usage: ./benchmark_math.sh

set -e

APP_DIR="$(cd "$(dirname "$0")" && pwd)"
SRC_DIR="$APP_DIR/../../src"
BUILD_BASE="/tmp/zephyr-math-benchmark"

# Source Zephyr environment
source ~/zephyrproject/zephyr/zephyr-env.sh

echo "=============================================="
echo "  Math Implementation Benchmark"
echo "=============================================="
echo ""

# Backup original files
cp "$SRC_DIR/odometry.c" "$SRC_DIR/odometry.c.bak"
cp "$APP_DIR/prj.conf" "$APP_DIR/prj.conf.bak"

cleanup() {
    echo ""
    echo "Restoring original files..."
    mv "$SRC_DIR/odometry.c.bak" "$SRC_DIR/odometry.c"
    mv "$APP_DIR/prj.conf.bak" "$APP_DIR/prj.conf"
}
trap cleanup EXIT

# ============================================
# Test 1: Lookup Table (current implementation)
# ============================================
echo ">>> Building: LOOKUP TABLE (integer math)"
echo "    - No FPU, no libc math"
echo ""

cat > "$APP_DIR/prj.conf" << 'EOF'
# Lookup table version - minimal libc
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_SERIAL=y
CONFIG_PRINTK=y
EOF

rm -rf "$BUILD_BASE/lookup"
west build -b bbc_microbit_v2 "$APP_DIR" -d "$BUILD_BASE/lookup" -- -DBOARD_ROOT="$APP_DIR/../.."

LOOKUP_FLASH=$(arm-none-eabi-size "$BUILD_BASE/lookup/zephyr/zephyr.elf" | tail -1 | awk '{print $1}')
LOOKUP_RAM=$(arm-none-eabi-size "$BUILD_BASE/lookup/zephyr/zephyr.elf" | tail -1 | awk '{print $2}')
LOOKUP_TOTAL=$(arm-none-eabi-size "$BUILD_BASE/lookup/zephyr/zephyr.elf" | tail -1 | awk '{print $4}')

echo ""
echo "   LOOKUP: Flash=$LOOKUP_FLASH, RAM=$LOOKUP_RAM, Total=$LOOKUP_TOTAL"
echo ""

# ============================================
# Test 2: Picolibc + FPU
# ============================================
echo ">>> Building: PICOLIBC + FPU"
echo "    - Hardware FPU, picolibc math"
echo ""

# Switch to math.h version
cp "$SRC_DIR/odometry_math.c" "$SRC_DIR/odometry.c"

cat > "$APP_DIR/prj.conf" << 'EOF'
# Picolibc + FPU version
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_SERIAL=y
CONFIG_PRINTK=y

# Enable FPU and picolibc
CONFIG_FPU=y
CONFIG_PICOLIBC=y
EOF

rm -rf "$BUILD_BASE/picolibc"
west build -b bbc_microbit_v2 "$APP_DIR" -d "$BUILD_BASE/picolibc" -- -DBOARD_ROOT="$APP_DIR/../.." 2>&1 || {
    echo "   PICOLIBC build failed (may not be available)"
    PICO_FLASH="N/A"
    PICO_RAM="N/A"
    PICO_TOTAL="N/A"
}

if [ -f "$BUILD_BASE/picolibc/zephyr/zephyr.elf" ]; then
    PICO_FLASH=$(arm-none-eabi-size "$BUILD_BASE/picolibc/zephyr/zephyr.elf" | tail -1 | awk '{print $1}')
    PICO_RAM=$(arm-none-eabi-size "$BUILD_BASE/picolibc/zephyr/zephyr.elf" | tail -1 | awk '{print $2}')
    PICO_TOTAL=$(arm-none-eabi-size "$BUILD_BASE/picolibc/zephyr/zephyr.elf" | tail -1 | awk '{print $4}')
    echo ""
    echo "   PICOLIBC: Flash=$PICO_FLASH, RAM=$PICO_RAM, Total=$PICO_TOTAL"
fi
echo ""

# ============================================
# Test 3: Newlib + FPU
# ============================================
echo ">>> Building: NEWLIB + FPU"
echo "    - Hardware FPU, newlib math"
echo ""

cat > "$APP_DIR/prj.conf" << 'EOF'
# Newlib + FPU version
CONFIG_I2C=y
CONFIG_GPIO=y
CONFIG_LOG=y
CONFIG_LOG_DEFAULT_LEVEL=3
CONFIG_LOG_MODE_IMMEDIATE=y
CONFIG_CONSOLE=y
CONFIG_UART_CONSOLE=y
CONFIG_SERIAL=y
CONFIG_PRINTK=y

# Enable FPU and newlib
CONFIG_FPU=y
CONFIG_NEWLIB_LIBC=y
EOF

rm -rf "$BUILD_BASE/newlib"
west build -b bbc_microbit_v2 "$APP_DIR" -d "$BUILD_BASE/newlib" -- -DBOARD_ROOT="$APP_DIR/../.." 2>&1 || {
    echo "   NEWLIB build failed (may not be available)"
    NEWLIB_FLASH="N/A"
    NEWLIB_RAM="N/A"
    NEWLIB_TOTAL="N/A"
}

if [ -f "$BUILD_BASE/newlib/zephyr/zephyr.elf" ]; then
    NEWLIB_FLASH=$(arm-none-eabi-size "$BUILD_BASE/newlib/zephyr/zephyr.elf" | tail -1 | awk '{print $1}')
    NEWLIB_RAM=$(arm-none-eabi-size "$BUILD_BASE/newlib/zephyr/zephyr.elf" | tail -1 | awk '{print $2}')
    NEWLIB_TOTAL=$(arm-none-eabi-size "$BUILD_BASE/newlib/zephyr/zephyr.elf" | tail -1 | awk '{print $4}')
    echo ""
    echo "   NEWLIB: Flash=$NEWLIB_FLASH, RAM=$NEWLIB_RAM, Total=$NEWLIB_TOTAL"
fi
echo ""

# ============================================
# Summary
# ============================================
echo "=============================================="
echo "  RESULTS SUMMARY"
echo "=============================================="
echo ""
printf "%-12s %10s %10s %10s\n" "Method" "Flash" "RAM" "Total"
printf "%-12s %10s %10s %10s\n" "--------" "-------" "-------" "-------"
printf "%-12s %10s %10s %10s\n" "Lookup" "$LOOKUP_FLASH" "$LOOKUP_RAM" "$LOOKUP_TOTAL"
printf "%-12s %10s %10s %10s\n" "Picolibc" "${PICO_FLASH:-N/A}" "${PICO_RAM:-N/A}" "${PICO_TOTAL:-N/A}"
printf "%-12s %10s %10s %10s\n" "Newlib" "${NEWLIB_FLASH:-N/A}" "${NEWLIB_RAM:-N/A}" "${NEWLIB_TOTAL:-N/A}"
echo ""

if [ "$PICO_FLASH" != "N/A" ] && [ -n "$PICO_FLASH" ]; then
    DIFF_PICO=$((PICO_FLASH - LOOKUP_FLASH))
    echo "Picolibc vs Lookup: +$DIFF_PICO bytes Flash"
fi
if [ "$NEWLIB_FLASH" != "N/A" ] && [ -n "$NEWLIB_FLASH" ]; then
    DIFF_NEWLIB=$((NEWLIB_FLASH - LOOKUP_FLASH))
    echo "Newlib vs Lookup: +$DIFF_NEWLIB bytes Flash"
fi
echo ""
echo "micro:bit V2 has 512KB Flash, 128KB RAM"
echo "=============================================="
