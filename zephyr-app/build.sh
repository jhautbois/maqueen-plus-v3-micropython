#!/bin/bash
# Build script for Maqueen Plus V3 Zephyr application

set -e

ZEPHYR_BASE="${ZEPHYR_BASE:-/home/jm/zephyrproject/zephyr}"
BOARD="bbc_microbit_v2"
APP_DIR="$(dirname "$(readlink -f "$0")")"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== Maqueen Plus V3 Zephyr Build ===${NC}"

# Check Zephyr environment
if [ ! -d "$ZEPHYR_BASE" ]; then
    echo -e "${RED}Error: ZEPHYR_BASE not found at $ZEPHYR_BASE${NC}"
    echo "Set ZEPHYR_BASE environment variable or edit this script"
    exit 1
fi

# Source Zephyr environment
echo -e "${YELLOW}Sourcing Zephyr environment...${NC}"
source "$ZEPHYR_BASE/zephyr-env.sh"

# Parse arguments
FLASH=0
CLEAN=0
MONITOR=0

while [[ $# -gt 0 ]]; do
    case $1 in
        --flash|-f)
            FLASH=1
            shift
            ;;
        --clean|-c)
            CLEAN=1
            shift
            ;;
        --monitor|-m)
            MONITOR=1
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --flash, -f     Flash to micro:bit after build"
            echo "  --clean, -c     Clean build directory first"
            echo "  --monitor, -m   Open serial monitor after flash"
            echo "  --help, -h      Show this help"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# Clean if requested
if [ $CLEAN -eq 1 ]; then
    echo -e "${YELLOW}Cleaning build directory...${NC}"
    rm -rf "$APP_DIR/build"
fi

# Build
echo -e "${YELLOW}Building for $BOARD...${NC}"
cd "$APP_DIR"
west build -b "$BOARD" .

if [ $? -eq 0 ]; then
    echo -e "${GREEN}Build successful!${NC}"
    echo "Output: $APP_DIR/build/zephyr/zephyr.hex"

    # Show size
    echo ""
    echo -e "${YELLOW}Memory usage:${NC}"
    size "$APP_DIR/build/zephyr/zephyr.elf"
else
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

# Flash if requested
if [ $FLASH -eq 1 ]; then
    echo ""
    echo -e "${YELLOW}Flashing to micro:bit...${NC}"
    west flash

    if [ $? -eq 0 ]; then
        echo -e "${GREEN}Flash successful!${NC}"
    else
        echo -e "${RED}Flash failed!${NC}"
        exit 1
    fi
fi

# Monitor if requested
if [ $MONITOR -eq 1 ]; then
    echo ""
    echo -e "${YELLOW}Opening serial monitor (Ctrl-A K to exit)...${NC}"
    sleep 1  # Wait for device to enumerate

    # Find serial device
    if [ -e /dev/ttyACM0 ]; then
        SERIAL=/dev/ttyACM0
    elif [ -e /dev/ttyUSB0 ]; then
        SERIAL=/dev/ttyUSB0
    else
        echo -e "${RED}No serial device found${NC}"
        exit 1
    fi

    screen "$SERIAL" 115200
fi

echo ""
echo -e "${GREEN}Done!${NC}"
