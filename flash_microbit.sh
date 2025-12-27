#!/bin/bash
# Script to automatically mount micro:bit and flash code

set -e  # Exit on error

MOUNT_POINT="/run/media/$USER/MICROBIT"
DEVICE="/dev/sda"

echo "=== micro:bit Auto Flash Tool ==="
echo ""

# Function to check if device exists
check_device() {
    if [ -b "$DEVICE" ]; then
        echo "✓ micro:bit detected at $DEVICE"
        return 0
    else
        echo "✗ micro:bit not found at $DEVICE"
        return 1
    fi
}

# Function to mount micro:bit
mount_microbit() {
    echo "Checking mount status..."

    if mount | grep -q "$DEVICE"; then
        MOUNT_POINT=$(mount | grep "$DEVICE" | awk '{print $3}')
        echo "✓ Already mounted at: $MOUNT_POINT"
        return 0
    fi

    echo "Mounting micro:bit with udisksctl..."

    # Use udisksctl (no sudo required)
    udisksctl mount -b "$DEVICE"

    if [ $? -eq 0 ]; then
        # Get the actual mount point
        sleep 1
        MOUNT_POINT=$(mount | grep "$DEVICE" | awk '{print $3}')
        echo "✓ Mounted at: $MOUNT_POINT"
        return 0
    else
        echo "✗ Failed to mount"
        return 1
    fi
}

# Function to flash file
flash_file() {
    local file=$1

    if [ ! -f "$file" ]; then
        echo "✗ File not found: $file"
        return 1
    fi

    echo "Flashing $file to micro:bit..."

    # Get current mount point
    CURRENT_MOUNT=$(mount | grep "$DEVICE" | awk '{print $3}')

    if [ -z "$CURRENT_MOUNT" ]; then
        echo "✗ micro:bit not mounted"
        return 1
    fi

    uflash "$file" "$CURRENT_MOUNT/"

    if [ $? -eq 0 ]; then
        echo "✓ Flash successful!"
        return 0
    else
        echo "✗ Flash failed"
        return 1
    fi
}

# Function to upload libraries
upload_libs() {
    local dir=$1
    echo ""
    echo "Uploading libraries..."

    # Wait for micro:bit to be ready after flash
    sleep 3

    for lib in "maqueen_plus_v3.py" "laser_matrix.py"; do
        if [ -f "$dir/$lib" ]; then
            echo "  Uploading $lib..."
            ufs put "$dir/$lib" 2>/dev/null || {
                echo "  Retrying $lib..."
                sleep 1
                ufs put "$dir/$lib"
            }
        fi
    done
    echo "✓ Libraries uploaded!"
}

# Main workflow
main() {
    # Check if device exists
    if ! check_device; then
        echo ""
        echo "Please plug in the micro:bit and try again"
        exit 1
    fi

    echo ""

    # Mount if necessary
    if ! mount_microbit; then
        exit 1
    fi

    echo ""

    # Flash file if provided
    if [ -n "$1" ]; then
        if ! flash_file "$1"; then
            exit 1
        fi

        # Get directory of flashed file
        FILE_DIR=$(dirname "$1")

        # Upload libraries if they exist in the same directory
        if [ -f "$FILE_DIR/maqueen_plus_v3.py" ] || [ -f "$FILE_DIR/laser_matrix.py" ]; then
            upload_libs "$FILE_DIR"
        fi

        echo ""
        echo "✓ All done!"
        echo ""
        echo "To view output, run:"
        echo "  screen /dev/ttyACM0 115200"
        echo "  (or: make repl)"
        echo ""
        echo "Then press RESET button on micro:bit"
    else
        echo "✓ micro:bit is ready"
        echo ""
        echo "Usage: $0 <file.py>"
        echo "Example: $0 build/main.py"
    fi
}

# Run main with all arguments
main "$@"
