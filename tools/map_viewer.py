#!/usr/bin/env python3
"""
Map Viewer for Zephyr Mapper Application

Reads occupancy grid from micro:bit serial port and displays it.

Usage:
    python map_viewer.py                    # Read from serial, ASCII display
    python map_viewer.py --port /dev/ttyACM0
    python map_viewer.py --plot             # Matplotlib visualization
    python map_viewer.py --file map.txt     # Read from saved file
    python map_viewer.py --save map.txt     # Save export to file

Requirements:
    pip install pyserial matplotlib
"""

import argparse
import sys
import time


def read_map_serial(port='/dev/ttyACM0', timeout=10):
    """Read map from micro:bit serial port using shell command"""
    try:
        import serial
    except ImportError:
        print("Error: pyserial not installed. Run: pip install pyserial")
        sys.exit(1)

    print(f"Connecting to {port}...")
    try:
        ser = serial.Serial(port, 115200, timeout=timeout)
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

    # Clear any pending data
    ser.reset_input_buffer()
    time.sleep(0.1)

    # Send map export command
    print("Requesting map export...")
    ser.write(b'\r\n')
    time.sleep(0.1)
    ser.write(b'map export\r\n')

    # Read response
    data = {}
    in_map = False
    start_time = time.time()

    while time.time() - start_time < timeout:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception:
            continue

        if not line:
            continue

        if '---MAP-START---' in line:
            in_map = True
            continue

        if '---MAP-END---' in line:
            break

        if in_map:
            if line.startswith('DATA:'):
                data['DATA'] = line[5:].strip()
            elif 'DATA' in data and line and line[0] in '0123456789abcdefABCDEF':
                # DATA continuation line (hex chars only)
                data['DATA'] += line.strip()
            elif ':' in line:
                key, val = line.split(':', 1)
                data[key] = val.strip()

    ser.close()

    if not data:
        print("No map data received")
        return None

    return data


def read_map_file(filename):
    """Read map from saved file"""
    data = {}

    try:
        with open(filename, 'r') as f:
            in_map = False
            for line in f:
                line = line.strip()

                if '---MAP-START---' in line:
                    in_map = True
                    continue

                if '---MAP-END---' in line:
                    break

                if in_map:
                    if line.startswith('DATA:'):
                        data['DATA'] = line[5:].strip()
                    elif 'DATA' in data and line and line[0] in '0123456789abcdefABCDEF':
                        # DATA continuation line (hex chars only)
                        data['DATA'] += line.strip()
                    elif ':' in line:
                        key, val = line.split(':', 1)
                        data[key] = val.strip()
    except FileNotFoundError:
        print(f"File not found: {filename}")
        return None

    return data


def save_map_file(data, filename):
    """Save map data to file"""
    with open(filename, 'w') as f:
        f.write("---MAP-START---\n")
        for key, val in data.items():
            if key == 'DATA':
                # Split long data lines
                f.write(f"DATA:{val[:64]}\n")
                for i in range(64, len(val), 64):
                    f.write(f"     {val[i:i+64]}\n")
            else:
                f.write(f"{key}:{val}\n")
        f.write("---MAP-END---\n")

    print(f"Map saved to {filename}")


def parse_grid(data):
    """Parse grid data from hex string"""
    size = int(data.get('SIZE', 48))
    hex_str = data.get('DATA', '')

    if not hex_str:
        return None, size

    try:
        grid = bytes.fromhex(hex_str)
    except ValueError:
        print(f"Invalid hex data (length={len(hex_str)})")
        return None, size

    return grid, size


def render_ascii(data, scale=1, show_legend=True):
    """Render map as ASCII art"""
    grid, size = parse_grid(data)

    if grid is None:
        print("No grid data to display")
        return

    display_size = size // scale
    is_logodds = len(grid) == size * size

    # Parse robot position
    pose_str = data.get('POSE', '0,0,0')
    try:
        pose = [int(x) for x in pose_str.split(',')]
        robot_x, robot_y = pose[0], pose[1]
        robot_heading = pose[2] if len(pose) > 2 else 0
    except (ValueError, IndexError):
        robot_x, robot_y, robot_heading = 0, 0, 0

    # Cell size for robot position calculation
    cell_mm = int(data.get('CELL', 100))
    origin_str = data.get('ORIGIN', f'-{size * cell_mm // 2},-{size * cell_mm // 2}')
    try:
        origin = [int(x) for x in origin_str.split(',')]
        origin_x, origin_y = origin[0], origin[1]
    except (ValueError, IndexError):
        origin_x = -(size * cell_mm // 2)
        origin_y = -(size * cell_mm // 2)

    # Robot cell position
    robot_cx = (robot_x - origin_x) // cell_mm
    robot_cy = (robot_y - origin_y) // cell_mm

    # Direction characters for robot
    dir_chars = ['↑', '↗', '→', '↘', '↓', '↙', '←', '↖']
    robot_dir = ((robot_heading // 1000) + 22) // 45 % 8

    # Header
    print(f"\n{'='*60}")
    print(f"Occupancy Grid ({size}x{size}, {cell_mm}mm/cell)")
    print(f"Robot: ({robot_x}mm, {robot_y}mm) @ {robot_heading//1000}°")
    print(f"{'='*60}\n")

    # Grid border
    print("+" + "-" * display_size + "+")

    # Grid rows (Y increases downward in display)
    for y in range(display_size):
        row = "|"

        for x in range(display_size):
            # Sample center of scaled cell
            gx = x * scale + scale // 2
            gy = y * scale + scale // 2

            # Check if robot is here
            if (gx // scale == robot_cx // scale and
                gy // scale == robot_cy // scale):
                row += dir_chars[robot_dir]
                continue

            # Get cell value
            if is_logodds:
                idx = gy * size + gx
                if idx < len(grid):
                    val = grid[idx]
                    # Convert unsigned to signed
                    if val > 127:
                        val = val - 256

                    if val < -20:
                        row += ' '  # Free
                    elif val > 20:
                        row += '#'  # Occupied
                    else:
                        row += '.'  # Unknown
                else:
                    row += '?'
            else:
                # Compact 2-bit format
                cell = gy * size + gx
                bi = cell // 4
                bo = (cell % 4) * 2
                if bi < len(grid):
                    state = (grid[bi] >> bo) & 3
                    chars = ['.', ' ', '#', '?']
                    row += chars[state]
                else:
                    row += '?'

        print(row + "|")

    # Footer
    print("+" + "-" * display_size + "+")

    # Statistics
    if 'STATS' in data:
        try:
            stats = [int(x) for x in data['STATS'].split(',')]
            unknown, free, occupied = stats[0], stats[1], stats[2]
            total = unknown + free + occupied
            print(f"\nCells: {total} total")
            print(f"  Unknown:  {unknown:4d} ({100*unknown/total:.1f}%)")
            print(f"  Free:     {free:4d} ({100*free/total:.1f}%)")
            print(f"  Occupied: {occupied:4d} ({100*occupied/total:.1f}%)")
        except (ValueError, IndexError):
            pass

    if show_legend:
        print(f"\nLegend: '.' unknown, ' ' free, '#' occupied, '{dir_chars[0]}' robot")


def render_plot(data):
    """Render map using matplotlib"""
    try:
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Error: matplotlib not installed. Run: pip install matplotlib")
        sys.exit(1)

    grid, size = parse_grid(data)

    if grid is None:
        print("No grid data to display")
        return

    is_logodds = len(grid) == size * size

    # Build numpy array
    arr = np.zeros((size, size))

    for y in range(size):
        for x in range(size):
            if is_logodds:
                idx = y * size + x
                if idx < len(grid):
                    val = grid[idx]
                    if val > 127:
                        val = val - 256
                    arr[y, x] = val
            else:
                cell = y * size + x
                bi = cell // 4
                bo = (cell % 4) * 2
                if bi < len(grid):
                    state = (grid[bi] >> bo) & 3
                    # Map states to log-odds-like values
                    arr[y, x] = [-50, -100, 100, 0][state]

    # Parse robot position
    pose_str = data.get('POSE', '0,0,0')
    try:
        pose = [int(x) for x in pose_str.split(',')]
        robot_x, robot_y = pose[0], pose[1]
    except (ValueError, IndexError):
        robot_x, robot_y = 0, 0

    cell_mm = int(data.get('CELL', 100))
    robot_cx = size // 2 + robot_x // cell_mm
    robot_cy = size // 2 + robot_y // cell_mm

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 10))

    # Plot grid
    im = ax.imshow(arr, cmap='RdYlGn_r', vmin=-128, vmax=127,
                   origin='lower', extent=[0, size, 0, size])

    # Mark robot position
    ax.plot(robot_cx + 0.5, robot_cy + 0.5, 'bo', markersize=10, label='Robot')

    # Labels and title
    ax.set_xlabel('X (cells)')
    ax.set_ylabel('Y (cells)')
    ax.set_title(f'Occupancy Grid ({size}x{size}, {cell_mm}mm/cell)')

    # Colorbar
    cbar = plt.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label('Log-odds (- = free, + = occupied)')

    # Grid lines
    ax.set_xticks(range(0, size + 1, size // 8))
    ax.set_yticks(range(0, size + 1, size // 8))
    ax.grid(True, alpha=0.3)

    ax.legend()

    plt.tight_layout()
    plt.show()


def main():
    parser = argparse.ArgumentParser(
        description='Map viewer for Zephyr Mapper application',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python map_viewer.py                    # Read from serial, ASCII display
    python map_viewer.py --port /dev/ttyACM0
    python map_viewer.py --plot             # Matplotlib visualization
    python map_viewer.py --file map.txt     # Read from saved file
    python map_viewer.py --save map.txt     # Save export to file
""")

    parser.add_argument('--port', '-p', default='/dev/ttyACM0',
                        help='Serial port (default: /dev/ttyACM0)')
    parser.add_argument('--file', '-f',
                        help='Read from file instead of serial')
    parser.add_argument('--save', '-s',
                        help='Save export to file')
    parser.add_argument('--plot', action='store_true',
                        help='Show matplotlib plot')
    parser.add_argument('--scale', type=int, default=1, choices=[1, 2, 4],
                        help='ASCII display scale (1, 2, or 4)')
    parser.add_argument('--timeout', type=int, default=10,
                        help='Serial timeout in seconds')

    args = parser.parse_args()

    # Get map data
    if args.file:
        print(f"Reading from file: {args.file}")
        data = read_map_file(args.file)
    else:
        data = read_map_serial(args.port, args.timeout)

    if not data:
        print("Failed to get map data")
        sys.exit(1)

    # Save if requested
    if args.save:
        save_map_file(data, args.save)

    # Display
    if args.plot:
        render_plot(data)
    else:
        render_ascii(data, args.scale)


if __name__ == '__main__':
    main()
