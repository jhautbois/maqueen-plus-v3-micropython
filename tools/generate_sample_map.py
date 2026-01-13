#!/usr/bin/env python3
"""
Generate a sample map file for testing map_viewer.py

Creates a simulated room environment with:
- Walls around the edges
- A few obstacles inside
- Free space in the center
- Robot positioned near origin
"""

import sys

# Grid parameters
SIZE = 48
CELL_MM = 100

# Log-odds values (signed, stored as unsigned bytes)
UNKNOWN = 0       # 0x00 - neutral
FREE = 0xE0       # -32 as unsigned byte (free space)
OCCUPIED = 0x50   # +80 as unsigned byte (obstacle)


def create_sample_grid():
    """Create a sample 48x48 grid simulating a room with obstacles"""
    # Initialize as unknown
    grid = [[UNKNOWN for _ in range(SIZE)] for _ in range(SIZE)]

    # Create walls around the perimeter (rows 5-42, cols 5-42 = room)
    room_min = 8
    room_max = 40

    # Draw walls (occupied)
    for x in range(room_min, room_max):
        grid[room_min][x] = OCCUPIED  # Top wall
        grid[room_max-1][x] = OCCUPIED  # Bottom wall
    for y in range(room_min, room_max):
        grid[y][room_min] = OCCUPIED  # Left wall
        grid[y][room_max-1] = OCCUPIED  # Right wall

    # Fill room interior as free space
    for y in range(room_min + 1, room_max - 1):
        for x in range(room_min + 1, room_max - 1):
            grid[y][x] = FREE

    # Add some obstacles inside the room
    # Obstacle 1: Small box at (15, 15)
    for dy in range(3):
        for dx in range(3):
            grid[15 + dy][15 + dx] = OCCUPIED

    # Obstacle 2: L-shaped obstacle
    for i in range(5):
        grid[20][30 + i] = OCCUPIED
    for i in range(4):
        grid[20 + i][30] = OCCUPIED

    # Obstacle 3: Pillar at (30, 15)
    for dy in range(2):
        for dx in range(2):
            grid[30 + dy][15 + dx] = OCCUPIED

    # Mark robot's explored area around center (more certain free)
    center_y, center_x = SIZE // 2, SIZE // 2
    for dy in range(-3, 4):
        for dx in range(-3, 4):
            y, x = center_y + dy, center_x + dx
            if room_min < y < room_max - 1 and room_min < x < room_max - 1:
                grid[y][x] = 0xD0  # Very free (-48)

    return grid


def grid_to_hex(grid):
    """Convert 2D grid to hex string"""
    hex_chars = []
    for row in grid:
        for val in row:
            hex_chars.append(f"{val:02x}")
    return ''.join(hex_chars)


def save_map_file(filename, grid):
    """Save grid in map export format"""
    hex_data = grid_to_hex(grid)

    # Count cell types for stats
    flat = [c for row in grid for c in row]
    unknown = sum(1 for v in flat if -20 <= (v if v < 128 else v - 256) <= 20)
    free = sum(1 for v in flat if (v if v < 128 else v - 256) < -20)
    occupied = sum(1 for v in flat if (v if v < 128 else v - 256) > 20)

    with open(filename, 'w') as f:
        f.write("---MAP-START---\n")
        f.write("VERSION:2\n")
        f.write(f"SIZE:{SIZE}\n")
        f.write(f"CELL:{CELL_MM}\n")
        f.write(f"ORIGIN:{-SIZE * CELL_MM // 2},{-SIZE * CELL_MM // 2}\n")
        f.write("POSE:50,100,45000\n")  # Robot at (50mm, 100mm), heading 45 deg
        f.write(f"STATS:{unknown},{free},{occupied}\n")

        # Write data in 64-char lines
        f.write(f"DATA:{hex_data[:64]}\n")
        for i in range(64, len(hex_data), 64):
            f.write(f"     {hex_data[i:i+64]}\n")

        f.write("---MAP-END---\n")

    print(f"Generated sample map: {filename}")
    print(f"  Grid size: {SIZE}x{SIZE} = {SIZE*SIZE} cells")
    print(f"  Hex data: {len(hex_data)} characters ({len(hex_data)//2} bytes)")
    print(f"  Stats: {unknown} unknown, {free} free, {occupied} occupied")


if __name__ == '__main__':
    output = sys.argv[1] if len(sys.argv) > 1 else 'sample_map.txt'
    grid = create_sample_grid()
    save_map_file(output, grid)
