/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Occupancy Grid implementation
 * 48x48 grid with 8-bit log-odds and Bresenham ray casting
 */

#include "occupancy_grid.h"
#include "odometry.h"
#include "sen0628.h"

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(occupancy_grid, LOG_LEVEL_INF);

/* Use Zephyr's CLAMP macro from sys/util.h */

/*
 * LIDAR column angles (in millidegrees from center)
 * SEN0628 8x8 mode: columns span approximately -26° to +26° (60° FOV)
 * Column 0 = far left, Column 7 = far right
 */
#define LIDAR_FOV_MDEG      60000   /* 60 degrees total FOV */
#define LIDAR_HALF_FOV_MDEG 30000   /* 30 degrees each side */
#define LIDAR_NUM_COLS      8

/* Column angle offsets from robot heading (millidegrees) */
static const int32_t col_angle_mdeg[LIDAR_NUM_COLS] = {
    -26250,  /* Column 0: far left */
    -18750,  /* Column 1 */
    -11250,  /* Column 2 */
    -3750,   /* Column 3: center-left */
    3750,    /* Column 4: center-right */
    11250,   /* Column 5 */
    18750,   /* Column 6 */
    26250    /* Column 7: far right */
};

/*
 * Sin/Cos lookup table (scaled by 1000, 1-degree resolution)
 * From odometry.c - reused for consistency
 */
static const int16_t sin_table[] = {
    0, 17, 35, 52, 70, 87, 105, 122, 139, 156,        /* 0-9 */
    174, 191, 208, 225, 242, 259, 276, 292, 309, 326, /* 10-19 */
    342, 358, 375, 391, 407, 423, 438, 454, 469, 485, /* 20-29 */
    500, 515, 530, 545, 559, 574, 588, 602, 616, 629, /* 30-39 */
    643, 656, 669, 682, 695, 707, 719, 731, 743, 755, /* 40-49 */
    766, 777, 788, 799, 809, 819, 829, 839, 848, 857, /* 50-59 */
    866, 875, 883, 891, 899, 906, 914, 921, 927, 934, /* 60-69 */
    940, 946, 951, 956, 961, 966, 970, 974, 978, 982, /* 70-79 */
    985, 988, 990, 993, 995, 996, 998, 999, 999, 1000, /* 80-89 */
    1000  /* 90 */
};

/**
 * @brief Get sin value for angle in millidegrees, scaled by 1000
 */
static int32_t sin_mdeg(int32_t mdeg)
{
    int32_t deg = mdeg / 1000;
    int32_t sign = 1;

    /* Normalize to 0-359 */
    while (deg < 0) {
        deg += 360;
    }
    while (deg >= 360) {
        deg -= 360;
    }

    /* Map to first quadrant */
    if (deg > 270) {
        deg = 360 - deg;
        sign = -1;
    } else if (deg > 180) {
        deg = deg - 180;
        sign = -1;
    } else if (deg > 90) {
        deg = 180 - deg;
    }

    return sign * sin_table[deg];
}

/**
 * @brief Get cos value for angle in millidegrees, scaled by 1000
 */
static int32_t cos_mdeg(int32_t mdeg)
{
    return sin_mdeg(mdeg + 90000);
}

/* ==================== Initialization ==================== */

void grid_init(struct occupancy_grid *grid)
{
    memset(grid->data, LOG_ODDS_PRIOR, GRID_BYTES);

    /* Set origin so that grid center (24,24) = world (0,0) */
    grid->origin_x = -(GRID_SIZE / 2) * CELL_MM;
    grid->origin_y = -(GRID_SIZE / 2) * CELL_MM;
    grid->update_count = 0;

    LOG_INF("Grid initialized: %dx%d, %dmm/cell, coverage %dmm",
            GRID_SIZE, GRID_SIZE, CELL_MM, GRID_COVERAGE);
}

void grid_clear(struct occupancy_grid *grid)
{
    memset(grid->data, LOG_ODDS_PRIOR, GRID_BYTES);
    grid->update_count = 0;
    LOG_INF("Grid cleared");
}

/* ==================== Cell Access ==================== */

int8_t grid_get_logodds(const struct occupancy_grid *grid, int cx, int cy)
{
    if (!grid_is_valid(cx, cy)) {
        return LOG_ODDS_PRIOR;
    }
    return grid->data[cy * GRID_SIZE + cx];
}

void grid_set_logodds(struct occupancy_grid *grid, int cx, int cy, int8_t value)
{
    if (!grid_is_valid(cx, cy)) {
        return;
    }
    grid->data[cy * GRID_SIZE + cx] = value;
}

cell_state_t grid_get_state(const struct occupancy_grid *grid, int cx, int cy)
{
    int8_t lo = grid_get_logodds(grid, cx, cy);

    if (lo > THRESHOLD_OCCUPIED) {
        return CELL_OCCUPIED;
    } else if (lo < THRESHOLD_FREE) {
        return CELL_FREE;
    }
    return CELL_UNKNOWN;
}

/* ==================== Coordinate Conversion ==================== */

void grid_world_to_cell(const struct occupancy_grid *grid,
                        int32_t world_x, int32_t world_y,
                        int *cell_x, int *cell_y)
{
    /* Cell = (world - origin) / cell_size */
    *cell_x = (world_x - grid->origin_x) / CELL_MM;
    *cell_y = (world_y - grid->origin_y) / CELL_MM;
}

void grid_cell_to_world(const struct occupancy_grid *grid,
                        int cell_x, int cell_y,
                        int32_t *world_x, int32_t *world_y)
{
    /* World = origin + (cell + 0.5) * cell_size */
    *world_x = grid->origin_x + cell_x * CELL_MM + CELL_MM / 2;
    *world_y = grid->origin_y + cell_y * CELL_MM + CELL_MM / 2;
}

/* ==================== Sensor Update ==================== */

void grid_update_ray(struct occupancy_grid *grid,
                     int32_t robot_x, int32_t robot_y,
                     int32_t target_x, int32_t target_y,
                     bool hit_obstacle)
{
    int x0, y0, x1, y1;
    int dx, dy, sx, sy, err;

    grid_world_to_cell(grid, robot_x, robot_y, &x0, &y0);
    grid_world_to_cell(grid, target_x, target_y, &x1, &y1);

    /* Bresenham line algorithm */
    dx = abs(x1 - x0);
    dy = abs(y1 - y0);
    sx = (x0 < x1) ? 1 : -1;
    sy = (y0 < y1) ? 1 : -1;
    err = dx - dy;

    while (1) {
        bool is_endpoint = (x0 == x1 && y0 == y1);

        if (grid_is_valid(x0, y0)) {
            int8_t val = grid->data[y0 * GRID_SIZE + x0];

            if (is_endpoint && hit_obstacle) {
                /* Mark as occupied */
                val = CLAMP(val + LOG_ODDS_OCCUPIED, LOG_ODDS_MIN, LOG_ODDS_MAX);
            } else if (!is_endpoint) {
                /* Mark as free (ray passed through) */
                val = CLAMP(val + LOG_ODDS_FREE, LOG_ODDS_MIN, LOG_ODDS_MAX);
            }

            grid->data[y0 * GRID_SIZE + x0] = val;
        }

        if (is_endpoint) {
            break;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }

    grid->update_count++;
}

void grid_update_from_lidar(struct occupancy_grid *grid,
                            const struct sen0628_columns *cols,
                            const struct odometry *odom)
{
    int32_t robot_x = odom->x;
    int32_t robot_y = odom->y;
    int32_t heading = odom->heading_mdeg;

    /* Process each column */
    for (int col = 0; col < LIDAR_NUM_COLS; col++) {
        uint16_t dist = cols->min[col];

        /* Skip invalid readings */
        if (dist < 50 || dist >= 4000) {
            continue;
        }

        /* Calculate angle for this column */
        int32_t angle = heading + col_angle_mdeg[col];

        /* Calculate target point in world coordinates */
        int32_t target_x = robot_x + (dist * cos_mdeg(angle)) / 1000;
        int32_t target_y = robot_y + (dist * sin_mdeg(angle)) / 1000;

        /* Update grid with ray casting */
        bool hit_obstacle = (dist < 3500);  /* Consider < 3.5m as obstacle */
        grid_update_ray(grid, robot_x, robot_y, target_x, target_y, hit_obstacle);
    }
}

/* ==================== Statistics ==================== */

void grid_get_stats(const struct occupancy_grid *grid, struct grid_stats *stats)
{
    stats->unknown_count = 0;
    stats->free_count = 0;
    stats->occupied_count = 0;
    stats->update_count = grid->update_count;

    for (int i = 0; i < GRID_BYTES; i++) {
        int8_t val = grid->data[i];

        if (val > THRESHOLD_OCCUPIED) {
            stats->occupied_count++;
        } else if (val < THRESHOLD_FREE) {
            stats->free_count++;
        } else {
            stats->unknown_count++;
        }
    }
}

bool grid_is_explored(const struct occupancy_grid *grid, int threshold)
{
    struct grid_stats stats;
    grid_get_stats(grid, &stats);
    return (stats.unknown_count < threshold);
}

/* ==================== Serialization ==================== */

int grid_export_compact(const struct occupancy_grid *grid,
                        uint8_t *out, size_t out_size)
{
    size_t compact_size = (GRID_SIZE * GRID_SIZE) / 4;

    if (out_size < compact_size) {
        return -ENOMEM;
    }

    memset(out, 0, compact_size);

    for (int i = 0; i < GRID_BYTES; i++) {
        int8_t val = grid->data[i];
        uint8_t state;

        if (val > THRESHOLD_OCCUPIED) {
            state = 2;  /* Occupied */
        } else if (val < THRESHOLD_FREE) {
            state = 1;  /* Free */
        } else {
            state = 0;  /* Unknown */
        }

        /* Pack 4 cells per byte */
        int byte_idx = i / 4;
        int bit_offset = (i % 4) * 2;
        out[byte_idx] |= (state << bit_offset);
    }

    return (int)compact_size;
}

int grid_import_compact(struct occupancy_grid *grid,
                        const uint8_t *data, size_t data_size)
{
    size_t compact_size = (GRID_SIZE * GRID_SIZE) / 4;

    if (data_size < compact_size) {
        return -EINVAL;
    }

    for (int i = 0; i < GRID_BYTES; i++) {
        int byte_idx = i / 4;
        int bit_offset = (i % 4) * 2;
        uint8_t state = (data[byte_idx] >> bit_offset) & 0x03;

        /* Convert state to log-odds */
        switch (state) {
        case 1:  /* Free */
            grid->data[i] = LOG_ODDS_MIN / 2;
            break;
        case 2:  /* Occupied */
            grid->data[i] = LOG_ODDS_MAX / 2;
            break;
        default: /* Unknown */
            grid->data[i] = LOG_ODDS_PRIOR;
            break;
        }
    }

    return 0;
}

int grid_to_hex(const struct occupancy_grid *grid, char *out, size_t out_size)
{
    size_t needed = GRID_BYTES * 2 + 1;

    if (out_size < needed) {
        return -ENOMEM;
    }

    static const char hex_chars[] = "0123456789ABCDEF";
    char *p = out;

    for (int i = 0; i < GRID_BYTES; i++) {
        uint8_t val = (uint8_t)grid->data[i];
        *p++ = hex_chars[(val >> 4) & 0x0F];
        *p++ = hex_chars[val & 0x0F];
    }
    *p = '\0';

    return (int)(p - out);
}

static int hex_char_to_int(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

int grid_from_hex(struct occupancy_grid *grid, const char *hex, size_t hex_len)
{
    if (hex_len < GRID_BYTES * 2) {
        return -EINVAL;
    }

    for (int i = 0; i < GRID_BYTES; i++) {
        int hi = hex_char_to_int(hex[i * 2]);
        int lo = hex_char_to_int(hex[i * 2 + 1]);

        if (hi < 0 || lo < 0) {
            return -EINVAL;
        }

        grid->data[i] = (int8_t)((hi << 4) | lo);
    }

    return 0;
}
