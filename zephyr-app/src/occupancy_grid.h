/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Occupancy Grid for SLAM mapping
 * 48x48 grid with 8-bit log-odds representation
 */

#ifndef OCCUPANCY_GRID_H
#define OCCUPANCY_GRID_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Grid configuration */
#define GRID_SIZE       48      /* 48x48 cells */
#define CELL_MM         100     /* 100mm per cell */
#define GRID_BYTES      (GRID_SIZE * GRID_SIZE)  /* 2304 bytes */
#define GRID_COVERAGE   (GRID_SIZE * CELL_MM)    /* 4800mm = 4.8m */

/* Log-odds constants */
#define LOG_ODDS_PRIOR       0      /* Initial value (unknown) */
#define LOG_ODDS_FREE      -15      /* Increment for free space */
#define LOG_ODDS_OCCUPIED   30      /* Increment for obstacle */
#define LOG_ODDS_MIN      -128      /* Minimum (definitely free) */
#define LOG_ODDS_MAX       127      /* Maximum (definitely occupied) */

/* Classification thresholds */
#define THRESHOLD_FREE     -20      /* Below = free */
#define THRESHOLD_OCCUPIED  20      /* Above = occupied */

/**
 * @brief Cell state classification
 */
typedef enum {
    CELL_UNKNOWN = 0,
    CELL_FREE = 1,
    CELL_OCCUPIED = 2
} cell_state_t;

/**
 * @brief Occupancy grid structure
 *
 * Uses log-odds representation for probabilistic updates.
 * Grid center (24,24) corresponds to robot initial position.
 */
struct occupancy_grid {
    int8_t data[GRID_BYTES];    /* Log-odds values */
    int32_t origin_x;           /* Origin X in mm (world coords of cell 0,0) */
    int32_t origin_y;           /* Origin Y in mm */
    uint32_t update_count;      /* Total sensor updates */
};

/**
 * @brief Grid statistics
 */
struct grid_stats {
    int unknown_count;
    int free_count;
    int occupied_count;
    uint32_t update_count;
};

/* ==================== Initialization ==================== */

/**
 * @brief Initialize grid with all cells unknown
 *
 * Sets origin so that grid center corresponds to (0,0) in world coordinates.
 *
 * @param grid Pointer to grid structure
 */
void grid_init(struct occupancy_grid *grid);

/**
 * @brief Clear grid to unknown state
 *
 * @param grid Pointer to grid structure
 */
void grid_clear(struct occupancy_grid *grid);

/* ==================== Cell Access ==================== */

/**
 * @brief Get raw log-odds value for a cell
 *
 * @param grid Pointer to grid structure
 * @param cx Cell X coordinate (0 to GRID_SIZE-1)
 * @param cy Cell Y coordinate (0 to GRID_SIZE-1)
 * @return Log-odds value (-128 to 127), or 0 if out of bounds
 */
int8_t grid_get_logodds(const struct occupancy_grid *grid, int cx, int cy);

/**
 * @brief Set raw log-odds value for a cell
 *
 * @param grid Pointer to grid structure
 * @param cx Cell X coordinate
 * @param cy Cell Y coordinate
 * @param value Log-odds value to set
 */
void grid_set_logodds(struct occupancy_grid *grid, int cx, int cy, int8_t value);

/**
 * @brief Get classified state for a cell
 *
 * @param grid Pointer to grid structure
 * @param cx Cell X coordinate
 * @param cy Cell Y coordinate
 * @return Cell state (CELL_UNKNOWN, CELL_FREE, or CELL_OCCUPIED)
 */
cell_state_t grid_get_state(const struct occupancy_grid *grid, int cx, int cy);

/**
 * @brief Check if cell coordinates are within grid bounds
 *
 * @param cx Cell X coordinate
 * @param cy Cell Y coordinate
 * @return true if valid, false if out of bounds
 */
static inline bool grid_is_valid(int cx, int cy)
{
    return (cx >= 0 && cx < GRID_SIZE && cy >= 0 && cy < GRID_SIZE);
}

/* ==================== Coordinate Conversion ==================== */

/**
 * @brief Convert world coordinates to cell coordinates
 *
 * @param grid Pointer to grid structure
 * @param world_x World X in mm
 * @param world_y World Y in mm
 * @param cell_x Output cell X coordinate
 * @param cell_y Output cell Y coordinate
 */
void grid_world_to_cell(const struct occupancy_grid *grid,
                        int32_t world_x, int32_t world_y,
                        int *cell_x, int *cell_y);

/**
 * @brief Convert cell coordinates to world coordinates (cell center)
 *
 * @param grid Pointer to grid structure
 * @param cell_x Cell X coordinate
 * @param cell_y Cell Y coordinate
 * @param world_x Output world X in mm
 * @param world_y Output world Y in mm
 */
void grid_cell_to_world(const struct occupancy_grid *grid,
                        int cell_x, int cell_y,
                        int32_t *world_x, int32_t *world_y);

/* ==================== Sensor Update ==================== */

/**
 * @brief Update grid from a single ray measurement (Bresenham)
 *
 * Traces a ray from robot position to target. Cells along the ray
 * are marked as free, the endpoint is marked as occupied (if hit).
 *
 * @param grid Pointer to grid structure
 * @param robot_x Robot X position in mm
 * @param robot_y Robot Y position in mm
 * @param target_x Target X position in mm
 * @param target_y Target Y position in mm
 * @param hit_obstacle true if ray hit an obstacle, false if max range
 */
void grid_update_ray(struct occupancy_grid *grid,
                     int32_t robot_x, int32_t robot_y,
                     int32_t target_x, int32_t target_y,
                     bool hit_obstacle);

/* Forward declarations for sensor structures */
struct sen0628_columns;
struct odometry;

/**
 * @brief Update grid from LIDAR column measurements
 *
 * Processes 8 LIDAR columns, converting each to world coordinates
 * and updating the grid using ray casting.
 *
 * @param grid Pointer to grid structure
 * @param cols Pointer to LIDAR column data
 * @param odom Pointer to odometry data (robot pose)
 */
void grid_update_from_lidar(struct occupancy_grid *grid,
                            const struct sen0628_columns *cols,
                            const struct odometry *odom);

/* ==================== Statistics ==================== */

/**
 * @brief Get grid statistics
 *
 * @param grid Pointer to grid structure
 * @param stats Output statistics structure
 */
void grid_get_stats(const struct occupancy_grid *grid, struct grid_stats *stats);

/**
 * @brief Check if most of the grid has been explored
 *
 * @param grid Pointer to grid structure
 * @param threshold Maximum unknown cells to consider complete
 * @return true if exploration is complete
 */
bool grid_is_explored(const struct occupancy_grid *grid, int threshold);

/* ==================== Serialization ==================== */

/**
 * @brief Export grid as compact 2-bit format
 *
 * Converts log-odds to 3-state values, packing 4 cells per byte.
 * Output size is GRID_SIZE * GRID_SIZE / 4 = 576 bytes.
 *
 * @param grid Pointer to grid structure
 * @param out Output buffer (must be at least 576 bytes)
 * @param out_size Size of output buffer
 * @return Number of bytes written, or negative on error
 */
int grid_export_compact(const struct occupancy_grid *grid,
                        uint8_t *out, size_t out_size);

/**
 * @brief Import grid from compact 2-bit format
 *
 * @param grid Pointer to grid structure
 * @param data Input data (576 bytes)
 * @param data_size Size of input data
 * @return 0 on success, negative on error
 */
int grid_import_compact(struct occupancy_grid *grid,
                        const uint8_t *data, size_t data_size);

/**
 * @brief Convert grid to hex string for serial export
 *
 * Full log-odds export: 2304 bytes -> 4608 hex characters.
 *
 * @param grid Pointer to grid structure
 * @param out Output string buffer
 * @param out_size Size of output buffer (must be >= 4609)
 * @return Number of characters written, or negative on error
 */
int grid_to_hex(const struct occupancy_grid *grid, char *out, size_t out_size);

/**
 * @brief Import grid from hex string
 *
 * @param grid Pointer to grid structure
 * @param hex Input hex string
 * @param hex_len Length of hex string
 * @return 0 on success, negative on error
 */
int grid_from_hex(struct occupancy_grid *grid, const char *hex, size_t hex_len);

#ifdef __cplusplus
}
#endif

#endif /* OCCUPANCY_GRID_H */
