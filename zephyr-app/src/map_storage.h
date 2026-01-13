/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Map storage using Zephyr NVS (Non-Volatile Storage)
 * Persists occupancy grid to flash memory
 */

#ifndef MAP_STORAGE_H
#define MAP_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NVS item IDs */
#define MAP_NVS_ID_HEADER   1
#define MAP_NVS_ID_DATA     2
#define MAP_NVS_ID_CALIB    3

/* Map file magic and version */
#define MAP_MAGIC           0x4D415032  /* 'MAP2' */
#define MAP_VERSION         2

/* Forward declarations */
struct occupancy_grid;
struct odometry;

/**
 * @brief Map header stored in NVS
 */
struct map_header {
    uint32_t magic;             /* MAP_MAGIC for validation */
    uint16_t version;           /* MAP_VERSION */
    uint16_t grid_size;         /* Grid dimension (48) */
    uint16_t cell_mm;           /* Cell size in mm (100) */
    uint16_t reserved;          /* Padding */
    int32_t origin_x;           /* Grid origin X */
    int32_t origin_y;           /* Grid origin Y */
    int32_t robot_x;            /* Robot position X at save time */
    int32_t robot_y;            /* Robot position Y */
    int32_t robot_heading;      /* Robot heading (mdeg) */
    uint32_t update_count;      /* Total grid updates */
    uint32_t timestamp;         /* Save timestamp (uptime) */
    uint32_t checksum;          /* CRC32 of grid data */
};

/**
 * @brief Storage statistics
 */
struct map_storage_stats {
    bool initialized;           /* Storage initialized */
    bool map_exists;            /* Map saved in NVS */
    uint32_t save_count;        /* Total saves this session */
    uint32_t load_count;        /* Total loads this session */
    uint32_t last_save_ms;      /* Last save timestamp */
    size_t flash_used;          /* Bytes used in NVS */
};

/* ==================== Initialization ==================== */

/**
 * @brief Initialize map storage
 *
 * Must be called before any other map_storage functions.
 * Initializes NVS subsystem if not already done.
 *
 * @return 0 on success, negative errno on failure
 */
int map_storage_init(void);

/**
 * @brief Check if storage is ready
 *
 * @return true if initialized and ready
 */
bool map_storage_is_ready(void);

/* ==================== Save/Load ==================== */

/**
 * @brief Save map to NVS
 *
 * Saves grid data and robot pose to flash. Computes CRC32
 * checksum for data integrity verification.
 *
 * @param grid Occupancy grid to save
 * @param odom Odometry (for robot pose), can be NULL
 * @return 0 on success, negative errno on failure
 */
int map_storage_save(const struct occupancy_grid *grid,
                     const struct odometry *odom);

/**
 * @brief Load map from NVS
 *
 * Loads grid data and optionally restores robot pose.
 * Verifies CRC32 checksum before loading.
 *
 * @param grid Occupancy grid to load into
 * @param odom Odometry to restore pose (can be NULL)
 * @return 0 on success, negative errno on failure
 */
int map_storage_load(struct occupancy_grid *grid,
                     struct odometry *odom);

/**
 * @brief Clear stored map
 *
 * Erases map data from NVS.
 *
 * @return 0 on success, negative errno on failure
 */
int map_storage_clear(void);

/* ==================== Status ==================== */

/**
 * @brief Check if a map exists in storage
 *
 * @return true if valid map is stored
 */
bool map_storage_exists(void);

/**
 * @brief Get stored map header without loading data
 *
 * @param header Output header structure
 * @return 0 on success, negative errno on failure
 */
int map_storage_get_header(struct map_header *header);

/**
 * @brief Get storage statistics
 *
 * @param stats Output statistics structure
 */
void map_storage_get_stats(struct map_storage_stats *stats);

/* ==================== Utilities ==================== */

/**
 * @brief Compute CRC32 of data
 *
 * @param data Data buffer
 * @param len Data length
 * @return CRC32 checksum
 */
uint32_t map_storage_crc32(const uint8_t *data, size_t len);

/* ==================== Calibration Storage ==================== */

/**
 * @brief Calibration data structure
 */
struct calibration_data {
    uint32_t magic;             /* 0x43414C42 'CALB' */
    uint16_t turn_speed;        /* Motor speed for turns */
    uint16_t ms_per_degree;     /* Calibrated rotation timing */
};

#define CALIB_MAGIC 0x43414C42  /* 'CALB' */

/**
 * @brief Save calibration data to NVS
 *
 * @param turn_speed Motor turn speed
 * @param ms_per_deg Milliseconds per degree
 * @return 0 on success, negative errno on failure
 */
int map_storage_save_calibration(uint16_t turn_speed, uint16_t ms_per_deg);

/**
 * @brief Load calibration data from NVS
 *
 * @param turn_speed Output: motor turn speed
 * @param ms_per_deg Output: milliseconds per degree
 * @return 0 on success, negative errno on failure
 */
int map_storage_load_calibration(uint16_t *turn_speed, uint16_t *ms_per_deg);

/**
 * @brief Check if calibration data exists
 *
 * @return true if valid calibration is stored
 */
bool map_storage_calibration_exists(void);

/**
 * @brief Clear calibration data from NVS
 *
 * @return 0 on success, negative errno on failure
 */
int map_storage_clear_calibration(void);

#ifdef __cplusplus
}
#endif

#endif /* MAP_STORAGE_H */
