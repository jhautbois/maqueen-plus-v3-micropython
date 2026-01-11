/*
 * SEN0628 8x8 ToF LIDAR Sensor Public API
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef SEN0628_H_
#define SEN0628_H_

#include <zephyr/device.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Matrix modes
 */
#define SEN0628_MODE_4X4        4
#define SEN0628_MODE_8X8        8

/*
 * Distance constants
 */
#define SEN0628_DIST_INVALID    4000    /* Invalid/no-target value */
#define SEN0628_DIST_MIN        20      /* Minimum reliable distance */
#define SEN0628_DIST_MAX        4000    /* Maximum distance */

/**
 * @brief Scan result with metadata for SLAM
 */
struct sen0628_scan {
	uint16_t distances[64];         /* Distance matrix [y*mode + x] */
	uint32_t timestamp_ms;          /* Kernel uptime at scan start */
	uint32_t duration_ms;           /* Time to complete scan */
	uint8_t mode;                   /* Mode used (4 or 8) */
	uint8_t valid_count;            /* Number of valid readings */
	uint16_t min_distance;          /* Minimum distance in scan */
	uint8_t min_x;                  /* X coordinate of minimum */
	uint8_t min_y;                  /* Y coordinate of minimum */
};

/**
 * @brief Column statistics for obstacle avoidance
 */
struct sen0628_columns {
	uint16_t min[8];                /* Minimum distance per column */
	uint16_t avg[8];                /* Average distance per column */
	uint8_t valid_count[8];         /* Valid readings per column */
};

/**
 * @brief Initialize the sensor
 *
 * @param dev Device handle
 * @return 0 on success, negative errno on failure
 */
int sen0628_init(const struct device *dev);

/**
 * @brief Set the matrix ranging mode
 *
 * @param dev Device handle
 * @param mode SEN0628_MODE_4X4 or SEN0628_MODE_8X8
 * @return 0 on success, negative errno on failure
 *
 * @note This function blocks for ~5 seconds after mode change
 */
int sen0628_set_mode(const struct device *dev, uint8_t mode);

/**
 * @brief Get current matrix mode
 *
 * @param dev Device handle
 * @return Current mode (4 or 8), or negative errno on failure
 */
int sen0628_get_mode(const struct device *dev);

/**
 * @brief Read distance at a specific point
 *
 * @param dev Device handle
 * @param x X coordinate (0 to mode-1)
 * @param y Y coordinate (0 to mode-1)
 * @param distance Pointer to store distance in mm
 * @return 0 on success, negative errno on failure
 */
int sen0628_read_point(const struct device *dev,
		       uint8_t x, uint8_t y,
		       uint16_t *distance);

/**
 * @brief Read entire distance matrix
 *
 * Reads all points in the current mode. In 8x8 mode, this reads
 * 64 values and takes approximately 2-3 seconds.
 *
 * @param dev Device handle
 * @param distances Buffer for distances (must be mode*mode elements)
 * @return 0 on success, negative errno on failure
 */
int sen0628_read_matrix(const struct device *dev,
			uint16_t *distances);

/**
 * @brief Read matrix with full metadata (for SLAM)
 *
 * @param dev Device handle
 * @param scan Pointer to scan structure to fill
 * @return 0 on success, negative errno on failure
 */
int sen0628_read_scan(const struct device *dev,
		      struct sen0628_scan *scan);

/**
 * @brief Get column statistics
 *
 * Calculates minimum and average distance for each column.
 * Useful for obstacle avoidance where columns = directions.
 *
 * @param dev Device handle
 * @param cols Pointer to columns structure to fill
 * @return 0 on success, negative errno on failure
 *
 * @note Uses cached data from last scan. Call sen0628_read_scan first.
 */
int sen0628_get_columns(const struct device *dev,
			struct sen0628_columns *cols);

/**
 * @brief Read minimum distance per column (fast obstacle check)
 *
 * This is the primary function for navigation/obstacle avoidance.
 * Returns the minimum distance in each of the 8 (or 4) columns.
 *
 * @param dev Device handle
 * @param min_per_col Buffer for column minimums (mode elements)
 * @return 0 on success, negative errno on failure
 */
int sen0628_read_columns_min(const struct device *dev,
			     uint16_t *min_per_col);

/**
 * @brief Get the minimum distance in the entire field of view
 *
 * @param dev Device handle
 * @return Minimum distance in mm, or SEN0628_DIST_INVALID on error
 *
 * @note Uses cached data from last scan
 */
uint16_t sen0628_get_min_distance(const struct device *dev);

/**
 * @brief Check if a sector is clear of obstacles
 *
 * @param dev Device handle
 * @param sector Sector index (0 to mode-1, left to right)
 * @param threshold Minimum required distance in mm
 * @return true if sector is clear, false if blocked or error
 *
 * @note Uses cached data from last scan
 */
bool sen0628_sector_clear(const struct device *dev,
			  uint8_t sector,
			  uint16_t threshold);

/**
 * @brief Print scan data as ASCII art to console
 *
 * @param scan Pointer to scan data
 */
void sen0628_print_scan(const struct sen0628_scan *scan);

/**
 * @brief Print column minimums as bar chart
 *
 * @param cols Pointer to column data
 * @param mode Current mode (4 or 8)
 */
void sen0628_print_columns(const struct sen0628_columns *cols, uint8_t mode);

/**
 * @brief Get error count from last scan
 *
 * @param dev Device handle
 * @return Number of read errors in last scan (0 = all points valid)
 */
int sen0628_get_error_count(const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* SEN0628_H_ */
