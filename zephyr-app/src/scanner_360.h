/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * 360-degree scanner for mapping
 * Rotates robot to capture full environment with 60° FOV LIDAR
 */

#ifndef SCANNER_360_H
#define SCANNER_360_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Scan configuration */
#define SCAN_POSITIONS      6       /* 6 positions for 360° coverage */
#define SCAN_ANGLE_DEG      60      /* Angle between positions */
#define SCAN_OVERLAP_DEG    0       /* Overlap between scans (60° FOV) */

/* Default rotation parameters (can be calibrated) */
#define SCAN_DEFAULT_TURN_SPEED     100     /* Motor speed for rotation */
#define SCAN_DEFAULT_MS_PER_DEG     12      /* Milliseconds per degree (~95°/s at speed 100) */
#define SCAN_SETTLE_MS              200     /* Settle time after rotation */

/* Forward declarations */
struct occupancy_grid;
struct odometry;

/**
 * @brief Progress callback for scan updates
 *
 * @param position Current position (0 to SCAN_POSITIONS-1)
 * @param total Total positions (SCAN_POSITIONS)
 * @param user_data User-provided context
 */
typedef void (*scan_progress_cb)(int position, int total, void *user_data);

/**
 * @brief Abort check callback
 *
 * @param user_data User-provided context
 * @return true to abort scan, false to continue
 */
typedef bool (*scan_abort_cb)(void *user_data);

/**
 * @brief Scanner state structure
 */
struct scanner_360 {
    const struct device *i2c_dev;   /* I2C device for Maqueen motors */
    const struct device *lidar_dev; /* LIDAR device for scanning */
    struct occupancy_grid *grid;    /* Grid to update */
    struct odometry *odom;          /* Odometry for pose tracking */

    /* Rotation calibration */
    int turn_speed;                 /* Motor speed for turns */
    int ms_per_degree;              /* Calibrated timing */

    /* State */
    int current_position;           /* Current scan position (0-5) */
    int32_t initial_heading;        /* Heading at scan start (mdeg) */
    bool scan_in_progress;          /* Scan active flag */
    bool calibrated;                /* Rotation calibrated flag */

    /* Statistics */
    uint32_t last_scan_duration_ms; /* Duration of last full scan */
    int last_scan_obstacles;        /* Obstacles detected in last scan */
};

/**
 * @brief Scan result statistics
 */
struct scan_result {
    uint32_t duration_ms;           /* Total scan duration */
    int positions_completed;        /* Positions successfully scanned */
    int total_rays;                 /* Total rays cast */
    int obstacles_detected;         /* New obstacles found */
    int free_cells_marked;          /* Cells marked as free */
};

/* ==================== Initialization ==================== */

/**
 * @brief Initialize scanner
 *
 * @param scanner Scanner state structure
 * @param i2c_dev I2C device for Maqueen motor control
 * @param lidar_dev LIDAR device for distance sensing
 * @param grid Occupancy grid to update during scan
 * @param odom Odometry for pose tracking
 * @return 0 on success, negative errno on failure
 */
int scanner_init(struct scanner_360 *scanner,
                 const struct device *i2c_dev,
                 const struct device *lidar_dev,
                 struct occupancy_grid *grid,
                 struct odometry *odom);

/**
 * @brief Calibrate rotation timing using odometry
 *
 * Performs a test rotation and measures actual angle using odometry
 * to calibrate ms_per_degree.
 *
 * @param scanner Scanner state structure
 * @param abort_cb Optional abort check callback (can be NULL)
 * @param user_data User data for callback
 * @return 0 on success, -ECANCELED if aborted, negative errno on failure
 */
int scanner_calibrate_rotation(struct scanner_360 *scanner,
                               scan_abort_cb abort_cb,
                               void *user_data);

/**
 * @brief Set rotation parameters manually
 *
 * @param scanner Scanner state structure
 * @param turn_speed Motor speed for rotation (0-255)
 * @param ms_per_degree Milliseconds per degree of rotation
 */
void scanner_set_rotation_params(struct scanner_360 *scanner,
                                 int turn_speed, int ms_per_degree);

/* ==================== Scanning ==================== */

/**
 * @brief Perform full 360-degree scan (blocking)
 *
 * Rotates robot through 6 positions, reading LIDAR at each position
 * and updating the occupancy grid. Returns to initial heading.
 *
 * @param scanner Scanner state structure
 * @param progress_cb Optional progress callback (can be NULL)
 * @param abort_cb Optional abort check callback (can be NULL)
 * @param user_data User data for callbacks
 * @param result Optional result statistics (can be NULL)
 * @return 0 on success, -ECANCELED if aborted, negative errno on failure
 */
int scanner_full_scan(struct scanner_360 *scanner,
                      scan_progress_cb progress_cb,
                      scan_abort_cb abort_cb,
                      void *user_data,
                      struct scan_result *result);

/**
 * @brief Scan single position (non-blocking step)
 *
 * Reads LIDAR and updates grid for current position.
 * Does not rotate - caller manages rotation.
 *
 * @param scanner Scanner state structure
 * @return Number of obstacles detected, or negative errno
 */
int scanner_scan_position(struct scanner_360 *scanner);

/**
 * @brief Rotate to next scan position
 *
 * @param scanner Scanner state structure
 * @return 0 on success, negative errno on failure
 */
int scanner_rotate_to_next(struct scanner_360 *scanner);

/**
 * @brief Return to initial heading
 *
 * @param scanner Scanner state structure
 * @return 0 on success, negative errno on failure
 */
int scanner_return_to_start(struct scanner_360 *scanner);

/* ==================== Status ==================== */

/**
 * @brief Check if scan is in progress
 *
 * @param scanner Scanner state structure
 * @return true if scanning, false otherwise
 */
static inline bool scanner_is_scanning(const struct scanner_360 *scanner)
{
    return scanner->scan_in_progress;
}

/**
 * @brief Get current scan position
 *
 * @param scanner Scanner state structure
 * @return Position index (0 to SCAN_POSITIONS-1)
 */
static inline int scanner_get_position(const struct scanner_360 *scanner)
{
    return scanner->current_position;
}

/**
 * @brief Abort current scan
 *
 * Stops motors and clears scan state. Does NOT return to start heading.
 *
 * @param scanner Scanner state structure
 */
void scanner_abort(struct scanner_360 *scanner);

#ifdef __cplusplus
}
#endif

#endif /* SCANNER_360_H */
