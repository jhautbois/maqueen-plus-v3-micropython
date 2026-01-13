/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * 360-degree scanner implementation
 */

#include "scanner_360.h"
#include "occupancy_grid.h"
#include "odometry.h"
#include "maqueen.h"
#include "sen0628.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>

LOG_MODULE_REGISTER(scanner_360, LOG_LEVEL_INF);

/* ==================== Internal Helpers ==================== */

/**
 * @brief Rotate robot by specified degrees
 *
 * @param scanner Scanner state
 * @param degrees Degrees to rotate (positive = clockwise/right)
 * @return 0 on success, negative errno on failure
 */
#define ODOM_UPDATE_INTERVAL_MS 50

static int rotate_degrees(struct scanner_360 *scanner, int degrees)
{
    int duration_ms = abs(degrees) * scanner->ms_per_degree;
    int elapsed_ms = 0;
    int ret;

    /* Set motor directions for rotation */
    if (degrees > 0) {
        /* Clockwise: left forward, right backward */
        ret = maqueen_motor_set(scanner->i2c_dev, MAQUEEN_MOTOR_LEFT,
                                MAQUEEN_DIR_FORWARD, scanner->turn_speed);
        if (ret < 0) return ret;

        ret = maqueen_motor_set(scanner->i2c_dev, MAQUEEN_MOTOR_RIGHT,
                                MAQUEEN_DIR_REVERSE, scanner->turn_speed);
        if (ret < 0) return ret;

        /* Update odometry direction */
        odometry_set_direction(scanner->odom, true, false);
    } else {
        /* Counter-clockwise: left backward, right forward */
        ret = maqueen_motor_set(scanner->i2c_dev, MAQUEEN_MOTOR_LEFT,
                                MAQUEEN_DIR_REVERSE, scanner->turn_speed);
        if (ret < 0) return ret;

        ret = maqueen_motor_set(scanner->i2c_dev, MAQUEEN_MOTOR_RIGHT,
                                MAQUEEN_DIR_FORWARD, scanner->turn_speed);
        if (ret < 0) return ret;

        /* Update odometry direction */
        odometry_set_direction(scanner->odom, false, true);
    }

    /* Rotate with periodic odometry updates */
    while (elapsed_ms < duration_ms) {
        int sleep_ms = (duration_ms - elapsed_ms > ODOM_UPDATE_INTERVAL_MS)
                       ? ODOM_UPDATE_INTERVAL_MS
                       : (duration_ms - elapsed_ms);
        k_sleep(K_MSEC(sleep_ms));
        elapsed_ms += sleep_ms;
        odometry_update(scanner->odom);
    }

    /* Stop motors */
    ret = maqueen_stop_all(scanner->i2c_dev);
    if (ret < 0) return ret;

    /* Final odometry update after stop */
    k_sleep(K_MSEC(50));
    odometry_update(scanner->odom);

    /* Settle time */
    k_sleep(K_MSEC(SCAN_SETTLE_MS));

    return 0;
}

/**
 * @brief Read LIDAR and update grid
 *
 * @param scanner Scanner state
 * @return Number of valid columns read, or negative errno
 */
static int read_and_update_grid(struct scanner_360 *scanner)
{
    struct sen0628_columns cols;
    struct sen0628_scan scan;
    int ret;

    /* Read LIDAR scan */
    ret = sen0628_read_scan(scanner->lidar_dev, &scan);
    if (ret < 0) {
        LOG_WRN("LIDAR scan failed: %d", ret);
        /* Continue anyway - try to get columns */
    }

    /* Get column statistics */
    ret = sen0628_get_columns(scanner->lidar_dev, &cols);
    if (ret < 0) {
        LOG_ERR("Failed to get LIDAR columns: %d", ret);
        return ret;
    }

    /* Update odometry before grid update */
    odometry_update(scanner->odom);

    /* Update occupancy grid */
    grid_update_from_lidar(scanner->grid, &cols, scanner->odom);

    /* Count valid readings */
    int valid = 0;
    for (int i = 0; i < 8; i++) {
        if (cols.valid_count[i] > 0) {
            valid++;
        }
    }

    return valid;
}

/* ==================== Initialization ==================== */

int scanner_init(struct scanner_360 *scanner,
                 const struct device *i2c_dev,
                 const struct device *lidar_dev,
                 struct occupancy_grid *grid,
                 struct odometry *odom)
{
    if (!scanner || !i2c_dev || !lidar_dev || !grid || !odom) {
        return -EINVAL;
    }

    scanner->i2c_dev = i2c_dev;
    scanner->lidar_dev = lidar_dev;
    scanner->grid = grid;
    scanner->odom = odom;

    /* Default rotation parameters */
    scanner->turn_speed = SCAN_DEFAULT_TURN_SPEED;
    scanner->ms_per_degree = SCAN_DEFAULT_MS_PER_DEG;

    /* Initial state */
    scanner->current_position = 0;
    scanner->initial_heading = 0;
    scanner->scan_in_progress = false;
    scanner->calibrated = false;

    /* Statistics */
    scanner->last_scan_duration_ms = 0;
    scanner->last_scan_obstacles = 0;

    LOG_INF("Scanner initialized: %d positions, %d° each",
            SCAN_POSITIONS, SCAN_ANGLE_DEG);

    return 0;
}

#define CALIB_TARGET_DEG     90      /* Target rotation for calibration */
#define CALIB_MAX_ITERATIONS 8       /* Maximum calibration attempts */
#define CALIB_TOLERANCE_DEG  5       /* Acceptable error in degrees (~5%) */

int scanner_calibrate_rotation(struct scanner_360 *scanner,
                               scan_abort_cb abort_cb,
                               void *user_data)
{
    int32_t start_heading, end_heading, actual_deg;
    int error_deg;
    int ret;
    int iteration;

    LOG_INF("Calibrating rotation (target=%d°, tolerance=%d°)...",
            CALIB_TARGET_DEG, CALIB_TOLERANCE_DEG);

    for (iteration = 0; iteration < CALIB_MAX_ITERATIONS; iteration++) {
        /* Check for abort */
        if (abort_cb && abort_cb(user_data)) {
            LOG_INF("Calibration aborted");
            maqueen_stop_all(scanner->i2c_dev);
            return -ECANCELED;
        }

        LOG_INF("Iteration %d: ms_per_deg=%d", iteration + 1, scanner->ms_per_degree);

        /* Record start heading */
        odometry_update(scanner->odom);
        start_heading = scanner->odom->heading_mdeg;

        /* Rotate using current settings */
        ret = rotate_degrees(scanner, CALIB_TARGET_DEG);
        if (ret < 0) {
            LOG_ERR("Calibration rotation failed: %d", ret);
            return ret;
        }

        /* Record end heading */
        odometry_update(scanner->odom);
        end_heading = scanner->odom->heading_mdeg;

        /* Calculate actual rotation (clockwise: heading decreases) */
        actual_deg = (start_heading - end_heading) / 1000;
        if (actual_deg < 0) {
            actual_deg += 360;  /* Handle wraparound through 0° */
        }

        /* Calculate error */
        error_deg = actual_deg - CALIB_TARGET_DEG;

        LOG_INF("  Measured: %d° (error=%d°)", actual_deg, error_deg);

        /* Check if within tolerance */
        if (abs(error_deg) <= CALIB_TOLERANCE_DEG) {
            LOG_INF("Calibration converged after %d iteration(s)", iteration + 1);
            break;
        }

        /* Sanity check */
        if (actual_deg < 10) {
            LOG_WRN("Calibration failed: rotation too small (%d°)", actual_deg);
            return -EIO;
        }

        /* Adjust ms_per_degree proportionally */
        int old_ms = scanner->ms_per_degree;
        scanner->ms_per_degree = (old_ms * CALIB_TARGET_DEG) / actual_deg;

        /* Clamp to reasonable range */
        if (scanner->ms_per_degree < 2) {
            scanner->ms_per_degree = 2;
        } else if (scanner->ms_per_degree > 100) {
            scanner->ms_per_degree = 100;
        }

        LOG_INF("  Adjusted: ms_per_deg %d -> %d", old_ms, scanner->ms_per_degree);

        /* Rotate back before next iteration */
        ret = rotate_degrees(scanner, -actual_deg);
        if (ret < 0) {
            LOG_WRN("Return rotation failed: %d", ret);
        }

        /* Small pause between iterations */
        k_sleep(K_MSEC(500));
    }

    if (iteration >= CALIB_MAX_ITERATIONS) {
        LOG_WRN("Calibration did not converge after %d iterations (error=%d°)",
                CALIB_MAX_ITERATIONS, error_deg);
        /* Continue anyway with best estimate */
    }

    /* Final return to start position */
    if (abort_cb && abort_cb(user_data)) {
        LOG_INF("Calibration aborted (skipping final return)");
        maqueen_stop_all(scanner->i2c_dev);
        return -ECANCELED;
    }

    ret = rotate_degrees(scanner, -CALIB_TARGET_DEG);
    if (ret < 0) {
        LOG_WRN("Final return rotation failed: %d", ret);
    }

    LOG_INF("Calibration complete: ms_per_deg=%d", scanner->ms_per_degree);
    scanner->calibrated = true;
    return 0;
}

void scanner_set_rotation_params(struct scanner_360 *scanner,
                                 int turn_speed, int ms_per_degree)
{
    scanner->turn_speed = turn_speed;
    scanner->ms_per_degree = ms_per_degree;
    LOG_INF("Rotation params: speed=%d, ms/deg=%d", turn_speed, ms_per_degree);
}

/* ==================== Scanning ==================== */

int scanner_full_scan(struct scanner_360 *scanner,
                      scan_progress_cb progress_cb,
                      scan_abort_cb abort_cb,
                      void *user_data,
                      struct scan_result *result)
{
    uint32_t start_time = k_uptime_get_32();
    int total_obstacles = 0;
    int ret;
    bool aborted = false;

    LOG_INF("Starting 360° scan (%d positions)", SCAN_POSITIONS);

    /* Initialize state */
    scanner->scan_in_progress = true;
    scanner->current_position = 0;
    odometry_update(scanner->odom);
    scanner->initial_heading = scanner->odom->heading_mdeg;

    /* Scan each position */
    for (int pos = 0; pos < SCAN_POSITIONS; pos++) {
        /* Check for abort request */
        if (abort_cb && abort_cb(user_data)) {
            LOG_INF("Scan aborted by user");
            aborted = true;
            break;
        }

        scanner->current_position = pos;

        /* Progress callback */
        if (progress_cb) {
            progress_cb(pos, SCAN_POSITIONS, user_data);
        }

        LOG_DBG("Scanning position %d/%d", pos + 1, SCAN_POSITIONS);

        /* Read LIDAR and update grid */
        ret = read_and_update_grid(scanner);
        if (ret < 0) {
            LOG_WRN("Position %d scan failed: %d", pos, ret);
        } else {
            total_obstacles += ret;
        }

        /* Check for abort again after scan */
        if (abort_cb && abort_cb(user_data)) {
            LOG_INF("Scan aborted by user");
            aborted = true;
            break;
        }

        /* Rotate to next position (except after last) */
        if (pos < SCAN_POSITIONS - 1) {
            ret = rotate_degrees(scanner, SCAN_ANGLE_DEG);
            if (ret < 0) {
                LOG_ERR("Rotation failed at position %d: %d", pos, ret);
                scanner_abort(scanner);
                return ret;
            }
        }
    }

    /* Handle abort */
    if (aborted) {
        maqueen_stop_all(scanner->i2c_dev);
        scanner->scan_in_progress = false;
        return -ECANCELED;
    }

    /* Return to initial heading */
    ret = scanner_return_to_start(scanner);
    if (ret < 0) {
        LOG_WRN("Return to start failed: %d", ret);
    }

    /* Final callback */
    if (progress_cb) {
        progress_cb(SCAN_POSITIONS, SCAN_POSITIONS, user_data);
    }

    /* Update statistics */
    uint32_t duration = k_uptime_get_32() - start_time;
    scanner->last_scan_duration_ms = duration;
    scanner->last_scan_obstacles = total_obstacles;
    scanner->scan_in_progress = false;

    LOG_INF("Scan complete: %dms, %d obstacle readings",
            duration, total_obstacles);

    /* Fill result if provided */
    if (result) {
        result->duration_ms = duration;
        result->positions_completed = SCAN_POSITIONS;
        result->total_rays = SCAN_POSITIONS * 8;  /* 8 columns per position */
        result->obstacles_detected = total_obstacles;

        struct grid_stats stats;
        grid_get_stats(scanner->grid, &stats);
        result->free_cells_marked = stats.free_count;
    }

    return 0;
}

int scanner_scan_position(struct scanner_360 *scanner)
{
    return read_and_update_grid(scanner);
}

int scanner_rotate_to_next(struct scanner_360 *scanner)
{
    int ret = rotate_degrees(scanner, SCAN_ANGLE_DEG);
    if (ret == 0) {
        scanner->current_position++;
        if (scanner->current_position >= SCAN_POSITIONS) {
            scanner->current_position = 0;
        }
    }
    return ret;
}

int scanner_return_to_start(struct scanner_360 *scanner)
{
    int32_t current_heading, diff_mdeg, diff_deg;
    int ret;

    odometry_update(scanner->odom);
    current_heading = scanner->odom->heading_mdeg;

    /* Calculate shortest rotation back to start */
    diff_mdeg = scanner->initial_heading - current_heading;

    /* Normalize to -180000 to +180000 */
    while (diff_mdeg > 180000) {
        diff_mdeg -= 360000;
    }
    while (diff_mdeg < -180000) {
        diff_mdeg += 360000;
    }

    diff_deg = diff_mdeg / 1000;

    if (abs(diff_deg) < 5) {
        LOG_DBG("Already near start heading");
        return 0;
    }

    LOG_DBG("Returning to start: rotating %d°", diff_deg);

    ret = rotate_degrees(scanner, diff_deg);
    if (ret < 0) {
        return ret;
    }

    scanner->current_position = 0;
    return 0;
}

void scanner_abort(struct scanner_360 *scanner)
{
    maqueen_stop_all(scanner->i2c_dev);
    scanner->scan_in_progress = false;
    LOG_WRN("Scan aborted");
}
