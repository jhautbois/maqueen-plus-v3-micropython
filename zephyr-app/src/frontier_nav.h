/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frontier-based navigation for exploration
 * Finds best direction toward unexplored areas
 */

#ifndef FRONTIER_NAV_H
#define FRONTIER_NAV_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Number of discrete directions */
#define NAV_NUM_DIRECTIONS  8

/* Direction indices: 0=N(forward), 1=NE, 2=E(right), etc. */
#define NAV_DIR_N   0   /* Forward (0°) */
#define NAV_DIR_NE  1   /* Forward-right (45°) */
#define NAV_DIR_E   2   /* Right (90°) */
#define NAV_DIR_SE  3   /* Back-right (135°) */
#define NAV_DIR_S   4   /* Backward (180°) */
#define NAV_DIR_SW  5   /* Back-left (225°) */
#define NAV_DIR_W   6   /* Left (270°) */
#define NAV_DIR_NW  7   /* Forward-left (315°) */

/* Maximum ray cast distance (cells) */
#define NAV_MAX_RAY_DIST    20

/* Exploration completion threshold */
#define NAV_DEFAULT_COMPLETE_THRESHOLD  100

/* Forward declaration */
struct occupancy_grid;

/**
 * @brief Result of frontier search
 */
struct frontier_result {
    int direction;              /* Best direction (0-7) */
    int score;                  /* Exploration score (higher = better) */
    int distance_cells;         /* Distance to nearest frontier */
    int unknown_count;          /* Unknown cells in this direction */
    bool path_blocked;          /* True if obstacle blocks path */
    bool exploration_complete;  /* True if no more frontiers */
};

/**
 * @brief Direction vectors (cosine × 1000, sine × 1000)
 *
 * Index by direction (0-7) to get unit vector components.
 */
extern const int16_t nav_dir_cos[NAV_NUM_DIRECTIONS];
extern const int16_t nav_dir_sin[NAV_NUM_DIRECTIONS];

/* ==================== Direction Utilities ==================== */

/**
 * @brief Convert direction index to angle in degrees
 *
 * @param direction Direction index (0-7)
 * @return Angle in degrees (0, 45, 90, ..., 315)
 */
int frontier_dir_to_angle(int direction);

/**
 * @brief Convert angle to nearest direction index
 *
 * @param angle_deg Angle in degrees (0-359)
 * @return Direction index (0-7)
 */
int frontier_angle_to_dir(int angle_deg);

/**
 * @brief Calculate signed difference between directions
 *
 * Returns the shortest rotation from 'from' to 'to'.
 *
 * @param from Starting direction (0-7)
 * @param to Target direction (0-7)
 * @return Difference (-4 to +3), negative = left, positive = right
 */
int frontier_dir_diff(int from, int to);

/* ==================== Frontier Search ==================== */

/**
 * @brief Find best direction toward unexplored area
 *
 * Casts rays in 8 directions from robot position, counting
 * unknown cells and checking for obstacles. Returns direction
 * with highest exploration score.
 *
 * Score formula:
 *   score = unknown_count * 10 + (MAX_DIST - first_unknown) * 2 - turn_penalty * 3
 *
 * @param grid Occupancy grid
 * @param robot_cx Robot cell X coordinate
 * @param robot_cy Robot cell Y coordinate
 * @param robot_heading_deg Robot heading in degrees (for turn penalty)
 * @param result Output result structure
 * @return 0 on success, negative errno on failure
 */
int frontier_find_best(const struct occupancy_grid *grid,
                       int robot_cx, int robot_cy,
                       int robot_heading_deg,
                       struct frontier_result *result);

/**
 * @brief Check exploration status for single direction
 *
 * @param grid Occupancy grid
 * @param robot_cx Robot cell X coordinate
 * @param robot_cy Robot cell Y coordinate
 * @param direction Direction to check (0-7)
 * @param result Output result (optional, can be NULL)
 * @return Number of unknown cells in direction
 */
int frontier_check_direction(const struct occupancy_grid *grid,
                             int robot_cx, int robot_cy,
                             int direction,
                             struct frontier_result *result);

/**
 * @brief Check if exploration is complete
 *
 * @param grid Occupancy grid
 * @param threshold Maximum unknown cells to consider complete
 * @return true if exploration is complete
 */
bool frontier_is_complete(const struct occupancy_grid *grid, int threshold);

/**
 * @brief Get nearest obstacle distance in direction
 *
 * @param grid Occupancy grid
 * @param robot_cx Robot cell X coordinate
 * @param robot_cy Robot cell Y coordinate
 * @param direction Direction to check (0-7)
 * @return Distance in cells to nearest obstacle, or NAV_MAX_RAY_DIST if clear
 */
int frontier_obstacle_distance(const struct occupancy_grid *grid,
                               int robot_cx, int robot_cy,
                               int direction);

#ifdef __cplusplus
}
#endif

#endif /* FRONTIER_NAV_H */
