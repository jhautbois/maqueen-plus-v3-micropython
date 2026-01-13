/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Frontier-based navigation implementation
 */

#include "frontier_nav.h"
#include "occupancy_grid.h"

#include <stdlib.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(frontier_nav, LOG_LEVEL_INF);

/*
 * Direction vectors scaled by 1000
 * Index 0 = North (forward), increasing clockwise
 *
 * Direction:  N     NE    E     SE    S     SW    W     NW
 * Angle:      0°    45°   90°   135°  180°  225°  270°  315°
 */
const int16_t nav_dir_cos[NAV_NUM_DIRECTIONS] = {
    1000,   /* N:  cos(0°)   = 1.000 */
    707,    /* NE: cos(45°)  = 0.707 */
    0,      /* E:  cos(90°)  = 0.000 */
    -707,   /* SE: cos(135°) = -0.707 */
    -1000,  /* S:  cos(180°) = -1.000 */
    -707,   /* SW: cos(225°) = -0.707 */
    0,      /* W:  cos(270°) = 0.000 */
    707     /* NW: cos(315°) = 0.707 */
};

const int16_t nav_dir_sin[NAV_NUM_DIRECTIONS] = {
    0,      /* N:  sin(0°)   = 0.000 */
    707,    /* NE: sin(45°)  = 0.707 */
    1000,   /* E:  sin(90°)  = 1.000 */
    707,    /* SE: sin(135°) = 0.707 */
    0,      /* S:  sin(180°) = 0.000 */
    -707,   /* SW: sin(225°) = -0.707 */
    -1000,  /* W:  sin(270°) = -1.000 */
    -707    /* NW: sin(315°) = -0.707 */
};

/* ==================== Direction Utilities ==================== */

int frontier_dir_to_angle(int direction)
{
    /* Normalize to 0-7 */
    direction = direction & 7;
    return direction * 45;
}

int frontier_angle_to_dir(int angle_deg)
{
    /* Normalize to 0-359 */
    while (angle_deg < 0) {
        angle_deg += 360;
    }
    while (angle_deg >= 360) {
        angle_deg -= 360;
    }

    /* Round to nearest 45° */
    return ((angle_deg + 22) / 45) & 7;
}

int frontier_dir_diff(int from, int to)
{
    int diff = (to - from) & 7;

    /* Convert to signed: 0-3 stay positive, 4-7 become negative */
    if (diff > 4) {
        diff -= 8;
    } else if (diff == 4) {
        /* 180° - prefer positive (right turn) */
        diff = 4;
    }

    return diff;
}

/* ==================== Frontier Search ==================== */

int frontier_check_direction(const struct occupancy_grid *grid,
                             int robot_cx, int robot_cy,
                             int direction,
                             struct frontier_result *result)
{
    int unknown_count = 0;
    int first_unknown = 0;
    bool blocked = false;
    int cos_val = nav_dir_cos[direction];
    int sin_val = nav_dir_sin[direction];

    /* Ray cast in this direction */
    for (int dist = 1; dist <= NAV_MAX_RAY_DIST; dist++) {
        /* Calculate cell coordinates */
        int cx = robot_cx + (dist * cos_val) / 1000;
        int cy = robot_cy + (dist * sin_val) / 1000;

        /* Check bounds */
        if (!grid_is_valid(cx, cy)) {
            break;
        }

        /* Get cell state */
        cell_state_t state = grid_get_state(grid, cx, cy);

        if (state == CELL_OCCUPIED) {
            blocked = true;
            break;
        }

        if (state == CELL_UNKNOWN) {
            unknown_count++;
            if (first_unknown == 0) {
                first_unknown = dist;
            }
        }
    }

    /* Fill result if provided */
    if (result) {
        result->direction = direction;
        result->unknown_count = unknown_count;
        result->distance_cells = first_unknown;
        result->path_blocked = blocked;
    }

    return unknown_count;
}

int frontier_find_best(const struct occupancy_grid *grid,
                       int robot_cx, int robot_cy,
                       int robot_heading_deg,
                       struct frontier_result *result)
{
    int best_dir = 0;
    int best_score = -1;
    int best_unknown = 0;
    int best_dist = NAV_MAX_RAY_DIST;
    bool any_frontier = false;

    int robot_dir = frontier_angle_to_dir(robot_heading_deg);

    LOG_DBG("Finding frontier from (%d,%d) heading %d°",
            robot_cx, robot_cy, robot_heading_deg);

    /* Check all 8 directions */
    for (int d = 0; d < NAV_NUM_DIRECTIONS; d++) {
        struct frontier_result dir_result;

        frontier_check_direction(grid, robot_cx, robot_cy, d, &dir_result);

        /* Skip blocked or empty directions */
        if (dir_result.path_blocked && dir_result.distance_cells == 0) {
            continue;
        }

        if (dir_result.unknown_count == 0) {
            continue;
        }

        any_frontier = true;

        /* Calculate score */
        int turn_penalty = abs(frontier_dir_diff(robot_dir, d));

        int score = dir_result.unknown_count * 10
                  + (NAV_MAX_RAY_DIST - dir_result.distance_cells) * 2
                  - turn_penalty * 3;

        /* Bonus for forward direction (prefer not turning) */
        if (d == robot_dir) {
            score += 5;
        }

        LOG_DBG("  Dir %d: unknown=%d, dist=%d, turn=%d, score=%d%s",
                d, dir_result.unknown_count, dir_result.distance_cells,
                turn_penalty, score, dir_result.path_blocked ? " (blocked)" : "");

        if (score > best_score) {
            best_score = score;
            best_dir = d;
            best_unknown = dir_result.unknown_count;
            best_dist = dir_result.distance_cells;
        }
    }

    /* Fill result */
    result->direction = best_dir;
    result->score = best_score;
    result->unknown_count = best_unknown;
    result->distance_cells = best_dist;
    result->path_blocked = false;  /* Best direction is not blocked */
    result->exploration_complete = !any_frontier;

    if (any_frontier) {
        LOG_INF("Best frontier: dir=%d (%d°), score=%d, unknown=%d",
                best_dir, frontier_dir_to_angle(best_dir),
                best_score, best_unknown);
    } else {
        LOG_INF("No frontiers found - exploration complete");
    }

    return any_frontier ? 0 : -ENOENT;
}

bool frontier_is_complete(const struct occupancy_grid *grid, int threshold)
{
    struct grid_stats stats;
    grid_get_stats(grid, &stats);

    bool complete = (stats.unknown_count < threshold);

    if (complete) {
        LOG_INF("Exploration complete: %d unknown cells (threshold=%d)",
                stats.unknown_count, threshold);
    }

    return complete;
}

int frontier_obstacle_distance(const struct occupancy_grid *grid,
                               int robot_cx, int robot_cy,
                               int direction)
{
    int cos_val = nav_dir_cos[direction];
    int sin_val = nav_dir_sin[direction];

    for (int dist = 1; dist <= NAV_MAX_RAY_DIST; dist++) {
        int cx = robot_cx + (dist * cos_val) / 1000;
        int cy = robot_cy + (dist * sin_val) / 1000;

        if (!grid_is_valid(cx, cy)) {
            return dist - 1;
        }

        if (grid_get_state(grid, cx, cy) == CELL_OCCUPIED) {
            return dist;
        }
    }

    return NAV_MAX_RAY_DIST;
}
