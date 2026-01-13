/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Shell commands for map operations
 */

#include <zephyr/shell/shell.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>

#include "occupancy_grid.h"
#include "map_storage.h"
#include "odometry.h"
#include "maqueen.h"

LOG_MODULE_REGISTER(map_shell, LOG_LEVEL_INF);

/* External references - must be provided by application */
extern struct occupancy_grid *map_shell_grid;
extern struct odometry *map_shell_odom;
extern const struct device *map_shell_i2c;

/* ==================== Command Handlers ==================== */

/**
 * @brief Show map as ASCII art
 *
 * Usage: map show [scale]
 * Scale: 1 = full size, 2 = half, 4 = quarter
 */
static int cmd_map_show(const struct shell *sh, size_t argc, char **argv)
{
    int scale = 1;

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    if (argc > 1) {
        scale = atoi(argv[1]);
        if (scale < 1 || scale > 4) {
            scale = 1;
        }
    }

    int display_size = GRID_SIZE / scale;

    /* Header line */
    shell_fprintf(sh, SHELL_NORMAL, "+");
    for (int x = 0; x < display_size; x++) {
        shell_fprintf(sh, SHELL_NORMAL, "-");
    }
    shell_fprintf(sh, SHELL_NORMAL, "+\n");

    /* Grid rows */
    for (int y = 0; y < display_size; y++) {
        shell_fprintf(sh, SHELL_NORMAL, "|");

        for (int x = 0; x < display_size; x++) {
            /* Sample center of scaled cell */
            int gx = x * scale + scale / 2;
            int gy = y * scale + scale / 2;

            cell_state_t state = grid_get_state(map_shell_grid, gx, gy);
            char c;

            switch (state) {
            case CELL_FREE:
                c = ' ';
                break;
            case CELL_OCCUPIED:
                c = '#';
                break;
            default:
                c = '.';
                break;
            }

            /* Mark robot position */
            if (map_shell_odom) {
                int rx, ry;
                grid_world_to_cell(map_shell_grid, map_shell_odom->x,
                                   map_shell_odom->y, &rx, &ry);
                if (gx / scale == rx / scale && gy / scale == ry / scale) {
                    c = '@';
                }
            }

            shell_fprintf(sh, SHELL_NORMAL, "%c", c);
        }

        shell_fprintf(sh, SHELL_NORMAL, "|\n");
    }

    /* Footer line */
    shell_fprintf(sh, SHELL_NORMAL, "+");
    for (int x = 0; x < display_size; x++) {
        shell_fprintf(sh, SHELL_NORMAL, "-");
    }
    shell_fprintf(sh, SHELL_NORMAL, "+\n");

    return 0;
}

/**
 * @brief Export map in machine-readable format
 *
 * Usage: map export
 */
static int cmd_map_export(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    struct grid_stats stats;
    grid_get_stats(map_shell_grid, &stats);

    shell_print(sh, "---MAP-START---");
    shell_print(sh, "VERSION:%d", MAP_VERSION);
    shell_print(sh, "SIZE:%d", GRID_SIZE);
    shell_print(sh, "CELL:%d", CELL_MM);
    shell_print(sh, "ORIGIN:%d,%d",
                (int)map_shell_grid->origin_x,
                (int)map_shell_grid->origin_y);

    if (map_shell_odom) {
        shell_print(sh, "POSE:%d,%d,%d",
                    (int)map_shell_odom->x,
                    (int)map_shell_odom->y,
                    (int)map_shell_odom->heading_mdeg);
    } else {
        shell_print(sh, "POSE:0,0,0");
    }

    shell_print(sh, "UPDATES:%u", stats.update_count);

    /* Export grid data as hex */
    shell_fprintf(sh, SHELL_NORMAL, "DATA:");

    for (int i = 0; i < GRID_BYTES; i++) {
        shell_fprintf(sh, SHELL_NORMAL, "%02X",
                      (uint8_t)map_shell_grid->data[i]);

        /* Line break every 64 bytes for readability */
        if ((i + 1) % 64 == 0 && i < GRID_BYTES - 1) {
            shell_fprintf(sh, SHELL_NORMAL, "\n     ");
        }
    }
    shell_fprintf(sh, SHELL_NORMAL, "\n");

    shell_print(sh, "STATS:%d,%d,%d",
                stats.unknown_count, stats.free_count, stats.occupied_count);

    uint32_t crc = map_storage_crc32((const uint8_t *)map_shell_grid->data,
                                     GRID_BYTES);
    shell_print(sh, "CRC:%08X", crc);

    shell_print(sh, "---MAP-END---");

    return 0;
}

/**
 * @brief Save map to NVS
 *
 * Usage: map save
 */
static int cmd_map_save(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    int ret = map_storage_save(map_shell_grid, map_shell_odom);
    if (ret < 0) {
        shell_error(sh, "Save failed: %d", ret);
        return ret;
    }

    shell_print(sh, "Map saved to NVS");
    return 0;
}

/**
 * @brief Load map from NVS
 *
 * Usage: map load
 */
static int cmd_map_load(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    int ret = map_storage_load(map_shell_grid, map_shell_odom);
    if (ret < 0) {
        shell_error(sh, "Load failed: %d", ret);
        return ret;
    }

    shell_print(sh, "Map loaded from NVS");
    return 0;
}

/**
 * @brief Clear map (RAM only)
 *
 * Usage: map clear
 */
static int cmd_map_clear(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    grid_clear(map_shell_grid);
    shell_print(sh, "Map cleared");
    return 0;
}

/**
 * @brief Erase map from NVS
 *
 * Usage: map erase
 */
static int cmd_map_erase(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    int ret = map_storage_clear();
    if (ret < 0) {
        shell_error(sh, "Erase failed: %d", ret);
        return ret;
    }

    shell_print(sh, "NVS map erased");
    return 0;
}

/**
 * @brief Show map statistics
 *
 * Usage: map stats
 */
static int cmd_map_stats(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    struct grid_stats grid_stats;
    grid_get_stats(map_shell_grid, &grid_stats);

    shell_print(sh, "=== Map Statistics ===");
    shell_print(sh, "Grid size:    %dx%d (%d cells)", GRID_SIZE, GRID_SIZE,
                GRID_SIZE * GRID_SIZE);
    shell_print(sh, "Cell size:    %dmm", CELL_MM);
    shell_print(sh, "Coverage:     %dmm x %dmm", GRID_COVERAGE, GRID_COVERAGE);
    shell_print(sh, "Memory:       %d bytes", GRID_BYTES);
    shell_print(sh, "");
    shell_print(sh, "Unknown:      %d cells (%.1f%%)",
                grid_stats.unknown_count,
                (double)(100.0f * grid_stats.unknown_count / (GRID_SIZE * GRID_SIZE)));
    shell_print(sh, "Free:         %d cells (%.1f%%)",
                grid_stats.free_count,
                (double)(100.0f * grid_stats.free_count / (GRID_SIZE * GRID_SIZE)));
    shell_print(sh, "Occupied:     %d cells (%.1f%%)",
                grid_stats.occupied_count,
                (double)(100.0f * grid_stats.occupied_count / (GRID_SIZE * GRID_SIZE)));
    shell_print(sh, "");
    shell_print(sh, "Updates:      %u", grid_stats.update_count);

    /* NVS stats */
    struct map_storage_stats storage_stats;
    map_storage_get_stats(&storage_stats);

    shell_print(sh, "");
    shell_print(sh, "=== Storage Statistics ===");
    shell_print(sh, "NVS ready:    %s", storage_stats.initialized ? "yes" : "no");
    shell_print(sh, "Map in NVS:   %s", storage_stats.map_exists ? "yes" : "no");
    shell_print(sh, "Saves:        %u", storage_stats.save_count);
    shell_print(sh, "Loads:        %u", storage_stats.load_count);

    return 0;
}

/**
 * @brief Show NVS map info
 *
 * Usage: map info
 */
static int cmd_map_info(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    struct map_header header;
    int ret = map_storage_get_header(&header);

    if (ret < 0) {
        shell_print(sh, "No map stored in NVS");
        return 0;
    }

    shell_print(sh, "=== Stored Map Info ===");
    shell_print(sh, "Version:      %d", header.version);
    shell_print(sh, "Grid size:    %d", header.grid_size);
    shell_print(sh, "Cell size:    %dmm", header.cell_mm);
    shell_print(sh, "Origin:       (%d, %d)", (int)header.origin_x,
                (int)header.origin_y);
    shell_print(sh, "Robot pose:   (%d, %d) @ %d°",
                (int)header.robot_x, (int)header.robot_y,
                (int)(header.robot_heading / 1000));
    shell_print(sh, "Updates:      %u", header.update_count);
    shell_print(sh, "Saved at:     %ums uptime", header.timestamp);
    shell_print(sh, "CRC:          0x%08X", header.checksum);

    return 0;
}

/**
 * @brief Query specific cell
 *
 * Usage: map cell <x> <y>
 */
static int cmd_map_cell(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 3) {
        shell_error(sh, "Usage: map cell <x> <y>");
        return -EINVAL;
    }

    if (!map_shell_grid) {
        shell_error(sh, "No grid available");
        return -ENODEV;
    }

    int cx = atoi(argv[1]);
    int cy = atoi(argv[2]);

    if (!grid_is_valid(cx, cy)) {
        shell_error(sh, "Cell (%d,%d) out of bounds (0-%d)",
                    cx, cy, GRID_SIZE - 1);
        return -EINVAL;
    }

    int8_t logodds = grid_get_logodds(map_shell_grid, cx, cy);
    cell_state_t state = grid_get_state(map_shell_grid, cx, cy);
    const char *state_str = (state == CELL_FREE) ? "FREE" :
                            (state == CELL_OCCUPIED) ? "OCCUPIED" : "UNKNOWN";

    int32_t wx, wy;
    grid_cell_to_world(map_shell_grid, cx, cy, &wx, &wy);

    shell_print(sh, "Cell (%d,%d):", cx, cy);
    shell_print(sh, "  Log-odds: %d", logodds);
    shell_print(sh, "  State:    %s", state_str);
    shell_print(sh, "  World:    (%dmm, %dmm)", (int)wx, (int)wy);

    return 0;
}

/**
 * @brief Show robot pose
 *
 * Usage: map pose
 */
static int cmd_map_pose(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_odom) {
        shell_error(sh, "No odometry available");
        return -ENODEV;
    }

    int rx = 0, ry = 0;
    if (map_shell_grid) {
        grid_world_to_cell(map_shell_grid, map_shell_odom->x,
                           map_shell_odom->y, &rx, &ry);
    }

    shell_print(sh, "Robot pose:");
    shell_print(sh, "  World:   (%dmm, %dmm)",
                (int)map_shell_odom->x, (int)map_shell_odom->y);
    shell_print(sh, "  Heading: %d° (%d mdeg)",
                (int)(map_shell_odom->heading_mdeg / 1000),
                (int)map_shell_odom->heading_mdeg);
    shell_print(sh, "  Cell:    (%d, %d)", rx, ry);

    return 0;
}

/**
 * @brief Emergency stop - stops all motors immediately
 *
 * Usage: map stop
 */
static int cmd_map_stop(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    if (!map_shell_i2c) {
        shell_error(sh, "No I2C device available");
        return -ENODEV;
    }

    int ret = maqueen_stop_all(map_shell_i2c);
    if (ret < 0) {
        shell_error(sh, "Failed to stop motors: %d", ret);
        return ret;
    }

    shell_print(sh, "Motors stopped");
    return 0;
}

/**
 * @brief Clear calibration data (forces recalibration on next start)
 *
 * Usage: map recalib
 */
static int cmd_map_recalib(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);

    /* Check if calibration exists */
    if (!map_storage_calibration_exists()) {
        shell_print(sh, "No calibration data stored");
        return 0;
    }

    /* Show current calibration */
    uint16_t turn_speed, ms_per_deg;
    int ret = map_storage_load_calibration(&turn_speed, &ms_per_deg);
    if (ret == 0) {
        shell_print(sh, "Current calibration: speed=%d, ms/deg=%d",
                    turn_speed, ms_per_deg);
    }

    /* Clear calibration */
    ret = map_storage_clear_calibration();
    if (ret < 0) {
        shell_error(sh, "Failed to clear calibration: %d", ret);
        return ret;
    }

    shell_print(sh, "Calibration cleared. Restart to recalibrate.");
    return 0;
}

/* ==================== Shell Command Registration ==================== */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_map,
    SHELL_CMD(show, NULL, "Show map as ASCII art [scale]", cmd_map_show),
    SHELL_CMD(export, NULL, "Export map in machine format", cmd_map_export),
    SHELL_CMD(save, NULL, "Save map to NVS", cmd_map_save),
    SHELL_CMD(load, NULL, "Load map from NVS", cmd_map_load),
    SHELL_CMD(clear, NULL, "Clear map (RAM)", cmd_map_clear),
    SHELL_CMD(erase, NULL, "Erase map from NVS", cmd_map_erase),
    SHELL_CMD(stats, NULL, "Show map statistics", cmd_map_stats),
    SHELL_CMD(info, NULL, "Show stored map info", cmd_map_info),
    SHELL_CMD(cell, NULL, "Query cell <x> <y>", cmd_map_cell),
    SHELL_CMD(pose, NULL, "Show robot pose", cmd_map_pose),
    SHELL_CMD(stop, NULL, "Emergency stop motors", cmd_map_stop),
    SHELL_CMD(recalib, NULL, "Clear calibration (restart needed)", cmd_map_recalib),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(map, &sub_map, "Map commands", NULL);
