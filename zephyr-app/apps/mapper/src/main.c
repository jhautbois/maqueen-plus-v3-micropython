/*
 * Copyright (c) 2026 Léandre's Maqueen Project
 * SPDX-License-Identifier: Apache-2.0
 *
 * Mapper Application - Autonomous exploration with occupancy grid mapping
 *
 * Button A: Start/Pause exploration
 * Button B: Save map and export via shell
 * Shell: map show/export/save/load/stats/clear
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <stdlib.h>
#include <string.h>

#include "maqueen.h"
#include "odometry.h"
#include "sen0628.h"
#include "occupancy_grid.h"
#include "scanner_360.h"
#include "frontier_nav.h"
#include "map_storage.h"

LOG_MODULE_REGISTER(mapper, LOG_LEVEL_INF);

/* ==================== Hardware Configuration ==================== */

/* I2C device for external sensors (edge connector) */
#define I2C_NODE DT_NODELABEL(i2c1)

/* LIDAR device */
#define LIDAR_NODE DT_NODELABEL(lidar)

/* Buttons */
#define BTN_A_NODE DT_ALIAS(sw0)
#define BTN_B_NODE DT_ALIAS(sw1)

#if !DT_NODE_HAS_STATUS(BTN_A_NODE, okay) || !DT_NODE_HAS_STATUS(BTN_B_NODE, okay)
#error "Button nodes not properly defined"
#endif

static const struct gpio_dt_spec btn_a = GPIO_DT_SPEC_GET(BTN_A_NODE, gpios);
static const struct gpio_dt_spec btn_b = GPIO_DT_SPEC_GET(BTN_B_NODE, gpios);

/* LED Display */
static const struct device *display_dev;

/* 5x5 LED patterns (1 bit per LED, row by row) */
static const uint8_t icon_ready[] = {    /* Checkmark */
    0b00000,
    0b00001,
    0b00010,
    0b10100,
    0b01000,
};

static const uint8_t icon_scanning[] __unused = {  /* Rotating dot positions */
    0b00100,
    0b00000,
    0b00000,
    0b00000,
    0b00000,
};

static const uint8_t icon_exploring[] = { /* Arrow up */
    0b00100,
    0b01110,
    0b10101,
    0b00100,
    0b00100,
};

static const uint8_t icon_paused[] = {    /* Pause bars */
    0b01010,
    0b01010,
    0b01010,
    0b01010,
    0b01010,
};

static const uint8_t icon_error[] = {     /* X */
    0b10001,
    0b01010,
    0b00100,
    0b01010,
    0b10001,
};

static const uint8_t icon_need_calib[] = { /* ? for press A to calibrate */
    0b01110,
    0b10001,
    0b00010,
    0b00100,
    0b00100,
};

static const uint8_t icon_calibrating[] = { /* Rotating C */
    0b01110,
    0b10000,
    0b10000,
    0b10000,
    0b01110,
};

/* ==================== Application State ==================== */

/* State machine */
enum mapper_state {
    STATE_INIT,
    STATE_IDLE,
    STATE_NEED_CALIBRATION,
    STATE_CALIBRATING,
    STATE_SCANNING,
    STATE_EXPLORING,
    STATE_PAUSED,
    STATE_ERROR
};

static const char *state_names[] __unused = {
    "INIT", "IDLE", "NEED_CALIB", "CALIBRATING", "SCANNING", "EXPLORING", "PAUSED", "ERROR"
};

/* Global state */
static struct {
    enum mapper_state state;
    const struct device *i2c_dev;
    const struct device *lidar_dev;

    /* Mapping components */
    struct occupancy_grid grid;
    struct odometry odom;
    struct scanner_360 scanner;

    /* Exploration parameters */
    int explore_speed;
    int explore_loop_ms;
    uint32_t loop_count;

    /* Button state */
    bool btn_a_pressed;
    bool btn_b_pressed;
} ctx;

/* Shell interface - global pointers for map_shell.c */
struct occupancy_grid *map_shell_grid = NULL;
struct odometry *map_shell_odom = NULL;
const struct device *map_shell_i2c = NULL;

/* ==================== LED Display ==================== */

static void display_icon(const uint8_t *icon)
{
    if (!display_dev) {
        return;
    }

    /* Micro:bit display buffer: 5 rows, each row is 1 byte (5 bits used) */
    uint8_t buf[5];
    for (int i = 0; i < 5; i++) {
        buf[i] = icon[i];
    }

    struct display_buffer_descriptor desc = {
        .buf_size = sizeof(buf),
        .width = 5,
        .height = 5,
        .pitch = 8,  /* bits per row (padded to byte) */
    };

    display_write(display_dev, 0, 0, &desc, buf);
}

static void __unused led_clear(void)
{
    if (!display_dev) {
        return;
    }

    uint8_t buf[5] = {0};
    struct display_buffer_descriptor desc = {
        .buf_size = sizeof(buf),
        .width = 5,
        .height = 5,
        .pitch = 8,
    };

    display_write(display_dev, 0, 0, &desc, buf);
}

static void display_scanning_animation(int position)
{
    if (!display_dev) {
        return;
    }

    /* Rotating dot around the edge */
    uint8_t buf[5] = {0};
    int pos = position % 16;

    if (pos < 5) {
        buf[0] = 1 << (4 - pos);      /* Top row, left to right */
    } else if (pos < 9) {
        buf[pos - 4] = 0b00001;       /* Right column, top to bottom */
    } else if (pos < 13) {
        buf[4] = 1 << (pos - 8);      /* Bottom row, right to left */
    } else {
        buf[16 - pos] = 0b10000;      /* Left column, bottom to top */
    }

    struct display_buffer_descriptor desc = {
        .buf_size = sizeof(buf),
        .width = 5,
        .height = 5,
        .pitch = 8,
    };

    display_write(display_dev, 0, 0, &desc, buf);
}

/* ==================== Button Handling ==================== */

static bool btn_initialized = false;
static bool btn_a_last = false;
static bool btn_b_last = false;

static void check_buttons(void)
{
    bool btn_a_now = gpio_pin_get_dt(&btn_a) == 0;  /* Active low */
    bool btn_b_now = gpio_pin_get_dt(&btn_b) == 0;

    /* On first call, just initialize last state (no events) */
    if (!btn_initialized) {
        btn_a_last = btn_a_now;
        btn_b_last = btn_b_now;
        btn_initialized = true;
        return;
    }

    /* Detect rising edge (press) */
    if (btn_a_now && !btn_a_last) {
        ctx.btn_a_pressed = true;
    }
    if (btn_b_now && !btn_b_last) {
        ctx.btn_b_pressed = true;
    }

    btn_a_last = btn_a_now;
    btn_b_last = btn_b_now;
}

/* ==================== Exploration Logic ==================== */

static void explore_step(void)
{
    struct sen0628_columns cols;
    struct sen0628_scan scan;
    struct frontier_result frontier;
    int ret;

    /* Update odometry */
    odometry_update(&ctx.odom);

    /* Read LIDAR */
    ret = sen0628_read_scan(ctx.lidar_dev, &scan);
    if (ret < 0) {
        LOG_WRN("LIDAR scan failed: %d", ret);
    }

    ret = sen0628_get_columns(ctx.lidar_dev, &cols);
    if (ret < 0) {
        LOG_ERR("Failed to get columns: %d", ret);
        maqueen_stop_all(ctx.i2c_dev);
        return;
    }

    /* Update grid */
    grid_update_from_lidar(&ctx.grid, &cols, &ctx.odom);

    /* Get robot position in grid */
    int rx, ry;
    grid_world_to_cell(&ctx.grid, ctx.odom.x, ctx.odom.y, &rx, &ry);
    int heading_deg = ctx.odom.heading_mdeg / 1000;

    /* Find best frontier direction */
    ret = frontier_find_best(&ctx.grid, rx, ry, heading_deg, &frontier);

    /* Check if exploration complete */
    if (frontier.exploration_complete ||
        frontier_is_complete(&ctx.grid, NAV_DEFAULT_COMPLETE_THRESHOLD)) {
        LOG_INF("Exploration complete!");
        maqueen_stop_all(ctx.i2c_dev);
        map_storage_save(&ctx.grid, &ctx.odom);
        ctx.state = STATE_IDLE;
        return;
    }

    /* Navigate */
    int center_dist = (cols.min[3] + cols.min[4]) / 2;
    int current_dir = frontier_angle_to_dir(heading_deg);
    int turn_diff = frontier_dir_diff(current_dir, frontier.direction);

    /* Obstacle avoidance priority */
    if (center_dist < 150) {
        /* Too close - back up and turn */
        LOG_DBG("Obstacle close (%dmm), backing up", center_dist);
        maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                          MAQUEEN_DIR_REVERSE, 80);
        maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                          MAQUEEN_DIR_REVERSE, 80);
        odometry_set_direction(&ctx.odom, false, false);
        k_sleep(K_MSEC(300));

        /* Turn toward clear direction */
        if (turn_diff >= 0) {
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                              MAQUEEN_DIR_REVERSE, 80);
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                              MAQUEEN_DIR_FORWARD, 80);
            odometry_set_direction(&ctx.odom, false, true);
        } else {
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                              MAQUEEN_DIR_FORWARD, 80);
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                              MAQUEEN_DIR_REVERSE, 80);
            odometry_set_direction(&ctx.odom, true, false);
        }
        k_sleep(K_MSEC(200));
    }
    else if (abs(turn_diff) > 1) {
        /* Need significant turn */
        int speed_fast = ctx.explore_speed;
        int speed_slow = ctx.explore_speed * 60 / 100;

        if (turn_diff > 0) {
            /* Turn right */
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                              MAQUEEN_DIR_FORWARD, speed_fast);
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                              MAQUEEN_DIR_FORWARD, speed_slow);
        } else {
            /* Turn left */
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                              MAQUEEN_DIR_FORWARD, speed_slow);
            maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                              MAQUEEN_DIR_FORWARD, speed_fast);
        }
        odometry_set_direction(&ctx.odom, true, true);
    }
    else {
        /* Go straight toward frontier */
        int speed = (center_dist > 500) ? ctx.explore_speed :
                    ctx.explore_speed * 70 / 100;
        maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_LEFT,
                          MAQUEEN_DIR_FORWARD, speed);
        maqueen_motor_set(ctx.i2c_dev, MAQUEEN_MOTOR_RIGHT,
                          MAQUEEN_DIR_FORWARD, speed);
        odometry_set_direction(&ctx.odom, true, true);
    }

    ctx.loop_count++;

    /* Periodic status */
    if (ctx.loop_count % 50 == 0) {
        struct grid_stats stats;
        grid_get_stats(&ctx.grid, &stats);
        LOG_INF("Exploring: pos=(%d,%d) head=%d° unknown=%d",
                (int)ctx.odom.x, (int)ctx.odom.y, heading_deg,
                stats.unknown_count);
    }
}

/* ==================== Scan Progress Callback ==================== */

static void scan_progress(int position, int total, void *user_data)
{
    ARG_UNUSED(user_data);
    LOG_INF("Scan progress: %d/%d", position + 1, total);
    display_scanning_animation(position);
}

static bool scan_abort_check(void *user_data)
{
    ARG_UNUSED(user_data);
    /* Check button A or B for abort */
    check_buttons();
    return ctx.btn_a_pressed || ctx.btn_b_pressed;
}

/* ==================== State Machine ==================== */

static void handle_state(void)
{
    switch (ctx.state) {
    case STATE_NEED_CALIBRATION:
        /* Button A: Start calibration */
        if (ctx.btn_a_pressed) {
            ctx.btn_a_pressed = false;
            LOG_INF("Starting calibration...");
            display_icon(icon_calibrating);
            ctx.state = STATE_CALIBRATING;
        }
        break;

    case STATE_CALIBRATING:
        {
            int ret = scanner_calibrate_rotation(&ctx.scanner,
                                                  scan_abort_check, NULL);
            if (ret == -ECANCELED) {
                LOG_INF("Calibration aborted");
                ctx.btn_a_pressed = false;
                ctx.btn_b_pressed = false;
                maqueen_stop_all(ctx.i2c_dev);
                display_icon(icon_need_calib);
                ctx.state = STATE_NEED_CALIBRATION;
            } else if (ret < 0) {
                LOG_ERR("Calibration failed: %d", ret);
                display_icon(icon_error);
                ctx.state = STATE_ERROR;
            } else {
                /* Save calibration to NVS */
                map_storage_save_calibration(
                    ctx.scanner.turn_speed,
                    ctx.scanner.ms_per_degree);
                LOG_INF("Calibration complete. Press A to start scan.");
                display_icon(icon_ready);
                ctx.state = STATE_IDLE;
            }
        }
        break;

    case STATE_IDLE:
        /* Button A: Start exploration */
        if (ctx.btn_a_pressed) {
            ctx.btn_a_pressed = false;
            LOG_INF("Starting 360° scan...");
            ctx.state = STATE_SCANNING;
            /* Animation will be shown in scan_progress callback */
        }
        /* Button B: Save and export map */
        if (ctx.btn_b_pressed) {
            ctx.btn_b_pressed = false;
            LOG_INF("Saving map...");
            map_storage_save(&ctx.grid, &ctx.odom);
            LOG_INF("Map saved. Use 'map export' to view.");
        }
        break;

    case STATE_SCANNING:
        {
            struct scan_result result;
            int ret = scanner_full_scan(&ctx.scanner, scan_progress,
                                        scan_abort_check, NULL, &result);
            if (ret == -ECANCELED) {
                LOG_INF("Scan aborted");
                ctx.btn_a_pressed = false;
                ctx.btn_b_pressed = false;
                maqueen_stop_all(ctx.i2c_dev);
                ctx.state = STATE_IDLE;
                display_icon(icon_ready);
            } else if (ret < 0) {
                LOG_ERR("Scan failed: %d", ret);
                ctx.state = STATE_ERROR;
                display_icon(icon_error);
            } else {
                LOG_INF("Scan complete: %d obstacles in %dms",
                        result.obstacles_detected, result.duration_ms);
                ctx.state = STATE_EXPLORING;
                display_icon(icon_exploring);
                LOG_INF("Starting exploration...");
            }
        }
        break;

    case STATE_EXPLORING:
        /* Button A: Pause */
        if (ctx.btn_a_pressed) {
            ctx.btn_a_pressed = false;
            maqueen_stop_all(ctx.i2c_dev);
            ctx.state = STATE_PAUSED;
            display_icon(icon_paused);
            LOG_INF("Paused");
            break;
        }
        /* Button B: Emergency stop and return to IDLE */
        if (ctx.btn_b_pressed) {
            ctx.btn_b_pressed = false;
            maqueen_stop_all(ctx.i2c_dev);
            map_storage_save(&ctx.grid, &ctx.odom);
            ctx.state = STATE_IDLE;
            display_icon(icon_ready);
            LOG_INF("Emergency stop - map saved");
            break;
        }
        /* Exploration step */
        explore_step();
        break;

    case STATE_PAUSED:
        /* Button A: Resume exploration */
        if (ctx.btn_a_pressed) {
            ctx.btn_a_pressed = false;
            ctx.state = STATE_EXPLORING;
            display_icon(icon_exploring);
            LOG_INF("Resumed");
        }
        /* Button B: Save, stop, return to IDLE */
        if (ctx.btn_b_pressed) {
            ctx.btn_b_pressed = false;
            map_storage_save(&ctx.grid, &ctx.odom);
            ctx.state = STATE_IDLE;
            display_icon(icon_ready);
            LOG_INF("Map saved - returned to IDLE");
        }
        break;

    case STATE_ERROR:
        /* Button A: Try to recover */
        if (ctx.btn_a_pressed) {
            ctx.btn_a_pressed = false;
            maqueen_stop_all(ctx.i2c_dev);
            ctx.state = STATE_IDLE;
            display_icon(icon_ready);
            LOG_INF("Reset to IDLE");
        }
        break;

    default:
        break;
    }
}

/* ==================== Initialization ==================== */

static int init_hardware(void)
{
    int ret;

    /* LED Display */
    display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
    if (!device_is_ready(display_dev)) {
        LOG_WRN("Display not ready, continuing without LED feedback");
        display_dev = NULL;
    } else {
        display_blanking_off(display_dev);
        display_icon(icon_ready);  /* Show ready icon immediately */
    }

    /* I2C device (for Maqueen) */
    ctx.i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(ctx.i2c_dev)) {
        LOG_ERR("I2C device not ready");
        display_icon(icon_error);
        return -ENODEV;
    }

    /* LIDAR device (from device tree) */
    ctx.lidar_dev = DEVICE_DT_GET(LIDAR_NODE);
    if (!device_is_ready(ctx.lidar_dev)) {
        LOG_ERR("LIDAR device not ready");
        return -ENODEV;
    }

    /* Buttons (active low with pull-up) */
    ret = gpio_pin_configure_dt(&btn_a, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure button A: %d", ret);
        return ret;
    }

    ret = gpio_pin_configure_dt(&btn_b, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Failed to configure button B: %d", ret);
        return ret;
    }

    /* Maqueen motor controller */
    ret = maqueen_init(ctx.i2c_dev);
    if (ret < 0) {
        LOG_ERR("Maqueen init failed: %d", ret);
        return ret;
    }

    /* SEN0628 LIDAR - initialize */
    ret = sen0628_init(ctx.lidar_dev);
    if (ret < 0) {
        LOG_ERR("SEN0628 init failed: %d", ret);
        return ret;
    }

    /* Set LIDAR to 4x4 mode for faster scans */
    ret = sen0628_set_mode(ctx.lidar_dev, 4);
    if (ret < 0) {
        LOG_WRN("Failed to set LIDAR mode: %d", ret);
    }

    LOG_INF("Hardware initialized");
    return 0;
}

static int init_mapping(void)
{
    int ret;

    /* Initialize grid */
    grid_init(&ctx.grid);

    /* Initialize odometry */
    odometry_init(&ctx.odom, ctx.i2c_dev);

    /* Initialize scanner */
    ret = scanner_init(&ctx.scanner, ctx.i2c_dev, ctx.lidar_dev, &ctx.grid, &ctx.odom);
    if (ret < 0) {
        LOG_ERR("Scanner init failed: %d", ret);
        return ret;
    }

    /* Initialize storage */
    ret = map_storage_init();
    if (ret < 0) {
        LOG_WRN("Storage init failed: %d (continuing without persistence)", ret);
    }

    /* Try to load calibration data */
    if (map_storage_calibration_exists()) {
        uint16_t turn_speed, ms_per_deg;
        ret = map_storage_load_calibration(&turn_speed, &ms_per_deg);
        if (ret == 0) {
            scanner_set_rotation_params(&ctx.scanner, turn_speed, ms_per_deg);
            LOG_INF("Loaded calibration: speed=%d, ms/deg=%d", turn_speed, ms_per_deg);
        }
    }

    /* Try to load existing map */
    if (map_storage_exists()) {
        ret = map_storage_load(&ctx.grid, &ctx.odom);
        if (ret == 0) {
            LOG_INF("Loaded saved map");
        }
    }

    /* Set up shell interface */
    map_shell_grid = &ctx.grid;
    map_shell_odom = &ctx.odom;
    map_shell_i2c = ctx.i2c_dev;

    /* Exploration parameters */
    ctx.explore_speed = 100;
    ctx.explore_loop_ms = 100;

    LOG_INF("Mapping initialized");
    return 0;
}

/* ==================== Main ==================== */

int main(void)
{
    int ret;

    LOG_INF("=== Mapper Application ===");
    LOG_INF("Button A: Start/Pause exploration");
    LOG_INF("Button B: Save map");
    LOG_INF("Shell: 'map' for commands");

    ctx.state = STATE_INIT;

    /* Initialize hardware */
    ret = init_hardware();
    if (ret < 0) {
        LOG_ERR("Hardware init failed");
        ctx.state = STATE_ERROR;
        return ret;
    }

    /* Initialize mapping */
    ret = init_mapping();
    if (ret < 0) {
        LOG_ERR("Mapping init failed");
        ctx.state = STATE_ERROR;
        return ret;
    }

    /* Check if calibration is needed */
    if (map_storage_calibration_exists()) {
        ctx.state = STATE_IDLE;
        display_icon(icon_ready);
        LOG_INF("Ready - Press button A to start");
    } else {
        ctx.state = STATE_NEED_CALIBRATION;
        display_icon(icon_need_calib);
        LOG_INF("Calibration needed - Press button A to calibrate");
    }

    /* Main loop */
    while (1) {
        check_buttons();
        handle_state();

        /* Loop timing based on state */
        int delay_ms;
        switch (ctx.state) {
        case STATE_EXPLORING:
            delay_ms = ctx.explore_loop_ms;
            break;
        case STATE_SCANNING:
            delay_ms = 10;  /* Fast polling during scan */
            break;
        default:
            delay_ms = 100;  /* Idle polling */
            break;
        }

        k_sleep(K_MSEC(delay_ms));
    }

    return 0;
}
