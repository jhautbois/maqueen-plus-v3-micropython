/*
 * Proximity LED Visualization Application
 *
 * Displays obstacle proximity using LEDs:
 *   - Headlights: Left/Right based on obstacle direction
 *   - Underglow: 4 LEDs mapped to 4 LIDAR columns
 *   - Color: Green (safe) -> Yellow (caution) -> Red (danger)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "sen0628.h"
#include "maqueen.h"

LOG_MODULE_REGISTER(proximity, LOG_LEVEL_INF);

/*
 * Distance thresholds (mm)
 */
#define DIST_DANGER     300     /* Red zone */
#define DIST_CAUTION    500     /* Yellow zone */
#define DIST_SAFE       500     /* Green zone (anything above caution) */

/*
 * Device handles
 */
#define I2C_EXT_NODE    DT_NODELABEL(i2c1)
#define LIDAR_NODE      DT_NODELABEL(lidar)
#define STRIP_NODE      DT_ALIAS(led_strip)

static const struct device *i2c_ext = DEVICE_DT_GET(I2C_EXT_NODE);
static const struct device *lidar = DEVICE_DT_GET(LIDAR_NODE);

#if DT_NODE_HAS_STATUS(STRIP_NODE, okay)
#define HAS_LED_STRIP 1
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_NUM_PIXELS];
#else
#define HAS_LED_STRIP 0
#warning "LED strip not configured"
#endif

/*
 * RGB color definitions
 * Using brighter values for visibility
 */
#define COLOR_OFF       (struct led_rgb){ .r = 0,    .g = 0,    .b = 0    }
#define COLOR_GREEN     (struct led_rgb){ .r = 0,    .g = 0x80, .b = 0    }
#define COLOR_YELLOW    (struct led_rgb){ .r = 0x80, .g = 0x40, .b = 0    }
#define COLOR_RED       (struct led_rgb){ .r = 0xFF, .g = 0,    .b = 0    }  /* Full red */

/*
 * Convert distance to RGB color
 * Very close objects (< 20mm) are treated as RED (not off)
 */
static struct led_rgb distance_to_color(uint16_t distance)
{
	if (distance >= SEN0628_DIST_INVALID) {
		return COLOR_OFF;  /* No reading */
	}
	/* Very close or within danger zone -> RED */
	if (distance < DIST_DANGER) {
		return COLOR_RED;
	}
	if (distance < DIST_CAUTION) {
		return COLOR_YELLOW;
	}
	return COLOR_GREEN;
}

/*
 * Convert distance to Maqueen headlight color
 * Very close objects (< 20mm) are treated as RED (not off)
 */
static uint8_t distance_to_headlight(uint16_t distance)
{
	if (distance >= SEN0628_DIST_INVALID) {
		return MAQUEEN_COLOR_OFF;  /* No reading */
	}
	/* Very close or within danger zone -> RED */
	if (distance < DIST_DANGER) {
		return MAQUEEN_COLOR_RED;
	}
	if (distance < DIST_CAUTION) {
		return MAQUEEN_COLOR_YELLOW;
	}
	return MAQUEEN_COLOR_GREEN;
}

/*
 * Update all LEDs based on LIDAR column data
 *
 * LED mapping (4x4 LIDAR mode):
 *   LED 0 (front-left)  -> Col 0
 *   LED 1 (rear-left)   -> Col 1
 *   LED 2 (rear-right)  -> Col 2
 *   LED 3 (front-right) -> Col 3
 */
static void update_leds(const struct sen0628_columns *cols)
{
	uint16_t left_min, right_min;
	int ret;

	/* Find minimum distance for left side (cols 0-1) */
	left_min = MIN(cols->min[0], cols->min[1]);

	/* Find minimum distance for right side (cols 2-3) */
	right_min = MIN(cols->min[2], cols->min[3]);

	/* Update headlights */
	ret = maqueen_headlight_set(i2c_ext, 0, distance_to_headlight(left_min));
	if (ret < 0) {
		LOG_WRN("Failed to set left headlight: %d", ret);
	}

	ret = maqueen_headlight_set(i2c_ext, 1, distance_to_headlight(right_min));
	if (ret < 0) {
		LOG_WRN("Failed to set right headlight: %d", ret);
	}

#if HAS_LED_STRIP
	/* Update all 4 underglow LEDs - one per LIDAR column */
	for (int i = 0; i < STRIP_NUM_PIXELS && i < 4; i++) {
		pixels[i] = distance_to_color(cols->min[i]);
	}

	ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	if (ret < 0) {
		LOG_WRN("Failed to update underglow: %d", ret);
	}
#endif
}

/*
 * Print status to console
 */
static void print_status(const struct sen0628_columns *cols, uint32_t scan_ms)
{
	static const char *zone_char[] = {"[===]", "[###]", "[XXX]", "[   ]"};
	int zones[4];

	for (int i = 0; i < 4; i++) {
		uint16_t d = cols->min[i];
		if (d >= SEN0628_DIST_INVALID) {
			zones[i] = 3;  /* No reading */
		} else if (d < DIST_DANGER) {
			zones[i] = 2;  /* Danger */
		} else if (d < DIST_CAUTION) {
			zones[i] = 1;  /* Caution */
		} else {
			zones[i] = 0;  /* Safe */
		}
	}

	printk("\r%s %s %s %s  L:%4d C:%4d R:%4d  %3dms  ",
	       zone_char[zones[0]], zone_char[zones[1]],
	       zone_char[zones[2]], zone_char[zones[3]],
	       cols->min[0], MIN(cols->min[1], cols->min[2]), cols->min[3],
	       scan_ms);
}

/*
 * Turn off all LEDs
 */
static void leds_off(void)
{
	maqueen_headlight_set(i2c_ext, 0, MAQUEEN_COLOR_OFF);
	maqueen_headlight_set(i2c_ext, 1, MAQUEEN_COLOR_OFF);

#if HAS_LED_STRIP
	memset(pixels, 0, sizeof(pixels));
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
#endif
}

/*
 * Startup animation - COLOR TEST to find correct order
 */
static void startup_animation(void)
{
	LOG_INF("=== COLOR TEST - Note what you see! ===");

#if HAS_LED_STRIP
	/* Test each primary color separately */
	struct led_rgb pure_red   = {.r = 0xFF, .g = 0x00, .b = 0x00};
	struct led_rgb pure_green = {.r = 0x00, .g = 0xFF, .b = 0x00};
	struct led_rgb pure_blue  = {.r = 0x00, .g = 0x00, .b = 0xFF};

	/* Test RED */
	printk("\n>>> Sending RED (FF,00,00) - what color do you see? <<<\n");
	for (int i = 0; i < STRIP_NUM_PIXELS; i++) pixels[i] = pure_red;
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	k_sleep(K_MSEC(2000));

	/* Test GREEN */
	printk(">>> Sending GREEN (00,FF,00) - what color do you see? <<<\n");
	for (int i = 0; i < STRIP_NUM_PIXELS; i++) pixels[i] = pure_green;
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	k_sleep(K_MSEC(2000));

	/* Test BLUE */
	printk(">>> Sending BLUE (00,00,FF) - what color do you see? <<<\n");
	for (int i = 0; i < STRIP_NUM_PIXELS; i++) pixels[i] = pure_blue;
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	k_sleep(K_MSEC(2000));

	/* Test WHITE */
	printk(">>> Sending WHITE (FF,FF,FF) - what color do you see? <<<\n");
	struct led_rgb white = {.r = 0xFF, .g = 0xFF, .b = 0xFF};
	for (int i = 0; i < STRIP_NUM_PIXELS; i++) pixels[i] = white;
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
	k_sleep(K_MSEC(2000));
#endif

	leds_off();
	LOG_INF("Color test complete - starting main loop");
	k_sleep(K_MSEC(500));
}

int main(void)
{
	struct sen0628_scan scan;
	struct sen0628_columns cols;
	int ret;

	LOG_INF("=== Proximity LED Visualization ===");
	LOG_INF("Green: safe (>%dmm), Yellow: caution (%d-%dmm), Red: danger (<%dmm)",
		DIST_CAUTION, DIST_DANGER, DIST_CAUTION, DIST_DANGER);

	/* Check devices */
	if (!device_is_ready(i2c_ext)) {
		LOG_ERR("External I2C not ready");
		return -1;
	}

	if (!device_is_ready(lidar)) {
		LOG_ERR("LIDAR not ready");
		return -1;
	}

#if HAS_LED_STRIP
	if (!device_is_ready(strip)) {
		LOG_ERR("LED strip not ready");
		return -1;
	}
	LOG_INF("LED strip ready (%d LEDs)", STRIP_NUM_PIXELS);
#endif

	/* Initialize Maqueen */
	ret = maqueen_init(i2c_ext);
	if (ret < 0) {
		LOG_ERR("Failed to init Maqueen: %d", ret);
		return -1;
	}

	/* Initialize LIDAR */
	ret = sen0628_init(lidar);
	if (ret < 0) {
		LOG_ERR("Failed to init LIDAR: %d", ret);
		return -1;
	}

	/* Set 4x4 mode for faster refresh */
	ret = sen0628_set_mode(lidar, SEN0628_MODE_4X4);
	if (ret < 0) {
		LOG_WRN("Failed to set 4x4 mode, using default");
	}

	LOG_INF("LIDAR mode: %dx%d", sen0628_get_mode(lidar), sen0628_get_mode(lidar));

	/* Startup animation */
	startup_animation();

	LOG_INF("Starting proximity detection loop...");
	printk("\n[===]=safe [###]=caution [XXX]=danger\n\n");

	/* Main loop */
	while (1) {
		/* Read LIDAR scan */
		ret = sen0628_read_scan(lidar, &scan);
		if (ret < 0 && ret != -scan.valid_count) {
			LOG_WRN("Scan error: %d", ret);
		}

		/* Get column statistics */
		ret = sen0628_get_columns(lidar, &cols);
		if (ret < 0) {
			LOG_WRN("Column stats error: %d", ret);
			continue;
		}

		/* Update LEDs */
		update_leds(&cols);

		/* Print status */
		print_status(&cols, scan.duration_ms);
	}

	return 0;
}
