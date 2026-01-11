/*
 * Maqueen Plus V3 Test Application for Zephyr RTOS
 *
 * Tests:
 *   1. WS2812 underglow LEDs
 *   2. Motor control
 *   3. Headlights
 *   4. Encoders
 *   5. Accelerometer
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Set to 1 for USB-tethered testing (shorter/slower motor movements) */
#define TETHERED_MODE 0

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <string.h>

#include "maqueen.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Device handles */
#define I2C_INT_NODE DT_NODELABEL(i2c0)   /* Internal I2C - on-board sensors */
#define I2C_EXT_NODE DT_NODELABEL(i2c1)   /* External I2C - edge connector (Maqueen) */
#define STRIP_NODE DT_ALIAS(led_strip)
#define ACCEL_NODE DT_NODELABEL(lsm303agr_accel)

#if DT_NODE_HAS_STATUS(STRIP_NODE, okay)
#define HAS_LED_STRIP 1
#define STRIP_NUM_PIXELS DT_PROP(STRIP_NODE, chain_length)
static const struct device *strip = DEVICE_DT_GET(STRIP_NODE);
static struct led_rgb pixels[STRIP_NUM_PIXELS];
#else
#define HAS_LED_STRIP 0
#endif

static const struct device *i2c_int = DEVICE_DT_GET(I2C_INT_NODE);  /* Internal */
static const struct device *i2c_ext = DEVICE_DT_GET(I2C_EXT_NODE);  /* External - Maqueen */

#if DT_NODE_HAS_STATUS(ACCEL_NODE, okay)
#define HAS_ACCEL 1
static const struct device *accel = DEVICE_DT_GET(ACCEL_NODE);
#else
#define HAS_ACCEL 0
#endif

/* RGB helper */
#define RGB(_r, _g, _b) { .r = (_r), .g = (_g), .b = (_b) }

/* Underglow colors */
static const struct led_rgb underglow_colors[] = {
	RGB(0x20, 0x00, 0x00),  /* Red */
	RGB(0x00, 0x20, 0x00),  /* Green */
	RGB(0x00, 0x00, 0x20),  /* Blue */
	RGB(0x20, 0x20, 0x00),  /* Yellow */
	RGB(0x20, 0x00, 0x20),  /* Magenta */
	RGB(0x00, 0x20, 0x20),  /* Cyan */
	RGB(0x20, 0x20, 0x20),  /* White */
};

/**
 * Test 1: Cycle through underglow LED colors
 */
static int test_underglow(void)
{
#if HAS_LED_STRIP
	int ret;

	LOG_INF("=== Test 1: Underglow LEDs ===");

	if (!device_is_ready(strip)) {
		LOG_ERR("LED strip not ready");
		return -ENODEV;
	}

	LOG_INF("Cycling through %d colors on %d LEDs",
		ARRAY_SIZE(underglow_colors), STRIP_NUM_PIXELS);

	for (size_t c = 0; c < ARRAY_SIZE(underglow_colors); c++) {
		/* Set all LEDs to same color */
		for (size_t i = 0; i < STRIP_NUM_PIXELS; i++) {
			memcpy(&pixels[i], &underglow_colors[c], sizeof(struct led_rgb));
		}

		ret = led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		if (ret) {
			LOG_ERR("Failed to update LED strip: %d", ret);
			return ret;
		}

		LOG_INF("Color %d: R=%d G=%d B=%d", c,
			underglow_colors[c].r,
			underglow_colors[c].g,
			underglow_colors[c].b);

		k_sleep(K_MSEC(500));
	}

	/* Turn off */
	memset(pixels, 0, sizeof(pixels));
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	LOG_INF("Underglow test complete");
	return 0;
#else
	LOG_WRN("LED strip not configured");
	return -ENOTSUP;
#endif
}

/**
 * Test 2: Motor control - brief forward movement
 *
 * TETHERED_MODE=1: Very brief wheel twitch (safe with USB cable)
 * TETHERED_MODE=0: Full 1-second movements (untethered only!)
 */
static int test_motors(void)
{
	int ret;

#if TETHERED_MODE
	const uint8_t speed = 60;       /* Slow speed */
	const int duration_ms = 200;    /* Very brief - just a twitch */
#else
	const uint8_t speed = 100;      /* Normal speed */
	const int duration_ms = 1000;   /* 1 second */
#endif

	LOG_INF("=== Test 2: Motors ===");
#if TETHERED_MODE
	LOG_INF("TETHERED MODE: Brief motor twitch only");
#else
	LOG_WRN("UNTETHERED MODE: Robot will move! Disconnect USB first!");
	k_sleep(K_SECONDS(3));  /* Give time to read warning */
#endif

	ret = maqueen_init(i2c_ext);
	if (ret < 0) {
		LOG_ERR("Failed to initialize Maqueen: %d", ret);
		return ret;
	}

	/* Clear encoders */
	maqueen_encoder_clear(i2c_ext);

	LOG_INF("Moving forward (speed %d) for %d ms...", speed, duration_ms);
	maqueen_motor_set(i2c_ext, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_FORWARD, speed);
	maqueen_motor_set(i2c_ext, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_FORWARD, speed);
	k_sleep(K_MSEC(duration_ms));

	/* Stop */
	maqueen_stop_all(i2c_ext);
	LOG_INF("Stopped");

	/* Read encoders */
	int16_t left_enc, right_enc;
	maqueen_encoder_read(i2c_ext, MAQUEEN_MOTOR_LEFT, &left_enc);
	maqueen_encoder_read(i2c_ext, MAQUEEN_MOTOR_RIGHT, &right_enc);
	LOG_INF("Encoder counts: Left=%d, Right=%d", left_enc, right_enc);

	k_sleep(K_MSEC(300));

	LOG_INF("Moving backward (speed %d) for %d ms...", speed, duration_ms);
	maqueen_motor_set(i2c_ext, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_REVERSE, speed);
	maqueen_motor_set(i2c_ext, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_REVERSE, speed);
	k_sleep(K_MSEC(duration_ms));

	maqueen_stop_all(i2c_ext);
	LOG_INF("Stopped");

	/* Read encoders again */
	maqueen_encoder_read(i2c_ext, MAQUEEN_MOTOR_LEFT, &left_enc);
	maqueen_encoder_read(i2c_ext, MAQUEEN_MOTOR_RIGHT, &right_enc);
	LOG_INF("Encoder counts: Left=%d, Right=%d", left_enc, right_enc);

	LOG_INF("Motor test complete");
	return 0;
}

/**
 * Test 3: Headlights
 */
static int test_headlights(void)
{
	int ret;

	LOG_INF("=== Test 3: Headlights ===");

	static const uint8_t colors[] = {
		MAQUEEN_COLOR_RED,
		MAQUEEN_COLOR_GREEN,
		MAQUEEN_COLOR_BLUE,
		MAQUEEN_COLOR_WHITE,
		MAQUEEN_COLOR_OFF,
	};

	for (size_t i = 0; i < ARRAY_SIZE(colors); i++) {
		LOG_INF("Headlight color: %d", colors[i]);

		ret = maqueen_headlight_set(i2c_ext, 0, colors[i]);
		if (ret < 0) {
			LOG_ERR("Failed to set left headlight: %d", ret);
		}

		ret = maqueen_headlight_set(i2c_ext, 1, colors[i]);
		if (ret < 0) {
			LOG_ERR("Failed to set right headlight: %d", ret);
		}

		k_sleep(K_MSEC(500));
	}

	LOG_INF("Headlight test complete");
	return 0;
}

/**
 * Test 4: Accelerometer
 */
static int test_accelerometer(void)
{
#if HAS_ACCEL
	struct sensor_value accel_vals[3];
	int ret;

	LOG_INF("=== Test 4: Accelerometer ===");

	if (!device_is_ready(accel)) {
		LOG_ERR("Accelerometer not ready");
		return -ENODEV;
	}

	LOG_INF("Reading accelerometer 5 times...");

	for (int i = 0; i < 5; i++) {
		ret = sensor_sample_fetch(accel);
		if (ret < 0) {
			LOG_ERR("Failed to fetch sample: %d", ret);
			continue;
		}

		ret = sensor_channel_get(accel, SENSOR_CHAN_ACCEL_XYZ, accel_vals);
		if (ret < 0) {
			LOG_ERR("Failed to get channel: %d", ret);
			continue;
		}

		LOG_INF("Accel X=%d.%06d Y=%d.%06d Z=%d.%06d m/s^2",
			accel_vals[0].val1, accel_vals[0].val2,
			accel_vals[1].val1, accel_vals[1].val2,
			accel_vals[2].val1, accel_vals[2].val2);

		k_sleep(K_MSEC(500));
	}

	LOG_INF("Accelerometer test complete");
	return 0;
#else
	LOG_WRN("Accelerometer not configured");
	return -ENOTSUP;
#endif
}

/**
 * Demo: Rainbow underglow animation
 */
static int demo_rainbow(void)
{
#if HAS_LED_STRIP
	LOG_INF("=== Demo: Rainbow Animation ===");

	if (!device_is_ready(strip)) {
		return -ENODEV;
	}

	/* Simple rotating rainbow */
	for (int cycle = 0; cycle < 20; cycle++) {
		for (size_t i = 0; i < STRIP_NUM_PIXELS; i++) {
			int hue = (cycle * 30 + i * 90) % 360;

			/* Simple HSV to RGB (approximate) */
			int section = hue / 60;
			int f = (hue % 60) * 255 / 60;

			switch (section) {
			case 0:
				pixels[i].r = 0x20;
				pixels[i].g = f >> 3;
				pixels[i].b = 0;
				break;
			case 1:
				pixels[i].r = (255 - f) >> 3;
				pixels[i].g = 0x20;
				pixels[i].b = 0;
				break;
			case 2:
				pixels[i].r = 0;
				pixels[i].g = 0x20;
				pixels[i].b = f >> 3;
				break;
			case 3:
				pixels[i].r = 0;
				pixels[i].g = (255 - f) >> 3;
				pixels[i].b = 0x20;
				break;
			case 4:
				pixels[i].r = f >> 3;
				pixels[i].g = 0;
				pixels[i].b = 0x20;
				break;
			case 5:
				pixels[i].r = 0x20;
				pixels[i].g = 0;
				pixels[i].b = (255 - f) >> 3;
				break;
			}
		}

		led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);
		k_sleep(K_MSEC(100));
	}

	/* Turn off */
	memset(pixels, 0, sizeof(pixels));
	led_strip_update_rgb(strip, pixels, STRIP_NUM_PIXELS);

	LOG_INF("Rainbow demo complete");
	return 0;
#else
	return -ENOTSUP;
#endif
}

/**
 * Scan an I2C bus for devices
 */
static void i2c_scan_bus(const struct device *i2c, const char *name)
{
	uint8_t dummy = 0;
	int found = 0;

	LOG_INF("Scanning %s (0x08-0x77)...", name);

	for (uint8_t addr = 0x08; addr < 0x78; addr++) {
		struct i2c_msg msg = {
			.buf = &dummy,
			.len = 0,
			.flags = I2C_MSG_WRITE | I2C_MSG_STOP,
		};
		int ret = i2c_transfer(i2c, &msg, 1, addr);
		if (ret == 0) {
			LOG_INF("  0x%02x", addr);
			found++;
		}
	}

	LOG_INF("  Found %d devices", found);
}

/**
 * Scan both I2C buses
 */
static void i2c_scan(void)
{
	LOG_INF("=== I2C Bus Scan ===");

	/* Scan internal bus (on-board sensors) */
	i2c_scan_bus(i2c_int, "i2c0 (internal)");

	/* Scan external bus (edge connector - Maqueen) */
	i2c_scan_bus(i2c_ext, "i2c1 (external/edge)");

	/* Direct test of Maqueen on external bus */
	LOG_INF("Direct test of 0x10 on external bus:");
	uint8_t reg = 0x32;
	uint8_t val;
	int ret = i2c_write_read(i2c_ext, 0x10, &reg, 1, &val, 1);
	LOG_INF("  ret=%d, val=0x%02x", ret, val);
}

int main(void)
{
	LOG_INF("========================================");
	LOG_INF("Maqueen Plus V3 Zephyr Test Application");
	LOG_INF("========================================");

	/* Check I2C buses */
	if (!device_is_ready(i2c_int)) {
		LOG_ERR("Internal I2C (i2c0) not ready!");
		return 0;
	}
	if (!device_is_ready(i2c_ext)) {
		LOG_ERR("External I2C (i2c1) not ready!");
		return 0;
	}

	LOG_INF("Both I2C buses ready, starting tests...\n");

	/* Scan I2C buses first */
	i2c_scan();

	k_sleep(K_SECONDS(2));  /* Give user time to observe */

	/* Run tests */
	test_underglow();
	k_sleep(K_SECONDS(1));

	test_motors();
	k_sleep(K_SECONDS(1));

	test_headlights();
	k_sleep(K_SECONDS(1));

	test_accelerometer();
	k_sleep(K_SECONDS(1));

	demo_rainbow();

	LOG_INF("========================================");
	LOG_INF("All tests complete!");
	LOG_INF("========================================");

	/* Keep running - could add shell commands here */
	while (1) {
		k_sleep(K_SECONDS(10));
	}

	return 0;
}
