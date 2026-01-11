/*
 * Speed Test for Maqueen Plus V3
 *
 * V3 uses real-time speed register (0x4C) instead of encoder counts.
 *
 * Button A: Execute next move in sequence
 *           (forward, left, right, backward, left, right, ...)
 * Button B: Check current wheel speed
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "maqueen.h"

LOG_MODULE_REGISTER(encoder_test, LOG_LEVEL_INF);

/* I2C device - external bus on edge connector (P19/P20) for Maqueen */
#define I2C_EXT_NODE DT_NODELABEL(i2c1)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_EXT_NODE);

/* Buttons */
static const struct gpio_dt_spec button_a = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button_b = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

/* Test parameters */
#define MOTOR_SPEED     100
#define TEST_DURATION_MS 1000
#define POLL_INTERVAL_MS 100

/* Movement types */
enum move_type {
	MOVE_FORWARD,
	MOVE_BACKWARD,
	MOVE_LEFT,
	MOVE_RIGHT,
	MOVE_COUNT
};

static const char *move_names[] = {
	"Forward", "Backward", "Turn Left", "Turn Right"
};

static void do_move(enum move_type type)
{
	uint8_t speed_l, speed_r;
	int samples = 0;
	int sum_l = 0, sum_r = 0;

	printk("\n=== %s ===\n", move_names[type]);

	/* Execute movement */
	switch (type) {
	case MOVE_FORWARD:
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_FORWARD, MOTOR_SPEED);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_FORWARD, MOTOR_SPEED);
		break;
	case MOVE_BACKWARD:
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_REVERSE, MOTOR_SPEED);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_REVERSE, MOTOR_SPEED);
		break;
	case MOVE_LEFT:
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_REVERSE, 80);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_FORWARD, 80);
		break;
	case MOVE_RIGHT:
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_FORWARD, 80);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_REVERSE, 80);
		break;
	default:
		return;
	}

	/* Sample speed during movement */
	for (int i = 0; i < 10; i++) {
		k_msleep(100);
		if (maqueen_speed_read(i2c_dev, &speed_l, &speed_r) == 0) {
			printk("  speed: L=%3d R=%3d (cm/s: L=%d R=%d)\n",
			       speed_l, speed_r, speed_l / 5, speed_r / 5);
			sum_l += speed_l;
			sum_r += speed_r;
			samples++;
		}
	}

	maqueen_stop_all(i2c_dev);

	if (samples > 0) {
		printk("Avg speed: L=%d R=%d (cm/s: L=%d R=%d)\n",
		       sum_l / samples, sum_r / samples,
		       sum_l / samples / 5, sum_r / samples / 5);
	}
}

/* Sequence: fwd, left, right, back, left, right, ... */
static enum move_type sequence[] = {
	MOVE_FORWARD, MOVE_LEFT, MOVE_RIGHT,
	MOVE_BACKWARD, MOVE_LEFT, MOVE_RIGHT
};
static int seq_idx = 0;

int main(void)
{
	int ret;

	printk("\n=============================\n");
	printk("  Maqueen Speed Test (V3)\n");
	printk("=============================\n");
	printk("Button A: Next move in sequence\n");
	printk("  (fwd, left, right, back, left, right)\n");
	printk("Button B: Check current speed\n\n");

	/* Check I2C device */
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -1;
	}

	/* Initialize Maqueen */
	ret = maqueen_init(i2c_dev);
	if (ret < 0) {
		LOG_ERR("Failed to init Maqueen: %d", ret);
		return ret;
	}

	/* Configure buttons */
	if (!gpio_is_ready_dt(&button_a) || !gpio_is_ready_dt(&button_b)) {
		LOG_ERR("Buttons not ready");
		return -1;
	}
	gpio_pin_configure_dt(&button_a, GPIO_INPUT);
	gpio_pin_configure_dt(&button_b, GPIO_INPUT);

	printk("Ready! Press button A to start sequence.\n\n");

	/* Main loop */
	while (1) {
		/* Check buttons (active low on micro:bit) */
		if (gpio_pin_get_dt(&button_a) == 0) {
			/* Execute next move in sequence */
			do_move(sequence[seq_idx]);
			seq_idx = (seq_idx + 1) % ARRAY_SIZE(sequence);
			k_msleep(500); /* Debounce */
		} else if (gpio_pin_get_dt(&button_b) == 0) {
			/* Show current speed */
			uint8_t speed_l, speed_r;
			printk("\n--- Speed check ---\n");
			if (maqueen_speed_read(i2c_dev, &speed_l, &speed_r) == 0) {
				printk("Speed: L=%d R=%d (cm/s: L=%d R=%d)\n",
				       speed_l, speed_r, speed_l / 5, speed_r / 5);
			}
			k_msleep(500); /* Debounce */
		}

		k_msleep(POLL_INTERVAL_MS);
	}

	return 0;
}
