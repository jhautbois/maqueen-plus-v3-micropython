/*
 * Odometry Test for Maqueen Plus V3
 *
 * V3 uses real-time speed register (0x4C) instead of encoder counts.
 * Odometry integrates speed to calculate position and heading.
 *
 * Button A: Execute next move in sequence
 *           (forward, left, right, backward, left, right, ...)
 * Button B: Show current position and heading
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
#include "odometry.h"

LOG_MODULE_REGISTER(odometry_test, LOG_LEVEL_INF);

/* I2C device - external bus on edge connector (P19/P20) for Maqueen */
#define I2C_EXT_NODE DT_NODELABEL(i2c1)
static const struct device *i2c_dev = DEVICE_DT_GET(I2C_EXT_NODE);

/* Buttons */
static const struct gpio_dt_spec button_a = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec button_b = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

/* Test parameters */
#define MOTOR_SPEED     100
#define TEST_DURATION_MS 1000
#define POLL_INTERVAL_MS 50
#define ODOM_UPDATE_MS  50

/* Odometry state */
static struct odometry odom;

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
	int32_t start_heading = odom.heading_mdeg;
	int32_t start_dist = odometry_get_distance_mm(&odom);
	int32_t start_x = odom.x;
	int32_t start_y = odom.y;

	printk("\n=== %s ===\n", move_names[type]);
	printk("Start: x=%d y=%d heading=%d deg\n",
	       odom.x, odom.y, odometry_get_heading_deg(&odom));

	/* Execute movement - set direction for odometry first! */
	switch (type) {
	case MOVE_FORWARD:
		odometry_set_direction(&odom, true, true);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_FORWARD, MOTOR_SPEED);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_FORWARD, MOTOR_SPEED);
		break;
	case MOVE_BACKWARD:
		odometry_set_direction(&odom, false, false);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_REVERSE, MOTOR_SPEED);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_REVERSE, MOTOR_SPEED);
		break;
	case MOVE_LEFT:
		odometry_set_direction(&odom, false, true);  /* Left reverse, right forward */
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_REVERSE, 80);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_FORWARD, 80);
		break;
	case MOVE_RIGHT:
		odometry_set_direction(&odom, true, false);  /* Left forward, right reverse */
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_LEFT, MAQUEEN_DIR_FORWARD, 80);
		maqueen_motor_set(i2c_dev, MAQUEEN_MOTOR_RIGHT, MAQUEEN_DIR_REVERSE, 80);
		break;
	default:
		return;
	}

	/* Update odometry during movement */
	for (int i = 0; i < TEST_DURATION_MS / ODOM_UPDATE_MS; i++) {
		k_msleep(ODOM_UPDATE_MS);
		odometry_update(&odom);
	}

	maqueen_stop_all(i2c_dev);
	odometry_set_direction(&odom, true, true);  /* Reset to forward */

	/* Final odometry update */
	k_msleep(ODOM_UPDATE_MS);
	odometry_update(&odom);

	/* Report results */
	int32_t delta_heading = odom.heading_mdeg - start_heading;
	int32_t delta_dist = odometry_get_distance_mm(&odom) - start_dist;

	printk("End:   x=%d y=%d heading=%d deg\n",
	       odom.x, odom.y, odometry_get_heading_deg(&odom));
	printk("Delta: dist=%dmm, rotation=%d deg\n",
	       delta_dist, delta_heading / 1000);
	printk("Movement: dx=%d dy=%d\n",
	       odom.x - start_x, odom.y - start_y);
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

	printk("\n==============================\n");
	printk("  Maqueen Odometry Test (V3)\n");
	printk("==============================\n");
	printk("Button A: Next move in sequence\n");
	printk("  (fwd, left, right, back, left, right)\n");
	printk("Button B: Show position / Reset odometry\n\n");

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

	/* Initialize odometry */
	odometry_init(&odom, i2c_dev);

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
			/* Show position and reset */
			printk("\n--- Current Position ---\n");
			printk("Position: x=%d mm, y=%d mm\n", odom.x, odom.y);
			printk("Heading:  %d deg\n", odometry_get_heading_deg(&odom));
			printk("Distance: L=%d mm, R=%d mm, total=%d mm\n",
			       odom.dist_left, odom.dist_right,
			       odometry_get_distance_mm(&odom));
			printk("\nResetting odometry...\n");
			odometry_reset(&odom);
			k_msleep(500); /* Debounce */
		}

		k_msleep(POLL_INTERVAL_MS);
	}

	return 0;
}
