/*
 * PID Test Application
 *
 * Button A: Test carré 50cm x 50cm
 * Button B: Test ligne droite 100cm
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/display.h>
#include <zephyr/logging/log.h>
#include <math.h>

#include "maqueen.h"
#include "pid_control.h"
#include "odometry.h"

LOG_MODULE_REGISTER(pid_test, LOG_LEVEL_INF);

/* Buttons */
#define BTN_A_NODE DT_ALIAS(sw0)
#define BTN_B_NODE DT_ALIAS(sw1)

static const struct gpio_dt_spec btn_a = GPIO_DT_SPEC_GET(BTN_A_NODE, gpios);
static const struct gpio_dt_spec btn_b = GPIO_DT_SPEC_GET(BTN_B_NODE, gpios);

/* Display */
static const struct device *display_dev;

/* Odometry */
static struct odometry odom;

/* LED icons */
static const uint8_t icon_ready[] = {
	0b00100,
	0b01110,
	0b10101,
	0b00100,
	0b00100,
};

static const uint8_t icon_done[] = {
	0b00000,
	0b00001,
	0b00010,
	0b10100,
	0b01000,
};

static const uint8_t icon_error[] = {
	0b10001,
	0b01010,
	0b00100,
	0b01010,
	0b10001,
};

static void display_icon(const uint8_t *icon)
{
	struct display_buffer_descriptor desc = {
		.buf_size = 5,
		.width = 5,
		.height = 5,
		.pitch = 8,
	};
	display_write(display_dev, 0, 0, &desc, (void *)icon);
}

static void test_square_50cm(void)
{
	int ret;

	LOG_INF("=== Test Carré 50cm x 50cm ===");
	display_icon(icon_ready);
	
	odometry_reset(&odom);

	for (int i = 0; i < 4; i++) {
		LOG_INF("Côté %d: avancer 50cm", i + 1);
		
		ret = pid_forward(50);
		if (ret < 0) {
			LOG_ERR("PID forward failed: %d", ret);
			display_icon(icon_error);
			return;
		}
		odometry_update_pid_distance(&odom, 500);
		
		LOG_INF("Position: (%d, %d) mm", odom.x, odom.y);
		k_sleep(K_MSEC(500));

		LOG_INF("Tourner 90° CW");
		
		ret = pid_rotate_cw(90);
		if (ret < 0) {
			LOG_ERR("PID rotate failed: %d", ret);
			display_icon(icon_error);
			return;
		}
		odometry_update_pid_rotation(&odom, 90000);
		
		LOG_INF("Heading: %d°", odometry_get_heading_deg(&odom));
		k_sleep(K_MSEC(500));
	}

	LOG_INF("=== Carré terminé ===");
	LOG_INF("Position finale: (%d, %d) mm", odom.x, odom.y);
	LOG_INF("Heading final: %d°", odometry_get_heading_deg(&odom));
	LOG_INF("Erreur de fermeture: %d mm", 
		(int)sqrtf(odom.x * odom.x + odom.y * odom.y));
	
	display_icon(icon_done);
}

static void test_straight_100cm(void)
{
	int ret;

	LOG_INF("=== Test Ligne Droite 100cm ===");
	display_icon(icon_ready);
	
	odometry_reset(&odom);

	LOG_INF("Avancer 100cm");
	
	ret = pid_forward(100);
	if (ret < 0) {
		LOG_ERR("PID forward failed: %d", ret);
		display_icon(icon_error);
		return;
	}
	odometry_update_pid_distance(&odom, 1000);
	
	LOG_INF("=== Ligne droite terminée ===");
	LOG_INF("Position finale: (%d, %d) mm", odom.x, odom.y);
	LOG_INF("Distance totale: %d mm", odometry_get_distance_mm(&odom));
	
	display_icon(icon_done);
}

int main(void)
{
	const struct device *i2c_dev;
	int ret;

	LOG_INF("PID Test Application starting...");

	/* Get I2C device */
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));
	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -1;
	}

	/* Initialize Maqueen */
	ret = maqueen_init(i2c_dev);
	if (ret < 0) {
		LOG_ERR("Maqueen init failed: %d", ret);
		return ret;
	}

	/* Initialize PID */
	ret = pid_init();
	if (ret < 0) {
		LOG_ERR("PID init failed: %d", ret);
		return ret;
	}

	/* Initialize odometry */
	odometry_init(&odom, i2c_dev);

	/* Initialize buttons */
	if (!gpio_is_ready_dt(&btn_a) || !gpio_is_ready_dt(&btn_b)) {
		LOG_ERR("Buttons not ready");
		return -1;
	}

	ret = gpio_pin_configure_dt(&btn_a, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure button A: %d", ret);
		return ret;
	}

	ret = gpio_pin_configure_dt(&btn_b, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure button B: %d", ret);
		return ret;
	}

	/* Initialize display */
	display_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_display));
	if (!device_is_ready(display_dev)) {
		LOG_ERR("Display not ready");
		return -1;
	}
	display_blanking_off(display_dev);

	LOG_INF("Ready! Press button A or B");
	display_icon(icon_ready);

	/* Main loop */
	/* Initialize last state to avoid false edge detection on startup */
	bool btn_a_last = (gpio_pin_get_dt(&btn_a) == 0);
	bool btn_b_last = (gpio_pin_get_dt(&btn_b) == 0);

	while (true) {
		bool btn_a_now = (gpio_pin_get_dt(&btn_a) == 0);  /* Active low */
		bool btn_b_now = (gpio_pin_get_dt(&btn_b) == 0);

		/* Button A pressed (rising edge) */
		if (btn_a_now && !btn_a_last) {
			test_square_50cm();
			k_sleep(K_MSEC(2000));
			display_icon(icon_ready);
		}

		/* Button B pressed (rising edge) */
		if (btn_b_now && !btn_b_last) {
			test_straight_100cm();
			k_sleep(K_MSEC(2000));
			display_icon(icon_ready);
		}

		btn_a_last = btn_a_now;
		btn_b_last = btn_b_now;

		k_sleep(K_MSEC(50));
	}

	return 0;
}
