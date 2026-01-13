/**
 * @file pid_control.c
 * @brief Maqueen Plus V3 PID Control Implementation
 *
 * Hardware PID controller via STM8 motor controller (I2C 0x10).
 * Tested calibration: 1 unit = 1 cm (distances: 12cm, 25cm, 50cm, 100cm)
 * Angle precision: ±5° (tested: 90°, 180°)
 */

#include "pid_control.h"
#include "maqueen.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <errno.h>

LOG_MODULE_REGISTER(pid_control, LOG_LEVEL_INF);

/* PID Control Registers (STM8 @ 0x10) */
#define PID_REG_DIR_DISTANCE    0x40  /**< Distance direction (1=forward, 2=backward) */
#define PID_REG_DISTANCE_HIGH   0x41  /**< Distance high byte */
#define PID_REG_DISTANCE_LOW    0x42  /**< Distance low byte */
#define PID_REG_DIR_ANGLE       0x43  /**< Rotation direction (1=CW, 2=CCW) */
#define PID_REG_ANGLE           0x44  /**< Angle in degrees (0-255) */
#define PID_REG_SPEED_DISTANCE  0x55  /**< Speed for distance movement */
#define PID_REG_SPEED_ANGLE     0x56  /**< Speed for rotation */
#define PID_REG_MODE_CONTROL    0x3C  /**< Mode control register */
#define PID_REG_STATUS          0x57  /**< Status register */

/* Mode control values */
#define PID_MODE_START          0x06  /**< Start PID operation */
#define PID_MODE_STOP           0x10  /**< Stop PID operation */

/* Status values */
#define PID_STATUS_DONE         0x01  /**< Movement complete */

/* Default speed */
#define PID_DEFAULT_SPEED       2

/* I2C device pointer */
static const struct device *i2c_dev;

int pid_init(void)
{
	i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	LOG_INF("PID control initialized");
	return 0;
}

/**
 * @brief Write a single register value
 */
static int pid_write_reg(uint8_t reg, uint8_t value)
{
	uint8_t buf[2] = { reg, value };
	int ret;

	if (i2c_dev == NULL) {
		LOG_ERR("PID not initialized");
		return -EINVAL;
	}

	ret = i2c_write(i2c_dev, buf, sizeof(buf), MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to write register 0x%02x: %d", reg, ret);
	}
	return ret;
}

/**
 * @brief Read a single register value
 */
static int pid_read_reg(uint8_t reg, uint8_t *value)
{
	int ret;

	if (i2c_dev == NULL) {
		LOG_ERR("PID not initialized");
		return -EINVAL;
	}

	ret = i2c_write(i2c_dev, &reg, 1, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to set register pointer 0x%02x: %d", reg, ret);
		return ret;
	}

	/* Add small delay between write and read (V3 requires this) */
	k_msleep(5);

	ret = i2c_read(i2c_dev, value, 1, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		LOG_ERR("Failed to read register 0x%02x: %d", reg, ret);
	}
	return ret;
}

int pid_move_distance(uint16_t distance_cm, enum pid_direction direction, uint8_t speed)
{
	int ret;

	if (speed == 0 || speed > 10) {
		LOG_WRN("Invalid speed %u, using default %u", speed, PID_DEFAULT_SPEED);
		speed = PID_DEFAULT_SPEED;
	}

	LOG_INF("PID distance: %u cm, dir=%s, speed=%u",
		distance_cm,
		direction == PID_FORWARD ? "forward" : "backward",
		speed);

	/* Set direction */
	ret = pid_write_reg(PID_REG_DIR_DISTANCE, direction);
	if (ret < 0) {
		return ret;
	}

	/* Set speed */
	ret = pid_write_reg(PID_REG_SPEED_DISTANCE, speed);
	if (ret < 0) {
		return ret;
	}

	/* Set distance (high byte) */
	ret = pid_write_reg(PID_REG_DISTANCE_HIGH, (distance_cm >> 8) & 0xFF);
	if (ret < 0) {
		return ret;
	}

	/* Set distance (low byte) */
	ret = pid_write_reg(PID_REG_DISTANCE_LOW, distance_cm & 0xFF);
	if (ret < 0) {
		return ret;
	}

	/* Start PID operation */
	ret = pid_write_reg(PID_REG_MODE_CONTROL, PID_MODE_START);
	if (ret < 0) {
		return ret;
	}

	/* Wait for completion */
	ret = pid_wait_done(10000);  /* 10 second timeout */
	if (ret < 0) {
		LOG_ERR("PID distance operation failed or timed out");
		pid_stop();
		return ret;
	}

	LOG_INF("PID distance complete");
	return 0;
}

int pid_rotate_angle(uint8_t angle_deg, enum pid_rotation direction, uint8_t speed)
{
	int ret;

	if (speed == 0 || speed > 10) {
		LOG_WRN("Invalid speed %u, using default %u", speed, PID_DEFAULT_SPEED);
		speed = PID_DEFAULT_SPEED;
	}

	LOG_INF("PID rotate: %u°, dir=%s, speed=%u",
		angle_deg,
		direction == PID_ROTATE_CW ? "CW" : "CCW",
		speed);

	/* Set rotation direction */
	ret = pid_write_reg(PID_REG_DIR_ANGLE, direction);
	if (ret < 0) {
		return ret;
	}

	/* Set speed */
	ret = pid_write_reg(PID_REG_SPEED_ANGLE, speed);
	if (ret < 0) {
		return ret;
	}

	/* Set angle */
	ret = pid_write_reg(PID_REG_ANGLE, angle_deg);
	if (ret < 0) {
		return ret;
	}

	/* Start PID operation */
	ret = pid_write_reg(PID_REG_MODE_CONTROL, PID_MODE_START);
	if (ret < 0) {
		return ret;
	}

	/* Wait for completion */
	ret = pid_wait_done(5000);  /* 5 second timeout */
	if (ret < 0) {
		LOG_ERR("PID rotation operation failed or timed out");
		pid_stop();
		return ret;
	}

	LOG_INF("PID rotation complete");
	return 0;
}

int pid_stop(void)
{
	LOG_DBG("PID stop");
	return pid_write_reg(PID_REG_MODE_CONTROL, PID_MODE_STOP);
}

bool pid_is_done(void)
{
	uint8_t status = 0;
	int ret;

	ret = pid_read_reg(PID_REG_STATUS, &status);
	if (ret < 0) {
		return false;
	}

	return (status == PID_STATUS_DONE);
}

int pid_wait_done(uint32_t timeout_ms)
{
	uint32_t start_time = k_uptime_get_32();
	uint32_t elapsed;
	uint8_t speed_left, speed_right;
	int ret;
	bool movement_detected = false;

	/* Initial delay to let PID start */
	k_msleep(100);

	while (true) {
		/* Read wheel speeds (registers 0x4C and 0x4D) */
		ret = pid_read_reg(0x4C, &speed_left);
		if (ret < 0) {
			LOG_ERR("Failed to read left speed");
			return ret;
		}

		ret = pid_read_reg(0x4D, &speed_right);
		if (ret < 0) {
			LOG_ERR("Failed to read right speed");
			return ret;
		}

		LOG_DBG("PID speeds: L=%u R=%u (detected=%d)",
			speed_left, speed_right, movement_detected);

		/* Detect if movement has started */
		if (speed_left > 0 || speed_right > 0) {
			movement_detected = true;
		}

		/* Check if movement complete (speeds back to 0) */
		if (movement_detected && speed_left == 0 && speed_right == 0) {
			LOG_DBG("PID complete (speeds=0)");
			k_msleep(100);  /* Extra delay for stability */
			return 0;
		}

		/* Check timeout */
		if (timeout_ms > 0) {
			elapsed = k_uptime_get_32() - start_time;
			if (elapsed >= timeout_ms) {
				LOG_ERR("PID wait timeout after %u ms (L=%u, R=%u)",
					elapsed, speed_left, speed_right);
				return -ETIMEDOUT;
			}
		}

		/* Poll every 100ms */
		k_msleep(100);
	}
}
