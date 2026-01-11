/*
 * DFRobot Maqueen Plus V3 Motor Controller Driver
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "maqueen.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(maqueen, LOG_LEVEL_INF);

int maqueen_init(const struct device *i2c_dev)
{
	uint8_t version;
	uint8_t reset_cmd[2] = { 0x49, 0x01 };
	int ret;
	int retries = 10;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Send reset command (from MakeCode library) */
	ret = i2c_write(i2c_dev, reset_cmd, sizeof(reset_cmd), MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		LOG_WRN("Reset command failed: %d (continuing anyway)", ret);
	} else {
		LOG_DBG("Reset command sent");
	}

	/* Wait for STM8 to initialize */
	k_msleep(100);

	/* Poll version register until ready */
	while (retries-- > 0) {
		ret = maqueen_get_version(i2c_dev, &version);
		if (ret == 0 && version != 0) {
			break;
		}
		k_msleep(50);
	}

	if (ret < 0) {
		LOG_ERR("Failed to communicate with Maqueen at 0x%02x", MAQUEEN_I2C_ADDR);
		return ret;
	}

	LOG_INF("Maqueen Plus V3 initialized (version: 0x%02x)", version);

	/* Stop all motors on init */
	maqueen_stop_all(i2c_dev);

	/* Clear encoder counters */
	maqueen_encoder_clear(i2c_dev);

	return 0;
}

int maqueen_motor_set(const struct device *i2c_dev,
                      uint8_t motor, uint8_t direction, uint8_t speed)
{
	uint8_t reg;
	uint8_t buf[3];

	if (motor == MAQUEEN_MOTOR_LEFT) {
		reg = MAQUEEN_REG_MOTOR_LEFT;
	} else if (motor == MAQUEEN_MOTOR_RIGHT) {
		reg = MAQUEEN_REG_MOTOR_RIGHT;
	} else {
		return -EINVAL;
	}

	buf[0] = reg;
	buf[1] = direction ? MAQUEEN_DIR_REVERSE : MAQUEEN_DIR_FORWARD;
	buf[2] = speed;

	return i2c_write(i2c_dev, buf, sizeof(buf), MAQUEEN_I2C_ADDR);
}

int maqueen_motor_stop(const struct device *i2c_dev, uint8_t motor)
{
	return maqueen_motor_set(i2c_dev, motor, MAQUEEN_DIR_FORWARD, 0);
}

int maqueen_stop_all(const struct device *i2c_dev)
{
	int ret;

	ret = maqueen_motor_stop(i2c_dev, MAQUEEN_MOTOR_LEFT);
	if (ret < 0) {
		return ret;
	}

	return maqueen_motor_stop(i2c_dev, MAQUEEN_MOTOR_RIGHT);
}

int maqueen_headlight_set(const struct device *i2c_dev,
                          uint8_t headlight, uint8_t color)
{
	uint8_t buf[2];

	if (headlight > 1 || color > MAQUEEN_COLOR_WHITE) {
		return -EINVAL;
	}

	buf[0] = (headlight == 0) ? MAQUEEN_REG_HEADLIGHT_L : MAQUEEN_REG_HEADLIGHT_R;
	buf[1] = color;

	return i2c_write(i2c_dev, buf, sizeof(buf), MAQUEEN_I2C_ADDR);
}

int maqueen_encoder_read(const struct device *i2c_dev,
                         uint8_t motor, int16_t *count)
{
	uint8_t reg = MAQUEEN_REG_ENCODER_L;  /* Read both encoders at once */
	uint8_t buf[4];
	int ret;

	/*
	 * Read 4 bytes from register 0x04 (like Arduino library):
	 * buf[0:1] = left encoder (big-endian)
	 * buf[2:3] = right encoder (big-endian)
	 */
	ret = i2c_write(i2c_dev, &reg, 1, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_read(i2c_dev, buf, 4, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		return ret;
	}

	/* Big-endian 16-bit values (as per Arduino library) */
	if (motor == MAQUEEN_MOTOR_LEFT) {
		*count = (int16_t)((buf[0] << 8) | buf[1]);
	} else if (motor == MAQUEEN_MOTOR_RIGHT) {
		*count = (int16_t)((buf[2] << 8) | buf[3]);
	} else {
		return -EINVAL;
	}

	return 0;
}

int maqueen_speed_read(const struct device *i2c_dev,
                       uint8_t *left, uint8_t *right)
{
	uint8_t reg = MAQUEEN_REG_SPEED;
	uint8_t buf[2];
	int ret;

	ret = i2c_write(i2c_dev, &reg, 1, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		return ret;
	}

	ret = i2c_read(i2c_dev, buf, 2, MAQUEEN_I2C_ADDR);
	if (ret < 0) {
		return ret;
	}

	*left = buf[0];
	*right = buf[1];

	return 0;
}

int maqueen_encoder_clear(const struct device *i2c_dev)
{
	uint8_t buf[2] = { MAQUEEN_REG_CLEAR_ENC, 0x01 };

	return i2c_write(i2c_dev, buf, sizeof(buf), MAQUEEN_I2C_ADDR);
}

int maqueen_get_version(const struct device *i2c_dev, uint8_t *version)
{
	uint8_t reg = MAQUEEN_REG_VERSION;

	return i2c_write_read(i2c_dev, MAQUEEN_I2C_ADDR, &reg, 1, version, 1);
}
