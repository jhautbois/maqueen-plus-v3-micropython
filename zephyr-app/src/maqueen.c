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
	int ret;

	if (!device_is_ready(i2c_dev)) {
		LOG_ERR("I2C device not ready");
		return -ENODEV;
	}

	/* Try to read version to verify communication */
	ret = maqueen_get_version(i2c_dev, &version);
	if (ret < 0) {
		LOG_ERR("Failed to communicate with Maqueen at 0x%02x", MAQUEEN_I2C_ADDR);
		return ret;
	}

	LOG_INF("Maqueen Plus V3 initialized (version: 0x%02x)", version);

	/* Stop all motors on init */
	maqueen_stop_all(i2c_dev);

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
	uint8_t reg;
	uint8_t buf[2];
	int ret;

	if (motor == MAQUEEN_MOTOR_LEFT) {
		reg = MAQUEEN_REG_ENCODER_L;
	} else if (motor == MAQUEEN_MOTOR_RIGHT) {
		reg = MAQUEEN_REG_ENCODER_R;
	} else {
		return -EINVAL;
	}

	ret = i2c_write_read(i2c_dev, MAQUEEN_I2C_ADDR, &reg, 1, buf, 2);
	if (ret < 0) {
		return ret;
	}

	/* Little-endian 16-bit value */
	*count = (int16_t)(buf[0] | (buf[1] << 8));

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
