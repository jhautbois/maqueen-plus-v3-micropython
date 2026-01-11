/*
 * DFRobot Maqueen Plus V3 Motor Controller Driver
 *
 * Simple driver for the STM8-based motor controller at I2C address 0x10
 */

#ifndef MAQUEEN_H_
#define MAQUEEN_H_

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>

/* I2C address */
#define MAQUEEN_I2C_ADDR        0x10

/* Motor registers */
#define MAQUEEN_REG_MOTOR_LEFT  0x00
#define MAQUEEN_REG_MOTOR_RIGHT 0x02
#define MAQUEEN_REG_ENCODER_L   0x04  /* Not used on V3 */
#define MAQUEEN_REG_ENCODER_R   0x06  /* Not used on V3 */
#define MAQUEEN_REG_SPEED       0x4C  /* Real-time speed (V3) */
#define MAQUEEN_REG_CLEAR_ENC   0x08
#define MAQUEEN_REG_HEADLIGHT_L 0x0B
#define MAQUEEN_REG_HEADLIGHT_R 0x0C
#define MAQUEEN_REG_VERSION     0x32

/* Motor IDs */
#define MAQUEEN_MOTOR_LEFT      0
#define MAQUEEN_MOTOR_RIGHT     1

/* Directions */
#define MAQUEEN_DIR_FORWARD     0
#define MAQUEEN_DIR_REVERSE     1

/* Headlight colors */
#define MAQUEEN_COLOR_OFF       0
#define MAQUEEN_COLOR_RED       1
#define MAQUEEN_COLOR_GREEN     2
#define MAQUEEN_COLOR_YELLOW    3
#define MAQUEEN_COLOR_BLUE      4
#define MAQUEEN_COLOR_PURPLE    5
#define MAQUEEN_COLOR_CYAN      6
#define MAQUEEN_COLOR_WHITE     7

/**
 * @brief Initialize Maqueen motor controller
 *
 * @param i2c_dev I2C device handle
 * @return 0 on success, negative errno on failure
 */
int maqueen_init(const struct device *i2c_dev);

/**
 * @brief Set motor speed and direction
 *
 * @param i2c_dev I2C device handle
 * @param motor MAQUEEN_MOTOR_LEFT or MAQUEEN_MOTOR_RIGHT
 * @param direction MAQUEEN_DIR_FORWARD or MAQUEEN_DIR_REVERSE
 * @param speed Speed 0-255
 * @return 0 on success, negative errno on failure
 */
int maqueen_motor_set(const struct device *i2c_dev,
                      uint8_t motor, uint8_t direction, uint8_t speed);

/**
 * @brief Stop a motor
 *
 * @param i2c_dev I2C device handle
 * @param motor MAQUEEN_MOTOR_LEFT or MAQUEEN_MOTOR_RIGHT
 * @return 0 on success, negative errno on failure
 */
int maqueen_motor_stop(const struct device *i2c_dev, uint8_t motor);

/**
 * @brief Stop all motors
 *
 * @param i2c_dev I2C device handle
 * @return 0 on success, negative errno on failure
 */
int maqueen_stop_all(const struct device *i2c_dev);

/**
 * @brief Set headlight color
 *
 * @param i2c_dev I2C device handle
 * @param headlight 0 = left, 1 = right
 * @param color Color index (MAQUEEN_COLOR_*)
 * @return 0 on success, negative errno on failure
 */
int maqueen_headlight_set(const struct device *i2c_dev,
                          uint8_t headlight, uint8_t color);

/**
 * @brief Read encoder value (V2 only - returns 0 on V3)
 *
 * @param i2c_dev I2C device handle
 * @param motor MAQUEEN_MOTOR_LEFT or MAQUEEN_MOTOR_RIGHT
 * @param count Pointer to store encoder count
 * @return 0 on success, negative errno on failure
 */
int maqueen_encoder_read(const struct device *i2c_dev,
                         uint8_t motor, int16_t *count);

/**
 * @brief Read real-time wheel speed (V3)
 *
 * @param i2c_dev I2C device handle
 * @param left Pointer to store left speed (raw value, divide by 5 for cm/s)
 * @param right Pointer to store right speed (raw value, divide by 5 for cm/s)
 * @return 0 on success, negative errno on failure
 */
int maqueen_speed_read(const struct device *i2c_dev,
                       uint8_t *left, uint8_t *right);

/**
 * @brief Clear encoder counters
 *
 * @param i2c_dev I2C device handle
 * @return 0 on success, negative errno on failure
 */
int maqueen_encoder_clear(const struct device *i2c_dev);

/**
 * @brief Get controller version
 *
 * @param i2c_dev I2C device handle
 * @param version Pointer to store version byte
 * @return 0 on success, negative errno on failure
 */
int maqueen_get_version(const struct device *i2c_dev, uint8_t *version);

#endif /* MAQUEEN_H_ */
