/*
 * Odometry module for Maqueen Plus V3
 *
 * Integrates wheel speed readings to estimate position and heading.
 * Uses differential drive kinematics.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include <zephyr/device.h>
#include <stdint.h>

/* Maqueen Plus V3 physical parameters (mm) */
#define ODOM_WHEELBASE_MM       82      /* Distance between wheels */
#define ODOM_WHEEL_DIAMETER_MM  43      /* Wheel diameter */
#define ODOM_SPEED_DIVISOR      5       /* Raw speed / 5 = cm/s */

/* Odometry state */
struct odometry {
	/* Position in mm */
	int32_t x;
	int32_t y;

	/* Heading in millidegrees (0-360000) */
	int32_t heading_mdeg;

	/* Total distance traveled per wheel (mm) */
	int32_t dist_left;
	int32_t dist_right;

	/* Last update timestamp */
	int64_t last_update_ms;

	/* Motor direction signs (1=forward, -1=reverse) */
	int8_t dir_left;
	int8_t dir_right;

	/* I2C device for speed reading */
	const struct device *i2c_dev;
};

/**
 * @brief Set motor directions for odometry calculation
 *
 * Call this when motor direction changes. The speed register only gives
 * magnitude, so we need to track direction separately.
 *
 * @param odom Odometry state structure
 * @param left_forward true if left motor is forward
 * @param right_forward true if right motor is forward
 */
void odometry_set_direction(struct odometry *odom,
			    bool left_forward, bool right_forward);

/**
 * @brief Initialize odometry
 *
 * @param odom Odometry state structure
 * @param i2c_dev I2C device for Maqueen communication
 */
void odometry_init(struct odometry *odom, const struct device *i2c_dev);

/**
 * @brief Reset odometry to origin
 *
 * @param odom Odometry state structure
 */
void odometry_reset(struct odometry *odom);

/**
 * @brief Update odometry by reading current speed
 *
 * Call this regularly (e.g., every 50-100ms) for accurate integration.
 *
 * @param odom Odometry state structure
 * @return 0 on success, negative errno on failure
 */
int odometry_update(struct odometry *odom);

/**
 * @brief Get current heading in degrees
 *
 * @param odom Odometry state structure
 * @return Heading in degrees (0-360)
 */
static inline int32_t odometry_get_heading_deg(const struct odometry *odom)
{
	return odom->heading_mdeg / 1000;
}

/**
 * @brief Get current heading in millidegrees
 *
 * @param odom Odometry state structure
 * @return Heading in millidegrees (0-360000)
 */
static inline int32_t odometry_get_heading_mdeg(const struct odometry *odom)
{
	return odom->heading_mdeg;
}

/**
 * @brief Get total distance traveled (average of both wheels)
 *
 * @param odom Odometry state structure
 * @return Distance in mm
 */
static inline int32_t odometry_get_distance_mm(const struct odometry *odom)
{
	return (odom->dist_left + odom->dist_right) / 2;
}

/**
 * @brief Get X position
 *
 * @param odom Odometry state structure
 * @return X position in mm
 */
static inline int32_t odometry_get_x(const struct odometry *odom)
{
	return odom->x;
}

/**
 * @brief Get Y position
 *
 * @param odom Odometry state structure
 * @return Y position in mm
 */
static inline int32_t odometry_get_y(const struct odometry *odom)
{
	return odom->y;
}

/**
 * @brief Update odometry after PID distance movement
 *
 * Call this after a PID distance command completes. Updates position
 * based on commanded distance (±1cm precision from PID).
 *
 * @param odom Odometry state structure
 * @param distance_mm Distance moved (positive=forward, negative=backward)
 */
void odometry_update_pid_distance(struct odometry *odom, int32_t distance_mm);

/**
 * @brief Update odometry after PID rotation
 *
 * Call this after a PID rotation command completes. Updates heading
 * based on commanded angle (±5° precision from PID).
 *
 * @param odom Odometry state structure
 * @param angle_mdeg Angle rotated in millidegrees (positive=CW, negative=CCW)
 */
void odometry_update_pid_rotation(struct odometry *odom, int32_t angle_mdeg);

#endif /* ODOMETRY_H_ */
