/*
 * Odometry module using math.h sin/cos (for benchmark comparison)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "odometry.h"
#include "maqueen.h"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(odometry, LOG_LEVEL_INF);

/* Convert millidegrees to radians */
#define MDEG_TO_RAD(mdeg) ((mdeg) * 3.14159265f / 180000.0f)

void odometry_init(struct odometry *odom, const struct device *i2c_dev)
{
	odom->i2c_dev = i2c_dev;
	odometry_reset(odom);
	LOG_INF("Odometry initialized (wheelbase=%dmm, using math.h)", ODOM_WHEELBASE_MM);
}

void odometry_reset(struct odometry *odom)
{
	odom->x = 0;
	odom->y = 0;
	odom->heading_mdeg = 0;
	odom->dist_left = 0;
	odom->dist_right = 0;
	odom->dir_left = 1;
	odom->dir_right = 1;
	odom->last_update_ms = k_uptime_get();
	LOG_INF("Odometry reset");
}

void odometry_set_direction(struct odometry *odom,
			    bool left_forward, bool right_forward)
{
	odom->dir_left = left_forward ? 1 : -1;
	odom->dir_right = right_forward ? 1 : -1;
}

int odometry_update(struct odometry *odom)
{
	uint8_t speed_left, speed_right;
	int64_t now_ms;
	int32_t dt_ms;
	int32_t v_left_mm_s, v_right_mm_s;
	int32_t d_left, d_right;
	int32_t d_center, d_theta_mdeg;
	int ret;

	/* Read current speed */
	ret = maqueen_speed_read(odom->i2c_dev, &speed_left, &speed_right);
	if (ret < 0) {
		return ret;
	}

	/* Calculate time delta */
	now_ms = k_uptime_get();
	dt_ms = (int32_t)(now_ms - odom->last_update_ms);
	odom->last_update_ms = now_ms;

	if (dt_ms <= 0 || dt_ms > 1000) {
		return 0;
	}

	/* Convert speed to mm/s */
	v_left_mm_s = (speed_left * 10 * odom->dir_left) / ODOM_SPEED_DIVISOR;
	v_right_mm_s = (speed_right * 10 * odom->dir_right) / ODOM_SPEED_DIVISOR;

	/* Calculate distance traveled */
	d_left = (v_left_mm_s * dt_ms) / 1000;
	d_right = (v_right_mm_s * dt_ms) / 1000;

	/* Accumulate total distance */
	odom->dist_left += d_left;
	odom->dist_right += d_right;

	/* Differential drive kinematics */
	d_center = (d_left + d_right) / 2;
	d_theta_mdeg = ((d_right - d_left) * 57296) / ODOM_WHEELBASE_MM;

	/* Update heading */
	odom->heading_mdeg += d_theta_mdeg;

	/* Normalize heading */
	while (odom->heading_mdeg < 0) {
		odom->heading_mdeg += 360000;
	}
	while (odom->heading_mdeg >= 360000) {
		odom->heading_mdeg -= 360000;
	}

	/* Update position using math.h sin/cos */
	int32_t mid_heading = odom->heading_mdeg - d_theta_mdeg / 2;
	float heading_rad = MDEG_TO_RAD(mid_heading);

	odom->x += (int32_t)(d_center * cosf(heading_rad));
	odom->y += (int32_t)(d_center * sinf(heading_rad));

	return 0;
}
