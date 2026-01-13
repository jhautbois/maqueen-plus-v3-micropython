/**
 * @file pid_control.h
 * @brief Maqueen Plus V3 PID Control Interface
 *
 * Hardware PID controller via STM8 (I2C 0x10)
 * Provides precise distance (±1cm) and angle (±5°) control.
 */

#ifndef PID_CONTROL_H
#define PID_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief PID movement direction for distance control
 */
enum pid_direction {
	PID_FORWARD = 1,   /**< Move forward */
	PID_BACKWARD = 2   /**< Move backward */
};

/**
 * @brief PID rotation direction
 */
enum pid_rotation {
	PID_ROTATE_CW = 1,   /**< Rotate clockwise (positive angle) */
	PID_ROTATE_CCW = 2   /**< Rotate counter-clockwise (negative angle) */
};

/**
 * @brief Initialize PID control system
 * @return 0 on success, negative errno on failure
 */
int pid_init(void);

/**
 * @brief Move a specific distance using PID control
 *
 * @param distance_cm Distance in centimeters (1-65535)
 * @param direction Forward or backward
 * @param speed Speed parameter (1-10, default=2)
 * @return 0 on success, negative errno on failure
 *
 * @note Blocks until movement is complete (status register = 0x01)
 * @note Calibration: 1 unit = 1 cm (tested with 12cm, 25cm, 50cm, 100cm)
 */
int pid_move_distance(uint16_t distance_cm, enum pid_direction direction, uint8_t speed);

/**
 * @brief Rotate a specific angle using PID control
 *
 * @param angle_deg Angle in degrees (0-255)
 * @param direction Clockwise or counter-clockwise
 * @param speed Speed parameter (1-10, default=2)
 * @return 0 on success, negative errno on failure
 *
 * @note Blocks until movement is complete (status register = 0x01)
 * @note Calibration: ±5° precision (tested 90° and 180°)
 */
int pid_rotate_angle(uint8_t angle_deg, enum pid_rotation direction, uint8_t speed);

/**
 * @brief Stop current PID operation
 * @return 0 on success, negative errno on failure
 */
int pid_stop(void);

/**
 * @brief Check if PID movement is complete
 * @return true if done (status=0x01), false otherwise
 */
bool pid_is_done(void);

/**
 * @brief Wait for PID movement to complete
 * @param timeout_ms Maximum wait time in milliseconds (0 = infinite)
 * @return 0 on success, -ETIMEDOUT on timeout
 */
int pid_wait_done(uint32_t timeout_ms);

/**
 * @brief Convenience function: move forward in cm
 */
static inline int pid_forward(uint16_t cm) {
	return pid_move_distance(cm, PID_FORWARD, 2);
}

/**
 * @brief Convenience function: move backward in cm
 */
static inline int pid_backward(uint16_t cm) {
	return pid_move_distance(cm, PID_BACKWARD, 2);
}

/**
 * @brief Convenience function: rotate clockwise in degrees
 */
static inline int pid_rotate_cw(uint8_t degrees) {
	return pid_rotate_angle(degrees, PID_ROTATE_CW, 2);
}

/**
 * @brief Convenience function: rotate counter-clockwise in degrees
 */
static inline int pid_rotate_ccw(uint8_t degrees) {
	return pid_rotate_angle(degrees, PID_ROTATE_CCW, 2);
}

#endif /* PID_CONTROL_H */
