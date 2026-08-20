/*
 * motors.c
 *
 *  Created on: Jul 19, 2026
 *  Author: Thomas Bourgeois
 *
 *  This software is written to control either an Afro30A or BlueRobotics Basic ESC.
 *
 *	Refer to STM32f446xx datasheet for TIM and TIM_CHANNEL.
 *
 *	Note: Static - means the function is contained only within this file.
 *		  Inline - removes overhead (only works for small functions).
 */

#include "motors.h"
#include "main.h"
#include <math.h>
#include "my_utils.h"

/*
 * @brief Converts throttle percentage to pulse width between 1100us and 1900us.
 * @param throttlePercent: throttle input, expected range -100% to +100%.
 *        Values outside this range are clamped before mapping, so a bad
 *        input saturates at min/max pulse width instead of being
 *        extrapolated by map() into an out-of-range compare value.
 * @retval Pulse width in microseconds.
 */
static inline uint32_t throttleToPulse(int32_t throttlePercent) {
	if (throttlePercent > 100) {
		throttlePercent = 100;
	} else if (throttlePercent < -100) {
		throttlePercent = -100;
	}

	return lroundf(map(throttlePercent, -100.0f, 100.0f, 1100.0f, 1900.0f));

}

/**
 * @brief Writes a motor throttle value to a timer PWM channel.
 * @param htim Pointer to the timer handle driving the PWM output.
 * @param channel Timer channel to update.
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 *        Out-of-range values are clamped in throttleToPulse().
 */
static inline void motor_write(TIM_HandleTypeDef *htim, uint32_t channel,
		int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(htim, channel, throttleToPulse(throttlePercent));
}

/**
 * @brief Resets all motors to 0 throttle and clears any break flags.
 * This function is useful for emergency stops or when initializing the system.
 */
void reset_motors() {

	motor1(0);
	motor2(0);
	motor3(0);
	motor4(0);
	motor5(0);
	motor6(0);

	__HAL_TIM_MOE_ENABLE(&htim1);
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);

	__HAL_TIM_MOE_ENABLE(&htim8);
	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
}

/**
 * @brief Sets motor 1 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor1(int32_t throttlePercent) {
	motor_write(MOTOR1_TIM, MOTOR1_CHANNEL, throttlePercent);
}

/**
 * @brief Sets motor 2 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor2(int32_t throttlePercent) {
	motor_write(MOTOR2_TIM, MOTOR2_CHANNEL, throttlePercent);
}

/**
 * @brief Sets motor 3 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor3(int32_t throttlePercent) {
	motor_write(MOTOR3_TIM, MOTOR3_CHANNEL, throttlePercent);
}

/**
 * @brief Sets motor 4 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor4(int32_t throttlePercent) {
	motor_write(MOTOR4_TIM, MOTOR4_CHANNEL, throttlePercent);
}

/**
 * @brief Sets motor 5 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor5(int32_t throttlePercent) {
	motor_write(MOTOR5_TIM, MOTOR5_CHANNEL, throttlePercent);
}

/**
 * @brief Sets motor 6 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor6(int32_t throttlePercent) {
	motor_write(MOTOR6_TIM, MOTOR6_CHANNEL, throttlePercent);
}
