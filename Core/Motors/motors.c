/*
 * motors.c
 *
 *  Created on: Jul 19, 2026
 *  Author: Thomas Bourgeois
 *
 *  This software is written to control either an Afro30A or BlueRobotics Basic ESC.
 *
 *	Nothing in this file needs to be changed. Any user changes are done in motors.h. *
 *
 *	Note: Static - means the function is contained only within this file.
 *		  Inline - removes overhead (only works for small functions).
 */

#include "motors.h"
#include "main.h"
#include <math.h>
#include "my_utils.h"

/*
 * Per-motor pulse limits. Each motor's limits are defined as structs and
 * passed into the single shared throttleToPulse(). Values are stored in motors.h
 */

static const MotorPulseLimits motor1_limits = { MOTOR1_REVERSE_ONSET_PULSE,
		MOTOR1_FORWARD_ONSET_PULSE };
static const MotorPulseLimits motor2_limits = { MOTOR2_REVERSE_ONSET_PULSE,
		MOTOR2_FORWARD_ONSET_PULSE };
static const MotorPulseLimits motor3_limits = { MOTOR3_REVERSE_ONSET_PULSE,
		MOTOR3_FORWARD_ONSET_PULSE };
static const MotorPulseLimits motor4_limits = { MOTOR4_REVERSE_ONSET_PULSE,
		MOTOR4_FORWARD_ONSET_PULSE };
static const MotorPulseLimits motor5_limits = { MOTOR5_REVERSE_ONSET_PULSE,
		MOTOR5_FORWARD_ONSET_PULSE };
static const MotorPulseLimits motor6_limits = { MOTOR6_REVERSE_ONSET_PULSE,
		MOTOR6_FORWARD_ONSET_PULSE };

/**
 * @brief Converts a throttle percentage to a pulse width in microseconds, using the
 *        given motor's onset-pulse limits.
 * @param limits Pointer to the calling motor's MotorPulseLimits (its reverse/forward
 *               onset pulse widths).
 * @param throttlePercent Throttle input, expected range -100% to +100%. Values outside
 *        this range are clamped before mapping, so a bad input saturates at min/max
 *        pulse width instead of being extrapolated by map() into an out-of-range
 *        compare value.
 * @retval Pulse width in microseconds.
 */
static inline uint32_t throttleToPulse(const MotorPulseLimits *limits,
		int32_t throttlePercent) {
	if (throttlePercent > 100) {
		throttlePercent = 100;
	} else if (throttlePercent < -100) {
		throttlePercent = -100;
	}

	if (throttlePercent == 0) {
		return (uint32_t) MOTOR_NEUTRAL_PULSE;
	} else if (throttlePercent > 0) {
		return lroundf(
				util_map(throttlePercent, 0.0f, 100.0f, limits->forwardOnsetPulse,
						MOTOR_MAX_PULSE));
	} else {
		return lroundf(
				util_map(throttlePercent, -100.0f, 0.0f, MOTOR_MIN_PULSE,
						limits->reverseOnsetPulse));
	}
}

/**
 * @brief Resets all motors to 0% throttle and re-arms the timer break
 *        (fault) protection on TIM1/TIM8.
 *
 * Beyond zeroing throttle, this function re-enables the PWM outputs
 * (MOE, gated off by BDTR after a break event), clears any latched
 * break flag, and re-enables the break interrupt. This makes it safe
 * to call both at system init and after a fault has been cleared, to
 * bring the motor subsystem back to a known, ready-to-run state.
 *
 */
void motors_reinit() {

	motor1(0);
	motor2(0);
	motor3(0);
	motor4(0);
	motor5(0);
	motor6(0);

	__HAL_TIM_MOE_ENABLE(&htim1); // Required: output stays gated off until MOE is set (BDTR.MOE) (Only advanced timers)
	__HAL_TIM_MOE_ENABLE(&htim8);

	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK); // clear interrupt clear for BKIN
	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);

	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); // <-- enable the break interrupt
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
}

/**
 * @brief Sets motor 1 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor1(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR1_TIM, MOTOR1_CHANNEL,
			throttleToPulse(&motor1_limits, throttlePercent));
}

/**
 * @brief Sets motor 2 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor2(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR2_TIM, MOTOR2_CHANNEL,
			throttleToPulse(&motor2_limits, throttlePercent));
}

/**
 * @brief Sets motor 3 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor3(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR3_TIM, MOTOR3_CHANNEL,
			throttleToPulse(&motor3_limits, throttlePercent));
}

/**
 * @brief Sets motor 4 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor4(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR4_TIM, MOTOR4_CHANNEL,
			throttleToPulse(&motor4_limits, throttlePercent));
}

/**
 * @brief Sets motor 5 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor5(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR5_TIM, MOTOR5_CHANNEL,
			throttleToPulse(&motor5_limits, throttlePercent));
}

/**
 * @brief Sets motor 6 throttle
 * @param throttlePercent Throttle value as a percentage (-100 to 100).
 */
void motor6(int32_t throttlePercent) {
	__HAL_TIM_SET_COMPARE(MOTOR6_TIM, MOTOR6_CHANNEL,
			throttleToPulse(&motor6_limits, throttlePercent));
}
