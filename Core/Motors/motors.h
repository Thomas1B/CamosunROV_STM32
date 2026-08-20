/*
 * motors.h
 *
 *  Created on: Jul 19, 2026
 *  Author: Thomas Bourgeois
 *
 *  This software is written to control either an Afro30A or BlueRobotics' Basic ESC.
 *
 *	Refer to STM32f446xx datasheet for TIM and TIM_CHANNEL.
 *
 */

#ifndef MOTORS_MOTORS_H_
#define MOTORS_MOTORS_H_

#include <stdint.h>
#include "main.h"          /* gives us TIM_HandleTypeDef, TIM_CHANNEL_x */

/*
 * DO NOT CHANGE
 */
extern TIM_HandleTypeDef htim1; /* the real variable lives in main.c */
extern TIM_HandleTypeDef htim8; /* the real variable lives in main.c */

/*
 * Motor -> timer/channel hardware mapping.
 *
 * Maps each of the 6 motors to the STM32 timer instance and channel that
 * generates its PWM signal. Motors 1-4 share TIM1 (channels 4 through 1),
 * motors 5-6 share TIM8 (channels 4 and 3). motors.c uses these directly
 * when calling HAL PWM functions (e.g. HAL_TIM_PWM_Start, __HAL_TIM_SET_COMPARE)
 * for each motorN() implementation, so this is the single place to update
 * if a motor is rewired to a different timer/channel.
 */

#define MOTOR1_TIM      &htim1
#define MOTOR1_CHANNEL  TIM_CHANNEL_4

#define MOTOR2_TIM      &htim1
#define MOTOR2_CHANNEL  TIM_CHANNEL_3

#define MOTOR3_TIM      &htim1
#define MOTOR3_CHANNEL  TIM_CHANNEL_2

#define MOTOR4_TIM      &htim1
#define MOTOR4_CHANNEL  TIM_CHANNEL_1

#define MOTOR5_TIM      &htim8
#define MOTOR5_CHANNEL  TIM_CHANNEL_4

#define MOTOR6_TIM      &htim8
#define MOTOR6_CHANNEL  TIM_CHANNEL_3

/*
 * DO NOT CHANGE
 * Fixed pulse limits (microseconds), same for every motor -- these are the ESC's
 * hardware min/max/neutral pulse width, not per-motor tuning.
 */
#define MOTOR_MIN_PULSE      1100.0f
#define MOTOR_NEUTRAL_PULSE  1500.0f
#define MOTOR_MAX_PULSE      1900.0f

/*
 * Per-motor pulse calibration, in microseconds.
 *
 * REVERSE_ONSET / FORWARD_ONSET mark the ESC's "deadband" boundary either side of
 * neutral -- the pulse width at the smallest throttle command (+/-1%) that still
 * moves the motor. Push these too close to NEUTRAL_PULSE and small throttle values
 * may not register; push them too far out and you lose fine control near zero
 * throttle. Tune per motor by testing where each ESC actually starts responding,
 * since it varies with the ESC and motor combination. Edit whichever motor needs
 * re-tuning; motors.c uses these constants directly, so no other file needs to change.
 */

#define MOTOR1_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR1_FORWARD_ONSET_PULSE  1550.0f

#define MOTOR2_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR2_FORWARD_ONSET_PULSE  1550.0f

#define MOTOR3_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR3_FORWARD_ONSET_PULSE  1550.0f

#define MOTOR4_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR4_FORWARD_ONSET_PULSE  1550.0f

#define MOTOR5_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR5_FORWARD_ONSET_PULSE  1550.0f

#define MOTOR6_REVERSE_ONSET_PULSE  1450.0f
#define MOTOR6_FORWARD_ONSET_PULSE  1550.0f

/*
 * DO NOT CHANGE ANYTHING PASS THIS POINT
 */

/*
 * Per-motor onset-pulse limits as struct, so throttleToPulse() can take
 * one argument per motor instead of two.
 */
typedef struct {
	float reverseOnsetPulse;
	float forwardOnsetPulse;
} MotorPulseLimits;

void reset_motors();

void motor1(int32_t throttlePercent);
void motor2(int32_t throttlePercent);
void motor3(int32_t throttlePercent);
void motor4(int32_t throttlePercent);
void motor5(int32_t throttlePercent);
void motor6(int32_t throttlePercent);

#endif
