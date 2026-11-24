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
extern TIM_HandleTypeDef htim3; /* the real variable lives in main.c */

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
 * Pulse limits (microseconds), same for every motor.
 *
 * MOTOR_NEUTRAL_PULSE is the ESC's stop point -- DO NOT CHANGE.
 *
 * MOTOR_MIN_PULSE / MOTOR_MAX_PULSE are hard output limits. The code never
 * sends a pulse outside them, so a 100 % command from the command centre
 * is capped to the real throttle shown below. The ESC hardware range is
 * 1100 .. 1900 us; do not set limits outside it.
 *
 * Real throttle at a given pulse width (ESC is linear about 1500 us):
 *
 *   pulse (us)   = 1500 + 4 * throttle (%)
 *   throttle (%) = (pulse - 1500) / 4
 *
 *        REVERSE LIMIT                    FORWARD LIMIT
 *   MIN_PULSE (us) | Real cap (%)    MAX_PULSE (us) | Real cap (%)
 *   ---------------+-------------    ---------------+-------------
 *        1100      |   -100.0             1700      |    50.0
 *        1120      |    -95.0             1720      |    55.0
 *        1140      |    -90.0             1740      |    60.0
 *        1160      |    -85.0             1760      |    65.0
 *        1180      |    -80.0             1780      |    70.0
 *        1200      |    -75.0             1800      |    75.0
 *        1220      |    -70.0             1820      |    80.0
 *        1240      |    -65.0             1840      |    85.0
 *        1260      |    -60.0             1860      |    90.0
 *        1280      |    -55.0             1880      |    95.0
 *        1300      |    -50.0             1900      |   100.0
 *
 *   e.g. to cap forward thrust at 80 %, set MOTOR_MAX_PULSE to 1820.0f.
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
 * DO NOT CHANGE
 * Fixed pulse limits (microseconds) for the MG90S servo, taken from the
 * datasheet: 1000 us = -90 deg, 1500 us = center, 2000 us = +90 deg,
 * at a 50 Hz (20 ms) PWM period. These are the hardware min/max/neutral
 * pulse widths, not per-servo tuning. Pulses outside this range can drive
 * the servo past its mechanical stop and cause stalling or gear damage.
 */
#define SERVO_TIM 			 &htim3
#define SERVO_CHANNEL 		 TIM_CHANNEL_3
#define SERVO_MIN_PULSE      1000.0f
#define SERVO_NEUTRAL_PULSE  1500.0f
#define SERVO_MAX_PULSE      2000.0f

/*
 * Commanded tilt limits (deci-degrees, x10). Commands outside this range
 * are clamped, so the camera stops at +/-45.0 deg (1000 us .. 2000 us),
 * which is the full travel this servo has inside the pulse limits above.
 * Matches the cam_tilt_dc field in the UART protocol, and TILT_MIN/TILT_MAX
 * (+/-45) in the command centre's config.py.
 */
#define SERVO_MIN_ANGLE_DC   (-450.0f)
#define SERVO_MAX_ANGLE_DC   ( 450.0f)

/*
 * Servo angle (deci-degrees, x10) at SERVO_MIN_PULSE and SERVO_MAX_PULSE.
 * Used only to convert angle -> pulse width.
 *
 * MEASURED, not from the datasheet: the MG90S datasheet claims +/-90 deg over
 * 1000..2000 us, but on our servo a +60 deg command (1833 us) only tilted the
 * camera ~30 deg (2026-11-02). Real travel is ~+/-45 deg over 1000..2000 us
 * (~11.1 us per degree). With +/-90 here, every command moved half as far
 * as asked. Re-measure if the servo is ever replaced.
 */
#define SERVO_PULSE_MIN_ANGLE_DC (-450.0f)
#define SERVO_PULSE_MAX_ANGLE_DC ( 450.0f)

/*
 * Camera tilt direction. The protocol defines + = camera tilts UP. If the
 * servo is mounted so that a + command tilts the camera DOWN, set this to 1.
 */
#define SERVO_REVERSED 0

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

void motors_reinit();

void motor1(int32_t throttlePercent);
void motor2(int32_t throttlePercent);
void motor3(int32_t throttlePercent);
void motor4(int32_t throttlePercent);
void motor5(int32_t throttlePercent);
void motor6(int32_t throttlePercent);

void servo(int32_t tiltDeciDeg);


#endif