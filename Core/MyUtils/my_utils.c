/*
 * my_utils.c
 *
 *  Created on: Jul 17, 2026
 *      Author: Thomas Bourgeois
 *
 *
 * Helper Functions for the STM32F4 project, including mapping values and converting ADC readings to voltages.
 */

#include "my_utils.h"


/**
 * @brief  Maps a value from one range to another (like Arduino's map()).
 * @param  x: The input value to map.
 * @param  in_min: The lower bound of the input value's current range.
 * @param  in_max: The upper bound of the input value's current range.
 * @param  out_min: The lower bound of the target range.
 * @param  out_max: The upper bound of the target range.
 * @retval The value of x mapped from [in_min, in_max] to [out_min, out_max].
 * @note   Does not clamp the result — if x is outside [in_min, in_max],
 *         the returned value will be outside [out_min, out_max] as well.
 * @note   If in_min == in_max, this will divide by zero.
 */
float util_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/**
 * @brief Converts a 12-bit ADC reading to a voltage.
 * @param val Raw ADC value (0–4095).
 * @return Voltage in volts (0.0–3.3V), assuming 3.3V reference.
 */
float util_adc_to_voltage(uint16_t val) {
	return ((float) val * 3.3f) / 4095.0f;
}

