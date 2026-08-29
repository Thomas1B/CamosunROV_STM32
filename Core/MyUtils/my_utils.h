/*
 * my_utils.h
 *
 *  Created on: Jul 17, 2026
 *      Author: Thomas Bourgeois
 *
 *
 * Helper Functions for the STM32F4 project, including mapping values and converting ADC readings to voltages.
 */

#ifndef MYUTILS_MY_UTILS_H_
#define MYUTILS_MY_UTILS_H_

#include <stdint.h>
#include <stdbool.h>

float util_map(float x, float in_min, float in_max, float out_min,
		float out_max);

float util_adc_to_voltage(uint16_t val);

void util_set_led_brightness_pct(uint8_t percent);

float util_average_channel(const volatile uint16_t *buf, uint32_t num_channels,
		uint32_t samples_per_channel, uint32_t channel_index);

bool has_fault(uint32_t current_fault, uint32_t faults_to_check);

bool has_all_faults(uint32_t current_fault, uint32_t faults_to_check);

#endif /* MYUTILS_MY_UTILS_H_ */
