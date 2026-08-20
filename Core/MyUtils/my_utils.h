/*
 * my_utils.c
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


float map(float x, float in_min, float in_max, float out_min, float out_max);


float analogToVoltage(uint16_t val);

#endif /* MYUTILS_MY_UTILS_H_ */
