/*
 * thermistor.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Thomas Bourgeois
 *
 *
 *      This firmware is for a NTC thermistor, it uses the Steinhart and Hart equation to solve for temperature.
 *      Refer to your datasheet for coefficient values.
 */

#include "thermistor.h"
#include <math.h>
#include "main.h"
#include "my_utils.h"

/**
 * @brief   Converts a raw ADC count to NTC resistance.
 *
 * @details Circuit: VCC -> R1 -> ADC node -> NTC -> GND. The ADC samples the
 *          node between R1 and the NTC, so as the NTC heats up (resistance
 *          drops), the node voltage - and ADC count - drops too.
 *
 *          Derived from the divider equation:
 *              Vadc = VCC * Rntc / (R1 + Rntc)
 *          which rearranges (substituting Vadc = adcValue/ADC_MAX * VCC,
 *          VCC cancels out) to:
 *              Rntc = R1 * adcValue / (ADC_MAX - adcValue)
 *
 * @param   adcValue  Raw ADC reading, 0 to ADC_MAX.
 *
 * @return  Thermistor resistance, in Ohms.
 *
 * @note    Not valid for adcValue == ADC_MAX (division by zero) - this would
 *          correspond to Rntc = infinity, i.e. an open circuit / disconnected
 *          sensor. Check for this fault condition before calling.
 */
float therm_get_ntc_resistance(float adcValue) {
	return R1 * (float) adcValue / (4095.0f - (float) adcValue);
}

/*
 * @brief   Converts a measured NTC thermistor resistance to temperature.
 *
 * @details Uses the Steinhart-Hart equation:
 *
 *
 * @param   R  Measured thermistor resistance, in Ohms.
 *
 * @return  Temperature in degrees Celsius.
 *
 * @note    All internal math is single-precision (logf, float literals) to
 *          match the STM32F446RE's single-precision hardware FPU.
 */
float therm_get_temperature(float R) {

	float lnR = logf(R / RREF); // ln(R / R_ref)
	float lnR2 = lnR * lnR; // ln^2(R / R_ref)
	float lnR3 = lnR2 * lnR; // ln^3(R / R_ref)

	float invT = A1 + (B1 * lnR) + (C1 * lnR2) + (D1 * lnR3);
	return (1.0f / invT) - 273.15;
}
