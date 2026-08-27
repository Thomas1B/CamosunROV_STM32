/*
 * thermistor.c
 *
 *  Created on: Aug 26, 2026
 *      Author: Thomas Bourgeois
 *
 *  Description:
 *      Driver support header for an NTC thermistor (Vishay NTCLE100E3103JB0,
 *      10 kΩ @ 25°C, B25/85 = 3977K material).
 *
 *      Temperature is computed from measured NTC resistance using the
 *      extended Steinhart-Hart equation (per the Vishay NTCLE100E3 datasheet):
 *
 *          1/T(K) = A1 + B1*ln(R/Rref) + C1*ln^2(R/Rref) + D1*ln^3(R/Rref)
 *
 *      Coefficients A1-D1 are specific to the thermistor.
 *      Refer to the datasheet for these values.

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
