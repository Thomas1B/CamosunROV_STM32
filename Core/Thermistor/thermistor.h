/*
 * thermistor.h
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
 *
 *      All constants are declared as float (with 'f' suffix) rather than
 *      double, since the STM32F446RE (Cortex-M4F) has a single-precision
 *      hardware FPU only.
 */

#ifndef THERMISTOR_THERMISTOR_H_
#define THERMISTOR_THERMISTOR_H_

#include <stdint.h>

#define R1  10e3f // R1 of the voltage divider with NTC.

#define RREF 10e3f      // Rref: NTC nominal resistance at 25 degC, in Ohms (R25)
#define A1   3.354016e-3f  // Steinhart-Hart coefficient A1, units: K^-1
#define B1   2.569850e-4f  // Steinhart-Hart coefficient B1, units: K^-1
#define C1   2.620131e-6f  // Steinhart-Hart coefficient C1, units: K^-1
#define D1   6.383091e-8f  // Steinhart-Hart coefficient D1, units: K^-1

float therm_get_ntc_resistance(float adcValue);
float therm_get_temperature(float R);

#endif /* THERMISTOR_THERMISTOR_H_ */
