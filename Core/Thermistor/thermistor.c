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

