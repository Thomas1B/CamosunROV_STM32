/*
 * Bar30.h
 *
 *  Created on: Jul 27, 2026
 *      Author: Thomas Bourgeois
 *
 *  Original Code is from https://github.com/bluerobotics/BlueRobotics_MS5837_Library/tree/master
 *  Claude AI was used to convert the code to work with STM32f446.
 */

#ifndef BAR30_BAR30_H_
#define BAR30_BAR30_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define MS5837_I2C_ADDR        (0x76 << 1)   /* HAL wants 8-bit (shifted) address */

/* Oversampling rate options - higher = more accurate but slower conversion */
typedef enum {
	MS5837_OSR_256 = 0x00, /* ~0.6 ms conversion  */
	MS5837_OSR_512 = 0x02, /* ~1.2 ms conversion  */
	MS5837_OSR_1024 = 0x04, /* ~2.3 ms conversion  */
	MS5837_OSR_2048 = 0x06, /* ~4.6 ms conversion  */
	MS5837_OSR_4096 = 0x08, /* ~9.1 ms conversion  */
	MS5837_OSR_8192 = 0x0A /* ~18.1 ms conversion, most accurate */
} MS5837_OSR_t;

/* Sensor variant - the MS5837 family covers two different depth ranges.
 * This is NOT auto-detected from the chip; you tell the driver which one
 * you have. Your Bar30 uses the 30BA variant (default). */
typedef enum {
	MS5837_MODEL_30BA = 0, /* Blue Robotics Bar30, ~300m depth rating */
	MS5837_MODEL_02BA = 1, /* shallower-rated variant, ~2 bar */
	MS5837_MODEL_UNRECOGNIZED = 255
} MS5837_Model_t;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint16_t prom[7]; /* factory calibration coefficients, index 1-6 used */
	MS5837_OSR_t osr;
	MS5837_Model_t model; /* which MS5837 variant - defaults to 30BA, explicitly set in Init() */
	float pressure_mbar;
	float temperature_C;
	float fluidDensity_kg_m3;
	float surface_pressure_mbar; /* zero-depth reference, defaults to 1013.25 in Init() */
	bool secondOrderCalculation; /* apply low-temp second-order compensation - default false, set true from main.c to enable */
	bool _model_set; /* INTERNAL USE ONLY. Set to true by MS5837_SetModel(). Lets
	 Init() tell "user already picked a model" apart from
	 "never touched", so it knows whether to apply the 30BA
	 default or leave your override alone. Don't read or
	 write this from main.c. */
} MS5837_t;

/**
 * @brief  Resets the sensor and reads factory calibration data from PROM.
 *         Also applies default values (surface_pressure_mbar = 1013.25,
 *         fluidDensity_kg_m3 = 1026, secondOrderCalculation = false), and
 *         sets model to MS5837_MODEL_30BA UNLESS MS5837_SetModel() was
 *         already called on this dev, in which case your choice is kept.
 * @param  dev: pointer to MS5837 handle (hi2c and osr must be set before calling)
 * @retval true if init + PROM read succeeded, false otherwise
 */
bool MS5837_Init(MS5837_t *dev);

/**
 * @brief  Performs one full pressure + temperature reading (blocking).
 *         Updates dev->pressure_mbar and dev->temperature_C.
 * @param  dev: pointer to MS5837 handle
 * @retval true on success, false on I2C error
 */
bool MS5837_Read(MS5837_t *dev);

/**
 * @brief  Sets which MS5837 variant this device is. Safe to call either
 *         BEFORE or AFTER Init() - once called, Init() will not overwrite
 *         it with the 30BA default.
 */
void MS5837_SetModel(MS5837_t *dev, MS5837_Model_t model);

/**
 * @brief  Returns which MS5837 variant is currently configured.
 */
MS5837_Model_t MS5837_GetModel(MS5837_t *dev);

/**
 * @brief  Verifies the PROM data read during Init() against its built-in
 *         CRC4 checksum (stored in the top 4 bits of prom[0]). Catches
 *         corrupted calibration data that a simple "is it all-zero" check
 *         would miss (e.g. a single flipped bit from I2C noise).
 * @param  dev: pointer to MS5837 handle - prom[] must already be populated
 * @retval true if the calculated CRC matches the stored CRC, false otherwise
 */
bool MS5837_CheckCRC(MS5837_t *dev);

/**
 * @brief  Sets the zero-depth reference pressure (mbar). Call this with a
 *         fresh reading taken while the sensor is still in air at the
 *         surface, right before deployment, to account for the day's
 *         actual atmospheric pressure instead of assuming a fixed value.
 *         Defaults to 1013.25 mbar (standard atmosphere) if never called.
 * @param  dev: pointer to MS5837 handle
 * @param  pressure_mbar: the reference pressure to use as "0 m depth"
 */
void MS5837_SetSurfaceReference(MS5837_t *dev, float pressure_mbar);

/**
 * @brief  Returns the currently configured zero-depth reference pressure (mbar).
 */
float MS5837_GetSurfaceReference(MS5837_t *dev);

void MS5837_SetFluidDensity(MS5837_t *dev, float density_kg_m3);

float MS5837_GetFluidDensity(MS5837_t *dev);

float MS5837_GetDepth(MS5837_t *dev);

/**
 * @brief  Sets the I2C handle the sensor is connected on. Must be called
 *         before Init() (Init() needs a valid dev->hi2c to talk to the
 *         sensor over I2C).
 * @param  dev: pointer to MS5837 handle
 * @param  hi2c: pointer to the HAL I2C handle (e.g. &hi2c1)
 */
void MS5837_SetI2C(MS5837_t *dev, I2C_HandleTypeDef *hi2c);

/**
 * @brief  Sets the oversampling rate (OSR) used for pressure/temperature
 *         conversions. Higher OSR = more accurate but slower reads (see
 *         MS5837_OSR_t for conversion time per setting). Must be called
 *         before the first Read() - it's used directly in Read() to pick
 *         the conversion delay and command bits, and is not defaulted by
 *         Init(), so set it explicitly.
 * @param  dev: pointer to MS5837 handle
 * @param  osr: desired oversampling rate
 */
void MS5837_SetOSR(MS5837_t *dev, MS5837_OSR_t osr);

/**
 * @brief  Returns the currently configured oversampling rate.
 */
MS5837_OSR_t MS5837_GetOSR(MS5837_t *dev);

#endif /* BAR30_BAR30_H_ */
