/*
 * TSYS01.h
 *
 *  Created on: Oct 12, 2026
 *      Author: Thomas Bourgeois
 *
 *  Original Code is from https://github.com/bluerobotics/BlueRobotics_TSYS01_Library
 *  Claude AI was used to convert the code to work with STM32F446. Some changes were made.
 *
 *  Driver for the Blue Robotics Celsius Fast-Response Temperature Sensor (TSYS01).
 */

#ifndef TSYS01_TSYS01_H_
#define TSYS01_TSYS01_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define TSYS01_I2C_ADDR        (0x77 << 1)   /* HAL wants 8-bit (shifted) address */

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint16_t prom[8]; /* PROM words 0-7. Coefficients k4..k0 live in prom[1]..prom[5] */
	uint32_t adc24; /* last raw 24-bit ADC result */
	float temperature_C;
} TSYS01_t;

/**
 * @brief  Resets the sensor, reads the PROM coefficients and verifies
 *         the PROM checksum.
 * @param  dev: pointer to TSYS01 handle (hi2c must be set before calling)
 * @retval true if reset, PROM read and checksum all succeeded, false otherwise
 */
bool TSYS01_Init(TSYS01_t *dev);

/**
 * @brief  Performs one full temperature reading (blocking, about 10 ms).
 *         Updates dev->temperature_C.
 * @param  dev: pointer to TSYS01 handle
 * @retval true on success, false on I2C error
 */
bool TSYS01_Read(TSYS01_t *dev);

/**
 * @brief  Non-blocking alternative to TSYS01_Read(), part 1. Starts a
 *         conversion and returns immediately. Wait at least
 *         TSYS01_CONV_TIME_MS before calling TSYS01_ReadResult().
 * @retval true on success, false on I2C error
 */
bool TSYS01_StartConversion(TSYS01_t *dev);

/**
 * @brief  Non-blocking alternative to TSYS01_Read(), part 2. Reads the
 *         finished conversion and updates dev->temperature_C.
 * @retval true on success, false on I2C error or if no conversion was ready
 */
bool TSYS01_ReadResult(TSYS01_t *dev);

/**
 * @brief  Returns the last temperature reading in degrees Celsius.
 */
float TSYS01_GetTemperature(TSYS01_t *dev);

/**
 * @brief  Verifies the PROM data read during Init(). Per the datasheet,
 *         the sum of all 16 PROM bytes (0xA0..0xAE) must be 0 modulo 256.
 * @param  dev: pointer to TSYS01 handle - prom[] must already be populated
 * @retval true if the checksum is valid, false otherwise
 */
bool TSYS01_CheckCRC(TSYS01_t *dev);

/**
 * @brief  Loads the datasheet example coefficients and ADC value, then
 *         runs the conversion. Use to verify the math with no sensor
 *         connected. Overwrites dev->prom[], so call TSYS01_Init()
 *         again afterwards before reading a real sensor.
 * @retval true if the result is within 0.01 degC of the datasheet's 10.58 degC
 */
bool TSYS01_SelfTest(TSYS01_t *dev);

/**
 * @brief  Sets the I2C handle the sensor is connected on. Must be called
 *         before Init().
 * @param  dev: pointer to TSYS01 handle
 * @param  hi2c: pointer to the HAL I2C handle (e.g. &hi2c1)
 */
void TSYS01_SetI2C(TSYS01_t *dev, I2C_HandleTypeDef *hi2c);

#define TSYS01_CONV_TIME_MS 10  /* datasheet max 8.22 ms, rounded up with margin */

#endif /* TSYS01_TSYS01_H_ */
