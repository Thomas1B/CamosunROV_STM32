/*
 * TSYS01.c
 *
 *  Created on: Oct 12, 2026
 *      Author: Thomas Bourgeois
 *
 *  Original Code is from https://github.com/bluerobotics/BlueRobotics_TSYS01_Library
 *  Claude AI was used to convert the code to work with STM32F446. Some changes were made.
 *
 */

#include "TSYS01.h"

/* TSYS01 command bytes, per datasheet */
#define CMD_RESET           0x1E
#define CMD_PROM_READ_BASE  0xA0   /* PROM addr N -> CMD_PROM_READ_BASE + 2*N */
#define CMD_ADC_READ        0x00
#define CMD_CONVERT         0x48   /* start temperature conversion */

/* Send a single command byte (TSYS01 commands ARE the byte) */
static bool send_command(TSYS01_t *dev, uint8_t cmd) {
	return (HAL_I2C_Master_Transmit(dev->hi2c, TSYS01_I2C_ADDR, &cmd, 1, 10)
			== HAL_OK);
}

/* Read one 16-bit PROM word */
static bool read_prom_word(TSYS01_t *dev, uint8_t index, uint16_t *word) {
	uint8_t cmd = CMD_PROM_READ_BASE + (2 * index);
	uint8_t buf[2];

	if (!send_command(dev, cmd)) {
		return false;
	}
	if (HAL_I2C_Master_Receive(dev->hi2c, TSYS01_I2C_ADDR, buf, 2, 10)
			!= HAL_OK) {
		return false;
	}
	*word = ((uint16_t) buf[0] << 8) | buf[1];
	return true;
}

/* Read the 24-bit ADC result (3 bytes, MSB first) */
static bool read_adc(TSYS01_t *dev, uint32_t *result) {
	uint8_t buf[3];

	if (!send_command(dev, CMD_ADC_READ)) {
		return false;
	}
	if (HAL_I2C_Master_Receive(dev->hi2c, TSYS01_I2C_ADDR, buf, 3, 10)
			!= HAL_OK) {
		return false;
	}
	*result = ((uint32_t) buf[0] << 16) | ((uint32_t) buf[1] << 8) | buf[2];
	return true;
}

/*
 * Converts the raw 24-bit ADC value to degrees Celsius using the datasheet
 * polynomial:
 *
 *   T = -2*k4*1e-21*ADC16^4 + 4*k3*1e-16*ADC16^3 - 2*k2*1e-11*ADC16^2
 *       + 1*k1*1e-6*ADC16 - 1.5*k0*1e-2          where ADC16 = ADC24 / 256
 *
 * The Arduino library evaluates this literally with pow() and constants
 * like 1e-21, which wastes accuracy in single-precision float. Here ADC16
 * is scaled to x = ADC16 / 1e5 (so x is roughly 0..0.66), which folds the
 * powers of ten into the coefficients:
 *
 *   T = -0.2*k4*x^4 + 0.4*k3*x^3 - 0.2*k2*x^2 + 0.1*k1*x - 0.015*k0
 *
 * and it is evaluated in Horner form. Every intermediate value stays in a
 * range float handles well, and the F446 FPU does it in a few cycles with
 * no pow() call. ADC16 is kept fractional (not truncated to an integer
 * like the Arduino version) to keep the full ADC resolution.
 */
static float calculate(const uint16_t *prom, uint32_t adc24) {
	const float k4 = (float) prom[1];
	const float k3 = (float) prom[2];
	const float k2 = (float) prom[3];
	const float k1 = (float) prom[4];
	const float k0 = (float) prom[5];

	float x = ((float) adc24 / 256.0f) / 100000.0f;

	return ((((-0.2f * k4) * x + (0.4f * k3)) * x + (-0.2f * k2)) * x
			+ (0.1f * k1)) * x + (-0.015f * k0);
}

bool TSYS01_CheckCRC(TSYS01_t *dev) {
	uint8_t sum = 0;
	for (uint8_t i = 0; i < 8; i++) {
		sum += (uint8_t) (dev->prom[i] >> 8);
		sum += (uint8_t) (dev->prom[i] & 0xFF);
	}
	return (sum == 0);
}

bool TSYS01_Init(TSYS01_t *dev) {
	/* Reset sequence - required before first PROM read */
	if (!send_command(dev, CMD_RESET)) {
		return false;
	}
	HAL_Delay(10); /* datasheet: reset takes 2.8 ms, use margin */

	/* Read all 8 PROM words (index 0-7). Words 1-5 are the coefficients k4..k0. */
	for (uint8_t i = 0; i < 8; i++) {
		if (!read_prom_word(dev, i, &dev->prom[i])) {
			return false;
		}
	}

	/* Basic sanity check - no sensor / bad read returns all 0 or all 0xFFFF */
	if (dev->prom[1] == 0 || dev->prom[1] == 0xFFFF) {
		return false;
	}

	/* Checksum - catches corrupted coefficients the check above would miss */
	if (!TSYS01_CheckCRC(dev)) {
		return false;
	}

	dev->adc24 = 0;
	dev->temperature_C = 0.0f;
	return true;
}

bool TSYS01_StartConversion(TSYS01_t *dev) {
	return send_command(dev, CMD_CONVERT);
}

bool TSYS01_ReadResult(TSYS01_t *dev) {
	uint32_t adc24 = 0;
	if (!read_adc(dev, &adc24)) {
		return false;
	}
	/* The sensor returns 0 if the ADC is read before a conversion finished */
	if (adc24 == 0) {
		return false;
	}
	dev->adc24 = adc24;
	dev->temperature_C = calculate(dev->prom, adc24);
	return true;
}

bool TSYS01_Read(TSYS01_t *dev) {
	if (!TSYS01_StartConversion(dev)) {
		return false;
	}
	HAL_Delay(TSYS01_CONV_TIME_MS);
	return TSYS01_ReadResult(dev);
}

float TSYS01_GetTemperature(TSYS01_t *dev) {
	return dev->temperature_C;
}

bool TSYS01_SelfTest(TSYS01_t *dev) {
	/* Example values from the TSYS01 datasheet */
	dev->prom[0] = 0;
	dev->prom[1] = 28446; /* 0xA2, k4 */
	dev->prom[2] = 24926; /* 0xA4, k3 */
	dev->prom[3] = 36016; /* 0xA6, k2 */
	dev->prom[4] = 32791; /* 0xA8, k1 */
	dev->prom[5] = 40781; /* 0xAA, k0 */
	dev->prom[6] = 0;
	dev->prom[7] = 0;

	dev->adc24 = 9378708;
	dev->temperature_C = calculate(dev->prom, dev->adc24);

	float err = dev->temperature_C - 10.58f;
	if (err < 0.0f) {
		err = -err;
	}
	return (err < 0.01f);
}

void TSYS01_SetI2C(TSYS01_t *dev, I2C_HandleTypeDef *hi2c) {
	dev->hi2c = hi2c;
}
