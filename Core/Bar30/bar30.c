/*
 * bar30.c
 *
 *  Created on: Jul 27, 2026
 *      Author: Thomas Bourgeois
 *
 *  Original Code is from https://github.com/bluerobotics/BlueRobotics_MS5837_Library/tree/master
 *  Claude AI was used to convert the code to work with STM32f446.
 */

#include "bar30.h"

/* MS5837 command bytes, per datasheet */
#define CMD_RESET           0x1E
#define CMD_PROM_READ_BASE  0xA0   /* PROM addr N -> CMD_PROM_READ_BASE + 2*N */
#define CMD_ADC_READ        0x00
#define CMD_CONVERT_D1_BASE 0x40   /* pressure conversion,   OR with OSR bits */
#define CMD_CONVERT_D2_BASE 0x50   /* temperature conversion, OR with OSR bits */

/* Conversion wait time in ms, indexed to match MS5837_OSR_t values (>>1) */
static const uint8_t conv_delay_ms[6] = { 1, 2, 3, 5, 10, 19 /* rounded up from datasheet max times, with margin */
};

static uint8_t get_conv_delay(MS5837_OSR_t osr) {
	return conv_delay_ms[osr >> 1];
}

/* Send a single command byte (no register address, MS5837 commands ARE the byte) */
static bool send_command(MS5837_t *dev, uint8_t cmd) {
	return (HAL_I2C_Master_Transmit(dev->hi2c, MS5837_I2C_ADDR, &cmd, 1, 10)
			== HAL_OK);
}

/* Read a 24-bit ADC result (3 bytes, MSB first) */
static bool read_adc(MS5837_t *dev, uint32_t *result) {
	uint8_t buf[3];
	if (HAL_I2C_Master_Receive(dev->hi2c, MS5837_I2C_ADDR, buf, 3, 10)
			!= HAL_OK) {
		return false;
	}
	*result = ((uint32_t) buf[0] << 16) | ((uint32_t) buf[1] << 8) | buf[2];
	return true;
}

/* Read one 16-bit PROM word */
static bool read_prom_word(MS5837_t *dev, uint8_t index, uint16_t *word) {
	uint8_t cmd = CMD_PROM_READ_BASE + (2 * index);
	uint8_t buf[2];

	if (!send_command(dev, cmd)) {
		return false;
	}
	if (HAL_I2C_Master_Receive(dev->hi2c, MS5837_I2C_ADDR, buf, 2, 10)
			!= HAL_OK) {
		return false;
	}
	*word = ((uint16_t) buf[0] << 8) | buf[1];
	return true;
}

/*
 * CRC4 check, per the standard algorithm defined in the manufacturer's
 * datasheet for this sensor family (MS56xx/MS58xx/MS5837). Operates on
 * an 8-word PROM image; word 7 is unused on the MS5837 so it's padded
 * with 0. The top 4 bits of word 0 hold the factory-programmed checksum,
 * which gets masked out before recalculating it here for comparison.
 */
static uint8_t calculate_crc4(uint16_t *prom) {
	uint16_t words[8];
	uint16_t remainder = 0;

	for (uint8_t i = 0; i < 7; i++) {
		words[i] = prom[i];
	}
	words[7] = 0; /* MS5837 only uses 7 words - pad the 8th for the algorithm */

	words[0] = words[0] & 0x0FFF; /* mask out the stored CRC bits before recalculating */

	for (uint8_t byte_idx = 0; byte_idx < 16; byte_idx++) {
		if (byte_idx % 2 == 1) {
			remainder ^= (uint16_t) (words[byte_idx >> 1] & 0x00FF);
		} else {
			remainder ^= (uint16_t) (words[byte_idx >> 1] >> 8);
		}

		for (uint8_t bit = 8; bit > 0; bit--) {
			if (remainder & 0x8000) {
				remainder = (remainder << 1) ^ 0x3000;
			} else {
				remainder = (remainder << 1);
			}
		}
	}

	return (uint8_t) ((remainder >> 12) & 0x000F);
}

bool MS5837_CheckCRC(MS5837_t *dev) {
	uint8_t stored_crc = (uint8_t) ((dev->prom[0] >> 12) & 0x000F);
	uint8_t calculated_crc = calculate_crc4(dev->prom);
	return (stored_crc == calculated_crc);
}

bool MS5837_Init(MS5837_t *dev) {
	/* Reset sequence - required before first PROM read */
	if (!send_command(dev, CMD_RESET)) {
		return false;
	}
	HAL_Delay(10); /* datasheet: allow 2.8 ms min, use margin */

	/* Read all 7 PROM words (index 0-6). Words 1-6 are the calibration coefficients. */
	for (uint8_t i = 0; i < 7; i++) {
		if (!read_prom_word(dev, i, &dev->prom[i])) {
			return false;
		}
	}

	/* Basic sanity check - PROM shouldn't be all zero or all 0xFFFF (no sensor / bad read) */
	if (dev->prom[1] == 0 || dev->prom[1] == 0xFFFF) {
		return false;
	}

	/* CRC4 check - catches corrupted calibration data that the above
	 * check alone would miss (e.g. a single flipped bit from I2C noise) */
	if (!MS5837_CheckCRC(dev)) {
		return false;
	}

	/* Model default: only apply it if SetModel() hasn't already been called
	 * on this dev. This is what lets SetModel() work whether it's called
	 * before or after Init(). */
	if (!dev->_model_set) {
		dev->model = MS5837_MODEL_30BA; /* your Bar30 - default */
	}

	dev->surface_pressure_mbar = 1013.25f; /* standard atmosphere - default, override via SetSurfaceReference() */
	dev->fluidDensity_kg_m3 = 1026; // default density is of average ocean water 1026 kg/m3
	dev->secondOrderCalculation = false; /* first-order only by default - set true from main.c to enable */

	return true;
}

void MS5837_SetSurfaceReference(MS5837_t *dev, float pressure_mbar) {
	dev->surface_pressure_mbar = pressure_mbar;
}

float MS5837_GetSurfaceReference(MS5837_t *dev) {
	return dev->surface_pressure_mbar;
}

void MS5837_SetModel(MS5837_t *dev, MS5837_Model_t model) {
	dev->model = model;
	dev->_model_set = true;
}

MS5837_Model_t MS5837_GetModel(MS5837_t *dev) {
	return dev->model;
}

/*
 * Converts raw D1 (pressure) and D2 (temperature) ADC values into
 * compensated pressure (mbar) and temperature (deg C), using the PROM
 * calibration coefficients. Passed in as explicit arguments (rather than
 * reading them off dev) so this can be called identically from polling
 * or interrupt-mode code, and tested standalone against known D1/D2/PROM
 * values without needing a live sensor.
 *
 * First-order compensation always runs. Second-order compensation (extra
 * accuracy below 20 degC, with a further correction below -15 degC) only
 * runs if dev->secondOrderCalculation is true - set that from your main.c
 * to enable it, e.g. bar30.secondOrderCalculation = true;
 */
static void calculate(MS5837_t *dev, uint32_t D1, uint32_t D2) {
	/* Coefficients: C1=prom[1] .. C6=prom[6], per datasheet compensation formula */
	int32_t dT = (int32_t) D2 - ((int32_t) dev->prom[5] << 8);
	int64_t SENS = ((int64_t) dev->prom[1] << 15)
			+ (((int64_t) dev->prom[3] * dT) >> 8);
	int64_t OFF = ((int64_t) dev->prom[2] << 16)
			+ (((int64_t) dev->prom[4] * dT) >> 7);

	int32_t temp_raw = 2000 + (int32_t) (((int64_t) dT * dev->prom[6]) >> 23);

	if (dev->secondOrderCalculation) {
		/* Second-order correction terms - only meaningfully non-zero below 20 degC */
		int64_t Ti = 0;
		int64_t OFFi = 0;
		int64_t SENSi = 0;

		if (temp_raw < 2000) {
			int32_t T_delta = temp_raw - 2000;
			Ti = (3LL * (int64_t) dT * (int64_t) dT) >> 33;
			OFFi = (3LL * (int64_t) T_delta * (int64_t) T_delta) / 2;
			SENSi = (5LL * (int64_t) T_delta * (int64_t) T_delta) / 8;

			if (temp_raw < -1500) {
				int32_t T_delta2 = temp_raw + 1500;
				OFFi += 7LL * (int64_t) T_delta2 * (int64_t) T_delta2;
				SENSi += 4LL * (int64_t) T_delta2 * (int64_t) T_delta2;
			}
		} else {
			Ti = (2LL * (int64_t) dT * (int64_t) dT) >> 37;
		}

		temp_raw -= (int32_t) Ti;
		OFF -= OFFi;
		SENS -= SENSi;
	}

	int32_t pressure_raw = (int32_t) ((((int64_t) D1 * SENS) >> 21) - OFF)
			>> 13;

	dev->temperature_C = temp_raw / 100.0f;
	dev->pressure_mbar = pressure_raw / 10.0f;
}

bool MS5837_Read(MS5837_t *dev) {
	uint32_t D1 = 0; /* raw pressure ADC value */
	uint32_t D2 = 0; /* raw temperature ADC value */
	uint8_t delay_ms = get_conv_delay(dev->osr);

	/* --- Pressure conversion --- */
	if (!send_command(dev, CMD_CONVERT_D1_BASE | dev->osr))
		return false;
	HAL_Delay(delay_ms);
	if (!send_command(dev, CMD_ADC_READ))
		return false;
	if (!read_adc(dev, &D1))
		return false;

	/* --- Temperature conversion --- */
	if (!send_command(dev, CMD_CONVERT_D2_BASE | dev->osr))
		return false;
	HAL_Delay(delay_ms);
	if (!send_command(dev, CMD_ADC_READ))
		return false;
	if (!read_adc(dev, &D2))
		return false;

	calculate(dev, D1, D2);

	return true;
}

void MS5837_SetFluidDensity(MS5837_t *dev, float density_kg_m3) {
	dev->fluidDensity_kg_m3 = density_kg_m3;
}

float MS5837_GetFluidDensity(MS5837_t *dev) {
	return dev->fluidDensity_kg_m3;
}

float MS5837_GetDepth(MS5837_t *dev) {
	const float GRAVITY = 9.80665f; // average gravity on earth
	float pressure_diff_pa = (dev->pressure_mbar - dev->surface_pressure_mbar) * 100.0f; // Converting to SI units, note: 1 mbar = 1 hPa
	return pressure_diff_pa / (dev->fluidDensity_kg_m3 * GRAVITY);
}

void MS5837_SetI2C(MS5837_t *dev, I2C_HandleTypeDef *hi2c) {
	dev->hi2c = hi2c;
}


void MS5837_SetOSR(MS5837_t *dev, MS5837_OSR_t osr) {
	dev->osr = osr;
}

MS5837_OSR_t MS5837_GetOSR(MS5837_t *dev) {
	return dev->osr;
}
