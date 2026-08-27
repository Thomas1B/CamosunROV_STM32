#include "bno055.h"
#include <string.h>

uint16_t accelScale = 100;
uint16_t tempScale = 1;
uint16_t angularRateScale = 16;
uint16_t eulerScale = 16;
uint16_t magScale = 16;
uint16_t quaScale = (1 << 14);    // 2^14



/**
 * @brief  Sets the current register page of the BNO055.
 * The BNO055 has two register pages (0 and 1) that contain different sets of registers.
 * This function allows you to switch between the two pages to access all the registers.
 *
 * @param page The register page to set (0 or 1)
 */
void bno055_setPage(uint8_t page) {
	bno055_writeData(BNO055_PAGE_ID, page);
}

/**
 * @brief  Gets the current operation mode of the BNO055.
 *
 * @retval bno055_opmode_t Current operation mode of the BNO055:
 * 	   - 0, 0x00, BNO055_OPERATION_MODE_CONFIG0,
 * 	   - 1, 0x01, BNO055_OPERATION_MODE_ACCONLY,
 * 	   - 2, 0x02, BNO055_OPERATION_MODE_MAGONLY,
 * 	   - 3, 0x03, BNO055_OPERATION_MODE_GYRONLY,
 * 	   - 4, 0x04, BNO055_OPERATION_MODE_ACCMAG,
 * 	   - 5, 0x05, BNO055_OPERATION_MODE_ACCGYRO,
 * 	   - 6, 0x06, BNO055_OPERATION_MODE_MAGGYRO,
 * 	   - 7, 0x07, BNO055_OPERATION_MODE_AMG (fusion mode),
 * 	   - 8, 0x08, BNO055_OPERATION_MODE_IMU,
 * 	   - 9, 0x09, BNO055_OPERATION_MODE_COMPASS,
 * 	   - 10, 0x0A, BNO055_OPERATION_MODE_M4G,
 * 	   - 11, 0x0B, BNO055_OPERATION_MODE_NDOF_FMC_OFF,
 * 	   - 12, 0x0C, BNO055_OPERATION_MODE_NDOF (fusion mode)
 */
bno055_opmode_t bno055_getOperationMode() {
	bno055_opmode_t mode;
	bno055_readData(BNO055_OPR_MODE, &mode, 1);
	return mode;
}

/**
 * @brief  Sets the current operation mode of the BNO055.
 *
 * @retval bno055_opmode_t Current operation mode of the BNO055:
 * 	   - 0x00 = Configuration mode (required to before changing mode)
 * 	   - 0x01 = Acceleration only
 * 	   - 0x02 = Magnetometer only
 * 	   - 0x03 = Gyroscope only
 * 	   - 0x04 = Accelerometer and Magnetometer
 * 	   - 0x05 = Accelerometer and Gyroscope
 * 	   - 0x06 = Magnetometer and Gyroscope
 * 	   - 0x07 = Accelerometer, Magnetometer, Gyroscope (no fusion algorithms)
 * 	   - 0x08 = Accel + Gyro (fusion algorithms, no  mag)
 * 	   - 0x09 = Accel + Mag (fusion for heading)
 * 	   - 0x0A = Mag + Accel (Gro-free fusion)
 * 	   - 0x0B = Full Fusion (no fast mag calibratiob)
 * 	   - 0x0C = Full Fusion 9-DOF fusion
 */
void bno055_setOperationMode(bno055_opmode_t mode) {
	bno055_writeData(BNO055_OPR_MODE, mode);
	if (mode == BNO055_OPERATION_MODE_CONFIG) {
		bno055_delay(19);
	} else {
		bno055_delay(7);
	}
}

/**
 * @brief  Sets the BNO055 to configuration mode. Must be in this mode to change settings.
 */
void bno055_setOperationModeConfig() {
	bno055_setOperationMode(BNO055_OPERATION_MODE_CONFIG);
}

/**
 * @brief  Sets the BNO055 to NDOF fusion mode.
 * This mode uses the accelerometer, magnetometer, and gyroscope to provide fused orientation data.
 * Full 9-DOF
 */
void bno055_setOperationModeNDOF() {
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
}

/**
 * @brief Enables or disables the use of a 32kHz external crystal oscillator on the BNO055.
 * Using the external crystal can improve the timing accuracy and sensor fusion stability.
 *
 * @param state Set to true to enable the external crystal, or false to disable it.
 * @note The external crystal bit is bit 7 of the SYS_TRIGGER register.
 */
void bno055_setExternalCrystalUse(bool state) {
	bno055_setPage(0);
	uint8_t tmp = 0;
	bno055_readData(BNO055_SYS_TRIGGER, &tmp, 1);
	tmp |= (state == true) ? 0x80 : 0x0;
	bno055_writeData(BNO055_SYS_TRIGGER, tmp);
	bno055_delay(700); // Required delay for the crystal to stabilize after enabling/disabling
}

/*
 * @brief Converience functions to enable use of a external crystal oscillator on the BNO055.
 */
void bno055_enableExternalCrystal() {
	bno055_setExternalCrystalUse(true);
}

/*
 * @brief Converience functions to disable use of a external crystal oscillator on the BNO055.
 */
void bno055_disableExternalCrystal() {
	bno055_setExternalCrystalUse(false);
}

/**
 * @brief  Resets the BNO055 sensor. This function will reset all registers to their default values and restart the sensor.
 * After calling this function, you should call `bno055_setup()` again to reinitialize the sensor and set the desired operation mode.
 *
 * @note The reset is triggered by writing 0x20 to the SYS_TRIGGER register.
 * @note Communication with the sensor should not occur until reset is complete.
 */
void bno055_reset() {
	bno055_writeData(BNO055_SYS_TRIGGER, 0x20);
	bno055_delay(700);
}

/**
 * @brief  Gets the temperature reading from the BNO055.
 * The temperature is stored in the BNO055_TEMP register and is represented as a signed 8-bit integer.
 * Value in degrees Celsius.
 */
int8_t bno055_getTemp() {
	bno055_setPage(0);
	uint8_t t;
	bno055_readData(BNO055_TEMP, &t, 1);
	return t;
}

/**
 * @brief  Initializes the BNO055 sensor. This function should be called before any other functions.
 * It performs a reset, checks the chip ID, and sets the operation mode to configuration mode.
 */
void bno055_setup() {
	bno055_reset();

	uint8_t id = 0;
	bno055_readData(BNO055_CHIP_ID, &id, 1);
	if (id != BNO055_ID) {
		printf("Can't find BNO055, id: 0x%02x. Please check your wiring.\r\n",
				id);
	}
	bno055_setPage(0);
	bno055_writeData(BNO055_SYS_TRIGGER, 0x0);

	// Select BNO055 config mode
	bno055_setOperationModeConfig();
	bno055_delay(10);
}

/*
 * @brief  Gets the software revision from the BNO055.
 * The software revision is stored in the BNO055_SW_REV_ID_LSB and BNO055_SW_REV_ID_MSB registers and is represented as a signed 16-bit integer.
 *
 *
 */
int16_t bno055_getSWRevision() {
	bno055_setPage(0);
	uint8_t buffer[2];
	bno055_readData(BNO055_SW_REV_ID_LSB, buffer, 2);
	return (int16_t) ((buffer[1] << 8) | buffer[0]);
}

/*
 * @brief  Gets the bootloader revision from the BNO055.
 * The bootloader revision is stored in the BNO055_BL_REV_ID register and is represented as an unsigned 8-bit integer.
 *
 * @note: this is read only.
 */
uint8_t bno055_getBootloaderRevision() {
	bno055_setPage(0);
	uint8_t tmp;
	bno055_readData(BNO055_BL_REV_ID, &tmp, 1);
	return tmp;
}

/*
 * @brief  Gets the system status from the BNO055.
 * The system status is stored in the BNO055_SYS_STATUS register and is represented as an unsigned 8-bit integer.
 * System status code:
 * 	   - 0, 0x00 = Idle
 * 	   - 1, 0x01 = System error
 * 	   - 2, 0x02 = Initializing peripherals
 * 	   - 3, 0x03 = System initialization
 * 	   - 4, 0x04 = Executing self-test
 * 	   - 5, 0x05 = Sensor fusion algorithm running
 * 	   - 6, 0x06 = System running without fusion algorithms
 */
uint8_t bno055_getSystemStatus() {
	bno055_setPage(0);
	uint8_t tmp;
	bno055_readData(BNO055_SYS_STATUS, &tmp, 1);
	return tmp;
}

/*
 * @brief  Gets the self-test results from the BNO055.
 * The self-test results are stored in the BNO055_ST_RESULT register and are represented as a bitfield in an unsigned 8-bit integer.
 * MCU self-test result (0 = fail, 1 = pass)
 */
bno055_self_test_result_t bno055_getSelfTestResult() {
	bno055_setPage(0);
	uint8_t tmp;
	bno055_self_test_result_t res = { .mcuState = 0, .gyrState = 0, .magState =
			0, .accState = 0 };
	bno055_readData(BNO055_ST_RESULT, &tmp, 1);
	res.mcuState = (tmp >> 3) & 0x01;
	res.gyrState = (tmp >> 2) & 0x01;
	res.magState = (tmp >> 1) & 0x01;
	res.accState = (tmp >> 0) & 0x01;
	return res;
}

/**
 * @brief  Gets the system error code from the BNO055.
 *
 * @retval uint8_t System status code:
 *         - 0, 0x00 = No error
 *         - 1, 0x01 = Peripheral initialization error
 *         - 2, 0x02 = System initialization error
 *         - 3, 0x03 = Self test result failed
 *         - 4, 0x04 = Register map value out of range
 *         - 5, 0x05 = Register map address out of range
 *         - 6, 0x06 = Register map write error
 *         - 7, 0x07 = Low power mode not available for selected operation mode
 *         - 8, 0x08 = Accelerometer power mode not available
 *         - 9, 0x09 = Fusion algorithm configuration error
 *         - 10, 0x0A = Sensor configuration error *
 */
uint8_t bno055_getSystemError() {
	bno055_setPage(0);
	uint8_t tmp;
	bno055_readData(BNO055_SYS_ERR, &tmp, 1);
	return tmp;
}

bno055_calibration_state_t bno055_getCalibrationState() {
	bno055_setPage(0);
	bno055_calibration_state_t cal =
			{ .sys = 0, .gyro = 0, .mag = 0, .accel = 0 };
	uint8_t calState = 0;
	bno055_readData(BNO055_CALIB_STAT, &calState, 1);
	cal.sys = (calState >> 6) & 0x03;
	cal.gyro = (calState >> 4) & 0x03;
	cal.accel = (calState >> 2) & 0x03;
	cal.mag = calState & 0x03;
	return cal;
}

bno055_calibration_data_t bno055_getCalibrationData() {
	bno055_calibration_data_t calData;
	uint8_t buffer[22];
	bno055_opmode_t operationMode = bno055_getOperationMode();
	bno055_setOperationModeConfig();
	bno055_setPage(0);

	bno055_readData(BNO055_ACC_OFFSET_X_LSB, buffer, 22);

	// Assumes little endian processor
	memcpy(&calData.offset.accel, buffer, 6);
	memcpy(&calData.offset.mag, buffer + 6, 6);
	memcpy(&calData.offset.gyro, buffer + 12, 6);
	memcpy(&calData.radius.accel, buffer + 18, 2);
	memcpy(&calData.radius.mag, buffer + 20, 2);

	bno055_setOperationMode(operationMode);

	return calData;
}

void bno055_setCalibrationData(bno055_calibration_data_t calData) {
	uint8_t buffer[22];
	bno055_opmode_t operationMode = bno055_getOperationMode();
	bno055_setOperationModeConfig();
	bno055_setPage(0);

	// Assumes litle endian processor
	memcpy(buffer, &calData.offset.accel, 6);
	memcpy(buffer + 6, &calData.offset.mag, 6);
	memcpy(buffer + 12, &calData.offset.gyro, 6);
	memcpy(buffer + 18, &calData.radius.accel, 2);
	memcpy(buffer + 20, &calData.radius.mag, 2);

	for (uint8_t i = 0; i < 22; i++) {
		// TODO(oliv4945): create multibytes write
		bno055_writeData(BNO055_ACC_OFFSET_X_LSB + i, buffer[i]);
	}

	bno055_setOperationMode(operationMode);
}

/**
 * @brief  Gets a vector reading from the BNO055.
 * The type of vector is determined by the `vec` parameter.
 *
 * @param vec The type of vector to read:
 * 	   - BNO055_VECTOR_ACCELEROMETER: Acceleration vector (x, y, z) in m/s^2
 * 	   - BNO055_VECTOR_LINEARACCEL: Linear acceleration vector (x, y, z) in m/s^2 (acceleration minus gravity)
 * 	   - BNO055_VECTOR_GRAVITY: Gravity vector (x, y, z) in m/s^2
 * 	   - BNO055_VECTOR_MAGNETOMETER: Magnetometer vector (x, y, z) in microteslas
 * 	   - BNO055_VECTOR_GYROSCOPE: Gyroscope vector (x, y, z) in degrees per second
 * 	   - BNO055_VECTOR_EULER: Euler angles (heading, roll, pitch) in degrees
 * 	   - BNO055_VECTOR_QUATERNION: Quaternion (w, x, y, z) with no units
 */
bno055_vector_t bno055_getVector(uint8_t vec) {
	bno055_setPage(0);
	uint8_t buffer[8];    // Quaternion need 8 bytes

	if (vec == BNO055_VECTOR_QUATERNION)
		bno055_readData(vec, buffer, 8);
	else
		bno055_readData(vec, buffer, 6);

	double scale = 1;

	if (vec == BNO055_VECTOR_MAGNETOMETER) {
		scale = magScale;
	} else if (vec == BNO055_VECTOR_ACCELEROMETER
			|| vec == BNO055_VECTOR_LINEARACCEL
			|| vec == BNO055_VECTOR_GRAVITY) {
		scale = accelScale;
	} else if (vec == BNO055_VECTOR_GYROSCOPE) {
		scale = angularRateScale;
	} else if (vec == BNO055_VECTOR_EULER) {
		scale = eulerScale;
	} else if (vec == BNO055_VECTOR_QUATERNION) {
		scale = quaScale;
	}

	bno055_vector_t xyz = { .w = 0, .x = 0, .y = 0, .z = 0 };
	if (vec == BNO055_VECTOR_QUATERNION) {
		xyz.w = (int16_t) ((buffer[1] << 8) | buffer[0]) / scale;
		xyz.x = (int16_t) ((buffer[3] << 8) | buffer[2]) / scale;
		xyz.y = (int16_t) ((buffer[5] << 8) | buffer[4]) / scale;
		xyz.z = (int16_t) ((buffer[7] << 8) | buffer[6]) / scale;
	} else {
		xyz.x = (int16_t) ((buffer[1] << 8) | buffer[0]) / scale;
		xyz.y = (int16_t) ((buffer[3] << 8) | buffer[2]) / scale;
		xyz.z = (int16_t) ((buffer[5] << 8) | buffer[4]) / scale;
	}

	return xyz;
}

/*
 * @brief  Gets the accelerometer vector (x, y, z) from the BNO055 in m/s^2.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_ACCELEROMETER`.
 */
bno055_vector_t bno055_getVectorAccelerometer() {
	return bno055_getVector(BNO055_VECTOR_ACCELEROMETER);
}

/*
 * @brief  Gets the magnetometer vector (x, y, z) from the BNO055 in microteslas.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_MAGNETOMETER`.
 */
bno055_vector_t bno055_getVectorMagnetometer() {
	return bno055_getVector(BNO055_VECTOR_MAGNETOMETER);
}

/*
 * @brief  Gets the gyroscope vector (x, y, z) from the BNO055 in degrees per second.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_GYROSCOPE`.
 */
bno055_vector_t bno055_getVectorGyroscope() {
	return bno055_getVector(BNO055_VECTOR_GYROSCOPE);
}

/*
 * @brief  Gets the Euler angles (heading, roll, pitch) from the BNO055 in degrees.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_EULER`.
 */
bno055_vector_t bno055_getVectorEuler() {
	return bno055_getVector(BNO055_VECTOR_EULER);
}

/*
 * @brief  Gets the linear acceleration vector (x, y, z) from the BNO055 in m/s^2.
 * Linear acceleration is the acceleration minus gravity.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_LINEARACCEL`.
 */
bno055_vector_t bno055_getVectorLinearAccel() {
	return bno055_getVector(BNO055_VECTOR_LINEARACCEL);
}

/*
 * @brief  Gets the gravity vector (x, y, z) from the BNO055 in m/s^2.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_GRAVITY`.
 */
bno055_vector_t bno055_getVectorGravity() {
	return bno055_getVector(BNO055_VECTOR_GRAVITY);
}

/*
 * @brief  Gets the quaternion (w, x, y, z) from the BNO055 with no units.
 * This function is a wrapper around `bno055_getVector` with the vector type set to `BNO055_VECTOR_QUATERNION`.
 */
bno055_vector_t bno055_getVectorQuaternion() {
	return bno055_getVector(BNO055_VECTOR_QUATERNION);
}

/*
 * @brief Remaps and inverts the BNO055 coordinate axes.
 *
 * This function configures the axis remap and axis sign registers
 * of the BNO055. It allows the sensor orientation to be adjusted
 * in software when the sensor is mounted in a non-default orientation.
 *
 * Axis remapping changes which physical sensor axis is used as
 * the logical X, Y, and Z axes.
 *
 * Axis sign mapping allows individual axes to be inverted.
 *
 * The BNO055 should be placed into CONFIGMODE before calling
 * this function.
 *
 * @param axis Structure containing axis remap and sign configuration.
 */
void bno055_setAxisMap(bno055_axis_map_t axis) {
	uint8_t axisRemap = (axis.z << 4) | (axis.y << 2) | (axis.x);
	uint8_t axisMapSign = (axis.x_sign << 2) | (axis.y_sign << 1)
			| (axis.z_sign);
	bno055_writeData(BNO055_AXIS_MAP_CONFIG, axisRemap);
	bno055_writeData(BNO055_AXIS_MAP_SIGN, axisMapSign);
}

/*
 * @brief Gets the current axis remap and sign configuration from the BNO055.
 *
 * This function reads the axis remap and axis sign registers of the BNO055
 * and returns a structure containing the current configuration.
 *
 * @return Structure containing current axis remap and sign configuration.
 */
uint8_t bno055_getUnitSelection() {
	bno055_setPage(0);

	uint8_t unit;
	bno055_readData(BNO055_UNIT_SEL, &unit, 1);

	return unit;
}

/**
 * @brief Prints the current BNO055 unit configuration.
 *
 * Reads the UNIT_SEL register (0x3B) and prints the currently
 * selected units for acceleration, angular rate, Euler angles,
 * and temperature.
 *
 * UNIT_SEL bit layout:
 * Bit 0: Acceleration units (0 = m/s^2, 1 = mg)
 * Bit 1: Angular rate units (0 = degrees/s, 1 = radians/s)
 * Bit 2: Euler angle units (0 = degrees, 1 = radians)
 * Bit 4: Temperature units (0 = Celsius, 1 = Fahrenheit)
 * Bit 7: Fusion output format (0 = Windows, 1 = Android)
 *  *
 * Magnetometer units are always microtesla (uT).
 * Quaternion values are unitless.
 */
void bno055_printUnits() {

	uint8_t unit = bno055_getUnitSelection();

	printf("UNIT_SEL = 0x%02X\r\n", unit);

	// Bit 0: acceleration units
	printf("Bit 0 (Accel Units): %d -> %s\r\n", (unit >> 0) & 0x01,
			(unit & 0x01) ? "mg" : "m/s^2");

	// Linear acceleration + gravity use same accel setting
	printf("Linear Accel / Gravity: %s\r\n", (unit & 0x01) ? "mg" : "m/s^2");

	// Bit 1: angular rate units
	printf("Bit 1 (Gyro Units): %d -> %s\r\n", (unit >> 1) & 0x01,
			(unit & 0x02) ? "Rps" : "Dps");

	// Bit 2: Euler angle units
	printf("Bit 2 (Euler Units): %d -> %s\r\n", (unit >> 2) & 0x01,
			(unit & 0x04) ? "Radians" : "Degrees");

	// Bit 4: temperature units
	printf("Bit 4 (Temperature Units): %d -> %s\r\n", (unit >> 4) & 0x01,
			(unit & 0x10) ? "F" : "C");

	printf("Bit 7 (Fusion Format): %d -> %s\r\n", (unit >> 7) & 0x01,
			(unit & 0x80) ? "Android" : "Windows");

	// Fixed units
	printf("Magnetometer: microtesla (fixed)\r\n");
	printf("Quaternion: quaternion units (fixed)\r\n");
}

/*
 * @brief  Runs the calibration process for the BNO055 sensor.
 *
 * This function guides the user through the calibration process for the gyroscope, accelerometer, and magnetometer.
 * It continuously checks the calibration status and provides instructions to the user until all sensors are fully calibrated.
 * Once calibration is complete, it prints the calibration data and system status.
 */
void bno055_runCalibration() {
	printf("Starting calibration process...\r\n");
	bno055_calibration_state_t cal = bno055_getCalibrationState();
	printf("Calibration: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\r\n\n", cal.sys,
			cal.gyro, cal.accel, cal.mag);
	bno055_delay(500);

	while (cal.gyro < 3) {
		printf("Calibrating Gyroscope... "
				"Keep the sensor still.\r\n");
		bno055_delay(500);
		cal = bno055_getCalibrationState();
		printf("Calibration: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\r\n\n", cal.sys,
				cal.gyro, cal.accel, cal.mag);
	}

	while (cal.accel < 3) {
		printf("Calibrating Accelerometer... "
				"Move the sensor in 45 degree increments around one axis.\r\n");
		bno055_delay(500);
		cal = bno055_getCalibrationState();
		printf("Calibration: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\r\n\n", cal.sys,
				cal.gyro, cal.accel, cal.mag);
	}

	while (cal.mag < 3) {
		printf("Calibrating Magnetometer... "
				"Move the sensor in a figure 8 pattern.\r\n");
		bno055_delay(500);
		cal = bno055_getCalibrationState();
		printf("Calibration: Sys=%d, Gyro=%d, Accel=%d, Mag=%d\r\n\n", cal.sys,
				cal.gyro, cal.accel, cal.mag);
	}

	printf("Calibration complete!\r\n");
	printf("Calibration Data:\r\n");

	bno055_calibration_data_t calData = bno055_getCalibrationData();
	printf("Accel Offsets: X=%d, Y=%d, Z=%d\r\n", calData.offset.accel.x,
			calData.offset.accel.y, calData.offset.accel.z);

	printf("Mag Offsets: X=%d, Y=%d, Z=%d\r\n", calData.offset.mag.x,
			calData.offset.mag.y, calData.offset.mag.z);

	printf("Gyro Offsets: X=%d, Y=%d, Z=%d\r\n", calData.offset.gyro.x,
			calData.offset.gyro.y, calData.offset.gyro.z);

	printf("Accel Radius: %d\r\n", calData.radius.accel);
	printf("Mag Radius: %d\r\n", calData.radius.mag);

	printf("Status: %d \r\n", bno055_getSystemStatus());
	printf("Error Code: %d \r\n", bno055_getSystemError());
	while (1) {
	};
}
