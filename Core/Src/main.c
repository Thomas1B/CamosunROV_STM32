/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include "my_utils.h"
#include "motors.h"
#include "thermistor.h"
#include "bno055_stm32.h"
#include "bar30.h"
#include "TSYS01.h"
#include "UART_DMA.h"
#include "status_led.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Top-level system state.
 *
 * Fault bits. These are LIVE: a bit is set while its fault is present and
 * cleared when it recovers (sensor answers again, Pi link comes back).
 * The one exception is LEAK, which stays set until the Pi sends a recover
 * command (leak sensor must be dry, see emergency_recover()), because the
 * emergency shutdown can't be undone without a reset.
 * The same bits are sent to the Pi as fault_flags in every telemetry frame.
 */
typedef enum {
	SYS_STATE_FAULT_NONE = 0,        // Normal operation; no faults
	SYS_STATE_FAULT_LEAK = (1 << 0), // Leak detected (latched until recover command)
	SYS_STATE_FAULT_IMU = (1 << 1), // IMU not responding / reset itself (live)
	SYS_STATE_FAULT_DEPTH = (1 << 2), // Depth sensor not responding (live)
	SYS_STATE_FAULT_TEMP = (1 << 3), // Water temperature sensor not responding (live)
	SYS_STATE_FAULT_UART = (1 << 4), // Pi link lost: command timeout after link was up (live)
} SystemState_t;

/**
 * @brief ADC1 channel selection.
 *
 * Identifies which physical input is being read via ADC1.
 */
typedef enum {
	CH_TEMPERATURE = 0, // thermistor channel
	CH_BATTERY, // battery channel
} ADC1_channel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SURFACE_PRESSURE 1013.25f // surface barometric pressure reference (units are either hPa or mbar)
#define WATER_DENSITY 1000.00f // density of water, change this depending on freshwater or saltwater: 1000 kg/m^3 or 1025 kg/m^3 (avg)

#define ADC1_NUM_CHANNELS 2 // number of channels use on ADC1
#define ADC1_SAMPLES_PER_CHANNEL 16 // number of samples to take / note: TIM2 (1KHz)

#define BATTERY_MONITOR_R1 100000.0f // R1 (100K) of voltage divider for battery monitor / MUST BE DEFINED AS FLOATS.
#define BATTERY_MONITOR_R2 18000.0f // R2 (18K) of voltage divider for battery monitor / MUST BE DEFINED AS FLOATS.

/* UART link to the Pi */
#define CMD_TIMEOUT_MS   150U  // no valid command for this long -> stop thrusters (~7 missed frames at 50 Hz)
#define TELEM_PERIOD_MS  20U   // read sensors + send telemetry every 20 ms (50 Hz)
#define DEBUG_PRINT_MS   1500U // SWO debug printf period (same as before)

/* I2C sensor health (IMU, Bar30 depth, Celsius water temp) */
#define SENSOR_FAIL_LIMIT 3U    // consecutive failed reads before a sensor is faulted (3 x 20 ms = 60 ms)
#define SENSOR_RETRY_MS   1000U // how often faulted sensors are re-initialized, and the IMU mode is checked

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;

/* USER CODE BEGIN PV */

volatile SystemState_t system_state_t = SYS_STATE_FAULT_NONE; // current system state // RUN or FAULT
volatile uint16_t adc1_raw_buffer[ADC1_NUM_CHANNELS * ADC1_SAMPLES_PER_CHANNEL]; // array to store ADC1 raw data. / data[CH0, CH1, CH0, CH1, ..., CH0, CH1]

bno055_vector_t imu_vec; // instance of the BNO055 sensor
bno055_calibration_data_t savedCalData = { .offset.accel = { .x = -7, .y = 6,
		.z = -33 }, .offset.mag = { .x = -76, .y = -423, .z = -216 },
		.offset.gyro = { .x = -2, .y = -3, .z = -1 }, .radius.accel = 1000,
		.radius.mag = 1484 };

MS5837_t bar30; //  instance of bar30 pressure sensor
TSYS01_t celsius; // instance of Celsius (TSYS01) water temperature sensor
static bool celsius_converting = false; // true = a TSYS01 conversion was started last loop and is ready to read

float battery_voltage; // var to store battery voltage
float internal_temperature; // var to store internal temperature
float water_temperature; // var to store water temperature
float depth; // var to store depth
float pressure_kpa; // var to store absolute pressure from the Bar30, kPa

static bool emergency_shutdown_done = false;

/* UART link to the Pi */
static cmd_data_t latestCmd; // newest valid command; written in UART ISR, copied in main with IRQs off
static volatile uint8_t cmdReady = 0; // 1 = latestCmd holds a command main hasn't applied yet
static volatile uint32_t lastCmdTick = 0;  // HAL tick of the last valid command
static uint8_t failsafeActive = 1; // 1 = thrusters stopped by the command timeout
static uint8_t linkEstablished = 0; // 1 = at least one valid command received since boot
static uint8_t lastRecover = 0;     // recover byte of the previous command, for 0 -> 1 detection

/*
 * I2C sensor faults that are active RIGHT NOW (IMU / DEPTH / TEMP bits only).
 * A bit is set after SENSOR_FAIL_LIMIT failed reads in a row, and cleared
 * again when the sensor is successfully re-initialized (tried every
 * SENSOR_RETRY_MS). Drives the yellow LED and is sent to the Pi through
 * get_faults(). Only touched from the main loop, so no interrupt race.
 */
static uint8_t sensorFaults = SYS_STATE_FAULT_NONE;
static uint8_t imuFailCount = 0;   // consecutive failed reads, per sensor
static uint8_t depthFailCount = 0;
static uint8_t tempFailCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

void emergency_shutdown(void);
bool emergency_recover(void);
float get_battery_voltage(void);
float get_internal_temperature(void);
void bno055_check_connection(void);

static uint16_t to_u16_x100(float v);
static uint16_t to_u16_x10(float v);
static int16_t to_i16_x100(float v);
static void apply_command(const cmd_data_t *cmd);
static void apply_failsafe(void);
static void read_sensors(void);
static void send_telemetry(void);
static void print_faults(uint8_t faults);

static bool imu_start(void);
static bool imu_reconfigure(void);
static bool celsius_start(void);
static void sensor_read_result(uint8_t faultBit, bool ok, uint8_t *failCount);
static void sensor_health_check(void);
static led_link_state_t get_link_state(void);
static uint8_t get_faults(void);
static bool leak_detected(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_TIM1_Init();
	MX_TIM8_Init();
	MX_ADC1_Init();
	MX_TIM2_Init();
	MX_I2C1_Init();
	MX_UART4_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	/*
	 * These must be at the top to catch any leaks on startup.
	 */
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); // <-- enable the break interrupt
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

	// Give each sensor driver its I2C handle and settings. No bus traffic here,
	// so it is done even on a leak at boot: emergency_recover() needs it.
	bno055_assignI2C(&hi2c1); // Assign the I2C handle to the BNO055 IMU
	MS5837_SetI2C(&bar30, &hi2c1); // Assign the I2C handle to the Bar30
	MS5837_SetOSR(&bar30, MS5837_OSR_256); // set the oversampling rate for the sensor
	MS5837_SetModel(&bar30, MS5837_MODEL_30BA); // set the sensor model
	MS5837_SetSurfaceReference(&bar30, SURFACE_PRESSURE); // pressure read as 0 m depth
	MS5837_SetFluidDensity(&bar30, WATER_DENSITY); // fresh or salt water
	bar30.secondOrderCalculation = false; // true = enable second-order calculation, compensates for water below 20 C
	TSYS01_SetI2C(&celsius, &hi2c1); // Assign the I2C handle to the TSYS01 (0x77, Bar30 is 0x76)

	if (system_state_t == SYS_STATE_FAULT_LEAK) {
		// Leak at power-up: skip the lamp test and boot sequence, go
		// straight to the LEAK pattern (red only, solid) and stay there.
		// Then fall through so the UART link still starts and the Pi gets
		// leak = 1 telemetry (sensor fields will be 0, they were never read).
		emergency_shutdown();
	} else {
		// Power-up LED sequence (see status_led.h):
		status_led_lamp_test();  // 1. all three on ~1 s
		status_led_boot_start(); // 2. green on (solid) during sensor init
		HAL_Delay(250);

		// Each sensor that fails to start is marked as faulted (yellow LED).
		// It is retried every SENSOR_RETRY_MS from the main loop, so a sensor
		// that is plugged in late, or recovers, clears its fault on its own.

		// Setting BNO055 IMU device
		if (!imu_start()) {
			sensorFaults |= SYS_STATE_FAULT_IMU;
		}

		// Setting bar30 pressure sensor
		if (!MS5837_Init(&bar30)) {
			sensorFaults |= SYS_STATE_FAULT_DEPTH;
		}

		// Setting Celsius (TSYS01) water temperature sensor
		if (!celsius_start()) {
			sensorFaults |= SYS_STATE_FAULT_TEMP;
		}

		// Starting ADC DMA with timer
		HAL_TIM_Base_Start(&htim2);
		HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_raw_buffer,
		ADC1_NUM_CHANNELS * ADC1_SAMPLES_PER_CHANNEL);

		// Initialize Motors (throttle to 0%) and start their PWM signal
		motors_reinit(); // # initialize motors (sets throttle to 0%)
		HAL_TIM_PWM_Start(MOTOR1_TIM, MOTOR1_CHANNEL); // Start PWM for Motor1
		HAL_TIM_PWM_Start(MOTOR2_TIM, MOTOR2_CHANNEL); // Start PWM for Motor2
		HAL_TIM_PWM_Start(MOTOR3_TIM, MOTOR3_CHANNEL); // Start PWM for Motor3
		HAL_TIM_PWM_Start(MOTOR4_TIM, MOTOR4_CHANNEL); // Start PWM for Motor4
		HAL_TIM_PWM_Start(MOTOR5_TIM, MOTOR5_CHANNEL); // Start PWM for Motor5
		HAL_TIM_PWM_Start(MOTOR6_TIM, MOTOR6_CHANNEL); // Start PWM for Motor6

		// Initialize Camera tilt Servo
		HAL_TIM_PWM_Start(SERVO_TIM, SERVO_CHANNEL);

		// 3. Fast green flicker to show boot up sequence is done.
		status_led_boot_done();
	}

	/* ---- UART link to the Pi start-up, runs once ---- */
	proto_start_rx(&huart4);                   // start listening for commands
	uint32_t lastTelemTick = HAL_GetTick();
	uint32_t lastSensorCheckTick = HAL_GetTick();
	cmd_data_t lastAppliedCmd = { 0 }; // newest command applied, kept for the debug print

	uint32_t Debug_lastPrint = 0;

	// 4. Switch from the boot sequence to the status table.
	status_led_update(leak_detected(), get_link_state(), sensorFaults != 0U);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		/* ---- 1. Faults ------------------------------------------------ */
		// LEAK is set by the break ISR and stays set until a recover command.
		// While leaking, ONLY the UART link and status LEDs run: motor/tilt
		// values are ignored, sensors are not read, no SWO debug print.
		if (leak_detected()) {
			emergency_shutdown(); // idempotent: the shutdown sequence only runs once

			cmd_data_t rcmd;
			uint8_t gotCmd = 0;
			__disable_irq();
			if (cmdReady) {
				rcmd = latestCmd;
				cmdReady = 0;
				gotCmd = 1;
			}
			__enable_irq();

			// Recover only when the recover byte goes 0 -> 1 (not while it is
			// held at 1). emergency_recover() refuses if the sensor is still wet.
			if (gotCmd) {
				uint8_t risingEdge = (rcmd.recover == 1U) && (lastRecover == 0U);
				lastRecover = rcmd.recover;
				if (risingEdge && emergency_recover()) {
					continue; // back to normal operation from the next pass
				}
			}
			proto_rx_keepalive(&huart4);

			// Telemetry: leak = 1, LEAK flag, sensor fields frozen at last reading
			if ((HAL_GetTick() - lastTelemTick) >= TELEM_PERIOD_MS) {
				lastTelemTick = HAL_GetTick();
				send_telemetry();
			}
			status_led_show_leak();
			continue;
		}

		/* ---- 2. Commands from the Pi ---------------------------------- */
		cmd_data_t cmd;
		uint8_t haveCmd = 0;

		__disable_irq(); // copy atomically so the UART ISR can't change it mid-copy
		if (cmdReady) {
			cmd = latestCmd;
			cmdReady = 0;
			haveCmd = 1;
		}
		uint32_t sinceCmd = HAL_GetTick() - lastCmdTick;
		__enable_irq();

		if (haveCmd) {
			lastRecover = cmd.recover; // so a 1 held from before a leak can't trigger recovery
			failsafeActive = 0;
			linkEstablished = 1;
			apply_command(&cmd);
			lastAppliedCmd = cmd;
		} else if ((sinceCmd > CMD_TIMEOUT_MS) && !failsafeActive) {
			failsafeActive = 1;
			apply_failsafe(); // link lost -> stop thrusters
		}

		proto_rx_keepalive(&huart4); // restarts reception if a re-arm ever failed

		/* ---- 3. Sensors + telemetry, every 20 ms ----------------------- */
		if ((HAL_GetTick() - lastTelemTick) >= TELEM_PERIOD_MS) {
			lastTelemTick = HAL_GetTick();
			read_sensors();
			send_telemetry();
		}

		/* ---- 3b. Sensor health, every 1 s ------------------------------ */
		// Re-initializes faulted sensors (clears their fault if they answer)
		// and checks the IMU hasn't silently reset itself.
		if ((HAL_GetTick() - lastSensorCheckTick) >= SENSOR_RETRY_MS) {
			lastSensorCheckTick = HAL_GetTick();
			sensor_health_check();
		}

		/* ---- 3c. Status LEDs (incl. 1 Hz blink, see status_led.h) ------ */
		status_led_update(leak_detected(), get_link_state(),
				sensorFaults != 0U);

		/* ---- 4. Debug print over SWO (uses the values read above) ------ */
		if ((HAL_GetTick() - Debug_lastPrint) > DEBUG_PRINT_MS) {
			if (!linkEstablished) {
				printf("Pi cmd: none received yet\r\n");
			} else {
				printf(
						"Pi cmd: M1=%d M2=%d M3=%d M4=%d M5=%d M6=%d, Tilt=%0.1f deg\r\n",
						lastAppliedCmd.motor[0], lastAppliedCmd.motor[1],
						lastAppliedCmd.motor[2], lastAppliedCmd.motor[3],
						lastAppliedCmd.motor[4], lastAppliedCmd.motor[5],
						lastAppliedCmd.tilt_dc / 10.0f);
				printf("Pi link: last cmd %lu ms ago%s\r\n",
						(unsigned long) sinceCmd,
						failsafeActive ? " [FAILSAFE: motors stopped]" : "");
				printf("Pi link: lost cmds (checksum) = %lu\r\n",
						(unsigned long) proto_get_cmd_checksum_errors());


			}
			printf("Euler: Heading = %0.2f \xB0, Roll = %0.2f \xB0, Pitch = %0.2f \xB0\r\n",
					imu_vec.x, imu_vec.y, imu_vec.z);
			printf("Battery Voltage = %0.3f V\n", battery_voltage);
			printf("Internal Temperature = %0.3f C\n", internal_temperature);
			printf("Water Temperature = %0.2f C\n", water_temperature);
			printf("Pressure = %0.1f kPa\n", pressure_kpa);
			printf("Depth = %0.2f\n", depth);
			print_faults(get_faults()); // same bits the Pi receives
			printf("\n");

			Debug_lastPrint = HAL_GetTick();
		}

	}

	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 180;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 2;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 180;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 3333;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 90;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 90;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 20000;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1500;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM8 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM8_Init(void) {

	/* USER CODE BEGIN TIM8_Init 0 */

	/* USER CODE END TIM8_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM8_Init 1 */

	/* USER CODE END TIM8_Init 1 */
	htim8.Instance = TIM8;
	htim8.Init.Prescaler = 180;
	htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim8.Init.Period = 3333;
	htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim8.Init.RepetitionCounter = 0;
	htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM8_Init 2 */

	/* USER CODE END TIM8_Init 2 */
	HAL_TIM_MspPostInit(&htim8);

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
	/* DMA1_Stream4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, RED_LED_Pin | GREEN_LED_Pin | YELLOW_LED_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin YELLOW_LED_Pin */
	GPIO_InitStruct.Pin = RED_LED_Pin | GREEN_LED_Pin | YELLOW_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int __io_putchar(int ch) {
	ITM_SendChar(ch); // Send the character to the SWO console (for debugging)
	return ch;
}


/**
 * @brief Takes a PWM pin away from its timer and holds it LOW as a plain GPIO
 *        output. ODR is written first so the pin never glitches HIGH.
 */
static void pwm_pin_force_low(GPIO_TypeDef *port, uint16_t pins) {
	GPIO_InitTypeDef g = { 0 };
	HAL_GPIO_WritePin(port, pins, GPIO_PIN_RESET);
	g.Pin = pins;
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(port, &g);
}


/**
 * @brief  Leak shutdown: stops everything except the status LEDs and UART4.
 *         Thruster and servo PWM stopped and pins held LOW, ADC + TIM2
 *         stopped, I2C1 de-initialized, break and ADC DMA IRQs disabled.
 *
 * @note   Main loop only (the break ISR just latches the LEAK flag).
 *         Runs once; later calls return immediately.
 *
 * @note   Undone by emergency_recover() (Pi sends recover = 1), not by
 *         clearing flags by hand, or leak detection stays disabled.
 */
void emergency_shutdown(void) {

	if (emergency_shutdown_done) {
		return;
	}
	emergency_shutdown_done = true;
	system_state_t = SYS_STATE_FAULT_LEAK;

	/* Thrusters: BKIN already cleared MOE, forcing all six outputs LOW in
	 * hardware. These calls also stop the TIM1/TIM8 counters, stop software
	 * (e.g. motors_reinit()) from re-enabling the outputs, and leave the HAL
	 * channel state ready for a later restart. Pins are then held LOW as GPIO
	 * so the ESCs see no pulses and stop. */
	HAL_TIM_PWM_Stop(MOTOR1_TIM, MOTOR1_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR2_TIM, MOTOR2_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR3_TIM, MOTOR3_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR4_TIM, MOTOR4_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR5_TIM, MOTOR5_CHANNEL);
	HAL_TIM_PWM_Stop(MOTOR6_TIM, MOTOR6_CHANNEL);
	pwm_pin_force_low(GPIOA, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11); // TIM1 CH1-4
	pwm_pin_force_low(GPIOC, GPIO_PIN_8 | GPIO_PIN_9);                              // TIM8 CH3-4

	/* Camera tilt servo (TIM3 CH3, PB0): stop PWM, hold LOW -> servo goes limp. */
	HAL_TIM_PWM_Stop(SERVO_TIM, SERVO_CHANNEL);
	pwm_pin_force_low(GPIOB, GPIO_PIN_0);

	/* Battery/thermistor ADC + its 1 kHz trigger timer. */
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_Base_Stop(&htim2);

	/* I2C sensors (IMU, Bar30, Celsius): the main loop no longer reads them. */
	HAL_I2C_DeInit(&hi2c1);

	/* Interrupts that are no longer needed. UART4, its DMA streams and
	 * SysTick (HAL_GetTick for the telemetry period) stay enabled. */
	HAL_NVIC_DisableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
	HAL_NVIC_DisableIRQ(DMA2_Stream0_IRQn);  // ADC1 DMA

	status_led_show_leak();     // LEAK = red only, solid
}

/**
 * @brief  Undoes emergency_shutdown() when the Pi sends recover = 1.
 *         Thrusters and servo restart at neutral, ADC and I2C sensors
 *         restart, leak detection is re-armed.
 *
 * @note   Main loop only. Refused (returns false, nothing changes) while
 *         either BKIN pin (PB12 = TIM1, PA6 = TIM8) still reads HIGH = wet.
 *
 * @retval true if recovered, false if the leak sensor is still wet.
 */
bool emergency_recover(void) {
	if (!emergency_shutdown_done) {
		return true; // nothing to undo
	}

	/* 1. Leak sensor must be dry. BKIN is active HIGH. */
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_SET
			|| HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET) {
		return false;
	}

	/* 2. Thrusters + servo: load 0 % throttle and centre tilt BEFORE the pins
	 *    go back to the timers, so the first pulse out is neutral. */
	apply_failsafe();               // motor1..6 = 0 % (neutral pulse)
	servo(0);                       // camera level
	HAL_TIM_MspPostInit(&htim1);    // PA8-PA11 back to TIM1 (undo pwm_pin_force_low)
	HAL_TIM_MspPostInit(&htim8);    // PC8-PC9 back to TIM8
	HAL_TIM_MspPostInit(&htim3);    // PB0 back to TIM3
	HAL_TIM_PWM_Start(MOTOR1_TIM, MOTOR1_CHANNEL); // also sets MOE on TIM1/TIM8
	HAL_TIM_PWM_Start(MOTOR2_TIM, MOTOR2_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR3_TIM, MOTOR3_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR4_TIM, MOTOR4_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR5_TIM, MOTOR5_CHANNEL);
	HAL_TIM_PWM_Start(MOTOR6_TIM, MOTOR6_CHANNEL);
	HAL_TIM_PWM_Start(SERVO_TIM, SERVO_CHANNEL);

	/* 3. Battery/thermistor ADC + its trigger timer. */
	HAL_NVIC_ClearPendingIRQ(DMA2_Stream0_IRQn);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc1_raw_buffer,
	ADC1_NUM_CHANNELS * ADC1_SAMPLES_PER_CHANNEL);
	HAL_TIM_Base_Start(&htim2);

	/* 4. I2C sensors: restart I2C1 and mark all three faulted, so
	 *    sensor_health_check() re-initializes them within SENSOR_RETRY_MS. */
	MX_I2C1_Init();
	celsius_converting = false;
	imuFailCount = 0;
	depthFailCount = 0;
	tempFailCount = 0;
	sensorFaults = SYS_STATE_FAULT_IMU | SYS_STATE_FAULT_DEPTH
			| SYS_STATE_FAULT_TEMP;

	/* 5. State. Thrusters stay at 0 % until the Pi sends a new command. */
	failsafeActive = 1;
	emergency_shutdown_done = false;
	system_state_t = SYS_STATE_FAULT_NONE;

	/* 6. Re-arm leak detection LAST, so a new leak can't be overwritten by
	 *    the state reset above. */
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_BREAK);
	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_BREAK);
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK);
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);
	HAL_NVIC_ClearPendingIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_ClearPendingIRQ(TIM8_BRK_TIM12_IRQn);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
	HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);

	return true;
}

/**
 * @brief  Break interrupt callback for advanced timers (TIM1/TIM8).
 *
 * Called by the HAL when a break event is detected on TIM1 or TIM8
 * (e.g. triggered by an external fault input, such as a leak sensor
 * tied to BKIN). Runs in interrupt context, so it only dispatches to
 * emergency_shutdown() rather than doing any blocking work itself.
 *
 * @param  htim  Pointer to the TIM handle that generated the break event.
 * @retval None
 */
void HAL_TIMEx_BreakCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1 || htim->Instance == TIM8) {
		system_state_t = SYS_STATE_FAULT_LEAK;
		__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_BREAK); // stop new break IRQs while faulted; must be re-enabled + cleared by any future reset routine
		__HAL_TIM_DISABLE_IT(&htim8, TIM_IT_BREAK);
	}

//	if (htim->Instance == TIM1) {			// for debugging only
//		printf("BREAK: TIM1\r\n");
//	} else if (htim->Instance == TIM8) {
//		printf("BREAK: TIM8\r\n");
//	}
	// do not anything in this function, especially blocking calls.
	// Use the emergency_shutdown function.
}

/**
 * @brief Calculates the battery voltage from the ADC reading.
 *
 * Averages the raw ADC samples for the battery channel, converts the
 * result to a voltage using the ADC reference voltage (3.3V) and
 * resolution (12-bit, 0-4095), then scales it by the voltage divider
 * ratio (BATTERY_MONITOR_R1 + BATTERY_MONITOR_R2) / BATTERY_MONITOR_R2
 * to recover the actual battery voltage.
 *
 * @return float Battery voltage in volts.
 */
float get_battery_voltage(void) {
	float raw_avg = util_average_channel(adc1_raw_buffer, ADC1_NUM_CHANNELS,
	ADC1_SAMPLES_PER_CHANNEL, CH_BATTERY);
	float ratio = (BATTERY_MONITOR_R1 + BATTERY_MONITOR_R2) / BATTERY_MONITOR_R2;
	return ratio * (raw_avg / 4095.0f) * 3.3f;
}

/**
 * @brief Calculates the NTC thermistor temperature from the ADC reading.
 *
 * Averages the raw ADC samples for the temperature channel, then converts
 * ADC count -> resistance -> temperature (see thermistor.c).
 *
 * @return float Temperature in degrees Celsius.
 */
float get_internal_temperature(void) {
	float raw_avg = util_average_channel(adc1_raw_buffer, ADC1_NUM_CHANNELS,
	ADC1_SAMPLES_PER_CHANNEL, CH_TEMPERATURE);
	float R = therm_get_ntc_resistance(raw_avg);
	return therm_get_temperature(R);
}

/**
 * @brief Refreshes the sensor variables.
 *
 * Every I2C read is checked. After SENSOR_FAIL_LIMIT failures in a row the
 * sensor is marked faulted in sensorFaults (yellow LED) and its last good
 * value is kept. A faulted sensor is skipped here, so a missing/failed I2C
 * device can't stall the loop with I2C timeouts every 20 ms. It is retried
 * once a second by sensor_health_check() instead.
 */
static void read_sensors(void) {
	battery_voltage = get_battery_voltage();
	internal_temperature = get_internal_temperature();

	if (!(sensorFaults & SYS_STATE_FAULT_DEPTH)) {
		bool ok = MS5837_Read(&bar30);
		if (ok) {
			depth = MS5837_GetDepth(&bar30);
			pressure_kpa = bar30.pressure_mbar / 10.0f; // absolute, 1 kPa = 10 mbar
		}
		sensor_read_result(SYS_STATE_FAULT_DEPTH, ok, &depthFailCount);
	}

	// Water temperature sensor (Celsius / TSYS01, separate device from the Bar30).
	// Skipped while SYS_STATE_FAULT_TEMP is active.
	//
	// Non-blocking: a conversion takes up to ~10 ms, and TSYS01_Read() would
	// stall the loop for that long every 20 ms. Instead, each call reads the
	// conversion started on the previous call (20 ms ago, so it's finished),
	// then starts the next one. water_temperature is therefore one telemetry
	// period (20 ms) old, which doesn't matter for water temperature.
	if (!(sensorFaults & SYS_STATE_FAULT_TEMP)) {
		bool ok = true;
		if (celsius_converting) {
			ok = TSYS01_ReadResult(&celsius);
			if (ok) {
				water_temperature = TSYS01_GetTemperature(&celsius);
			}
		}
		celsius_converting = TSYS01_StartConversion(&celsius);
		sensor_read_result(SYS_STATE_FAULT_TEMP, ok && celsius_converting,
				&tempFailCount);
	}

	if (!(sensorFaults & SYS_STATE_FAULT_IMU)) {
		bno055_vector_t v;
		bool ok = bno055_readVector(BNO055_VECTOR_EULER, &v);
		if (ok) {
			imu_vec = v;
		}
		sensor_read_result(SYS_STATE_FAULT_IMU, ok, &imuFailCount);
	}
}

/**
 * @brief Counts consecutive failed reads for one sensor and faults it after
 *        SENSOR_FAIL_LIMIT in a row. One good read resets the count, so a
 *        single glitch on the bus doesn't light the yellow LED.
 *
 * @param faultBit   SYS_STATE_FAULT_IMU / _DEPTH / _TEMP
 * @param ok         true if this read succeeded
 * @param failCount  that sensor's consecutive-failure counter
 */
static void sensor_read_result(uint8_t faultBit, bool ok, uint8_t *failCount) {
	if (ok) {
		*failCount = 0;
		return;
	}
	if (*failCount < SENSOR_FAIL_LIMIT) {
		(*failCount)++;
	}
	if (*failCount >= SENSOR_FAIL_LIMIT) {
		sensorFaults |= faultBit;
	}
}

/**
 * @brief Runs every SENSOR_RETRY_MS from the main loop.
 *
 *  - Each faulted sensor is re-initialized. If that works, its fault is
 *    cleared: yellow LED off, and its bit drops out of fault_flags so the
 *    Pi knows its data is valid again.
 *  - A working IMU has its operation mode checked. If the BNO055 browns out
 *    or resets, it still answers on I2C but comes back in CONFIG mode with
 *    frozen Euler angles, which a read check alone wouldn't catch.
 *
 * Blocking time: ~10 ms per Bar30/TSYS01 retry and ~35 ms for an IMU retry,
 * but only when the sensor answers. A missing sensor NACKs straight away.
 */
static void sensor_health_check(void) {
	if (sensorFaults & SYS_STATE_FAULT_DEPTH) {
		if (MS5837_Init(&bar30)) {
			sensorFaults &= (uint8_t) ~SYS_STATE_FAULT_DEPTH;
			depthFailCount = 0;
		}
	}

	if (sensorFaults & SYS_STATE_FAULT_TEMP) {
		if (celsius_start()) {
			sensorFaults &= (uint8_t) ~SYS_STATE_FAULT_TEMP;
			tempFailCount = 0;
		}
	}

	if (sensorFaults & SYS_STATE_FAULT_IMU) {
		if (imu_reconfigure()) {
			sensorFaults &= (uint8_t) ~SYS_STATE_FAULT_IMU;
			imuFailCount = 0;
		}
	} else {
		uint8_t mode = 0;
		if (!bno055_readData(BNO055_OPR_MODE, &mode, 1)) {
			sensor_read_result(SYS_STATE_FAULT_IMU, false, &imuFailCount);
		} else if (mode != BNO055_OPERATION_MODE_NDOF) {
			sensorFaults |= SYS_STATE_FAULT_IMU; // reset itself, reconfigure next check
		}
	}
}

/**
 * @brief Full IMU start-up, used once at boot. Includes the BNO055 reset
 *        (~700 ms), so it must NOT be used from the main loop.
 * @retval true if the IMU was found and configured.
 */
static bool imu_start(void) {
	if (!bno055_setup()) {
		return false;
	}
	// These needs to be here, since they write to device and can raise errors if called.
	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
	bno055_setCalibrationData(savedCalData);
	return true;
}

/**
 * @brief IMU recovery used by the main loop. Same as imu_start() but without
 *        the 700 ms reset, which would stall the loop longer than
 *        CMD_TIMEOUT_MS. (~35 ms when the IMU answers.)
 *
 * HAL_I2C_IsDeviceReady() is checked first. It is quiet when the IMU is
 * missing, whereas bno055_writeData() prints an error on every failure.
 *
 * @retval true if the IMU answered and is back in NDOF mode.
 */
static bool imu_reconfigure(void) {
	if (HAL_I2C_IsDeviceReady(&hi2c1, BNO055_I2C_ADDR << 1, 1, 5) != HAL_OK) {
		return false;
	}

	bno055_setPage(0);
	uint8_t id = 0;
	if (!bno055_readData(BNO055_CHIP_ID, &id, 1) || id != BNO055_ID) {
		return false;
	}

	bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
	bno055_setCalibrationData(savedCalData); // goes to CONFIG, writes, back to NDOF

	uint8_t mode = 0;
	return bno055_readData(BNO055_OPR_MODE, &mode, 1)
			&& (mode == BNO055_OPERATION_MODE_NDOF);
}

/**
 * @brief Initializes the Celsius (TSYS01) sensor. Used at boot and by the
 *        1 s retry. Resets the sensor, reads + checksums its PROM.
 * @retval true if the sensor answered and its PROM checksum was good.
 */
static bool celsius_start(void) {
	celsius_converting = false; // any conversion in progress is lost on reset
	return TSYS01_Init(&celsius);
}

/**
 * @brief Pi link state for the LEDs, from the command-timeout flags.
 */
static led_link_state_t get_link_state(void) {
	if (!linkEstablished) {
		return LED_LINK_WAITING;
	}
	if (failsafeActive) {
		return LED_LINK_LOST;
	}
	return LED_LINK_CONNECTED;
}

/**
 * @brief true once a leak has been detected (set by the break ISR or the
 *        boot check). Stays true until emergency_recover() succeeds.
 */
static bool leak_detected(void) {
	return (system_state_t & SYS_STATE_FAULT_LEAK) != 0U;
}

/**
 * @brief Faults active right now. Sent to the Pi as fault_flags and shown
 *        in the debug print. Every bit clears when its fault recovers,
 *        except LEAK:
 *          LEAK         - leak detected (stays set until recover command)
 *          IMU/DEPTH/TEMP - sensor currently faulted (sensorFaults)
 *          UART         - Pi link currently lost (command timeout after the
 *                         link was up; clears when commands resume)
 */
static uint8_t get_faults(void) {
	uint8_t faults = sensorFaults;
	if (leak_detected()) {
		faults |= SYS_STATE_FAULT_LEAK;
	}
	if (get_link_state() == LED_LINK_LOST) {
		faults |= SYS_STATE_FAULT_UART;
	}
	return faults;
}

/**
 * @brief Converts a float to an unsigned x100 integer for the telemetry frame.
 *
 * Most telemetry values are sent as whole numbers scaled by 100, so two
 * decimal places survive without sending a float over UART. The Pi divides
 * by 100.0 to get the real value back.
 *
 * Used for the unsigned fields: battery_x100, heading_x100.
 *
 * @note  Clamped to the uint16_t range: anything below 0.00 is sent as 0,
 *        anything above 655.35 is sent as 655.35. (Without the clamp, an
 *        out-of-range float -> integer cast is undefined behaviour in C.)
 *
 * @param  v  Value in real units, e.g. 15.84 (V), 359.94 (deg).
 * @return    v x 100, rounded, e.g. 15.84 -> 1584.
 */
static uint16_t to_u16_x100(float v) {
	float s = v * 100.0f + 0.5f;
	if (s <= 0.0f) {
		return 0U;
	}
	if (s >= (float) UINT16_MAX) {
		return UINT16_MAX;
	}
	return (uint16_t) s;
}

/**
 * @brief Converts a float to an unsigned x10 integer for the telemetry frame.
 *
 * Same as to_u16_x100() but scaled by 10. Used only for pressure_x10, since
 * pressure in kPa x100 would overflow 16 bits (Bar30 reads up to ~3000 kPa).
 * 0.1 kPa steps = 100 Pa ~= 1 cm of water, same resolution as depth_x100.
 *
 * @note  Clamped to 0.0 .. 6553.5.
 *
 * @param  v  Value in real units, e.g. 101.33 (kPa).
 * @return    v x 10, rounded, e.g. 101.33 -> 1013.
 */
static uint16_t to_u16_x10(float v) {
	float s = v * 10.0f + 0.5f;
	if (s <= 0.0f) {
		return 0U;
	}
	if (s >= (float) UINT16_MAX) {
		return UINT16_MAX;
	}
	return (uint16_t) s;
}

/**
 * @brief Converts a float to a signed x100 integer for the telemetry frame.
 *
 * Same idea as to_u16_x100(), but for values that can be negative. Rounding
 * must go AWAY from zero on both sides, so the 0.5 is added for positive
 * values and subtracted for negative ones.
 *
 * Used for the signed fields: depth_x100, water_temp_x100,
 * inside_temp_x100, roll_x100, pitch_x100. (depth is signed because the
 * Bar30 can read slightly negative at the surface.)
 *
 * @note  Clamped to the int16_t range: -327.68 to 327.67. Enough for depth
 *        (Bar30 is rated to 300 m), temperatures, and roll/pitch (+-180 deg).
 *
 * @param  v  Value in real units, e.g. -5.27 (degC), -45.03 (deg).
 * @return    v x 100, rounded, e.g. -5.27 -> -527.
 */
static int16_t to_i16_x100(float v) {
	float s = v * 100.0f;
	s = (s >= 0.0f) ? (s + 0.5f) : (s - 0.5f);
	if (s >= (float) INT16_MAX) {
		return INT16_MAX;
	}
	if (s <= (float) INT16_MIN) {
		return INT16_MIN;
	}
	return (int16_t) s;
}

/**
 * @brief Applies a command received from the Pi.
 *
 * motor[0..5] in the frame drive motor1()..motor6() from motors.h (those
 * functions clamp their own input). Thrusters are skipped while a leak is
 * latched: TIM1/TIM8 BKIN has already cut their PWM outputs in hardware.
 */
static void apply_command(const cmd_data_t *cmd) {
	if (!leak_detected()) {
		motor1(cmd->motor[0]);
		motor2(cmd->motor[1]);
		motor3(cmd->motor[2]);
		motor4(cmd->motor[3]);
		motor5(cmd->motor[4]);
		motor6(cmd->motor[5]);


		// Camera tilt (deci-degrees, e.g. -155 = -15.5 deg). servo() clamps
		// its own input. During a leak TIM3 is stopped by emergency_shutdown().
		servo(cmd->tilt_dc);
	}
}

/**
 * @brief Command timeout: stop all thrusters (camera tilt left where it is).
 *
 * Uses motorN(0) rather than motors_reinit(): motors_reinit() also re-enables
 * MOE and the break interrupt, which would undo a leak shutdown.
 *
 * The UART fault bit isn't stored here: get_faults() works it out from the
 * link state, so it is set while the link is lost and clears when commands
 * resume. (Not set while still waiting for the Pi to finish booting.)
 */
static void apply_failsafe(void) {
	motor1(0);
	motor2(0);
	motor3(0);
	motor4(0);
	motor5(0);
	motor6(0);
}

/**
 * @brief Prints the current faults by name over SWO, e.g.
 *        "Faults: 0x09 [ LEAK TEMP ]" or "Faults: none".
 *
 * Names match SystemState_t (and FAULT_NAMES in UART_DMA.py on the Pi).
 */
static void print_faults(uint8_t faults) {
	if (faults == SYS_STATE_FAULT_NONE) {
		printf("Faults: none\r\n");
		return;
	}
	printf("Faults: 0x%02X [", faults);
	if (faults & SYS_STATE_FAULT_LEAK)  printf(" LEAK");
	if (faults & SYS_STATE_FAULT_IMU)   printf(" IMU");
	if (faults & SYS_STATE_FAULT_DEPTH) printf(" DEPTH");
	if (faults & SYS_STATE_FAULT_TEMP)  printf(" TEMP");
	if (faults & SYS_STATE_FAULT_UART)  printf(" UART");
	printf(" ]\r\n");
}

/**
 * @brief Packs the current sensor values and sends one telemetry frame.
 *
 * leak is taken from the LEAK fault (the leak sensor drives TIM1/TIM8 BKIN,
 * which is what sets that fault). fault_flags are the faults active right
 * now (see get_faults()), so a sensor that recovers drops out of them.
 *
 * While a sensor is faulted its field keeps the last good value; the Pi
 * should ignore that field while the sensor's fault bit is set.
 */
static void send_telemetry(void) {
	telem_data_t data;

	data.pressure_x10 = to_u16_x10(pressure_kpa); // absolute, kPa x10 (= hPa)
	data.depth_x100 = to_i16_x100(depth);
	data.water_temp_x100 = to_i16_x100(water_temperature);
	data.battery_x100 = to_u16_x100(battery_voltage);
	data.inside_temp_x100 = to_i16_x100(internal_temperature);
	data.heading_x100 = to_u16_x100((float) imu_vec.x); // x = heading
	data.roll_x100 = to_i16_x100((float) imu_vec.y);    // y = roll
	data.pitch_x100 = to_i16_x100((float) imu_vec.z);   // z = pitch
	// leak = 1 if the LEAK fault bit is set, otherwise 0
	if (leak_detected()) {
		data.leak = 1U;
	} else {
		data.leak = 0U;
	}
	data.fault_flags = get_faults();

	proto_send_telem(&huart4, &data); // HAL_BUSY = previous frame still sending, skip this one
}

/**
 * @brief Called by HAL when UART4 reception pauses (idle line) or the DMA
 *        buffer fills. Hands the received bytes to the protocol parser.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
	if (huart->Instance == UART4) {
		cmd_data_t cmd;
		if (proto_rx_event_handler(huart, Size, &cmd)) {
			latestCmd = cmd;              // main loop copies it with IRQs off
			lastCmdTick = HAL_GetTick();
			cmdReady = 1;
		}
	}
}

/**
 * @brief Called by HAL on a UART error (noise, framing, overrun). The frame
 *        in progress is broken: drop it and restart reception.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		proto_rx_error_handler(huart);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
