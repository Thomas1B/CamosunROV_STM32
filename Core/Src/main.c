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
 * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "UART_DMA.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/**
 * @brief Top-level system state.
 *
 * Tracks whether the system is operating normally or has latched a fault.
 * Once a fault state is entered, it remains latched until explicitly
 * cleared/reset (e.g. via a fault-reset routine), even if the underlying
 * condition clears.
 */
typedef enum {
	SYS_STATE_FAULT_NONE = 0,        // Normal operation; no faults latched
	SYS_STATE_FAULT_LEAK = (1 << 0), // Fault latched: leak detected
	SYS_STATE_FAULT_IMU = (1 << 1), // Fault latched: IMU failure or invalid data
	SYS_STATE_FAULT_DEPTH = (1 << 2), // Fault latched: depth sensor failure or out-of-range reading
	SYS_STATE_FAULT_TEMP = (1 << 3), // Fault latched: temperature sensor failure or out-of-range reading
	SYS_STATE_FAULT_UART = (1 << 4), // Fault latched: UART communication failure
} SystemState_t;

/**
 * @brief ADC1 channel selection.
 *
 * Identifies which physical input is being read via ADC1.
 */
typedef enum {
	CH_BATTERY = 0, // battery channel
	CH_TEMPERATURE // thermistor channel
} ADC1_channel_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define SURFACE_PRESSURE 1050.25f // surface barometric pressure reference (units are either hPa or mbar)
#define WATER_DENSITY 1000.00f // density of water, change this depending on freshwater or saltwater: 1000 kg/m^3 or 1025 kg/m^3 (avg)

#define ADC1_NUM_CHANNELS 2 // number of channels use on ADC1
#define ADC1_SAMPLES_PER_CHANNEL 16 // number of samples to take / note: TIM2 (1KHz)

#define BATTERY_MONITOR_R1 100000.0f // R1 (100K) of voltage divider for battery monitor / MUST BE DEFINED AS FLOATS.
#define BATTERY_MONITOR_R2 18000.0f // R2 (18K) of voltage divider for battery monitor / MUST BE DEFINED AS FLOATS.

/* UART link to the Pi */
#define CMD_TIMEOUT_MS   150U  // no valid command for this long -> stop thrusters (~7 missed frames at 50 Hz)
#define TELEM_PERIOD_MS  20U   // read sensors + send telemetry every 20 ms (50 Hz)
#define DEBUG_PRINT_MS   1500U // SWO debug printf period (same as before)


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

float battery_voltage; // var to store battery voltage
float internal_temperature; // var to store internal temperature
float water_temperature; // var to store water temperature
float depth; // var to store depth

static bool emergency_shutdown_done = false;

/* UART link to the Pi */
static cmd_data_t latestCmd;               // newest valid command; written in UART ISR, copied in main with IRQs off
static volatile uint8_t cmdReady = 0;      // 1 = latestCmd holds a command main hasn't applied yet
static volatile uint32_t lastCmdTick = 0;  // HAL tick of the last valid command
static uint8_t failsafeActive = 1;         // 1 = thrusters stopped by the command timeout
static uint8_t linkEstablished = 0;        // 1 = at least one valid command received since boot

/*
 * Faults reported to the Pi. Kept separately from system_state_t because
 * emergency_shutdown() sets system_state_t = SYS_STATE_FAULT_LEAK, which
 * would erase any IMU/DEPTH/TEMP flags set at boot. Every loop merges
 * system_state_t into this, so nothing is lost. Only touched from the main
 * loop, so no interrupt race. A future fault-reset routine must clear this
 * too.
 */
static uint8_t reportedFaults = SYS_STATE_FAULT_NONE;

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
/* USER CODE BEGIN PFP */

void emergency_shutdown(void);
float get_battery_voltage(void);
float get_internal_temperature(void);
void bno055_check_connection(void);

static uint16_t to_u16_x10(float v);
static int16_t to_i16_x10(float v);
static void apply_command(const cmd_data_t *cmd);
static void apply_failsafe(void);
static void read_sensors(void);
static void send_telemetry(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

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
  /* USER CODE BEGIN 2 */
	/*
	 * These must be at the top to catch any leaks on startup.
	 */
	__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_BREAK); // <-- enable the break interrupt
	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_BREAK);

	if (system_state_t == SYS_STATE_FAULT_LEAK) {
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		emergency_shutdown();
		while (1)
			; // do nothing
	} else {
		// Flashing green to notify boot up starting
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);
		HAL_Delay(1000);

		// Setting BNO055 IMU device
		bno055_assignI2C(&hi2c1); // Assign the I2C handle to the BNO055 IMU
		if (bno055_setup()) {
			// These needs to be here, since they write to device and can raise errors if called.
			bno055_setOperationMode(BNO055_OPERATION_MODE_NDOF);
			bno055_setCalibrationData(savedCalData);
		} else {
			system_state_t += SYS_STATE_FAULT_IMU;
		}

		// Setting bar30 pressure sensor
		MS5837_SetI2C(&bar30, &hi2c1); // // Assign the I2C handle to the Bar30
		if (!MS5837_Init(&bar30)) {
			system_state_t += SYS_STATE_FAULT_DEPTH;
		}
		// These dont need don't need in an else statement. They just write to vars, not the device. (BNO055 is different)
		MS5837_SetSurfaceReference(&bar30, SURFACE_PRESSURE);
		MS5837_SetFluidDensity(&bar30, WATER_DENSITY);
		MS5837_SetOSR(&bar30, MS5837_OSR_256); // set the oversampling rate for the sensor
		MS5837_SetModel(&bar30, MS5837_MODEL_30BA); // set the sensor model
		bar30.secondOrderCalculation = false; // enable second order calculation, compensates for water below 15C.


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

		// Flashing green LED to show boot up sequence is done.
		for (int i = 0; i < 12; i++) {
			HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
			HAL_Delay(50);
		}
	}

	// TODO: add if statements to catch if sensors not working. no need to use while loops, just notify leds.
//	if (has_fault(system_state_t, SYS_STATE_FAULT_DEPTH)) {
//		HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, GPIO_PIN_SET);
//	}

	/* ---- UART link to the Pi start-up, runs once ---- */
	reportedFaults = (uint8_t) system_state_t; // keep the boot-time sensor faults
	proto_start_rx(&huart4);                   // start listening for commands
	uint32_t lastTelemTick = HAL_GetTick();
	cmd_data_t lastAppliedCmd = { 0 };       // newest command applied, kept for the debug print


	uint32_t Debug_lastPrint = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		/* ---- 1. Faults ------------------------------------------------ */
		// Merge in anything the break ISR (or boot code) has set.
		reportedFaults |= (uint8_t) system_state_t;

		if (reportedFaults & SYS_STATE_FAULT_LEAK) {
			emergency_shutdown(); // idempotent: the shutdown sequence only runs once
			// The Pi is told through telemetry (leak = 1, LEAK fault flag),
			// which keeps being sent below.
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

		/* ---- 4. Debug print over SWO (uses the values read above) ------ */
		if ((HAL_GetTick() - Debug_lastPrint) > DEBUG_PRINT_MS) {
			printf("Euler: Heading=%0.2f, Roll=%0.2f, Pitch=%0.2f\r\n",
					imu_vec.x, imu_vec.y, imu_vec.z);

			printf("Battery Voltage = %0.3f, ", battery_voltage);
			printf("Internal Temperature = %0.3f\n", internal_temperature);
			printf("Depth = %0.2f \n", depth);

			if (!linkEstablished) {
				printf("Pi cmd: none received yet\r\n");
			} else {
				printf("Pi cmd: M1=%d M2=%d M3=%d M4=%d M5=%d M6=%d, Tilt=%0.1f deg\r\n",
						lastAppliedCmd.motor[0], lastAppliedCmd.motor[1],
						lastAppliedCmd.motor[2], lastAppliedCmd.motor[3],
						lastAppliedCmd.motor[4], lastAppliedCmd.motor[5],
						lastAppliedCmd.tilt_dc / 10.0f);
				printf("Pi link: last cmd %lu ms ago%s, faults=0x%02X\r\n",
						(unsigned long) sinceCmd,
						failsafeActive ? " [FAILSAFE: motors stopped]" : "",
						reportedFaults);
			}

			Debug_lastPrint = HAL_GetTick();
		}

	}




  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
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
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 180;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3333;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 180;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 3333;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
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
static void MX_UART4_Init(void)
{

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
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_LED_Pin|GREEN_LED_Pin|YELLOW_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RED_LED_Pin GREEN_LED_Pin YELLOW_LED_Pin */
  GPIO_InitStruct.Pin = RED_LED_Pin|GREEN_LED_Pin|YELLOW_LED_Pin;
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
 * @brief  Latches the leak-fault state and shuts down non-essential
 *         peripherals: stops motors' PWM outputs are left running at their
 *         last duty cycle (see note below), stops the viewing light PWM,
 *         stops the battery/thermistor ADC+DMA sampling, and disables the
 *         TIM1/TIM8 break interrupts.
 *
 * @note   Only ever called from the main loop (never from ISR context) —
 *         HAL_TIMEx_BreakCallback() only sets system_state_t; this function
 *         does the actual (non-ISR-safe) shutdown work.
 *
 * @note   Idempotent via the emergency_shutdown_done flag: safe to call
 *         repeatedly (e.g. every main loop iteration while faulted) without
 *         re-running the shutdown sequence.
 *
 * @note   Disabling TIM_IT_BREAK here means a second leak event will NOT
 *         generate another interrupt while faulted. Any future fault-clear
 *         / reset routine MUST re-enable (and clear the pending flag on)
 *         TIM_IT_BREAK for htim1 and htim8, and must reset
 *         emergency_shutdown_done = false, or the system will silently stop
 *         detecting leaks after a reset.
 *
 * @retval None
 */
void emergency_shutdown(void) {

	// No need to stop Motor TIMs or PWMs. TIM1 and TIM8 BKIN handle that automatically.

	if (emergency_shutdown_done) {
		return;
	}
	emergency_shutdown_done = true;
	system_state_t = SYS_STATE_FAULT_LEAK;

	HAL_ADC_Stop_DMA(&hadc1);   // stop battery/thermistor ADC sampling
	HAL_TIM_Base_Stop(&htim2);  // stop the ADC trigger timer
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);

	// TODO: decide to whether to shutdown I2C communication...
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
	return  ratio * (raw_avg / 4095.0f) * 3.3f;
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
 * A sensor whose fault is latched is skipped, so a missing/failed I2C device
 * can't stall the loop with I2C timeouts every 20 ms.
 */
static void read_sensors(void) {
	battery_voltage = get_battery_voltage();
	internal_temperature = get_internal_temperature();

	if (!(reportedFaults & SYS_STATE_FAULT_DEPTH)) {
		MS5837_Read(&bar30);
		depth = MS5837_GetDepth(&bar30);
	}

	// Water temperature sensor (separate device from the Bar30).
	// Skipped if SYS_STATE_FAULT_TEMP is latched -- set that flag in your
	// init code if this sensor fails to start.
	if (!(reportedFaults & SYS_STATE_FAULT_TEMP)) {
		// TODO: water_temperature = <your water temperature sensor read>;
	}

	if (!(reportedFaults & SYS_STATE_FAULT_IMU)) {
		imu_vec = bno055_getVectorEuler();
	}
}


/**
 * @brief Converts a float to an unsigned x10 integer ("deci-units") for the
 *        telemetry frame.
 *
 * Every telemetry value is sent as a whole number scaled by 10, so one
 * decimal place survives without sending a float over UART. The Pi divides
 * by 10.0 to get the real value back.
 *
 * Used for the unsigned fields: depth_dm, battery_dv, heading_dc.
 *
 * @note  No clamping. v must be 0.0 to 6553.5 to fit a uint16_t. Only for
 *        values that are never negative -- adding +0.5 rounds the wrong way
 *        for negative numbers (use to_i16_x10() for those).
 *
 * @param  v  Value in real units, e.g. 299.5 (m), 15.8 (V), 359.9 (deg).
 * @return    v x 10, rounded, e.g. 299.5 -> 2995.
 */
static uint16_t to_u16_x10(float v) {
	return (uint16_t) (v * 10.0f + 0.5f);
}

/**
 * @brief Converts a float to a signed x10 integer ("deci-units") for the
 *        telemetry frame.
 *
 * Same idea as to_u16_x10(), but for values that can be negative. Rounding
 * must go AWAY from zero on both sides, so the 0.5 is added for positive
 * values and subtracted for negative ones.
 *
 * Used for the signed fields: water_temp_dc, inside_temp_dc, roll_dc,
 * pitch_dc.
 *
 * @note  No clamping. v must be -3276.8 to 3276.7 to fit an int16_t.
 *
 * @param  v  Value in real units, e.g. -5.2 (degC), -45.0 (deg).
 * @return    v x 10, rounded, e.g. -5.2 -> -52.
 */
static int16_t to_i16_x10(float v) {
	float s = v * 10.0f;
	return (int16_t) ((s >= 0.0f) ? (s + 0.5f) : (s - 0.5f));
}


/**
 * @brief Applies a command received from the Pi.
 *
 * motor[0..5] in the frame drive motor1()..motor6() from motors.h (those
 * functions clamp their own input). Thrusters are skipped while a leak is
 * latched: TIM1/TIM8 BKIN has already cut their PWM outputs in hardware.
 */
static void apply_command(const cmd_data_t *cmd) {
	if (!(reportedFaults & SYS_STATE_FAULT_LEAK)) {
		motor1(cmd->motor[0]);
		motor2(cmd->motor[1]);
		motor3(cmd->motor[2]);
		motor4(cmd->motor[3]);
		motor5(cmd->motor[4]);
		motor6(cmd->motor[5]);
	}

	// TODO: camera tilt -- call your function from motors.c here with
	//       cmd->tilt_dc (deci-degrees, e.g. -155 = -15.5 deg).
}


/**
 * @brief Command timeout: stop all thrusters (camera tilt left where it is).
 *
 * Uses motorN(0) rather than motors_reinit(): motors_reinit() also re-enables
 * MOE and the break interrupt, which would undo a leak shutdown.
 *
 * Latches SYS_STATE_FAULT_UART, but only if the link was up before -- not
 * while waiting for the Pi to finish booting. Because it is latched, the Pi
 * still sees it in telemetry once the link recovers.
 */
static void apply_failsafe(void) {
	motor1(0);
	motor2(0);
	motor3(0);
	motor4(0);
	motor5(0);
	motor6(0);

	if (linkEstablished) {
		reportedFaults |= SYS_STATE_FAULT_UART;
	}
}


/**
 * @brief Packs the current sensor values and sends one telemetry frame.
 *
 * leak is taken from the latched LEAK fault (the leak sensor drives
 * TIM1/TIM8 BKIN, which is what sets that fault).
 */
static void send_telemetry(void) {
	telem_data_t data;

	data.depth_dm = to_i16_x10(depth);
	data.water_temp_dc = to_i16_x10(water_temperature);
	data.battery_dv = to_u16_x10(battery_voltage);
	data.inside_temp_dc = to_i16_x10(internal_temperature);
	data.heading_dc = to_u16_x10((float) imu_vec.x); // x = heading
	data.roll_dc = to_i16_x10((float) imu_vec.y);    // y = roll
	data.pitch_dc = to_i16_x10((float) imu_vec.z);   // z = pitch
	// leak = 1 if the LEAK fault bit is set, otherwise 0
	if (reportedFaults & SYS_STATE_FAULT_LEAK) {
		data.leak = 1U;
	} else {
		data.leak = 0U;
	}
	data.fault_flags = reportedFaults;

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
void Error_Handler(void)
{
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
