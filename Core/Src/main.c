/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//static uint8_t tx_buffer[1000];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM17_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */

static void platform_delay(uint32_t ms);
static void update_filter();
static uint8_t init_imu_regs(stmdev_ctx_t *dev_ctx_imu,
		stmdev_ctx_t *dev_ctx_mag, lsm9ds1_id_t *whoamI, uint8_t *rst);
static void read_encoder_values();
static void set_motor_speed(int motor, int speed);
static void set_motor_speeds(int leftSpeed, int rightSpeed);
static void parse_command(char *command);
void process_i2c_request();
void update_imu_buffer();
static void update_pid();
static void do_pid(SetPointInfo *p);
static void reset_pid();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// SERIAL COMMUNICATION
volatile uint8_t rx_data; // Single byte buffer for UART reception
volatile uint8_t rx_buffer[BUFFER_SIZE_CMD]; // Command buffer
volatile uint8_t rx_index = 0; // Index for received characters
volatile uint8_t command_ready = 0; // Flag to indicate a complete command

// ENCODERS AND MOTORS
int current_servo_pos_us = MID_TURN_US;
volatile uint32_t timestamp_motor_command = 0;
volatile int16_t left_encoder_value = 0;
volatile int16_t right_encoder_value = 0;

// AHRS
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float_t acceleration_mg[3];
static float_t angular_rate_dps[3];
static float_t magnetic_field_mgauss[3];
// AHRS END

// IMU I2C comms
volatile uint8_t i2c_rx_buffer[1];  // Separate buffer for I2C reception
uint8_t imu_data_buffer[I2C_BUFFER_SIZE];

// IMU I2C comms end

// PID
const int PID_INTERVAL = 1000 / PID_RATE;
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.05;
float Ko = 1.0;
unsigned char moving = 0;

SetPointInfo leftPID, rightPID;

unsigned long lastPIDTime = 0;
// PID END

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM17_Init();
	MX_I2C2_Init();
	/* USER CODE BEGIN 2 */

	uint32_t timestamp = 0;
	//uint32_t timestamp_print = 0;
	uint32_t nextPID = PID_INTERVAL;

	static lsm9ds1_id_t whoamI;
	static lsm9ds1_status_t reg;
	static uint8_t rst;
	static sensbus_t mag_bus = { &hi2c1,
	LSM9DS1_MAG_I2C_ADD_H, 0, 0 };
	static sensbus_t imu_bus = { &hi2c1,
	LSM9DS1_IMU_I2C_ADD_H, 0, 0 };

	stmdev_ctx_t dev_ctx_imu;
	stmdev_ctx_t dev_ctx_mag;
	/* Initialize inertial sensors (IMU) driver interface */
	dev_ctx_imu.write_reg = platform_write_imu;
	dev_ctx_imu.read_reg = platform_read_imu;
	dev_ctx_imu.handle = (void*) &imu_bus;
	/* Initialize magnetic sensors driver interface */
	dev_ctx_mag.write_reg = platform_write_mag;
	dev_ctx_mag.read_reg = platform_read_mag;
	dev_ctx_mag.handle = (void*) &mag_bus;
	/* Wait sensor boot time */
	platform_delay(BOOT_TIME);

	if (init_imu_regs(&dev_ctx_imu, &dev_ctx_mag, &whoamI, &rst) == 1) {
		uint8_t msg1[] = "FAILED!\n";
		HAL_UART_Transmit(&huart2, msg1, sizeof(msg1) - 1, HAL_MAX_DELAY);
		while (1) {
			/* manage here device not found */
		}
	}

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);
	//HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*)&i2c_rx_buffer, 1);
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
		if (command_ready) {
			command_ready = 0;
			parse_command((char*) rx_buffer);
		}

		if (HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY) {
			// If the slave is ready and a request is made, transmit data
			HAL_I2C_Slave_Transmit(&hi2c2, imu_data_buffer, 20 * sizeof(uint8_t), 100); // Timeout for non-blocking
		}

		if ((HAL_GetTick() - timestamp) > (1000 / FILTER_UPDATE_RATE_HZ)) {
			timestamp = HAL_GetTick();
			lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
			read_accel_gyro(&dev_ctx_imu, reg, acceleration_mg,
					angular_rate_dps);
			read_magnetometer(&dev_ctx_mag, reg, magnetic_field_mgauss);

			calibrate_data(angular_rate_dps, acceleration_mg,
					magnetic_field_mgauss);

			update_filter();
			update_imu_buffer();

			// motors
			read_encoder_values();

		}

		if (HAL_GetTick() > nextPID) {
			update_pid();
			nextPID = HAL_GetTick() +  PID_INTERVAL;
		}

		if ((HAL_GetTick() - timestamp_motor_command) > AUTO_STOP_INTERVAL) {
			set_motor_speeds(0, 0);
			moving = 0;
		}

//		if ((HAL_GetTick() - timestamp_print) > (1000 / PRINT_RATE_HZ)) {
//			timestamp_print = HAL_GetTick();
//
//			snprintf((char*) tx_buffer, sizeof(tx_buffer),
//					"%4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f\r\n",
//					q0, q1, q2, q3, acceleration_mg[0], acceleration_mg[1],
//					acceleration_mg[2], angular_rate_dps[0],
//					angular_rate_dps[1], angular_rate_dps[2]);
//
//			HAL_UART_Transmit(&huart2, tx_buffer,
//					strlen((char const*) tx_buffer),
//					HAL_MAX_DELAY);
//		}

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
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 10;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
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
	hi2c1.Init.Timing = 0x10D19CE4;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x10D19CE4;
	hi2c2.Init.OwnAddress1 = 40;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_ENABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_3);
	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 4294967295;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
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
	htim3.Init.Prescaler = 79;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 99;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 79;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 19999;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief TIM17 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM17_Init(void) {

	/* USER CODE BEGIN TIM17_Init 0 */

	/* USER CODE END TIM17_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM17_Init 1 */

	/* USER CODE END TIM17_Init 1 */
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 79;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 19999;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim17) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM17_Init 2 */
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
	/* USER CODE END TIM17_Init 2 */
	HAL_TIM_MspPostInit(&htim17);

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

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
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LD2_Pin */
	GPIO_InitStruct.Pin = LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
	HAL_Delay(ms);
}

static uint8_t init_imu_regs(stmdev_ctx_t *dev_ctx_imu,
		stmdev_ctx_t *dev_ctx_mag, lsm9ds1_id_t *whoamI, uint8_t *rst) {
	/* Check device ID */
	lsm9ds1_dev_id_get(dev_ctx_mag, dev_ctx_imu, whoamI);

	if ((*whoamI).imu != LSM9DS1_IMU_ID || (*whoamI).mag != LSM9DS1_MAG_ID) {
		return 1;
	}

	/* Restore default configuration */
	lsm9ds1_dev_reset_set(dev_ctx_mag, dev_ctx_imu, PROPERTY_ENABLE);

	do {
		lsm9ds1_dev_reset_get(dev_ctx_mag, dev_ctx_imu, rst);
	} while ((*rst));

	/* Enable Block Data Update */
	lsm9ds1_block_data_update_set(dev_ctx_mag, dev_ctx_imu, PROPERTY_ENABLE);
	/* Set full scale */
	lsm9ds1_xl_full_scale_set(dev_ctx_imu, LSM9DS1_4g);
	lsm9ds1_gy_full_scale_set(dev_ctx_imu, LSM9DS1_2000dps);
	lsm9ds1_mag_full_scale_set(dev_ctx_mag, LSM9DS1_16Ga);
	/* Configure filtering chain - See datasheet for filtering chain details */
	/* Accelerometer filtering chain */
	lsm9ds1_xl_filter_aalias_bandwidth_set(dev_ctx_imu, LSM9DS1_AUTO);
	lsm9ds1_xl_filter_lp_bandwidth_set(dev_ctx_imu, LSM9DS1_LP_ODR_DIV_50);
	lsm9ds1_xl_filter_out_path_set(dev_ctx_imu, LSM9DS1_LP_OUT);
	/* Gyroscope filtering chain */
	lsm9ds1_gy_filter_lp_bandwidth_set(dev_ctx_imu, LSM9DS1_LP_MEDIUM);
	lsm9ds1_gy_filter_hp_bandwidth_set(dev_ctx_imu, LSM9DS1_HP_ULTRA_LIGHT);
	lsm9ds1_gy_filter_out_path_set(dev_ctx_imu, LSM9DS1_LPF1_LPF2_OUT);
	/* Set Output Data Rate / Power mode */
	lsm9ds1_imu_data_rate_set(dev_ctx_imu, LSM9DS1_IMU_119Hz);
	lsm9ds1_mag_data_rate_set(dev_ctx_mag, LSM9DS1_MAG_UHP_155Hz);

	return 0;
}

static void update_filter() {
	updateAHRS(angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2],
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
			magnetic_field_mgauss[0], magnetic_field_mgauss[1],
			magnetic_field_mgauss[2], 1.f / FILTER_UPDATE_RATE_HZ, &q0, &q1,
			&q2, &q3);
}

static void read_encoder_values() {
	left_encoder_value = __HAL_TIM_GET_COUNTER(&htim2);
	right_encoder_value = __HAL_TIM_GET_COUNTER(&htim1);
}

static void set_motor_speed(int motor, int speed) {
	uint8_t reverse = 0;

	if (speed < 0) {
		speed = -speed;
		reverse = 1;
	}
	if (speed > 100) {
		speed = 100;
	}

	if (motor == LEFT) {
		if (reverse == 0) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);  // Forward
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);    // Stop Backward
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
		}
	} else { // RIGHT motor
		if (reverse == 0) {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, speed);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
		} else {
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, speed);
		}
	}
}

static void set_motor_speeds(int leftSpeed, int rightSpeed) {
	set_motor_speed(LEFT, leftSpeed);
	set_motor_speed(RIGHT, rightSpeed);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2) {
		if (rx_data == 13) { // End of command
			rx_buffer[rx_index] = '\0'; // Null-terminate the string
			rx_index = 0;
			command_ready = 1; // Set flag to process command
		} else {
			if (rx_index < BUFFER_SIZE_CMD - 1) {
				rx_buffer[rx_index++] = rx_data;
			}
		}

		// Restart reception for the next byte
		HAL_UART_Receive_IT(&huart2, (uint8_t*) &rx_data, 1);
	}
}

//void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c2.Instance) {
//    	update_imu_buffer();
//        if(i2c_rx_buffer[0] == 0x02) {
//        	HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Data Received\n", 18, HAL_MAX_DELAY);
//        	HAL_I2C_Slave_Transmit_IT(&hi2c2, imu_data_buffer, I2C_BUFFER_SIZE);
//        } else{
//        	HAL_UART_Transmit(&huart2, (uint8_t*)"UNK Data Received\n", 18, HAL_MAX_DELAY);
//        	HAL_I2C_Slave_Transmit_IT(&hi2c2, imu_data_buffer, I2C_BUFFER_SIZE);
//        }
//        HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*)i2c_rx_buffer, 1);
//    }
//}

//void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c2.Instance) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Read Request\n", 17, HAL_MAX_DELAY);
//        HAL_I2C_Slave_Transmit_IT(&hi2c2, imu_data_buffer, I2C_BUFFER_SIZE);
//    }
//}
//
//void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c2.Instance) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Transmit Done\n", 18, HAL_MAX_DELAY);
//        // Ready to listen for the next request
//        HAL_I2C_EnableListen_IT(&hi2c2);
//    }
//}
//
//void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
//    if (hi2c->Instance == hi2c2.Instance) {
//        HAL_UART_Transmit(&huart2, (uint8_t*)"I2C Error!\n", 10, HAL_MAX_DELAY);
//        HAL_I2C_EnableListen_IT(&hi2c2);  // Restart listening mode
//    }
//}

static void parse_command(char *command) {
	char cmd;
	int values[MAX_ARGS] = { 0 };
	int argCount = 0;
	char response[50]; // Buffer for response message

	// Extract command and arguments
	char *token = strtok(command, " "); // Get first token (command)
	if (token != NULL) {
		cmd = token[0]; // First character is the command
		while ((token = strtok(NULL, " ")) != NULL && argCount < MAX_ARGS) {
			values[argCount++] = atoi(token); // Convert argument to hint
		}
	}

	// Execute action based on command
	switch (cmd) {
	case MOTOR_SPEEDS: // Example: Move motors "m 100 200"
		if (argCount >= 3) {
			if (values[0] == 0 && values[1] == 0) {
				set_motor_speeds(0, 0);
				reset_pid();
				moving = 0;
			} else
				moving = 1;
			leftPID.TargetTicksPerFrame = values[0];
			rightPID.TargetTicksPerFrame = values[1];
			set_desired_position(values[2], &current_servo_pos_us);

			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, current_servo_pos_us);
			timestamp_motor_command = HAL_GetTick();
			snprintf(response, sizeof(response), "OK %d %d %d\r\n", values[0],
					values[1], current_servo_pos_us);
		}
		break;

	case SERVO_WRITE: // Example: Set servo position "s 45"
		if (argCount >= 1) {
			set_desired_position(values[0], &current_servo_pos_us);
			__HAL_TIM_SET_COMPARE(&htim17, TIM_CHANNEL_1, current_servo_pos_us);
			snprintf(response, sizeof(response), "OK %d \r\n",
					current_servo_pos_us);
		}
		break;

	case READ_ENCODERS: // Example: Set servo position "s 45"
		read_encoder_values();
		int servo_pos = get_position_in_rads(current_servo_pos_us);
		snprintf(response, sizeof(response), "%d %d %d \r\n",
				left_encoder_value, right_encoder_value, servo_pos);
		break;

	case 'i': // Example: Set servo position "s 45"
		snprintf(response, sizeof(response), "%4.4f %4.4f %4.4f %4.4f \r\n", q0,
				q1, q2, q3);
		break;

	default:
		snprintf(response, sizeof(response), "Unknown command %c \r\n", cmd);
		break;
	}
	HAL_UART_Transmit(&huart2, (uint8_t*) response, strlen(response),
	HAL_MAX_DELAY);
}

void update_imu_buffer() {
	int16_t ax = (int16_t) (acceleration_mg[0] * 1000);
	int16_t ay = (int16_t) (acceleration_mg[1] * 1000);
	int16_t az = (int16_t) (acceleration_mg[2] * 1000);

	int16_t gx = (int16_t) (angular_rate_dps[0] * 100);
	int16_t gy = (int16_t) (angular_rate_dps[1] * 100);
	int16_t gz = (int16_t) (angular_rate_dps[2] * 100);

	int16_t q0_int = (int16_t) (q0 * 10000);
	int16_t q1_int = (int16_t) (q1 * 10000);
	int16_t q2_int = (int16_t) (q2 * 10000);
	int16_t q3_int = (int16_t) (q3 * 10000);

	memcpy(imu_data_buffer, &q0_int, sizeof(int16_t));
	memcpy(imu_data_buffer + 2, &q1_int, sizeof(int16_t));
	memcpy(imu_data_buffer + 4, &q2_int, sizeof(int16_t));
	memcpy(imu_data_buffer + 6, &q3_int, sizeof(int16_t));
	memcpy(imu_data_buffer + 8, &ax, sizeof(int16_t));
	memcpy(imu_data_buffer + 10, &ay, sizeof(int16_t));
	memcpy(imu_data_buffer + 12, &az, sizeof(int16_t));
	memcpy(imu_data_buffer + 14, &gx, sizeof(int16_t));
	memcpy(imu_data_buffer + 16, &gy, sizeof(int16_t));
	memcpy(imu_data_buffer + 18, &gz, sizeof(int16_t));

}

static void reset_pid() {
	read_encoder_values();

	leftPID.TargetTicksPerFrame = 0.0;
	leftPID.Encoder = left_encoder_value;
	leftPID.PrevEnc = leftPID.Encoder;
	leftPID.output = 0;
	leftPID.PrevInput = 0;
	leftPID.ITerm = 0;

	rightPID.TargetTicksPerFrame = 0.0;
	rightPID.Encoder = right_encoder_value;
	rightPID.PrevEnc = rightPID.Encoder;
	rightPID.output = 0;
	rightPID.PrevInput = 0;
	rightPID.ITerm = 0;
}

static void do_pid(SetPointInfo *p) {
    long Perror;
    long output;
    int input;

    // Compute the error
    input = p->Encoder - p->PrevEnc;
    Perror = p->TargetTicksPerFrame - input;

    // Deadband: Ignore small errors to prevent jitter
    if (abs(Perror) < 2) Perror = 0;

    // Compute PID output
    output = (Kp * Perror - Kd * (input - p->PrevInput) + p->ITerm) / Ko;

    // Store the previous encoder value
    p->PrevEnc = p->Encoder;

    // Accumulate Integral error *or* Limit output to prevent windup
    if (output >= 100) {
        output = 100;
        p->ITerm = 0; // Prevent integral windup
    }
    else if (output <= -100) {
        output = -100;
        p->ITerm = 0; // Prevent integral windup
    }
    else {
        p->ITerm += Ki * Perror;  // Integral accumulation
    }

    p->output = output;
    p->PrevInput = input;
}


/* Read the encoder values and call the PID routine */
static void update_pid() {
	/* Read the encoders */
	read_encoder_values();
	leftPID.Encoder = left_encoder_value;
	rightPID.Encoder = right_encoder_value;

	/* If we're not moving there is nothing more to do */
	if (!moving) {
		/*
		 * Reset PIDs once, to prevent startup spikes,
		 * see http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-initialization/
		 * PrevInput is considered a good proxy to detect
		 * whether reset has already happened
		 */
		if (leftPID.PrevInput != 0 || rightPID.PrevInput != 0)
			reset_pid();
		return;
	}

	/* Compute PID update for each motor */
	do_pid(&rightPID);
	do_pid(&leftPID);

	/* Set the motor speeds accordingly */
	set_motor_speeds(leftPID.output, rightPID.output);
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

#ifdef  USE_FULL_ASSERT
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
