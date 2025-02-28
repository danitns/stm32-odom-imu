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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t tx_buffer[1000];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

static void platform_delay(uint32_t ms);
static void update_filter(float *q0, float *q1, float *q2, float *q3,
		float_t *angular_rate_dps, float_t *acceleration_mg,
		float_t *magnetic_field_mgauss);
static uint8_t init_imu_regs(stmdev_ctx_t *dev_ctx_imu,
		stmdev_ctx_t *dev_ctx_mag, lsm9ds1_id_t *whoamI, uint8_t *rst);

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
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	/* USER CODE BEGIN 2 */

	uint32_t timestamp = 0;
	uint32_t timestamp_print = 0;

	static float_t acceleration_mg[3];
	static float_t angular_rate_dps[3];
	static float_t magnetic_field_mgauss[3];
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

	// AHRS
	float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
	// AHRS END

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		/* Read device status register */

		if ((HAL_GetTick() - timestamp) > (1000 / FILTER_UPDATE_RATE_HZ)) {
			timestamp = HAL_GetTick();
			lsm9ds1_dev_status_get(&dev_ctx_mag, &dev_ctx_imu, &reg);
			read_accel_gyro(&dev_ctx_imu, reg, acceleration_mg,
					angular_rate_dps);
			read_magnetometer(&dev_ctx_mag, reg, magnetic_field_mgauss);

			calibrate_data(angular_rate_dps, acceleration_mg,
					magnetic_field_mgauss);

			update_filter(&q0, &q1, &q2, &q3, angular_rate_dps, acceleration_mg,
					magnetic_field_mgauss);

		}
		if ((HAL_GetTick() - timestamp_print) > (1000 / PRINT_RATE_HZ)) {
			timestamp_print = HAL_GetTick();
			snprintf((char*) tx_buffer, sizeof(tx_buffer),
					"%4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f %4.4f\r\n",
					q0, q1, q2, q3, acceleration_mg[0], acceleration_mg[1],
					acceleration_mg[2], angular_rate_dps[0],
					angular_rate_dps[1], angular_rate_dps[2]);
			HAL_UART_Transmit(&huart2, tx_buffer, sizeof(tx_buffer) - 1, HAL_MAX_DELAY);
		}

		/* USER CODE BEGIN 3 */
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

static void update_filter(float *q0, float *q1, float *q2, float *q3,
		float_t *angular_rate_dps, float_t *acceleration_mg,
		float_t *magnetic_field_mgauss) {
	updateAHRS(angular_rate_dps[0], angular_rate_dps[1], angular_rate_dps[2],
			acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
			magnetic_field_mgauss[0], magnetic_field_mgauss[1],
			magnetic_field_mgauss[2], 1.f / FILTER_UPDATE_RATE_HZ, q0, q1, q2,
			q3);
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
