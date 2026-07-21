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
#include "icm20948.h"
#include "fft_processor.h"
#include "uart_output.h"
#include "app_config.h"
#include "signal_filter_config.h"
#ifdef APP_MAVLINK_ENABLED
#include "mavlink_service.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define REQUIRED_SENSOR_COMPONENTS APP_REQUIRED_SENSOR_COMPONENTS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
ICM20948 imu;
ImuSensor imu_sensor;
AccelerometerData accelerometer_raw_data;
AccelerometerData accelerometer_filtered_data;
GyroscopeData gyroscope_data;
MagnetometerData magnetometer_data;
ImuCalibrationData calibration_data;
FFT_Processor fft_raw_processor[APP_ACCEL_AXIS_COUNT];
FFT_Processor fft_handled_processor[APP_ACCEL_AXIS_COUNT];
FFT_Result fft_raw_result[APP_ACCEL_AXIS_COUNT];
FFT_Result fft_handled_result[APP_ACCEL_AXIS_COUNT];
uint8_t fft_output_counter;
uint8_t fft_ready_axes_mask;
SignalFilterChain accelerometer_filter[APP_ACCEL_AXIS_COUNT];
volatile uint32_t accelerometer_comparison_time_ms;
volatile float accelerometer_comparison_raw_value[APP_ACCEL_AXIS_COUNT];
volatile float accelerometer_comparison_handled_value[APP_ACCEL_AXIS_COUNT];

#define FFT_OUTPUT_DECIMATION 4
UART_Output uart_output;
#ifdef APP_MAVLINK_ENABLED
MAVLink_Service mavlink_service;
#endif

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
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
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_Delay(100);
	ICM20948_Create(&imu, &hspi2, IMU_CS_GPIO_Port, IMU_CS_Pin);
	ICM20948_Bind(&imu_sensor, &imu);
	for (uint8_t axis = 0U; axis < APP_ACCEL_AXIS_COUNT; axis++) {
		FFT_Processor_Init(&fft_raw_processor[axis],
				APP_ACCEL_FFT_REMOVE_MEAN);
		FFT_Processor_Init(&fft_handled_processor[axis],
				APP_ACCEL_FFT_REMOVE_MEAN);
		if (!SignalFilterChain_Init(&accelerometer_filter[axis],
				&accelerometer_filter_config)) {
			Error_Handler();
		}
	}
	UART_Output_Init(&uart_output, &huart1);
#ifdef APP_MAVLINK_ENABLED
	MAVLink_Service_Init(&mavlink_service, &huart2);
#endif
#if APP_UART_STATUS_STREAM_ENABLED
	UART_Output_Status(&uart_output, "initializing sensor components");
#endif

	if (ImuSensor_Init(&imu_sensor, REQUIRED_SENSOR_COMPONENTS)) {
#if APP_UART_STATUS_STREAM_ENABLED
		UART_Output_Status(&uart_output, "sensor components initialized");
#endif
		if (REQUIRED_SENSOR_COMPONENTS
				& (SENSOR_ACCELEROMETER | SENSOR_GYROSCOPE)) {
#if APP_UART_STATUS_STREAM_ENABLED
			UART_Output_Status(&uart_output, "calibrating");
#endif
			if (!ImuSensor_Calibrate(&imu_sensor, &calibration_data)) {
				UART_Output_Error(&uart_output, "calibration failed",
				REQUIRED_SENSOR_COMPONENTS, imu_sensor.initialized_components);
			} else {
#if APP_UART_STATUS_STREAM_ENABLED
				UART_Output_FilterStatus(&uart_output, "calibration complete",
						&accelerometer_filter_config);
				UART_Output_Status(&uart_output, "streaming");
#endif
				HAL_TIM_Base_Start_IT(&htim3);
			}
		} else {
#if APP_UART_STATUS_STREAM_ENABLED
			UART_Output_Status(&uart_output, "streaming");
#endif
			HAL_TIM_Base_Start_IT(&htim3);
		}
	} else {
		UART_Output_Error(&uart_output, "sensor initialization failed",
		REQUIRED_SENSOR_COMPONENTS, imu_sensor.initialized_components);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		UART_Output_Process(&uart_output);

		for (uint8_t axis = 0U; axis < APP_ACCEL_AXIS_COUNT; axis++) {
			uint8_t axis_mask = (uint8_t) (1U << axis);
			if ((fft_ready_axes_mask & axis_mask) == 0U
					&& fft_raw_processor[axis].ready
					&& fft_handled_processor[axis].ready) {
				uint8_t raw_fft_ready = FFT_Processor_Process(
						&fft_raw_processor[axis], &fft_raw_result[axis]);
				uint8_t handled_fft_ready = FFT_Processor_Process(
						&fft_handled_processor[axis],
						&fft_handled_result[axis]);
				if (raw_fft_ready && handled_fft_ready) {
					fft_ready_axes_mask |= axis_mask;
				}
			}
		}

		if (fft_ready_axes_mask == APP_ACCEL_ALL_AXES_MASK) {
			fft_ready_axes_mask = 0U;
#if APP_UART_LEGACY_IMU_STREAM_ENABLED
			UART_Output_SensorData(&uart_output,
					(imu_sensor.initialized_components & SENSOR_ACCELEROMETER) ?
							&accelerometer_filtered_data : 0,
					(imu_sensor.initialized_components & SENSOR_GYROSCOPE) ?
							&gyroscope_data : 0,
					(imu_sensor.initialized_components & SENSOR_MAGNETOMETER) ?
							&magnetometer_data : 0);
#endif
#if APP_UART_ACCEL_COMPARISON_STREAM_ENABLED
			uint32_t interrupt_state = __get_PRIMASK();
			uint32_t comparison_time_ms;
			float comparison_raw_value[APP_ACCEL_AXIS_COUNT];
			float comparison_handled_value[APP_ACCEL_AXIS_COUNT];

			__disable_irq();
			comparison_time_ms = accelerometer_comparison_time_ms;
			for (uint8_t axis = 0U; axis < APP_ACCEL_AXIS_COUNT; axis++) {
				comparison_raw_value[axis] =
						accelerometer_comparison_raw_value[axis];
				comparison_handled_value[axis] =
						accelerometer_comparison_handled_value[axis];
			}
			if (interrupt_state == 0U) {
				__enable_irq();
			}

			UART_Output_AccelerometerComparison(&uart_output,
					comparison_time_ms, comparison_raw_value,
					comparison_handled_value);
#endif
#if APP_UART_FFT_STREAM_ENABLED
			fft_output_counter++;
			if (fft_output_counter >= FFT_OUTPUT_DECIMATION) {
				fft_output_counter = 0;
				for (uint8_t axis = 0U; axis < APP_ACCEL_AXIS_COUNT; axis++) {
					UART_Output_FFTComparison(&uart_output, axis,
							&fft_raw_result[axis], &fft_handled_result[axis]);
				}
			}
#endif
		}

#ifdef APP_MAVLINK_ENABLED
		MAVLink_Service_Process(&mavlink_service, &magnetometer_data);
#endif
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 976;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 460800;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
#ifndef APP_MAVLINK_ENABLED
	return;
#endif

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

  /* DMA interrupt init */
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IMU_CS_GPIO_Port, IMU_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : IMU_CS_Pin */
  GPIO_InitStruct.Pin = IMU_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(IMU_CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		if (ImuSensor_ReadAccelerometer(&imu_sensor, &accelerometer_raw_data)) {
			for (uint8_t axis = 0U; axis < APP_ACCEL_AXIS_COUNT; axis++) {
				accelerometer_filtered_data.acceleration[axis] =
						SignalFilterChain_Process(&accelerometer_filter[axis],
								accelerometer_raw_data.acceleration[axis]);
				accelerometer_comparison_raw_value[axis] =
						accelerometer_raw_data.acceleration[axis];
				accelerometer_comparison_handled_value[axis] =
						accelerometer_filtered_data.acceleration[axis];
				FFT_Processor_AddSample(&fft_raw_processor[axis],
						accelerometer_raw_data.acceleration[axis]);
				FFT_Processor_AddSample(&fft_handled_processor[axis],
						accelerometer_filtered_data.acceleration[axis]);
			}
			accelerometer_filtered_data.valid = accelerometer_raw_data.valid;
			accelerometer_comparison_time_ms = HAL_GetTick();
		}
		if (imu_sensor.initialized_components & SENSOR_GYROSCOPE) {
			ImuSensor_ReadGyroscope(&imu_sensor, &gyroscope_data);
		}
		if (imu_sensor.initialized_components & SENSOR_MAGNETOMETER) {
			ImuSensor_ReadMagnetometer(&imu_sensor, &magnetometer_data);
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == uart_output.uart) {
		UART_Output_TxComplete(&uart_output);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart == uart_output.uart) {
		UART_Output_TxError(&uart_output);
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
