/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"

#include "DHT.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include "scheduler.h"
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
#define MAX_SAMPLES 60  // Зберігати останні 60 вимірювань (1 година)

typedef enum {
  DISP_TEMP,
  DISP_HUM,
  DISP_TEMP_AVG,
  DISP_HUM_AVG,
} DisplayMode_t;

float temp_buffer[MAX_SAMPLES];  // Буфер для зберігання температури
float hum_buffer[MAX_SAMPLES]; // Буфер для зберігання вологості
uint8_t buffer_index = 0;

DisplayMode_t display_mode = DISP_TEMP;

DHT_sensor dth22;
uint8_t isPrepared = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
void StartTaskByMode(void);
void MeasureTask(void);
void DisplayTempTask(void);
void DisplayHumTask(void);
void DisplayTempAvgTask(void);
void DisplayHumAvgTask(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM3) {
		 SetTask(MeasureTask);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == KEY_Pin) {
		display_mode = (DisplayMode_t)((display_mode + 1) % 4);
		StartTaskByMode();
	}
}

void StartTaskByMode(void) {
	switch (display_mode) {
	case DISP_TEMP:
		SetTask(DisplayTempTask);
		break;
	case DISP_HUM:
		SetTask(DisplayHumTask);
		break;
	case DISP_TEMP_AVG:
		SetTask(DisplayTempAvgTask);
		break;
	case DISP_HUM_AVG:
		SetTask(DisplayHumAvgTask);
		break;
	}
}

void UpdateDisplay(char* name, char* value) {
	ssd1306_Fill(White);
	ssd1306_SetCursor(2, 10);
	ssd1306_WriteString(name, Font_11x18, Black);
	ssd1306_SetCursor(2, 30);
	ssd1306_WriteString(value, Font_16x26, Black);
	ssd1306_UpdateScreen();
}

float CalculateAverage(float* values, uint8_t size) {
	float sum = 0;
	for (uint8_t i = 0; i < size; i++) {
		sum += values[i];
	}
	return sum / size;
}

float CalculateMaxDeviation(float* values, uint8_t size, float average) {
	float maxDeviation = 0;
	for (uint8_t i = 0; i < size; i++) {
		float deviation = fabs(values[i] - average);
		if (deviation > maxDeviation) {
			maxDeviation = deviation;
		}
	}
	return maxDeviation;
}

void MeasureTask(void) {
	DHT_data dth22Data = DHT_getData(&dth22);
	temp_buffer[buffer_index] = dth22Data.temp;
	hum_buffer[buffer_index] = dth22Data.hum;
	buffer_index = (buffer_index + 1) % MAX_SAMPLES;
	StartTaskByMode();
}

void DisplayTempTask(void) {
	char msgTemp[20];
	float current_temp = temp_buffer[buffer_index -1];
	sprintf(msgTemp, "%d'C", (uint8_t) current_temp);
	UpdateDisplay("TEMP", &msgTemp);
}


void DisplayHumTask(void) {
	char msgHum[20];
	float current_hum = hum_buffer[buffer_index - 1];
	sprintf(msgHum, "%d%%", (uint8_t) current_hum);
	UpdateDisplay("HUM", &msgHum);
}

void DisplayTempAvgTask(void) {
	float temp_average = CalculateAverage(hum_buffer, MAX_SAMPLES);
	float temp_deviation = CalculateMaxDeviation(hum_buffer, MAX_SAMPLES, temp_average);
	char msg_max_deviation[20];
	sprintf(msg_max_deviation, "%d'C", (uint8_t) temp_deviation);
	UpdateDisplay("TEMP AVG", &msg_max_deviation);
}

void DisplayHumAvgTask(void) {
	float hum_average = CalculateAverage(hum_buffer, MAX_SAMPLES);
	float hum_deviation = CalculateMaxDeviation(hum_buffer, MAX_SAMPLES, hum_average);
	char msg_max_deviation[20];
	sprintf(msg_max_deviation, "%d%%", (uint8_t) hum_deviation);
	UpdateDisplay("HUM AVG", &msg_max_deviation);
}
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
	MX_SPI1_Init();
	MX_TIM3_Init();
	/* USER CODE BEGIN 2 */
	ssd1306_Init();

	dth22.DHT_Port = GPIOA;
	dth22.DHT_Pin = GPIO_PIN_3;
	dth22.type = DHT22;
	dth22.pullUp = GPIO_NOPULL;

	InitScheduler();
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		TaskManager();

		if (!isPrepared) {
			UpdateDisplay("Loading...", "");
			HAL_Delay(500);
			HAL_TIM_Base_Start_IT(&htim3);
			isPrepared = 1;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  htim3.Init.Prescaler = 63999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 59998;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|DTH_Pin|OLED_CS_Pin|OLED_Res_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin DTH_Pin OLED_CS_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|DTH_Pin|OLED_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_Res_Pin */
  GPIO_InitStruct.Pin = OLED_Res_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_Res_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_DC_Pin */
  GPIO_InitStruct.Pin = OLED_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OLED_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
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
