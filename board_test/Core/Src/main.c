/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "retarget.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef timer;
uint16_t duty_cycle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void tpMotor(void);
void tpTimer(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void TIM3_IRQHandler() {
	uint8_t counter = 0;
	__NVIC_ClearPendingIRQ(TIM3_IRQn);
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	if(counter++ == 1000) {
		duty_cycle = (duty_cycle + 250) % (1000 + 1);
		__HAL_TIM_SET_COMPARE(&timer,TIM_CHANNEL_2,duty_cycle);
		counter = 0;
	} else {
	}
}
void tpTimer(void) {
	// PB4 => TIM3_CH1 (IN1A)
	// PB5 => TIM3_CH2 (IN2A)
	TIM_OC_InitTypeDef timer_oc_config;
	GPIO_InitTypeDef in2_a = {0};
	HAL_StatusTypeDef status;

	// Enable TIM3 Clock
	__HAL_RCC_TIM3_CLK_ENABLE();
	// ENable GPIOB Clock
	__HAL_RCC_GPIOB_CLK_ENABLE();
	// Enable IT
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);

	in2_a.Pin = GPIO_PIN_5;
	in2_a.Pull = GPIO_NOPULL;
	in2_a.Speed = GPIO_SPEED_LOW;
	in2_a.Mode = GPIO_MODE_AF_PP;
	in2_a.Alternate = GPIO_AF2_TIM3;
	HAL_GPIO_Init(GPIOB, &in2_a);

	timer.Instance = TIM3;
	// FreqTimer / (FreqSouhaité * Period)
	timer.Init.Prescaler = 16000000 / (50 * 1000);
	timer.Init.Period = 1000;
	timer.Init.ClockDivision = 0;
	timer.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer.Init.RepetitionCounter = 0;

	status = HAL_TIM_PWM_Init(&timer);

	timer_oc_config.OCMode = TIM_OCMODE_PWM1;
	timer_oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;
	timer_oc_config.OCFastMode = TIM_OCFAST_DISABLE;
	timer_oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	timer_oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	timer_oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;

	timer_oc_config.Pulse = (1000 - 1) / 2;
	status = HAL_TIM_PWM_ConfigChannel(&timer, &timer_oc_config, TIM_CHANNEL_2);

	status = HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);
	//status = HAL_TIM_PWM_Start_IT(&timer, TIM_CHANNEL_2);

	//while(1);
}
void tpMotor(void) {
	// IN1A = Pont A input 1 (D3) => PB4
	// IN2A = Pont A input 2 (D4) => PB5
	// ENA = Pont A input (Bridge enable, OFF on LOW) => PA10
	// ENB = Pont B input (Bridge enable, OFF on LOW) => PC1

	GPIO_InitTypeDef in1_a = {0};
	GPIO_InitTypeDef in2_a = {0};
	GPIO_InitTypeDef en_a = {0};
	GPIO_InitTypeDef en_b = {0};

	// Enable Clocks
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	// Configure Pins
	in1_a.Pin = GPIO_PIN_4;
	in1_a.Pull = GPIO_NOPULL;
	in1_a.Speed = GPIO_SPEED_LOW;
	in1_a.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &in1_a);

	in2_a.Pin = GPIO_PIN_5;
	in2_a.Pull = GPIO_NOPULL;
	in2_a.Speed = GPIO_SPEED_LOW;
	in2_a.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &in2_a);

	en_a.Pin = GPIO_PIN_10;
	en_a.Pull = GPIO_NOPULL;
	en_a.Speed = GPIO_SPEED_LOW;
	en_a.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOA, &en_a);

	en_b.Pin = GPIO_PIN_1;
	en_b.Pull = GPIO_NOPULL;
	en_b.Speed = GPIO_SPEED_LOW;
	en_b.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOC, &en_b);

	// Enabling bridges
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, SET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, SET);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t dir = 0;
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
  /* USER CODE BEGIN 2 */
  printf("Demo is starting...\r\n");
  RetargetInit(&huart2);
  tpMotor();
  tpTimer();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#if 1
  while (1)
  {
	  // Motor Control Shield
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
	  //HAL_Delay(500);
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
	  if(dir == 0) {
		  duty_cycle = (duty_cycle + 100) % (1000 +1);
		  if(duty_cycle == 1000) {
			  dir = 1;
		  }
	  } else {
		  duty_cycle = (duty_cycle - 100) % (1000 +1);
		  if(duty_cycle == 0)
			  dir = 0;
	  }

	  __HAL_TIM_SET_COMPARE(&timer,TIM_CHANNEL_2,duty_cycle);
	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
#endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_GREEN_Pin */
  GPIO_InitStruct.Pin = LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GREEN_GPIO_Port, &GPIO_InitStruct);

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
  while (1)
  {
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

