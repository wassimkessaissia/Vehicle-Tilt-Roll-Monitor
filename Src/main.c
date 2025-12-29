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
#include"mpu6500.h"
#include<stdio.h>
#include<math.h>
#include"ssd1306.h"
#include"ssd1306_fonts.h"
#include<string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	STABLE ,
	WARNING ,
	DANGER ,
	ROLLOVER ,
}Tiltstate;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
Tiltstate Vehicle_state (float Roll , Tiltstate current_state){
	float roll_abs = fabs(Roll) ;
	switch (current_state) {
	case STABLE :
		if (roll_abs > 17) return WARNING  ;
		return STABLE ;
	case WARNING :
		if (roll_abs < 13) return STABLE;
		if(roll_abs>32) return DANGER ;
		return WARNING ;
	case DANGER :
		if (roll_abs < 28 ) return WARNING ;
		if ( roll_abs >47) return ROLLOVER   ;
		return DANGER ;
	case ROLLOVER :
		if (roll_abs < 43) return DANGER ;
		return ROLLOVER ;


	}
	return STABLE ;
}
void ControlOutput (Tiltstate state ){
	static uint32_t lastTick = 0;
	switch ( state){


	case STABLE :
		 HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin, GPIO_PIN_SET);
		 HAL_GPIO_WritePin(GPIOB, YELLOW_LED_Pin|RED_LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 999);
		 break ;
	case WARNING :
		if (HAL_GetTick() - lastTick >= 700)
				{
			 lastTick = HAL_GetTick();
					HAL_GPIO_TogglePin(GPIOB,BUZZER_Pin);
				}
		HAL_GPIO_WritePin(GPIOB,YELLOW_LED_Pin , GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|RED_LED_Pin, GPIO_PIN_RESET);
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 699);
		break ;
	case DANGER :
		if (HAL_GetTick() - lastTick >= 333)
						{
					 lastTick = HAL_GetTick();
							HAL_GPIO_TogglePin(GPIOB,BUZZER_Pin);
						}
		HAL_GPIO_WritePin(GPIOB, RED_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|YELLOW_LED_Pin, GPIO_PIN_RESET);
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 399);
		break ;
	case ROLLOVER :

		HAL_GPIO_WritePin(GPIOB,BUZZER_Pin , GPIO_PIN_SET);
		 __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		if (HAL_GetTick() - lastTick >= 200)
		{
	     lastTick = HAL_GetTick();
		HAL_GPIO_TogglePin(GPIOB,RED_LED_Pin);
		}
		HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|YELLOW_LED_Pin, GPIO_PIN_RESET);
	}

}
void OLED(Tiltstate state, float roll) {
    char buffer[20];
    char state_text[15];
    uint8_t motor_percent;

    // Convert state enum to text
    switch (state) {
        case STABLE:
            sprintf(state_text, "STABLE");
            motor_percent = 100;
            break;
        case WARNING:
            sprintf(state_text, "WARNING");
            motor_percent = 70;
            break;
        case DANGER:
            sprintf(state_text, "DANGER");
            motor_percent = 40;
            break;
        case ROLLOVER:
            sprintf(state_text, "ROLLOVER!");
            motor_percent = 0;
            break;
    }

    // Clear OLED
    ssd1306_Fill(Black);

    // Line 1: Title
        ssd1306_SetCursor(15, 0);
        ssd1306_WriteString("Tilt Monitor", Font_7x10, White);

        // Line 2: State (smaller font)
        ssd1306_SetCursor(2, 18);
        ssd1306_WriteString("State: ", Font_7x10, White);
        ssd1306_WriteString(state_text, Font_7x10, White);

        // Line 3: Roll
        sprintf(buffer, "Roll: %.1f deg", roll);
        ssd1306_SetCursor(2, 35);
        ssd1306_WriteString(buffer, Font_7x10, White);

        // Line 4: Motor
        sprintf(buffer, "Motor: %d%%", motor_percent);
        ssd1306_SetCursor(2, 52);
        ssd1306_WriteString(buffer, Font_7x10, White);

    // Update screen
    ssd1306_UpdateScreen();
}
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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  printf("=== Vehicle Tilt Monitor ===\n\n");

  MPU6500_Init();
  MPU6500_WhoAmI();
  ssd1306_Init();

  printf("Calibrating...\n");
  HAL_Delay(2000);
  MPU6500_Calibrate();

  printf("\nDriver working! Ready for angle calculation.\n\n");
  ssd1306_Fill(Black);
    ssd1306_SetCursor(2, 0);
    ssd1306_WriteString("Distance:", Font_11x18, White);
    ssd1306_UpdateScreen();

  MPU6500_Data sensor_data;
  int16_t acc_x, acc_y, acc_z;
	Tiltstate  current_state = STABLE ;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  MPU6500_ReadAll(&sensor_data);
	  	 float roll =  MPU6500_GetRoll(&sensor_data);
	  	printf("Roll: %.2f deg \n", roll);

	  	Tiltstate  current_state = Vehicle_state(roll , current_state) ;
	  	OLED(current_state , roll) ;
	  	ControlOutput(current_state ) ;
	  	  HAL_Delay(100);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
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
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GREEN_LED_Pin|YELLOW_LED_Pin|RED_LED_Pin|BUZZER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : GREEN_LED_Pin YELLOW_LED_Pin RED_LED_Pin BUZZER_Pin */
  GPIO_InitStruct.Pin = GREEN_LED_Pin|YELLOW_LED_Pin|RED_LED_Pin|BUZZER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  while (1)
  {
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
