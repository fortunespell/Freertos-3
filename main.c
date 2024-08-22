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
#include "cmsis_os.h"
#include "fonts.h"
#include "ssd1306.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
osThreadId BlinkRedLEDHandle;
osThreadId BlinkBlueLEDHandle;
osThreadId BlinkGreenLEDHandle;
osThreadId PushButtonHandle;
//osThreadId UltrasonicHandle;
//osThreadId display_Handle;
/* USER CODE END Includes */
#define TRIG_PIN GPIO_PIN_10
#define TRIG_PORT GPIOC
#define ECHO_PIN GPIO_PIN_11
#define ECHO_PORT GPIOC
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);

/* USER CODE BEGIN PFP */

void StartTask01(void const * argument1);
void StartTask02(void const * argument2);
void StartTask03(void const * argument3);
void StartTask04(void const * argument4);
//void StartTask05(void const * argument);
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
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  HAL_TIM_Base_Start(&htim1);
  HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  /* definition and creation of defaultTask */
   osThreadDef(BlinkRedLED, StartTask01, osPriorityNormal , 0, 128);
   BlinkRedLEDHandle = osThreadCreate(osThread(BlinkRedLED), NULL);

    /* definition and creation of BlinkBlueLED */
   //osThreadDef(BlinkBlueLED, StartTask02, osPriorityNormal, 0, 128);
  // BlinkBlueLEDHandle = osThreadCreate(osThread(BlinkBlueLED), NULL);

    //* definition and creation of BlinkGreenLED */
  osThreadDef(BlinkGreenLED, StartTask03, osPriorityNormal, 0, 128);
   BlinkGreenLEDHandle = osThreadCreate(osThread(BlinkGreenLED), NULL);

    /* definition and creation of PushButton */
    //osThreadDef(PushButton, StartTask04, osPriorityNormal, 0, 128);
   // PushButtonHandle = osThreadCreate(osThread(PushButton), NULL);

    /* definition and creation of Ultrasonic */
   // osThreadDef(Ultrasonic, StartTask05, osPriorityRealtime , 0, 128);
   //  UltrasonicHandle = osThreadCreate(osThread(Ultrasonic), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
    osKernelStart();
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
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
  hi2c1.Init.Timing = 0x00301739;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartTask01(void const * argument1)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	 for (;;) {
	        // Toggle LED red
	        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	        vTaskDelay(pdMS_TO_TICKS(1500)); // Delay for 500 milliseconds
	        SSD1306_Init();
	        SSD1306_GotoXY (0,0);
	        SSD1306_Puts ("RTS", &Font_11x18, 1);
	        SSD1306_GotoXY (0, 30);
	        SSD1306_Puts ("EL", &Font_11x18, 1);
	        SSD1306_UpdateScreen();
	        HAL_Delay (1000);
	        SSD1306_ScrollRight(0,7);
	        HAL_Delay(3000);
	        SSD1306_ScrollLeft(0,7);
	        HAL_Delay(3000);
	        SSD1306_Stopscroll();
	        SSD1306_Clear();

	        int num=2024;
	        char snum[5];
	        SSD1306_GotoXY (30,20);
	        itoa(num, snum, 10);
	        SSD1306_Puts (snum, &Font_16x26, 1);
	        SSD1306_UpdateScreen();

	        //SSD1306_DrawLine(POINT1 X, POINT1 Y, POINT2 X, POINT2 Y, 1);
	        SSD1306_DrawLine(1,54,126,54,1);
	        SSD1306_UpdateScreen();
	        HAL_Delay (5000);
	        SSD1306_Clear();

	        // For Rectangle, we need to use two corner points
	        // SSD1306_DrawRectangle(POINT1 X, POINT1 Y, POINT2 X, POINT2 Y, 1);
	        SSD1306_DrawRectangle(17,1,115,14,1);
	        // SSD1306_DrawTriangle(POINT1X, POINT1Y, POINT2X, POINT2Y, POINT3X, POINT3Y, 1);
	        SSD1306_DrawTriangle(73,22,124,62,74,54,1);
	        // SSD1306_DrawCircle(CENTER POINT X, CENTER POINT Y, RADIUS, 1);
	        SSD1306_DrawCircle(34,50,13,1);
	        SSD1306_UpdateScreen();
	        /* USER CODE END 2 */
	    }


  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the BlinkBlueLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void const * argument2)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
    for (;;) {
        // Glow the blue LED three times for 1500ms each
        for (int i = 0; i < 3; i++)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // Turn on LED
            vTaskDelay(pdMS_TO_TICKS(1500)); // Delay for 1500 milliseconds
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off LED
            vTaskDelay(pdMS_TO_TICKS(1500)); // Delay for 1500 milliseconds
        }

        // Glow the blue LED two times for 100ms each
        for (int i = 0; i < 2; i++)
        {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // Turn on LED
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // Turn off LED
            vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
        }
    }

  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the BlinkGreenLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void const * argument3)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
	for (int i=0;i<5;i++){
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		while (__HAL_TIM_GET_COUNTER(&htim1) < 10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		// wait for the echo pin to go high
		while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
		Value1 = __HAL_TIM_GET_COUNTER(&htim1);

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		// wait for the echo pin to go low
		while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		Value2 = __HAL_TIM_GET_COUNTER(&htim1);

		Distance = (Value2 - Value1) * 0.034 / 2;

		// Check if the distance is less than 10 cm
		if (Distance < 10.0) {
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // turn on the LED on PB0
		} else {
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // turn off the LED on PB0
			void StartTask01();
		}

		HAL_Delay(5);
	}
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the PushButton thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void const * argument4)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
	for (;;) {
	        // Check if the push button is pressed
	        if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET) {
	            // If the button is pressed, blink the LED faster
	        	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
	            vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100 milliseconds
	        } else {
	            // If the button is not pressed, blink the LED normally
	        	void StartTask01();
	        }

	        // Toggle the LED

	    }
  /* USER CODE END StartTask04 */
}


/*void StartTask05(void const * argument)
{

	for (;;)
	{

		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		while (__HAL_TIM_GET_COUNTER(&htim1) < 10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
		// wait for the echo pin to go high
		while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
		Value1 = __HAL_TIM_GET_COUNTER(&htim1);

		pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
		// wait for the echo pin to go low
		while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
		Value2 = __HAL_TIM_GET_COUNTER(&htim1);

		Distance = (Value2 - Value1) * 0.034 / 2;

		// Check if the distance is less than 10 cm
		if (Distance < 10.0) {
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // turn on the LED on PB0
		} else {
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);  // turn off the LED on PB0
		}

		HAL_Delay(50);

	 }

}*/
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
