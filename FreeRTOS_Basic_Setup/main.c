/* FreeRTOS includes. */
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "boot.h"


/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

/* The task functions prototype*/
void vTaskLedBlinking(void *pvParameters);
void vTaskCommunication(void *pvParameters);
void vApplicationIdleHook(void);
void vTimerInterruptTask(void *p);
void vExternalInterruptTask(void *p);

void initGPIOs(void);
void MX_TIM2_Init(void);
void Error_Handler(void);
void SystemClock_Config(void);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT_modified(TIM_HandleTypeDef *htim);

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
TaskHandle_t timerHandle = NULL;
TaskHandle_t buttonHandle = NULL;


/*-----------------------------------------------------------*/

int main( void )
{
	//SystemInit();
	//SystemCoreClockUpdate();
	
	/* essential Board initializations */
	HAL_Init();
	SystemClock_Config();
	initGPIOs();
	MX_TIM2_Init();
	BootComInit();

	printf("FreeRTOS running on STM32F407 Discovery Board\n");

	/* Create one of the two tasks. */
	xTaskCreate(	vTaskLedBlinking,		/* Pointer to the function that implements the task. */
					"LED blinking",	/* Text name for the task.  This is to facilitate debugging only. */
					100,		/* Stack depth in words. */
					NULL,		/* We are not using the task parameter. */
					2,			/* This task will run at priority 1. */
					NULL );		/* We are not using the task handle. */
	
	/* Create the other task in exactly the same way. */
	xTaskCreate( vTaskCommunication, "Communication task", 100, NULL, 1, NULL );
	
	xTaskCreate( vExternalInterruptTask, "button triggered task", 100, NULL, 1, &buttonHandle );
	
	xTaskCreate( vTimerInterruptTask, "timerHandle task", 100, NULL, 1, &timerHandle );
	
	HAL_TIM_Base_Start_IT_modified(&htim2);
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vTaskLedBlinking( void *pvParameters )
{
	
	TickType_t xLastWakeTime;
	const TickType_t xDelay1s = pdMS_TO_TICKS( 1000 );
	xLastWakeTime = xTaskGetTickCount();

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		printf("Blinking!!! \n");
		//HAL_GPIO_TogglePin(GPIOD,GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 |GPIO_PIN_15);

		vTaskDelayUntil( &xLastWakeTime, xDelay1s );
	}
	
	vTaskDelete(NULL);
	
} 
/*-----------------------------------------------------------*/

/* UART communication */
void vTaskCommunication( void *pvParameters )
{
	const char *pcTaskName = "Task communication is running\n";
	volatile unsigned long ul;

	/* As per most tasks, this task is implemented in an infinite loop. */
	for( ;; )
	{
		/* Print out the name of this task. */
    printf("%s\n",pcTaskName);
		//vTaskDelay(1000 / portTICK_PERIOD_MS);

	}
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the button */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	
	BaseType_t checkIfYieldRequried;
	checkIfYieldRequried = xTaskResumeFromISR(buttonHandle);
	portYIELD_FROM_ISR(checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the timer */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
	
	BaseType_t checkIfYieldRequried;
	checkIfYieldRequried = xTaskResumeFromISR(timerHandle);
	portYIELD_FROM_ISR(checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

void vTimerInterruptTask(void *p)
{
	while(1)
	{
		vTaskSuspend(NULL);
		HAL_GPIO_TogglePin(GPIOD, (GPIO_PIN_12|GPIO_PIN_13));
		printf("Timer Interrupt\r\n");
	}
}
/*-----------------------------------------------------------*/

void vExternalInterruptTask(void *p)
{
	while(1)
	{
		vTaskSuspend(NULL);
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14 |GPIO_PIN_15);
		printf("External Interrupt\r\n");
	}
}
/*-----------------------------------------------------------*/

/* This is gonna use for checking if reprogramming flag is activated*/
void vApplicationIdleHook(void)
{
	for( ;; )
	{
		/* Print out the name of this task. */
    printf("idling\n");
	}
}
/*-----------------------------------------------------------*/

void initGPIOs(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
	/*Configure GPIO pin PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	/* EXTI config */
	HAL_NVIC_SetPriority(EXTI0_IRQn, 170, 0);
	HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}
/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* This function will only be called if an API call to create a task, queue
	or semaphore fails because there is too little heap RAM remaining. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed char *pcTaskName )
{
	/* This function will only be called if a task overflows its stack.  Note
	that stack overflow checking does slow down the context switch
	implementation. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
	/* This example does not use the tick hook to perform any processing. */
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

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
  htim2.Init.Prescaler = 42000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
* @brief TIM_Base MSP Initialization
* This function configures the hardware resources used in this example
* @param htim_base: TIM_Base handle pointer
* @retval None
*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 171, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }

}

HAL_StatusTypeDef HAL_TIM_Base_Start_IT_modified(TIM_HandleTypeDef *htim)
{
  uint32_t tmpsmcr;

  /* Check the parameters */
  assert_param(IS_TIM_INSTANCE(htim->Instance));

  /* Check the TIM state */
  if (htim->State != HAL_TIM_STATE_READY)
  {
    return HAL_ERROR;
  }

  /* Set the TIM state */
  htim->State = HAL_TIM_STATE_BUSY;

  /* Enable the TIM Update interrupt */
  __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

  /* Enable the Peripheral, except in trigger mode where enable is automatically done with trigger */
  if (IS_TIM_SLAVE_INSTANCE(htim->Instance))
  {
    tmpsmcr = htim->Instance->SMCR & TIM_SMCR_SMS;
    if (!IS_TIM_SLAVEMODE_TRIGGER_ENABLED(tmpsmcr))
    {
      __HAL_TIM_ENABLE(htim);
    }
  }
  else
  {
    __HAL_TIM_ENABLE(htim);
  }

  /* Return function status */
  return HAL_OK;
}


