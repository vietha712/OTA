/* FreeRTOS includes. */
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "boot.h"


/* Used as a loop counter to create a very crude delay. */
#define mainDELAY_LOOP_COUNT		( 0xfffff )

#define MAX_BYTE 512

#define SEND_DATA_SIZE 50
/** \brief Configure number of bytes in the host->target data packet. */
#define BOOT_COM_RS232_RX_MAX_DATA       (64)

/* The task functions prototype*/
void vTaskLedBlinking(void *pvParameters);
//void vTaskCommunication(void *pvParameters);
void vTaskSendDataFromUart3(void *pvParameters);
void vTaskFinishSendDataFromUart3(void *pvParameters);
void vTaskReceiveDataByUart2(void *pvParameters);
void vApplicationIdleHook(void);
void vTimerInterruptTask(void *p);
void vExternalInterruptTask(void *p);

void initGPIOs(void);
void MX_TIM2_Init(void);
void MX_USART2_UART_Init(void);
void MX_USART3_UART_Init(void);
void Error_Handler(void);
void SystemClock_Config(void);
void VectorBase_Config(void);
void Init(void);

HAL_StatusTypeDef HAL_TIM_Base_Start_IT_modified(TIM_HandleTypeDef *htim);

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
TaskHandle_t timerHandle = NULL;
TaskHandle_t buttonHandle = NULL;
TaskHandle_t uart3Handle = NULL;
TaskHandle_t endUart3Handle = NULL;
TaskHandle_t uart2Handle = NULL;

uint8_t uart2_receivingBuffer;
uint8_t pData[SEND_DATA_SIZE] = {0x00, 0x02, 0xff, 0x00, 0x00, 0x00, 0x01, 0x01, 0x01, 0x01,
																 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
																 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
																 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01,
																 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01, 0x01};

/*-----------------------------------------------------------*/


int main( void )
{
	/* essential Board initializations */
	Init();
	HAL_Init();
	SystemClock_Config();
	//SystemInit();
	//SystemCoreClockUpdate();
	initGPIOs();
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();

	printf("FreeRTOS running on STM32F407 Discovery Board\n");
	
	/* Create the other task in exactly the same way. */
	xTaskCreate( vTaskSendDataFromUart3, "vTaskSendDataFromUart3 task", 100, NULL, 2, &uart3Handle );
	xTaskCreate( vTaskFinishSendDataFromUart3, "vTaskSendDataFromUart3 task", 100, NULL, 1, &endUart3Handle );

	xTaskCreate( vTaskReceiveDataByUart2, "vTaskReceiveDataByUart2 task", 100, NULL, 2, &uart2Handle );
	
	xTaskCreate( vExternalInterruptTask, "button triggered task", 100, NULL, 1, &buttonHandle );
	
	xTaskCreate( vTimerInterruptTask, "timerHandle task for LED blinking", 100, NULL, 1, &timerHandle );
	
	/* Start Interrupt */
	HAL_TIM_Base_Start_IT_modified(&htim2);
	HAL_UART_Receive_IT(&huart2,&uart2_receivingBuffer, 1); //last argument indicates 1 byte transmitted trigger interrupt
	
	/* Start the scheduler so our tasks start executing. */
	vTaskStartScheduler();

	/* If all is well we will never reach here as the scheduler will now be
	running.  If we do reach here then it is likely that there was insufficient
	heap available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/
static void Init(void)
{
  /* Configure the vector table base address. */
  VectorBase_Config();

} /*** end of Init ***/
/*-----------------------------------------------------------*/

static void VectorBase_Config(void)
{
  /* The constant array with vectors of the vector table is declared externally in the
   * c-startup code.
   */
  extern const unsigned long __Vectors[];

  /* Remap the vector table to where the vector table is located for this program. */
  SCB->VTOR = (unsigned long)&__Vectors[0];
} /*** end of VectorBase_Config***/
/*-----------------------------------------------------------*/

/* UART communication */
void vTaskReceiveDataByUart2( void *pvParameters )
{
	static unsigned char xcpCtoReqPacket[BOOT_COM_RS232_RX_MAX_DATA+1];
  static unsigned char xcpCtoRxLength;
  static unsigned char xcpCtoRxInProgress = 0;
	static uint32_t numByteCopyFromUart2Buffer = 0;
	/* As per most tasks, this task is implemented in an infinite loop. */
	while(1)
	{
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
			
			if (uart2_receivingBuffer > 0) xcpCtoReqPacket[numByteCopyFromUart2Buffer++] = uart2_receivingBuffer;
			if(numByteCopyFromUart2Buffer > BOOT_COM_RS232_RX_MAX_DATA) numByteCopyFromUart2Buffer = 0;

			/* start of cto packet received? */
			if (xcpCtoRxInProgress == 0)
			{
				/* check that the length has a valid value. it should not be 0 */
				if ((xcpCtoReqPacket[0] <= BOOT_COM_RS232_RX_MAX_DATA) )
				{
					/* indicate that a cto packet is being received */
					xcpCtoRxInProgress = 1;
					/* reset packet data count */
					xcpCtoRxLength = 0;
				}
			}
			else
			{
				/* store the next packet byte */
				/* increment the packet data count */
				xcpCtoRxLength++;
		
				/* check to see if the entire packet was received */
				if (xcpCtoRxLength == xcpCtoReqPacket[0])
				{
					/* done with cto packet reception */
					xcpCtoRxInProgress = 0;
		
					/* check if this was an XCP CONNECT command */
					if ((xcpCtoReqPacket[1] == 0xff) && (xcpCtoRxLength == 2))
					{
						/* connection request received so start the bootloader */
						for(int i = 0; i < numByteCopyFromUart2Buffer; i++) 
						{
							xcpCtoReqPacket[i] = 0;
							numByteCopyFromUart2Buffer = 0;
						}
						BootActivate();
					}
				}
			}
		}
	}
}
/*-----------------------------------------------------------*/

/* UART communication */
void vTaskSendDataFromUart3( void *pvParameters )
{
	/* As per most tasks, this task is implemented in an infinite loop. */
	uint32_t i = 0;
	for( ;; )
	{
		HAL_UART_Transmit_IT(&huart3, &pData[i], 1);
		vTaskDelay(2000/portTICK_RATE_MS); //3s
		i++;
		if(i == SEND_DATA_SIZE) i = 0;
	}
}

/* UART communication */
void vTaskFinishSendDataFromUart3( void *pvParameters )
{
	/* As per most tasks, this task is implemented in an infinite loop. */
	while(1)
	{
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			//for(int i = 0; i < SEND_DATA_SIZE;i++) pData[i]++;
			//if(pData[9] == 0xff)
			//{
			//	for(int i = 0; i < SEND_DATA_SIZE;i++) pData[i]=0;
			//}
		}
	}
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the button */
void EXTI0_IRQHandler(void)
{
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
	BaseType_t checkIfYieldRequried;
	
	vTaskNotifyGiveFromISR(buttonHandle, &checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the timer */
void TIM2_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim2);
	BaseType_t checkIfYieldRequried;
	
	vTaskNotifyGiveFromISR(timerHandle, &checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the uart2 */
void USART2_IRQHandler(void)
{
	BaseType_t checkIfYieldRequried;

	if(huart2.Instance->SR & 0x20)
	{
			uart2_receivingBuffer = huart2.Instance->DR;
	}

	vTaskNotifyGiveFromISR(uart2Handle, &checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

/* Interrupt routine for the timer */
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart3);
	BaseType_t checkIfYieldRequried;
	
	vTaskNotifyGiveFromISR(endUart3Handle, &checkIfYieldRequried);
}
/*-----------------------------------------------------------*/

void vTimerInterruptTask(void *p)
{
	while(1)
	{
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
		}
	}
}
/*-----------------------------------------------------------*/
/* Task notification is used as a lightweight binary semaphore */
void vExternalInterruptTask(void *p)
{
	while(1)
	{
		if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14 |GPIO_PIN_15);
		}
	}
}
/*-----------------------------------------------------------*/

/* This is gonna use for checking if reprogramming flag is activated*/
void vApplicationIdleHook(void)
{
	for( ;; )
	{
		/* Print out the name of this task. */
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
	
	/* Peripheral clock enable. */
  __HAL_RCC_USART2_CLK_ENABLE();
	
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
	
	/* Configure UART pins */
	/* UART TX and RX GPIO pin configuration. */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	/* Set priority grouping. */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* MemoryManagement_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration. */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
	
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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
  huart2.Init.BaudRate = 57600;
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
	huart2.Instance->BRR = (0x2d9); //Workaround for the communication with flashing device
  /* USER CODE END USART2_Init 2 */


}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 57600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	huart3.Instance->BRR = (0x2d9); //Workaround for the communication with flashing device
  /* USER CODE END USART3_Init 2 */

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

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(huart->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 171, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(huart->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* Peripheral clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 170, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }

}


