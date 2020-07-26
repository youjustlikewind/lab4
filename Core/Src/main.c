/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "stdio.h"
#include "semphr.h"

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
/* Definitions for Task1 */
osThreadId_t Task1Handle;
const osThreadAttr_t Task1_attributes = {
  .name = "Task1",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Task02 */
osThreadId_t Task02Handle;
const osThreadAttr_t Task02_attributes = {
  .name = "Task02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTask03 */
osThreadId_t myTask03Handle;
const osThreadAttr_t myTask03_attributes = {
  .name = "myTask03",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartTask01(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
QueueHandle_t  xQueue1;
QueueHandle_t  xQueue2;
static SemaphoreHandle_t xSemaphore = NULL;


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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
  //Create two queue capable of containing 5 int values.
  xQueue1=xQueueCreate(5,sizeof(int32_t));
  xQueue2=xQueueCreate(5,sizeof(int32_t));
  if(xQueue1 == NULL || xQueue2 == NULL)
  {
	  /* Queue was not created and must not be used. */
	  printf("Queue was not created and must not be used.\n");
  }
  else
  {
	  //create Muxtext
	  xSemaphore=xSemaphoreCreateMutex();
	  if(xSemaphore == NULL)
	  {
		  printf("Mutex was not created");
	  }
	  else
	  {

			  /* Create the thread(s) */
			  /* creation of Task1 */
			  Task1Handle = osThreadNew(StartTask01, NULL, &Task1_attributes);

			  /* creation of Task02 */
			  Task02Handle = osThreadNew(StartTask02, NULL, &Task02_attributes);

			  /* creation of myTask03 */
			  myTask03Handle = osThreadNew(StartTask03, NULL, &myTask03_attributes);

			  xSemaphoreGive(xSemaphore);
	  }
  }

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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB14 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_14|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartTask01 */
/**
  * @brief  Function implementing the Task1 thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartTask01 */
void StartTask01(void *argument)
{
	  int32_t counter=1; //value to write
	  BaseType_t xStatus1;
	  BaseType_t xStatus2;
	  const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );
	  int flag=2;
	  /* USER CODE BEGIN 5 */
	  /* Infinite loop */
	  for(;;)
	  {
		//sends the “counter” value to Task 2 once per second
		xStatus1=xQueueSend( xQueue1,( void * ) &counter, xTicksToWait);
		if(xStatus1!=pdPASS)
		{
			printf("xQueue1 Failed to post the message, even after 1000 ms.");
		}

		//sends the “counter” value to Task 3 once per two seconds
		if(flag>=2)
		{
			xStatus2=xQueueSend( xQueue2,( void * ) &counter, xTicksToWait);
			if(xStatus2!=pdPASS)
			{
				printf("xQueue2 Failed to post the message, even after 1000 ms.");
			}
			flag=0;
		}

		//The incrementing of “counter” variable should be paused if the corresponding queue is full and resumed if the corresponding queue is not full.
		if(uxQueueMessagesWaiting(xQueue1)==5 || uxQueueMessagesWaiting(xQueue2)==5)
		{
			counter=counter;
		}
		else
			counter=counter+1;

		flag=flag+1;
		osDelay(1000);
	  }
	  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Task02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	  int32_t xReceivedValue;
	  BaseType_t xStatus;
	  const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );
	  /* USER CODE BEGIN StartTask02 */
	  /* Infinite loop */
	  for(;;)
	  {
		if(uxQueueMessagesWaiting(xQueue1) != 0)
		{
			xStatus = xQueueReceive( xQueue1, &xReceivedValue, xTicksToWait );
			if( xStatus == pdPASS )
			 {
				xSemaphoreTake(xSemaphore, portMAX_DELAY);//Realize mutual exclusive access to resources through binary mutually exclusive semaphores, and wait forever until resources are available
				for(int32_t i=0;i<xReceivedValue;i++)
				{
					 /* After the data is successfully acquired, flip the LED2 light level, the number of flips is the number read*/
					 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);//led2 Flashing
					 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//led1 Flashing,LED1 is used to check if LED2 flashes and conflicts with task 3
					 osDelay(550);
				}
				xSemaphoreGive(xSemaphore);//freed
				osDelay(550);//Free for a certain period of time for task 3 using LED2
			 }
			 else
			 {
				 /* Data was not received from the queue even after waiting for 1000ms. */
				 printf( "Could not receive from the queue.\r\n" );
			 }
		}
	  }
	  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the myTask03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	  int32_t xReceivedValue;
	  BaseType_t xStatus;
	  const TickType_t xTicksToWait = pdMS_TO_TICKS( 1000 );
	  /* USER CODE BEGIN StartTask03 */
	  /* Infinite loop */
	  for(;;)
	  {
		if(uxQueueMessagesWaiting(xQueue2) != 0)
		{
			xStatus = xQueueReceive( xQueue2, &xReceivedValue, xTicksToWait );
			if( xStatus == pdPASS )
			 {
				xSemaphoreTake(xSemaphore, portMAX_DELAY) ;
				for(int32_t i=0;i<xReceivedValue;i++)
				{
					 /* After the data is successfully acquired, flip the LED2 light level, the number of flips is the number read*/
					 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_7);// LED2 flashing
					 HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_14);//LED3 Flashing,LED3 is used to check if LED2 flashes and conflicts with task 2
					 osDelay(550);
				}
				xSemaphoreGive(xSemaphore);//freed
				osDelay(550);//Free for a certain period of time for task 2 using LED2
			 }
			 else
			 {
				 /* Data was not received from the queue even after waiting for 1000ms. */
				 printf( "Could not receive from the queue.\r\n" );
			 }
		}
	  }
  /* USER CODE END StartTask03 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
