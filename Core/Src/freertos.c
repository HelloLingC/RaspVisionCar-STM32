/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for oledTask */
osThreadId_t oledTaskHandle;
const osThreadAttr_t oledTask_attributes = {
  .name = "oledTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pidTask */
osThreadId_t pidTaskHandle;
const osThreadAttr_t pidTask_attributes = {
  .name = "pidTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTaskReceive(void *argument);
void StartTaskOLED(void *argument);
void StartTaskPID(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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

  /* Create the thread(s) */
  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(StartTaskReceive, NULL, &receiveTask_attributes);

  /* creation of oledTask */
  oledTaskHandle = osThreadNew(StartTaskOLED, NULL, &oledTask_attributes);

  /* creation of pidTask */
  pidTaskHandle = osThreadNew(StartTaskPID, NULL, &pidTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTaskReceive */
/**
  * @brief  Function implementing the receiveTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTaskReceive */
void StartTaskReceive(void *argument)
{
  /* USER CODE BEGIN StartTaskReceive */
  /* Infinite loop */
  for(;;)
  {
    if (xSerialSemaphore != NULL) {
      if (xSemaphoreTake(xSerialSemaphore, portMAX_DELAY) == pdTRUE) {
        usart_info((const char*)rxBuffer);
      }
    } else {
      // We could not obtain the semaphore
      // usart_error("FAILED to obtain samaphore in TaskReceive.");
    }
    osDelay(100);
  }
  /* USER CODE END StartTaskReceive */
}

/* USER CODE BEGIN Header_StartTaskOLED */
extern void OLED_task_update();
/**
* @brief Function implementing the oledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskOLED */
void StartTaskOLED(void *argument)
{
  /* USER CODE BEGIN StartTaskOLED */
  /* Infinite loop */
  for(;;)
  {
    OLED_task_update();
    osDelay(1000);
  }
  /* USER CODE END StartTaskOLED */
}

/* USER CODE BEGIN Header_StartTaskPID */
extern void task_pid_update();
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPID */
void StartTaskPID(void *argument)
{
  /* USER CODE BEGIN StartTaskPID */
  /* Infinite loop */
  for(;;)
  {
    task_pid_update();
    osDelay(20);
  }
  /* USER CODE END StartTaskPID */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

