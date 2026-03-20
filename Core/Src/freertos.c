/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "own_lib.h"
#include "mycan.h"
#include "myuart.h"
#include "filter.h"
#include "observer.h"
#include "pid.h"
#include "m3508.h"
#include "vofa.h"
#include "ANO_TC.h"
#include "arm_math.h"
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
float debug_test_time;//用于测试的万能小变量

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    data_send_vofa[0] = arm_cos_f32(debug_test_time * PI);
    data_send_vofa[1] = arm_cos_f32(debug_test_time*2.0f*PI);
    data_send_vofa[2] = arm_cos_f32(debug_test_time*4.0f*PI);
    data_send_vofa[3] = arm_cos_f32(debug_test_time*8.0f*PI);
    data_send_vofa[4] = arm_cos_f32(debug_test_time*16.0f*PI);
    data_send_vofa[5] = arm_cos_f32(debug_test_time*32.0f*PI);
    data_send_vofa[6] = arm_cos_f32(debug_test_time*64.0f*PI);
    data_send_vofa[7] = arm_cos_f32(debug_test_time*128.0f*PI);
    vofa_send_to_show_justfloat(&huart6, data_send_vofa);
    osDelay(1);

    // data_send_anotc[0] = arm_cos_f32(debug_test_time * PI);
    // data_send_anotc[1] = arm_cos_f32(debug_test_time*2.0f*PI);
    // data_send_anotc[2] = arm_cos_f32(debug_test_time*4.0f*PI);
    // data_send_anotc[3] = arm_cos_f32(debug_test_time*8.0f*PI);
    // data_send_anotc[4] = arm_cos_f32(debug_test_time*16.0f*PI);
    // data_send_anotc[5] = arm_cos_f32(debug_test_time*32.0f*PI);
    // data_send_anotc[6] = arm_cos_f32(debug_test_time*64.0f*PI);
    // data_send_anotc[7] = arm_cos_f32(debug_test_time*128.0f*PI);
    // Usart_Send_To_Show32(&huart6, data_send_anotc);
    // osDelay(5);

    debug_test_time += 0.001f;
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
