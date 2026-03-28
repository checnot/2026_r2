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
#include "robotic_arm.h"
#include "chassis.h"
#include "clamp.h"
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

int8_t debug_flag;
int32_t debug_angle_sum;
float debug_position;
uint32_t debug_time_start;
float debug_start_position = 0;
float debug_theta3;
float debug_theta4;

int32_t debug_lift;
int32_t debug_extern;
int32_t debug_rotate;
uint8_t clamp_open_flag;
//底盘升降控制相关变量
uint8_t lift_mode;
uint8_t lift_front_flag;
uint8_t lift_back_flag;
uint8_t lift_whole_flag;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId robotic_armHandle;
osThreadId chassisHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void entry_robotic_arm(void const * argument);
void entry_chassis(void const * argument);

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

  /* definition and creation of robotic_arm */
  osThreadDef(robotic_arm, entry_robotic_arm, osPriorityIdle, 0, 256);
  robotic_armHandle = osThreadCreate(osThread(robotic_arm), NULL);

  /* definition and creation of chassis */
  osThreadDef(chassis, entry_chassis, osPriorityIdle, 0, 128);
  chassisHandle = osThreadCreate(osThread(chassis), NULL);

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
    data_send_vofa[0] = m3508_fdcan1[1].Angle_Sum;
    data_send_vofa[1] = m3508_fdcan1[2].Angle_Sum;
		data_send_vofa[7] = debug_test_time;
    vofa_send_to_show_justfloat(&huart6, data_send_vofa);

    // data_send_anotc[0] = arm_cos_f32(debug_test_time * PI);
    // data_send_anotc[1] = arm_cos_f32(debug_test_time*2.0f*PI);
    // data_send_anotc[2] = arm_cos_f32(debug_test_time*4.0f*PI);
    // data_send_anotc[3] = arm_cos_f32(debug_test_time*8.0f*PI);
    // data_send_anotc[4] = arm_cos_f32(debug_test_time*16.0f*PI);
    // data_send_anotc[5] = arm_cos_f32(debug_test_time*32.0f*PI);
    // data_send_anotc[6] = arm_cos_f32(debug_test_time*64.0f*PI);
    // data_send_anotc[7] = arm_cos_f32(debug_test_time*128.0f*PI);
    // Usart_Send_To_Show32(&huart6, data_send_anotc);

    debug_test_time += 0.001f;
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_entry_robotic_arm */
/**
* @brief Function implementing the robotic_arm thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_robotic_arm */
void entry_robotic_arm(void const * argument)
{
  /* USER CODE BEGIN entry_robotic_arm */

  arm_joint_parameter_init(&arm);

  m3508_parameter_init(&m3508_fdcan1[0], 2.0f, 10.0f, 0.5f, 2.0f);
  m3508_parameter_init(&m3508_fdcan1[1], 2.0f, 10.0f, 0.5f, 2.0f);
  m3508_parameter_init(&m3508_fdcan1[2], 2.0f, 10.0f, 0.5f, 2.0f);

  /* Infinite loop */
  for(;;)
  {
    arm_joint_state_update(&arm);
    arm_route_point_calculate(&arm);

//发现气泵和遥控器开关的通信有问题，暂时注释掉气泵控制代码
		// if(HT10A.switch_b == 1)
    // {
    //   HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_SET);
    // }
    // else
    // {
    //   HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_RESET);
    // }
		
		if(debug_flag == 1)
    {
      arm_actuator_stable_control(&arm, 0.0f, 0.0f);
    }
    // arm_gravity_comp(&arm, 0.60f, 3000.0f);
// //此处输出表示为输出轴电流(力矩)
    // if(m3508_fdcan1[0].Initial_flag == 200 && A1_debug_485_2[1].Pos_init_flag == 200)
    // {
    //   // m3508_fdcan1[0].Current_Output = (int16_t)(arm.joint_torque_gc[1]/0.3f/10.0f*8192.0f);
    //   // m3508_fdcan1[0].Current_Output = 0;
    //   // A1_debug_485_2[1].target.T = arm.joint_torque_gc[0] / 9.1f;
    // }
    if(m3508_fdcan1[0].Initial_flag == 200 && A1_debug_485_2[1].Pos_init_flag == 200)
    {
      if(debug_flag == 0)
      {
        arm_joint_state_init(&arm);
      }
      if(debug_flag == 1)
      {
        if(arm_route_reachable_check(&arm) == IK_REACHABLE)
        {
          arm_route_execute(&arm);
        }      
        else 
        {
          //TODO目标点不可达时的处理，目前先简单地不执行路径控制
        }
      }
    }
    if(HT10A.right_x !=0 || HT10A.right_y != 0)
    {
      arm.x0_target += 0.0001f * arm_cos_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
      arm.y0_target += 0.0001f * arm_sin_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
    }
    else
    {
      
    }

    osDelay(1);
  }
  /* USER CODE END entry_robotic_arm */
}

/* USER CODE BEGIN Header_entry_chassis */
/**
* @brief Function implementing the chassis thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_chassis */
void entry_chassis(void const * argument)
{
  /* USER CODE BEGIN entry_chassis */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END entry_chassis */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
