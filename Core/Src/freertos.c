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
#include "ht10a.h"
#include "hi14.h"
#include "trajectory.h"
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

int32_t debug_flag;
float debug_position;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId roboticarm_taskHandle;
osThreadId chassis_taskHandle;
osThreadId clamp_taskHandle;
osThreadId cmd_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void entry_robotic_arm(void const * argument);
void entry_chassis(void const * argument);
void entry_clamp(void const * argument);
void entry_cmd_task(void const * argument);

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of roboticarm_task */
  osThreadDef(roboticarm_task, entry_robotic_arm, osPriorityBelowNormal, 0, 256);
  roboticarm_taskHandle = osThreadCreate(osThread(roboticarm_task), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, entry_chassis, osPriorityBelowNormal, 0, 128);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of clamp_task */
  osThreadDef(clamp_task, entry_clamp, osPriorityBelowNormal, 0, 128);
  clamp_taskHandle = osThreadCreate(osThread(clamp_task), NULL);

  /* definition and creation of cmd_task */
  osThreadDef(cmd_task, entry_cmd_task, osPriorityNormal, 0, 128);
  cmd_taskHandle = osThreadCreate(osThread(cmd_task), NULL);

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

  //测试用轨迹函数
  // A1_debug_485_1[1].id = 1;
  // A1_debug_485_1[1].mode = 10;
//=======================================================================
  //多段轨迹-初始化
  // // 1. 单段轨迹参数初始化
  // Traj_Adaptive_Init(&debug_traj_a1, 0.0, 0, 4.0f*PI, 8.0f*PI, PI/6.0f);
  // // 2. 多段轨迹初始化
  // Traj_Path_Queue_Init(&debug_traj_queue_a1, PI/90.0f); // 2度的融合半径
  // // 3. 上位机发来一段乱七八糟的心情画图路径：
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, PI/2.0f); // 抛入目标A
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, -PI/2.0f); // 抛入目标B
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, PI/2.0f); // 抛入目标C
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_a1, -PI/2.0f); // 抛入目标D

  // //-------------------------------------------------------------
  // Traj_Adaptive_Init(&debug_traj_m3508, 0.0, 0, 8192.0f*16.0f, 8192.0f*16.0f, 8192.0f);

  // Traj_Path_Queue_Init(&debug_traj_queue_m3508, 512.0f); // 512单位的融合半径
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_m3508, 40000.0f); // 抛入目标A
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_m3508, -40000.0f); // 抛入目标B
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_m3508, 40000.0f); // 抛入目标C
  // Traj_Path_Queue_AddPoint(&debug_traj_queue_m3508, -40000.0f); // 抛入目标D

  // m3508_parameter_init(&m3508_fdcan1[3], 2.4f, 2.0f, 1.6f, 0.5f);
  //多段轨迹-初始化
//=======================================================================

  /* Infinite loop */
  for(;;)
  {

//=======================================================================
  //多段轨迹-规划与执行
    // if(debug_flag == 1)
    // {
    //   //多段轨迹规划与执行示例（以A1电机为例，m3508同理）
    //   Traj_Path_Queue_Update(&debug_traj_queue_a1, &debug_traj_a1, A1_debug_485_1[1].measure.Pos);
    //   Traj_Adaptive_Update(&debug_traj_a1, A1_debug_485_1[1].measure.Pos, 0.001f);
    //   A1_Motor_Position_Control(&A1_debug_485_1[1], debug_traj_a1.out_pos, 2.0f, 0.025f);

    //   //------------------------------------------------------------

    //   // Traj_Path_Queue_Update(&debug_traj_queue_m3508, &debug_traj_m3508, m3508_fdcan1[3].Angle_Sum);
    //   // Traj_Adaptive_Update(&debug_traj_m3508, m3508_fdcan1[3].Angle_Sum, 0.001f);
    //   // m3508_anglesum_control(&m3508_fdcan1[3], (int32_t)debug_traj_m3508.out_pos, 8192);

    // }
  //多段轨迹-规划与执行
//=======================================================================

//=======================================================================
  //vofa显示调试信息

    // data_send_vofa[0] = A1_debug_485_1[1].measure.Pos;
    // data_send_vofa[1] = debug_traj_a1.out_pos;
    // if(debug_traj_queue_a1.tail > 0) {
    //     // 由于Traj_Path_Queue_Update的特性，需要-1才能显示当前正在前往的目标
    //     data_send_vofa[2] = debug_traj_queue_a1.points[debug_traj_queue_a1.tail - 1];
    // } else {
    //     data_send_vofa[2] = 0.0f;// 默认输出0
    // }
		// data_send_vofa[3] = (float)debug_count;

    // data_send_vofa[0] = arm.x0_now;
    // data_send_vofa[1] = arm_traj.traj_x.out_pos;
    // data_send_vofa[2] = arm_traj.traj_queue_x.points[arm_traj.traj_queue_x.tail > 0 ? arm_traj.traj_queue_x.tail - 1 : 0];
    // data_send_vofa[3] = arm.y0_now;
    // data_send_vofa[4] = arm_traj.traj_y.out_pos;
    // data_send_vofa[5] = arm_traj.traj_queue_y.points[arm_traj.traj_queue_y.tail > 0 ? arm_traj.traj_queue_y.tail - 1 : 0];

    data_send_vofa[0] = Radar_Pos.radar_x;
    data_send_vofa[1] = chassis_route_traj.traj_vx.out_pos;
    data_send_vofa[2] = chassis_route_traj.traj_vx.target_pos;
    data_send_vofa[3] = Radar_Pos.radar_y;        
    data_send_vofa[4] = chassis_route_traj.traj_vy.out_pos;
    data_send_vofa[5] = chassis_route_traj.traj_vy.target_pos;      
    data_send_vofa[6] = chassis_route_traj.traj_vx.out_vel;
    vofa_send_to_show_justfloat(&huart10, data_send_vofa);

  //vofa显示调试信息
//=======================================================================
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
// 机械臂dh参数初始化
  arm_joint_parameter_init(&arm);
// 3508参数初始化
  m3508_parameter_init(&m3508_fdcan1[0], 2.0f, 15.0f, 2.0f, 4.0f);
  m3508_parameter_init(&m3508_fdcan1[1], 3.0f, 10.0f, 1.6f, 2.0f);
  m3508_parameter_init(&m3508_fdcan1[2], 3.0f, 10.0f, 1.6f, 2.0f);
// // 机械臂路径参数初始化
//   arm_trajectory_init(&arm_traj, &arm);
// 给机械臂一个初始的待机位置
  arm_queue_add_safe(&arm_traj, 0.15f, 0.15f, 0.0f);

  // //预设控制位置
  // arm.x0_target = 0.15f;
  // arm.y0_target = 0.15f;

  /* Infinite loop */
  for(;;)
  {
    arm_joint_state_update(&arm);
    arm_route_point_calculate(&arm);

    //等待初始化完成
    if(m3508_fdcan1[0].Initial_flag == 200 && A1_debug_485_2[1].Pos_init_flag == 200)
    {    
    //保持执行器水平
      if(m3508_fdcan1[1].Initial_flag == 200 && m3508_fdcan1[2].Initial_flag == 200)
      {arm_actuator_stable_control(&arm, arm.effector_pitch_target, arm.theta_target[4]);}

    //启动机械臂
      if(HT10A.switch_d == 1)
      {
        if(debug_flag == 0)
        {
          arm_load_test_waypoints(); //预设一段测试路径，实际使用中可以替换为上位机发送的路径
          debug_flag += 1;
        }

        // 1. 调用多段直线插补协调器 (它会从队列中成组提取路点)
        arm_linear_interpolation_update(&arm, &arm_traj, 0.05f); 

        // 2. 分别推演 X 和 Y 的单段物理影子 (0.001f 代表 1ms)
        Traj_Adaptive_Update(&arm_traj.traj_x, arm.x0_now, 0.001f);
        Traj_Adaptive_Update(&arm_traj.traj_y, arm.y0_now, 0.001f);
        Traj_Adaptive_Update(&arm_traj.traj_w, A1_debug_485_2[0].measure.Pos, 0.001f);

        // 3. 将影子坐标交接给逆解器
        arm.x0_target = arm_traj.traj_x.out_pos;
        arm.y0_target = arm_traj.traj_y.out_pos;
        arm.theta_target[0] = arm_traj.traj_w.out_pos;

        // 4. 开始逆向解算出各个 theta_target，并转换成电机位置环
        arm_route_point_calculate(&arm);

      // 兜底：检测将要下发的位姿是否合法，若包含死点、反折点、越界：
        if(arm_route_reachable_check(&arm) != IK_REACHABLE)
        {
            // 清空荒诞的轨迹余党
            arm_traj.traj_queue_x.count = 0;
            arm_traj.traj_queue_y.count = 0;
            arm_traj.traj_queue_w.count = 0;
            // 将“影子进度”拽回到当前物理世界真正的坐标身上去！
            arm_trajectory_init(&arm_traj, &arm); 
            // 下发真实位置重新解算：机械臂瞬间进入抱死僵直锁定模式，不至于由于偏差而爆发堵转巨力
            arm.x0_target = arm.x0_now;
            arm.y0_target = arm.y0_now;
            arm.theta_target[0] = A1_debug_485_2[0].measure.Pos;
            arm_route_point_calculate(&arm);
        }

        arm_route_execute(&arm, 10.0f, 0.1f);
        
      }      
    //在不操纵机械臂的情况下默认举起到竖直位置，并时刻记录当前机械臂位姿，准备随时做路径切入
      else if(HT10A.switch_d == 0 || HT10A.switch_d == -1)
      {
        // 将机械臂抬起到竖直位置
        arm_joint_state_init(&arm);
        // 时刻初始化机械臂路径规划器的当前点为机械臂当前实际位姿，准备随时切入
        arm_trajectory_init(&arm_traj, &arm);
      }
    }
    osDelay(1);
  }
  /* USER CODE END entry_robotic_arm */
}

/* USER CODE BEGIN Header_entry_chassis */
/**
* @brief Function implementing the chassis_r2 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_chassis */
void entry_chassis(void const * argument)
{
  /* USER CODE BEGIN entry_chassis */
// 3508参数初始化
  m3508_parameter_init(&m3508_fdcan3[0], 2.5f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[1], 2.5f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[2], 2.5f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[3], 2.5f, 1.0f, 0.0f, 0.0f);
// 底盘升降初始化为收回状态
  chassis_r2.lift_front = 0;
  chassis_r2.lift_back = 0;
  chassis_lift_front_control(chassis_r2.lift_front);
  chassis_lift_back_control(chassis_r2.lift_back);

  /* Infinite loop */ 
  for(;;)
  {
   chassis_single_route_scheduler(&Radar_Pos, &chassis_route_traj,0.001);

   omni_chassis_calculate(&chassis_r2, chassis_route_traj.traj_vx.out_vel, chassis_route_traj.traj_vy.out_vel, chassis_route_traj.traj_vw.out_vel);
    // if(m3508_fdcan3[0].Initial_flag == 200 && m3508_fdcan3[1].Initial_flag == 200 
    //   && m3508_fdcan3[2].Initial_flag == 200 && m3508_fdcan3[3].Initial_flag == 200)
    // {
    //   //switch_d为0时控制底盘
    //   if(HT10A.switch_d == 0)
    //   {
    //     chassis_r2.vx = HT10A.left_y;
    //     chassis_r2.vy = -HT10A.left_x;
    //     chassis_r2.vw = -HT10A.right_x;
    //     //遥控器按钮控制升降!!!!!!!!!!已将切换逻辑转移至ht10a.c中，避免了升降状态与底盘控制状态的耦合
    //     // if(HT10A.switch_b == 1)       {chassis_r2.lift_front = 1;}
    //     // else if(HT10A.switch_b == -1) {chassis_r2.lift_front = 0;}
    //     // if(HT10A.switch_c == 1)       {chassis_r2.lift_back = 1;}
    //     // else if(HT10A.switch_c == -1) {chassis_r2.lift_back = 0;}
    //     chassis_lift_front_control(chassis_r2.lift_front);
    //     chassis_lift_back_control(chassis_r2.lift_back);
    //     omni_chassis_calculate(&chassis_r2, chassis_r2.vx, chassis_r2.vy, chassis_r2.vw);
    //   }
    //   //当不主动控制底盘时，默认停止底盘，但升降状态保留
    //   else if(HT10A.switch_d == 1 || HT10A.switch_d == -1)
    //   {
    //     chassis_r2.vx = 0;
    //     chassis_r2.vy = 0;
    //     chassis_r2.vw = 0;
    //     chassis_lift_front_control(chassis_r2.lift_front);
    //     chassis_lift_back_control(chassis_r2.lift_back);
    //     omni_chassis_calculate(&chassis_r2, chassis_r2.vx, chassis_r2.vy, chassis_r2.vw);
    //   }
    // }
    osDelay(1);
  }
  /* USER CODE END entry_chassis */
}

/* USER CODE BEGIN Header_entry_clamp */
/**
* @brief Function implementing the clamp_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_clamp */
void entry_clamp(void const * argument)
{
  /* USER CODE BEGIN entry_clamp */

  //夹爪电机初始化
	m3508_parameter_init(&m3508_fdcan2[0],5,2,0.5,0.2);
  m3508_parameter_init(&m3508_fdcan2[1],5,2,0.5,0.2);
  m3508_parameter_init(&m3508_fdcan2[2],5,2,0.5,0.2);
	m3508_parameter_init(&m3508_fdcan2[3],5,2,0.5,0.2);
  //标志位初始化
  clamp.catch_flag = 0;
  clamp.rotate_flag = 0;
  clamp.retractable_flag = 0;
  clamp.lift_flag = 0;

  /* Infinite loop */
  for(;;)
  {
    if(  m3508_fdcan2[1].Initial_flag == 200 
      && m3508_fdcan2[2].Initial_flag == 200 && m3508_fdcan2[3].Initial_flag == 200)
    {
      if(HT10A.switch_d == -1)
      {
        //!!!!!!!!!!!!!!!该部分切换逻辑转移至ht10a.c中

        //气缸驱动夹爪部分
        // if      (HT10A.switch_b == 1) {clamp.catch_flag = 1;}
        // else if (HT10A.switch_b == -1) {clamp.catch_flag = 0;}
        clamp_open_control(clamp.catch_flag);
        //夹爪旋转部分
        // if      (HT10A.knob_a >=1000) {clamp.rotate_flag = 1;}
        // else if (HT10A.knob_a <= -1000) {clamp.rotate_flag = 0;}
        clamp_rotate_control(clamp.rotate_flag * CLAMP_ROTATE_MAX_POSITION, &m3508_fdcan2[3]);
        //夹爪伸缩部分
        // if      (HT10A.switch_c == 1) {clamp.retractable_flag = 1;}
        // else if (HT10A.switch_c == -1) {clamp.retractable_flag = 0;}
        clamp_extend_control(clamp.retractable_flag * CLAMP_EXTEND_MAX_POSITION, &m3508_fdcan2[2]);
        //夹爪升降部分
        if(abs_int16(HT10A.left_y) > 100){m3508_speed_control(&m3508_fdcan2[1], HT10A.left_y, 8192);}
        else                             {m3508_speed_control(&m3508_fdcan2[1], 0, 8192);}
      }
      else if(HT10A.switch_d == 1 || HT10A.switch_d == 0)
      {
        clamp_open_control(clamp.catch_flag);
        clamp_rotate_control(clamp.rotate_flag * CLAMP_ROTATE_MAX_POSITION, &m3508_fdcan2[3]);
        clamp_extend_control(clamp.retractable_flag * CLAMP_EXTEND_MAX_POSITION, &m3508_fdcan2[2]);
        m3508_speed_control(&m3508_fdcan2[1], 0, 8192);
      }
    }

    osDelay(1);
  }
  /* USER CODE END entry_clamp */
}

/* USER CODE BEGIN Header_entry_cmd_task */
/**
* @brief Function implementing the cmd_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_cmd_task */
void entry_cmd_task(void const * argument)
{
  /* USER CODE BEGIN entry_cmd_task */
  /* Infinite loop */
  for(;;)
  {
    // 1. 先比较：本地上一次记录的旧状态(HT10A_last) 和 最新的全局状态(HT10A)
    customized_judgment(); //根据比较结果进行各种判断和操作，具体逻辑由用户在 ht10a.c 的 customized_judgment() 中编写

    // 2. 后更新：判断完之后，再把最新的全局状态拷贝到本地，留作下一次循环(1ms后)对比使用
    HT10A_last = HT10A;


    osDelay(1);
  }
  /* USER CODE END entry_cmd_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
