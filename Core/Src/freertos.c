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
#include "BT_customized.h"
#include "radar.h"
#include "MF_planner.h"
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
float debug_test_time; // 用于测试的万能小变量

int32_t debug_flag;
float debug_position;
uint8_t point_add = 0; // 测试用：是否添加路径点的标志位
uint8_t point_add1 = 0; // 测试用：是否添加路径点的标志位 
uint8_t kalman_update_times = 0; // 卡尔曼滤波更新次数统计
uint8_t kalman_init_flag = 0; // 卡尔曼滤波初始化flag


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

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize)
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

  MFPlanner_Init(&mf_planner_ctx);
  MFPlanner_LoadFixedDemoMap(&mf_planner_ctx);
  MFPlanner_BuildFullPlan(&mf_planner_ctx, 0, 1, 3); // 从入口区地面(0,1)出发，规划拾取3个R2KFS的完整动作列表

  /* Infinite loop */
  for (;;)
  {

    

  //=======================================================================
  // vofa显示调试信息

    // data_send_vofa[0] = arm.x0_now;
    // data_send_vofa[1] = arm_traj.traj_x.out_pos;
    // data_send_vofa[2] = arm_traj.traj_queue_x.points[arm_traj.traj_queue_x.tail > 0 ? arm_traj.traj_queue_x.tail - 1 : 0];
    // data_send_vofa[3] = arm.y0_now;
    // data_send_vofa[4] = arm_traj.traj_y.out_pos;
    // data_send_vofa[5] = arm_traj.traj_queue_y.points[arm_traj.traj_queue_y.tail > 0 ? arm_traj.traj_queue_y.tail - 1 : 0];

    data_send_vofa[0] = Radar_Pos.reliable_yaw;

    data_send_vofa[1] = chassis_route_traj.traj_vw.target_pos;
		data_send_vofa[2] = Radar_Pos.reliable_x;
    vofa_send_to_show_justfloat(&huart10, data_send_vofa);

  // vofa显示调试信息
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
// 机械臂上的3508参数初始化
  m3508_parameter_init(&m3508_fdcan1[0], 2.2f, 15.0f, 2.2f, 6.0f);
  m3508_parameter_init(&m3508_fdcan1[1], 1.8f, 3.5f, 1.5f, 1.0f);
  m3508_parameter_init(&m3508_fdcan1[2], 1.8f, 3.5f, 1.5f, 1.0f);

// 末端执行器默认目标姿态
  arm.effector_pitch_target = -PI/2.0f;
  arm.theta_target[4] = 0.0f;

// 向机械臂添加默认路径点，替代之前的单位置环上电复位
  arm_queue_add_point(&arm_traj, 0.0f, 0.40f, 0.0f, 500, 0);
  arm_queue_add_point(&arm_traj, 0.15f, 0.15f, 0.0f, 0, 0);

  /* Infinite loop */
  for (;;)
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
      // if(HT10A.switch_d == 1)
      // {

        // 1. 调用多段直线插补协调器 (它会从队列中成组提取路点)
        arm_linear_interpolation_update(&arm, &arm_traj, 0.03f); 

        // 2. 分别推演 X 和 Y 的单段物理影子 (0.001f 代表 1ms)
        Traj_Adaptive_Update(&arm_traj.traj_x, arm.x0_now, 0.001f);
        Traj_Adaptive_Update(&arm_traj.traj_y, arm.y0_now, 0.001f);
        Traj_Adaptive_Update(&arm_traj.traj_w, A1_debug_485_2[0].measure.Pos, 0.001f);

        // 3. 将影子坐标交接给逆解器
// ==============4月28日更新：去除遥控模式=====================

        arm.x0_target = arm_traj.traj_x.out_pos;
        arm.y0_target = arm_traj.traj_y.out_pos;
        arm.theta_target[0] = arm_traj.traj_w.out_pos;

        // 4. 开始逆向解算出各个 theta_target，并转换成电机位置环
        arm_route_point_calculate(&arm);
        // 兜底：检测将要下发的位姿是否合法，若包含死点、反折点、越界：
        if(arm_route_reachable_check(&arm) != IK_REACHABLE)
        {
            // 清空非法轨迹残留
            arm_traj.traj_queue_x.count = 0;
            arm_traj.traj_queue_y.count = 0;
            arm_traj.traj_queue_w.count = 0;
            // 将“影子进度”拽回到当前物理世界真正的坐标身上去！
            arm_trajectory_init(&arm_traj, &arm); 
            // 下发真实位置重新解算：机械臂进入阻尼缓下落模式
            arm.x0_target = arm.x0_now;
            arm.y0_target = arm.y0_now;
            arm.theta_target[0] = A1_debug_485_2[0].measure.Pos;
            arm_route_point_calculate(&arm);
        }
        arm_route_execute(&arm, 8.0f, 0.20f);
      // }
      
    //在不操纵机械臂的情况下默认举起到竖直位置，并时刻记录当前机械臂位姿，准备随时做路径切入
      // else if(HT10A.switch_d == 0 || HT10A.switch_d == -1)
      // {
      //   // 将机械臂抬起到竖直位置
      //   arm_joint_state_init(&arm);

      //   // 时刻初始化机械臂路径规划器的当前点为机械臂当前实际位姿，准备随时切入
      //   arm_trajectory_init(&arm_traj, &arm);
      // }
      // if(HT10A.switch_d == 1 && air_pump_flag_remote == 1) {HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_SET);} // 开启吸泵
      // else                          {HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_RESET);} // 关闭吸泵
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
  m3508_parameter_init(&m3508_fdcan3[0], 8.0f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[1], 8.0f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[2], 8.0f, 1.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[3], 8.0f, 1.0f, 0.0f, 0.0f);
  // 底盘升降初始化为收回状态
  chassis_r2.lift_front = 0;
  chassis_r2.lift_back = 0;
  chassis_lift_front_control(chassis_r2.lift_front);
  chassis_lift_back_control(chassis_r2.lift_back);
  chassis_route_traj_init();
  /* Infinite loop */
  for (;;)
  {
// 测试用添加路径点
    if(m3508_fdcan3[0].Initial_flag == 200 && m3508_fdcan3[1].Initial_flag == 200
      && m3508_fdcan3[2].Initial_flag == 200 && m3508_fdcan3[3].Initial_flag == 200)
    {
      if(point_add == 1)
      {
        //      chassis_route_point_add(&chassis_route_traj,55.0f,66.0f,0.0f);
        chassis_route_point_add(&chassis_route_traj, 1250.0f, 560.0f, 0.0f,0);
        // chassis_route_point_add(&chassis_route_traj, 2200.0f, 3000.0f, 180.0f);
        // chassis_route_point_add(&chassis_route_traj, 1200.0f, 3000.0f, -45.0f);
        point_add = 0;
      }
      if(point_add1 ==1)
      {
        chassis_route_point_add(&chassis_route_traj, 1200.0f,2000.0f, 180.0f,0);
        point_add1 = 0;
      }
// 测试用添加路径点

      // 滤波后的雷达数据更新
      Radar_Pos.radar_x_filtered = Kalman_Update(&Radar_Kalman_X, Radar_Pos.radar_x);
      Radar_Pos.radar_y_filtered = Kalman_Update(&Radar_Kalman_Y, Radar_Pos.radar_y);
      Radar_Pos.radar_z_filtered = Kalman_Update(&Radar_Kalman_Z, Radar_Pos.radar_z);
 // Radar_Pos.radar_yaw_filtered = Kalman_Update(&Radar_Kalman_Yaw, Radar_Pos.radar_yaw);
      // 利用电机速度计算底盘实际速度（逆解算）
      omni_chassis_inverse_calculate(&chassis_r2, m3508_fdcan3);

      // 当底盘实际速度足够小时，认为雷达数据不可靠，使用滤波后的雷达数据
      if (abs_float(chassis_r2.real_vx) < RADAR_Filter_Threshold && abs_float(chassis_r2.real_vy) < RADAR_Filter_Threshold)
      {
        // if(kalman_init_flag == 0)
        // {
        //   kalman_init_flag = 1; // 仅在第一次满足条件时初始化卡尔曼滤波器，避免频繁初始化
        //   Kalman_Init(&Radar_Kalman_X, 0.0001f, 100.0f, 0.0f, Radar_Pos.radar_x);
        //   Kalman_Init(&Radar_Kalman_Y, 0.0001f, 100.0f, 0.0f, Radar_Pos.radar_y);
        // }
        Radar_Pos.reliable_x = Radar_Pos.radar_x_filtered;
        Radar_Pos.reliable_y = Radar_Pos.radar_y_filtered;
        Radar_Pos.reliable_z = Radar_Pos.radar_z_filtered;
        Radar_Pos.reliable_yaw = Radar_Pos.radar_yaw;
      }
      // 否则认为雷达数据可靠，直接使用原始雷达数据
      else
      {
        Radar_Pos.reliable_x = Radar_Pos.radar_x;
        Radar_Pos.reliable_y = Radar_Pos.radar_y;
        Radar_Pos.reliable_z = Radar_Pos.radar_z;
        Radar_Pos.reliable_yaw = Radar_Pos.radar_yaw;
      }

      // 多段-单段-坐标系变换-底盘解算 路径执行部分
      chassis_multi_route_scheduler(&chassis_route_traj, &Radar_Pos);
      chassis_single_route_scheduler(&Radar_Pos, &chassis_route_traj, 1);
      chassis_world_to_robot_velocity(&chassis_r2, chassis_route_traj.traj_vx.out_vel, chassis_route_traj.traj_vy.out_vel, chassis_route_traj.traj_vw.out_vel, Radar_Pos.reliable_yaw);
      omni_chassis_calculate(&chassis_r2, chassis_r2.vx, chassis_r2.vy, chassis_r2.vw);
    }
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

  // 夹爪电机初始化
  m3508_parameter_init(&m3508_fdcan2[0], 5, 2, 0.5, 0.2);
  m3508_parameter_init(&m3508_fdcan2[1], 5, 2, 0.5, 0.2);
  m3508_parameter_init(&m3508_fdcan2[2], 5, 2, 0.5, 0.2);
  m3508_parameter_init(&m3508_fdcan2[3], 5, 2, 0.5, 0.2);
  // 标志位初始化
  clamp.catch_flag = 0;
  clamp.rotate_flag = 0;
  clamp.retractable_flag = 0;
  clamp.lift_flag = 0;

  /* Infinite loop */
  for (;;)
  {
    if (m3508_fdcan2[1].Initial_flag == 200 && m3508_fdcan2[2].Initial_flag == 200 && m3508_fdcan2[3].Initial_flag == 200)
    {
        // 气缸驱动夹爪部分
        clamp_open_control(clamp.catch_flag);
        // 夹爪旋转部分
        clamp_rotate_control(clamp.rotate_flag * CLAMP_ROTATE_MAX_POSITION, &m3508_fdcan2[3]);
        // 夹爪伸缩部分
        clamp_extend_control(clamp.retractable_flag * CLAMP_EXTEND_MAX_POSITION, &m3508_fdcan2[2]);
        // 夹爪升降部分
        clamp_lift_control(clamp.lift_flag, &m3508_fdcan2[1]);
    
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

  // 行为树相关初始化
  BT_Init_Combined_Task(); // 初始化行为树模块（黑板、状态机等）

  /* Infinite loop */
  for (;;)
  {

    // switch_d为 1 并且 switch_a 从 0 变为 1 时，发起一次自动组合任务请求
    if (HT10A.switch_d == 1 &&(HT10A_last.switch_a == 0 && HT10A.switch_a == 1)) 
    {
      BT_Start_Combined_Task();
    }

    // 调用 BT_Customized_Tick1ms_Combined（内部做 10ms 降频）
    BT_Customized_Tick1ms_Combined();

    HT10A_last = HT10A;
    osDelay(1);
  }
  /* USER CODE END entry_cmd_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
