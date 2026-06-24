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
#include "tim.h"
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
#include "go_m8010.h"
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

int32_t debug_flag; // 万能小变量
float debug_position;
uint8_t point_add = 0;           // 测试用：是否添加路径点的标志位
uint8_t point_add1 = 0;          // 测试用：是否添加路径点的标志位
uint8_t kalman_update_times = 0; // 卡尔曼滤波更新次数统计
uint8_t kalman_init_flag = 0; // 卡尔曼滤波初始化flag
float vx, vy, vw = 0.0f; // 用于测试的底盘速度变量
float speed = 0.0f; // 用于测试的速度变量 
uint8_t catch_flag = 0; // 测试用：夹爪开合标志位 
int32_t rotate_target_angle = 0; // 测试用：夹爪旋转目标角度  
uint8_t task_flag = 0; // 测试用：总开关，控制是否执行BT_Combined_Task_Tree_Root函数  
float debug_speed; // 用于测试的速度变量

uint8_t traj_init_flag_arm = 0;  // 机械臂轨迹初始化标志位
uint8_t traj_init_flag_lift = 0; // 抬升轨迹初始化标志位
uint8_t traj_init_flag_chassis = 0; // 底盘轨迹初始化标志位

extern volatile uint8_t a1_send_request;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId roboticarm_taskHandle;
osThreadId chassis_taskHandle;
osThreadId clamp_taskHandle;
osThreadId cmd_taskHandle;
osThreadId m8010_taskHandle;
osThreadId a1_taskHandle;
osSemaphoreId bisem_m8010Handle;
osStaticSemaphoreDef_t bisem_m8010ControlBlock;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//#ifndef bisem_m8010Handle
//	osSemaphoreId bisem_m8010Handle;
//#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void entry_robotic_arm(void const * argument);
void entry_chassis(void const * argument);
void entry_clamp(void const * argument);
void entry_cmd_task(void const * argument);
void entry_m8010_task(void const * argument);
void entry_a1_task(void const * argument);

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

  /* Create the semaphores(s) */
  /* definition and creation of bisem_m8010 */
  osSemaphoreStaticDef(bisem_m8010, &bisem_m8010ControlBlock);
  bisem_m8010Handle = osSemaphoreCreate(osSemaphore(bisem_m8010), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */

  // CubeMX 创建的信号量默认计数为 1，需要先 take 掉让它变为 0
  osSemaphoreWait(bisem_m8010Handle, 0); // 非阻塞地取走初始的 1，使计数变为 0

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

  /* definition and creation of m8010_task */
  osThreadDef(m8010_task, entry_m8010_task, osPriorityNormal, 0, 128);
  m8010_taskHandle = osThreadCreate(osThread(m8010_task), NULL);

  /* definition and creation of a1_task */
  osThreadDef(a1_task, entry_a1_task, osPriorityNormal, 0, 128);
  a1_taskHandle = osThreadCreate(osThread(a1_task), NULL);

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

  // start tim2
  TIM2->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim2);
  // start tim6
  TIM6->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim6);
  // start tim7
  TIM7->CNT = 0;
  HAL_TIM_Base_Start_IT(&htim7);

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

    // data_send_vofa[0] = Radar_Pos.reliable_yaw;
    // data_send_vofa[1] = chassis_route_traj.traj_vw.target_pos;
    // data_send_vofa[2] = Radar_Pos.reliable_x;

    // 此处measure.Pos是实际转子位置减去初始位置（即电机上电时的位置），加上初始位置的目的是为了显示实际的机械位置（相对于上电时的零点）
    // 因此此处将measure.pos加上init_pos显示的数值是电机的实际位置，更直观地和发给电机的位置环目标进行对比
    // data_send_vofa[0] =chassis_route_traj.traj_vx.out_pos ;
    // data_send_vofa[1] = Radar_Pos.reliable_x;
    // data_send_vofa[2] = chassis_route_traj.traj_vx.out_vel;
    // data_send_vofa[3] = chassis_route_traj.traj_vx.target_pos;
    // data_send_vofa[4] = Radar_Pos.radar_x;
    // data_send_vofa[5] = Radar_Pos.radar_x_filtered;
    // data_send_vofa[6] = m3508_fdcan3[0].Now_Speed;
    // data_send_vofa[7] = m3508_fdcan3[0].M3508_Speed_PID.Target;
    // data_send_vofa[0] = m3508_fdcan3[0].Now_Speed;
    // data_send_vofa[1] = m3508_fdcan3[0].M3508_Speed_PID.Target;
    // data_send_vofa[2] = m3508_fdcan3[1].Now_Speed;
    // data_send_vofa[3] = m3508_fdcan3[1].M3508_Speed_PID.Target;
    // data_send_vofa[4] = m3508_fdcan3[2].Now_Speed;
    // data_send_vofa[5] = m3508_fdcan3[2].M3508_Speed_PID.Target;
    // data_send_vofa[6] = m3508_fdcan3[3].Now_Speed;
    // data_send_vofa[7] = m3508_fdcan3[3].M3508_Speed_PID.Target;
    data_send_vofa[0] = arm.theta_target[0];
    data_send_vofa[1] = arm.joint_parameters[1].theta;
    data_send_vofa[2] = arm.theta_target[1];
    data_send_vofa[3] = arm.joint_parameters[2].theta;
    data_send_vofa[4] = arm.x0_now;
    data_send_vofa[5] = arm_traj.traj_x.out_pos;
    data_send_vofa[6] = arm.y0_now;
    data_send_vofa[7] = arm_traj.traj_y.out_pos;
    vofa_send_to_show_justfloat(&huart7, data_send_vofa);

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
   m3508_parameter_init(&m3508_fdcan1[0], 1.0f, 0.0f, 1.0f, 1.5f, 0.0f, 12.5f);
  // m3508_parameter_init(&m3508_fdcan1[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // 末端执行器默认目标姿态
  arm.effector_pitch_target = 0.0f;
  // arm.theta_target[4] = 0.0f;

  // 初始化机械臂工作区域
  arm.mode = ARM_MODE_FRONT;

  /* Infinite loop */
  for (;;)
  {
    arm_joint_state_update(&arm);
    arm_route_point_calculate_front(&arm);

  /* 调试相关代码，实际使用需要被注释掉 */
    if(debug_flag == 1){
      debug_flag = 0;
      arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_FRONT_TO_BACK);
    }
    if(debug_flag == 2){
      debug_flag = 0;
      arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_BACK_TO_FRONT);
    }
    if(debug_flag == 3){
      debug_flag = 0;
      arm_queue_add_point(&arm_traj, 0.4, 0.4, 0.0f, 500, 0);
    }
    if(debug_flag == 4){
      debug_flag = 0;
      arm_queue_add_point(&arm_traj, 0.5, 0.5, 0.0f, 500, 0);
    }
    if(debug_flag == 5){
      debug_flag = 0;
      arm_queue_add_point(&arm_traj, -0.4, 0.4, 0.0f, 500, 0);
    }
    if(debug_flag == 6){
      debug_flag = 0;
      arm_queue_add_point(&arm_traj, -0.5, 0.5, 0.0f, 500, 0);
    }

    // 等待初始化完成
    if(A1_debug_485_2[0].Pos_init_flag == 200 && m8010_bus_1.motors[2].measure.Pos_init_flag == 200)
    {
      if (traj_init_flag_arm == 0)
      {
        arm_trajectory_init(&arm_traj, &arm);   // 以当前位置初始化轨迹系统
        arm_queue_add_point(&arm_traj, 0.4, 0.5, 0.0f, 500, 0);
        traj_init_flag_arm = 1;
      }

      // 保持执行器水平
      if (m3508_fdcan1[0].Initial_flag == 200 && arm.mode != ARM_MODE_TRANSITION)
      {
        arm_actuator_stable_control(&arm, arm.effector_pitch_target, arm.theta_target[4]);
      }

      // 1. 调用多段直线插补协调器 (它会从队列中成组提取路点)
      arm_linear_interpolation_update(&arm, &arm_traj, 0.05f);

      // 2. 分别推演 X 和 Y 的单段物理影子 (0.001f 代表 1ms)
      Traj_Adaptive_Update(&arm_traj.traj_x, arm.x0_now, 0.001f);
      Traj_Adaptive_Update(&arm_traj.traj_y, arm.y0_now, 0.001f);
      // Traj_Adaptive_Update(&arm_traj.traj_w, m8010_bus_1.motors[0].measure.Pos, 0.001f);

      // 3. 将影子坐标交接给逆解器（当工作区不处于过渡模式时）
      if(arm.mode == ARM_MODE_FRONT || arm.mode == ARM_MODE_BACK)
      {
        arm.x0_target = arm_traj.traj_x.out_pos;
        arm.y0_target = arm_traj.traj_y.out_pos;
        // arm.theta_target[0] = arm_traj.traj_w.out_pos;
      }

//       // 4. 开始逆向解算出各个 theta_target，并转换成电机位置环
//       arm_route_point_calculate(&arm);
// //      arm_route_execute(&arm, 3.5f, 0.35f); // 位置环控制
//       arm_route_execute(&arm, 0.0f, 0.0f, 0.0f, 0.0f);  // 不控制
//       // arm_gravity_comp(); // 重力补偿

      // 4. 逆解算出各关节的 theta_target ，并转换成电机位置环
      arm_mode_update_execute(&arm, &arm_traj, &arm_trans_traj, 5.0f, 0.2f, 0.3f, 3.5f); // 位置环控制
      // arm_mode_update_execute(&arm, &arm_traj, &arm_trans_traj, 0.0f, 0.0f, 0.0f, 0.0f);

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
  // m3508_parameter_init(&m3508_fdcan3[0], 17.0f, 0.00f, 3.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan3[1], 15.0f, 0.00f, 3.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan3[2], 15.0f, 0.00f, 5.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan3[3], 16.0f, 0.00f, 3.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[0], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[1], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[2], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan3[3], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
	//抬升机构辅助轮电机初始化
	// m3508_parameter_init(&m3508_fdcan1[1], 15.0f, 0.00f, 2.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan1[2], 15.0f, 0.00f, 2.0f, 0.0f, 0.0f, 0.0f);
	m3508_parameter_init(&m3508_fdcan1[1], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan1[2], 0.0f, 0.00f, 0.0f, 0.0f, 0.0f, 0.0f);
  // 底盘升降初始化为收回状态
  // chassis_r2.lift_front = 0;
  // chassis_r2.lift_back = 0;
  // chassis_lift_front_control(chassis_r2.lift_front);
  // chassis_lift_back_control(chassis_r2.lift_back);
  chassis_route_traj_init();
  /* Infinite loop */
  for (;;)
  {
    if (m3508_fdcan3[0].Initial_flag == 200 && m3508_fdcan3[1].Initial_flag == 200 && m3508_fdcan3[2].Initial_flag == 200 && m3508_fdcan3[3].Initial_flag == 200)
    {
      // 测试用添加路径点
          //  if(point_add == 1)
          //  {
          //    chassis_route_point_add(&chassis_route_traj, 600.0f, 2000.0f, -180.0f,0);
          //    point_add = 0;
          //  }
          //  if(point_add1 ==1)
          //  {
          //    chassis_route_point_add(&chassis_route_traj, 450.0f,1400.0f, 0.0f,0);
          //    point_add1 = 0;
          //  }

      // 滤波后的雷达数据更新
      Radar_Pos.radar_x_filtered = Kalman_Update(&Radar_Kalman_X, Radar_Pos.radar_x);
      Radar_Pos.radar_y_filtered = Kalman_Update(&Radar_Kalman_Y, Radar_Pos.radar_y);
      Radar_Pos.radar_z_filtered = Kalman_Update(&Radar_Kalman_Z, Radar_Pos.radar_z);
      // 利用电机速度计算底盘实际速度（逆解算）
      omni_chassis_inverse_calculate(&chassis_r2, m3508_fdcan3);

      // 当底盘实际速度足够小时，认为雷达数据不可靠，使用滤波后的雷达数据
      if (abs_float(Radar_Pos.radar_x_filtered - Radar_Pos.radar_x) <5 && abs_float(Radar_Pos.radar_y_filtered - Radar_Pos.radar_y) <5 && abs_float(chassis_route_traj.traj_vy.target_pos - Radar_Pos.radar_y_filtered) < 10 && abs_float(chassis_route_traj.traj_vx.target_pos - Radar_Pos.radar_x_filtered) < 20 && abs_float(chassis_route_traj.traj_vx.target_pos - Radar_Pos.radar_x) < 10 && abs_float(chassis_route_traj.traj_vy.target_pos - Radar_Pos.radar_y) < 10)
      {
        Radar_Pos.reliable_x = Radar_Pos.radar_x_filtered;
        Radar_Pos.reliable_y = Radar_Pos.radar_y_filtered;
        Radar_Pos.reliable_z = Radar_Pos.radar_z_filtered;
        Radar_Pos.reliable_yaw = Radar_Pos.hi14_yaw; // 直接使用hi14的yaw数据，替代雷达的yaw数据
      }
      // 否则认为雷达数据可靠，直接使用原始雷达数据
      else
      {
        Radar_Pos.reliable_x = Radar_Pos.radar_x;
        Radar_Pos.reliable_y = Radar_Pos.radar_y;
        Radar_Pos.reliable_z = Radar_Pos.radar_z;
        Radar_Pos.reliable_yaw = Radar_Pos.hi14_yaw;
      }
      if(traj_init_flag_chassis == 0)
      {
        // 以当前坐标初始化底盘路径系统
        chassis_route_traj_init();
        traj_init_flag_chassis = 1;
      }


      // 多段-单段-坐标系变换-底盘解算 路径执行部分
      chassis_multi_route_scheduler(&chassis_route_traj, &Radar_Pos);
      chassis_single_route_scheduler(&Radar_Pos, &chassis_route_traj, 1);
      chassis_world_to_robot_velocity(&chassis_r2, chassis_route_traj.traj_vx.out_vel, chassis_route_traj.traj_vy.out_vel, chassis_route_traj.traj_vw.out_vel, Radar_Pos.reliable_yaw);
      omni_chassis_calculate(&chassis_r2, chassis_r2.vx, chassis_r2.vy, chassis_r2.vw);
    }

    /* ========================= 抬升部分 ========================= */

    if (m8010_bus_3.motors[0].measure.Pos_init_flag == 200 && m8010_bus_3.motors[1].measure.Pos_init_flag == 200)
    {
      // 1. 正向运动学更新当前升降高度
      chassis_lift_fk_update(&chassis_r2);
      if (traj_init_flag_lift == 0)
      {
        // 以当前升降高度初始化升降轨迹系统
        chassis_lift_traj_init();
        // 添加一个默认的升降目标点，作为上电复位
        chassis_lift_queue_add_point(&chassis_lift_traj, 0.220f, 0.020f, 0);
        traj_init_flag_lift = 1;
      }
      // if (debug_flag == 1)
      // {
      //   chassis_lift_queue_add_point(&chassis_lift_traj, -0.025f, -0.225f, 0);
      //   debug_flag = 0;
      // }
      // if (debug_flag == 2)
      // {
      //   chassis_lift_queue_add_point(&chassis_lift_traj, 0.230f, 0.030f, 0);
      //   debug_flag = 0;
      // }
      // 2. 多段路径调度器：当前段完成→从队列出队下一个目标
      chassis_lift_linear_interpolation_update(&chassis_r2, &chassis_lift_traj, CHASSIS_LIFT_QUEUE_BLEND_RADIUS);
      // 3. 单段轨迹推演（影子位置 + 速度 + 自动判敛）
      Traj_Adaptive_Update(&chassis_lift_traj.traj_front, chassis_r2.lift_front_current, 0.001f);
      Traj_Adaptive_Update(&chassis_lift_traj.traj_back, chassis_r2.lift_back_current, 0.001f);
      // 4. 影子位置 → 目标高度
      chassis_r2.lift_front_target = chassis_lift_traj.traj_front.out_pos;
      chassis_r2.lift_back_target = chassis_lift_traj.traj_back.out_pos;
      // 5. 逆运动学：目标高度 → 电机位置
      chassis_lift_ik_update(&chassis_r2);
      // 6. 位置环执行
      // chassis_lift_excute(&chassis_r2, 2.0f, 0.2f);
      chassis_lift_excute(&chassis_r2, 0.0f, 0.0f);
      // 7. 抬升轮速度执行
      // chassis_lift_wheel_excute(debug_speed, 2.0f, 0.2f);
      chassis_lift_wheel_excute(debug_speed, 0.0f, 0.0f);

    }

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
  // m3508_parameter_init(&m3508_fdcan2[0], 5.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  // m3508_parameter_init(&m3508_fdcan2[1], 5.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  // m3508_parameter_init(&m3508_fdcan2[2], 5.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan2[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan2[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan2[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

  // 标志位初始化
  clamp.catch_flag = 0;
  clamp.rotate_flag = 0;
  clamp.retractable_flag = 0;
  clamp.lift_flag = 0;

  /* Infinite loop */
  for (;;)
  {
    if (m3508_fdcan2[0].Initial_flag == 200 && m3508_fdcan2[1].Initial_flag == 200 && m3508_fdcan2[2].Initial_flag == 200)
    {

      // 气缸驱动夹爪部分
      clamp_open_control(clamp.catch_flag);
      // 夹爪旋转部分
      clamp_rotate_control(clamp.rotate_flag * CLAMP_ROTATE_MAX_POSITION, &m3508_fdcan2[2]);
      // 夹爪伸缩部分
      clamp_extend_control(clamp.retractable_flag * CLAMP_EXTEND_MAX_POSITION, &m3508_fdcan2[1]);
      // // 夹爪升降部分
      // clamp_lift_control(clamp.lift_flag, &m3508_fdcan2[0]);
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

  BT_MASTER_Init_Task();  // 初始化行为树任务
  // BT_MASTER_Start_Task(); // 启动行为树主控任务

  /* Infinite loop */
  for (;;)
  {
    // BT_MASTER_Tick();     // 每1ms调用，内部降频10ms，行为树主控任务周期调用

    osDelay(1);
  }
  /* USER CODE END entry_cmd_task */
}

/* USER CODE BEGIN Header_entry_m8010_task */
/**
* @brief Function implementing the m8010_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_m8010_task */
void entry_m8010_task(void const * argument)
{
  /* USER CODE BEGIN entry_m8010_task */
	
  m8010_bus_1.motors[0].target.id = 1;
  m8010_bus_1.motors[0].target.mode = 1;
  m8010_bus_1.motors[1].target.id = 2;
  m8010_bus_1.motors[1].target.mode = 1;
  m8010_bus_1.motors[2].target.id = 3;
  m8010_bus_1.motors[2].target.mode = 1;

  m8010_bus_3.motors[0].target.id = 1;
  m8010_bus_3.motors[0].target.mode = 1;
  m8010_bus_3.motors[1].target.id = 2;
  m8010_bus_3.motors[1].target.mode = 1;
  // m8010_bus_3.motors[2].target.id = 3;
  // m8010_bus_3.motors[2].target.mode = 1;

  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(bisem_m8010Handle, osWaitForever); // 等待信号量，确保在主循环中被正确触发

    m8010_bus_poll(&m8010_bus_1);
    // m8010_bus_poll(&m8010_bus_2);
    m8010_bus_poll(&m8010_bus_3);
  
    osDelay(1);
  }
  /* USER CODE END entry_m8010_task */
}

/* USER CODE BEGIN Header_entry_a1_task */
/**
* @brief Function implementing the a1_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entry_a1_task */
void entry_a1_task(void const * argument)
{
  /* USER CODE BEGIN entry_a1_task */
  uint8_t timer_a1_flag = 0;
  /* Infinite loop */
  for(;;)
  {
    if (a1_send_request)
    {
      a1_send_request = 0;

      switch(timer_a1_flag)
      {
        case 0:
          A1_Motor_Send_Data(&A1_debug_485_2[0], &huart6);
          break;
        case 1:
          A1_Motor_Send_Data(&A1_debug_485_2[1], &huart6);
          break;
      }

      timer_a1_flag++;
      if (timer_a1_flag > 1) timer_a1_flag = 0;
    }

    osDelay(1);
  }
  /* USER CODE END entry_a1_task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
