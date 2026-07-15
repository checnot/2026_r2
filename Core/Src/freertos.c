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
#include "lift.h"
#include "H7_F1_spi.h"
#include "ir_action.h"
//#include "Ana_keyboard.h"
#include "screen.h"
#include "keyboard.h"
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
uint8_t debug_mbt_start;  // 调试用行为树启动变量
float debug_position;
uint8_t point_add = 0;           // 测试用：是否添加路径点的标志位
uint8_t point_add1 = 0;          // 测试用：是否添加路径点的标志位
uint8_t kalman_update_times = 0; // 卡尔曼滤波更新次数统计
uint8_t kalman_init_flag = 0;    // 卡尔曼滤波初始化flag

/* 自适应一阶低通滤波变量（与卡尔曼对照） */
// 静止时深滤波（alpha小），运动时浅滤波（alpha大），解决静止雷达抖动
#define LOWPASS_ALPHA_STATIC     0.05f   // 静止：截止频率≈0.3Hz，强去噪
#define LOWPASS_ALPHA_MOVING     0.9f     // 运动：截止频率≈16Hz，快速跟踪
#define LOWPASS_SPEED_THRESH     50.0f    // 线速度阈值
#define LOWPASS_ANGULAR_THRESH   50.0f     // 角速度阈值 

float radar_x_lowpass = 0.0f;
float radar_y_lowpass = 0.0f;
float radar_z_lowpass = 0.0f;
uint8_t lowpass_init_flag = 0;   // 低通初始化标志

float vx, vy, vw = 0.0f;         // 用于测试的底盘速度变量
float speed = 0.0f;              // 用于测试的速度变量
uint8_t catch_flag = 0;          // 测试用：夹爪开合标志位
int32_t rotate_target_angle = 0; // 测试用：夹爪旋转目标角度
uint8_t task_flag = 0;           // A/B/C/D 主赛启动与断电重试入口
float debug_speed;               // 用于测试的速度变量
int16_t leidai_speed_1 = 300;    //由于雷达建图的低速
int16_t leidai_speed_2 = 1000;    //由于雷达建图的高速
uint8_t camera_send = 0;         //测试用：相机发送请求
float arm_target_x = 0.0f;        //测试用：机械臂轨迹点
float arm_target_y = 0.0f;
uint8_t arm_pump = 0;
uint8_t camera_command = 0;
float angle = 0.0f;
uint8_t traj_init_flag_arm = 0;     // 机械臂轨迹初始化标志位
uint8_t traj_init_flag_lift = 0;    // 抬升轨迹初始化标志位
uint8_t traj_init_flag_chassis = 0; // 底盘轨迹初始化标志位

extern volatile uint8_t a1_send_request;
static uint8_t c8_spi_poll_cnt = 0;
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
// #ifndef bisem_m8010Handle
//	osSemaphoreId bisem_m8010Handle;
// #endif
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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of roboticarm_task */
  osThreadDef(roboticarm_task, entry_robotic_arm, osPriorityBelowNormal, 0, 256);
  roboticarm_taskHandle = osThreadCreate(osThread(roboticarm_task), NULL);

  /* definition and creation of chassis_task */
  osThreadDef(chassis_task, entry_chassis, osPriorityBelowNormal, 0, 256);
  chassis_taskHandle = osThreadCreate(osThread(chassis_task), NULL);

  /* definition and creation of clamp_task */
  osThreadDef(clamp_task, entry_clamp, osPriorityBelowNormal, 0, 256);
  clamp_taskHandle = osThreadCreate(osThread(clamp_task), NULL);

  /* definition and creation of cmd_task */
  osThreadDef(cmd_task, entry_cmd_task, osPriorityNormal, 0, 1024);
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
    // Vofa 10通道：卡尔曼 vs 低通对比视图
    data_send_vofa[0] = Radar_Pos.radar_x;            // ch0: 雷达原始X
    data_send_vofa[1] = Radar_Pos.radar_x_filtered;   // ch1: 卡尔曼滤波X
    data_send_vofa[2] = radar_x_lowpass;              // ch2: 一阶低通X
    data_send_vofa[3] = Radar_Pos.reliable_x;         // ch3: 最终使用X
    // data_send_vofa[4] = chassis_route_traj.traj_vx.out_vel;          // ch4: 雷达原始Y
    // data_send_vofa[5] = Radar_Pos.radar_y_filtered;   // ch5: 卡尔曼滤波Y
    // data_send_vofa[6] = radar_y_lowpass;              // ch6: 一阶低通Y
    // data_send_vofa[7] = Radar_Pos.reliable_y;         // ch7: 最终使用Y
    // data_send_vofa[0] = m3508_fdcan3[0].M3508_Speed_PID.Target; // ch0: 目标速度
    // data_send_vofa[1] = m3508_fdcan3[0].Now_Speed;              // ch1: 实际速度
    // data_send_vofa[2] = m3508_fdcan3[1].M3508_Speed_PID.Target; // ch2: 目标速度
    // data_send_vofa[3] = m3508_fdcan3[1].Now_Speed;              // ch3: 实际速度
    // data_send_vofa[4] = m3508_fdcan3[2].M3508_Speed_PID.Target; // ch4: 目标速度
    // data_send_vofa[5] = m3508_fdcan3[2].Now_Speed;              // ch5: 实际速度
    // data_send_vofa[6] = m3508_fdcan3[3].M3508_Speed_PID.Target; // ch6: 目标速度
    // data_send_vofa[7] = m3508_fdcan3[3].Now_Speed;              // ch7: 实际速度


    // vofa_send_to_show_justfloat(&huart7, data_send_vofa);
    // vofa显示调试信息
    //=======================================================================



    // UART7 is now used by screen.c; do not send VOFA on the same port.
    /*
     * H7 通过 SPI1 轮询 F103C8T6 的红外接收结果，并交给 ir_action 处理。
     * IR_Action_Poll() 内部会调用 H7_F1_SPI_Task()：
     *   1. 有新红外帧时，更新 h7_f1_spi_last_frame 和 ir_action_rx_frame；
     *   2. 如果红外帧匹配指定数据包，则置位 MC_bb.DOCKING_SUCCESS = 1；
     *   3. 没有新红外帧时，显示用的数据包会清零，方便 Watch 观察。
     * 调试时可观察：
     *   h7_f1_spi_rx_ok_count / h7_f1_spi_last_ch / h7_f1_spi_last_frame
     *   ir_action_rx_count / ir_action_rx_frame / ir_action_docking_success_set_count
     */
//    c8_spi_poll_cnt++;
//    if (c8_spi_poll_cnt >= 5U)
//    {
//      c8_spi_poll_cnt = 0;
      IR_Action_Poll();
//    }

    debug_test_time += 0.001f;
		// Screen_TestDisplay();
    osDelay(40); // 25Hz
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
   m3508_parameter_init(&m3508_fdcan1[0], 2.0f, 0.0f, 2.0f, 4.5f, 0.0f, 25.0f);
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
    // if(debug_flag == 1){
    //   debug_flag = 0;
    //   arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_FRONT_TO_BACK);
    // }
    // if(debug_flag == 2){
    //   debug_flag = 0;
    //   arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_BACK_TO_FRONT);
    // }
    // if(debug_flag == 3){
    //   debug_flag = 0;
    //   arm_queue_add_point(&arm_traj, 0.4, 0.4, 0.0f, 500, 0);
    // }
    // if(debug_flag == 4){
    //   debug_flag = 0;
    //   arm_queue_add_point(&arm_traj, 0.5, 0.5, 0.0f, 500, 0);
    // }
    // if(debug_flag == 5){
    //   debug_flag = 0;
    //   arm_queue_add_point(&arm_traj, -0.4, 0.4, 0.0f, 500, 0);
    // }
    // if(debug_flag == 6){
    //   debug_flag = 0;
    //   arm_queue_add_point(&arm_traj, -0.5, 0.5, 0.0f, 500, 0);
    // }

    // 等待初始化完成
    if(A1_debug_485_2[0].Pos_init_flag == 200 
      && m8010_bus_1.motors[2].measure.Pos_init_flag == 200 
      && m3508_fdcan1[0].Initial_flag == 200)
    {
      if (traj_init_flag_arm == 0)
      {
        arm_trajectory_init(&arm_traj, &arm);   // 以当前位置初始化轨迹系统
        arm_queue_add_point(&arm_traj, PICK_PRE_REST_X, PICK_PRE_REST_Y, 200, ARM_PUMP_SUCK, 0.0f);
        arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 500, ARM_PUMP_SUCK, 0.0f);
        traj_init_flag_arm = 1;
      }
//       if(point_add == 1)
//       {
//         arm_queue_add_point(&arm_traj,arm_target_x,arm_target_y,0,0,arm_pump,angle * PI /180.0f);
//         point_add=0;
//       }
      // if(point_add == 2)
      //  {
      //    arm_queue_add_point(&arm_traj,PICK_REST_X,PICK_REST_Y,0,0,1);
      //    point_add=0;
      //  }
			//   if(point_add == 3)
      //  {
      //    arm_queue_add_point(&arm_traj,PICK_REST_X,PICK_REST_Y,0,0,2);
      //    point_add=0;
      //  }
      //  if(point_add == 1)
      // {
      //   arm_queue_add_point(&arm_traj,arm_target_x,arm_target_y,0,0,arm_pump);
      //   arm.effector_pitch_target = angle * 3.14f /180.0f;
      //   point_add=0;
      // }
      // if(point_add == 2)
      //  {
      //    arm_queue_add_point(&arm_traj,PICK_REST_X,PICK_REST_Y,0,0,1);
      //    point_add=0;
      //  }
			//   if(point_add == 3)
      //  {
      //    arm_queue_add_point(&arm_traj,PICK_REST_X,PICK_REST_Y,0,0,2);
      //    point_add=0;
      //  }
			// if(point_add == 4)
      // {
      //   arm_queue_add_point(&arm_traj,0.1f,1.0f,0,0,0);
			// 	arm_queue_add_point(&arm_traj,0.2f,1.0f,0,500,0);
      //   point_add=0;
      // }
      // 保持执行器水平
      // if (m3508_fdcan1[0].Initial_flag == 200 && arm.mode != ARM_MODE_TRANSITION)
      // {
      //   arm_actuator_stable_control(&arm, arm.effector_pitch_target, arm.theta_target[4]);
      // }

      // 1. 调用多段直线插补协调器 (它会从队列中成组提取路点)
      arm_linear_interpolation_update(&arm, &arm_traj, 0.05f);

      // 2. 分别推演 X 和 Y 的单段物理影子 (0.001f 代表 1ms)
      Traj_Adaptive_Update(&arm_traj.traj_x, arm.x0_now, 0.001f);
      Traj_Adaptive_Update(&arm_traj.traj_y, arm.y0_now, 0.001f);
      // Traj_Adaptive_Update(&arm_traj.traj_w, m8010_bus_1.motors[0].measure.Pos, 0.001f);

      // 3. 将影子坐标交接给逆解器（当工作区不处于过渡模式时）
      if (arm.mode == ARM_MODE_FRONT || arm.mode == ARM_MODE_BACK)
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
      arm_mode_update_execute(&arm, &arm_traj, &arm_trans_traj, 8.0f, 0.25f, 0.3f, 3.5f); // 位置环控制
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
  // 3508参数初始化（速度环 + 位置环）
  m3508_parameter_init(&m3508_fdcan3[0], 15.0f, 0.00f, 3.0f, 0.1f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan3[1], 15.0f, 0.00f, 3.0f, 0.1f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan3[2], 15.0f, 0.00f, 3.0f, 0.1f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan3[3], 15.0f, 0.00f, 3.0f, 0.1f, 0.0f, 0.2f);
  // 底盘使能
  chassis_r2.chassis_en_flag = 1;

  // 抬升机构辅助轮电机初始化
  m3508_parameter_init(&m3508_fdcan1[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  m3508_parameter_init(&m3508_fdcan1[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  // 底盘升降初始化为收回状态
  // chassis_r2.lift_front = 0;
  // chassis_r2.lift_back = 0;
  // chassis_lift_front_control(chassis_r2.lift_front);
  // chassis_lift_back_control(chassis_r2.lift_back);
  // chassis_route_traj_init();

  /* Infinite loop */
  for (;;)
  {
    if (m3508_fdcan3[0].Initial_flag == 200 && m3508_fdcan3[1].Initial_flag == 200 && m3508_fdcan3[2].Initial_flag == 200 && m3508_fdcan3[3].Initial_flag == 200)
    {
     // DR_Update(); 
      // //雷达建图用
      // if (HT10A.switch_d == -1)
      // {
      //   if (HT10A.left_y > 0)chassis_r2.vx = leidai_speed_1;
      //   else if (HT10A.left_y < 0)chassis_r2.vx = -leidai_speed_1;
      //   else if(HT10A.left_y == 0)chassis_r2.vx = 0;
      //   if (HT10A.left_x > 0)chassis_r2.vy = -leidai_speed_1;
      //   else if (HT10A.left_x < 0)chassis_r2.vy = leidai_speed_1;
      //   else if(HT10A.left_x == 0)chassis_r2.vy = 0;
      //    if (HT10A.right_y > 0)chassis_r2.vw = leidai_speed_1;
      //   else if (HT10A.right_y < 0)chassis_r2.vw = -leidai_speed_1;
      //   else if(HT10A.right_y == 0)chassis_r2.vw = 0;
      // }
      // if (HT10A.switch_d == 1)
      // {
      //   if (HT10A.left_y > 0)chassis_r2.vx = leidai_speed_2;
      //   else if (HT10A.left_y < 0)chassis_r2.vx = -leidai_speed_2;
      //   else if(HT10A.left_y == 0)chassis_r2.vx = 0;
      //   if (HT10A.left_x > 0)chassis_r2.vy = -leidai_speed_2;
      //   else if (HT10A.left_x < 0)chassis_r2.vy = leidai_speed_2;
      //    else if(HT10A.left_x == 0)chassis_r2.vy = 0;
      //    if (HT10A.right_y > 0)chassis_r2.vw = leidai_speed_2;
      //   else if (HT10A.right_y < 0)chassis_r2.vw = -leidai_speed_2;
      //   else if(HT10A.right_y == 0)chassis_r2.vw = 0;
      // }
      //一区平移测试
      // if (point_add== 1 )
      //  {
      //    chassis_route_point_add(&chassis_route_traj,1000.0f,-1400.0f,0.0f,0,0);
			//  	point_add = 0;
      //  }
      //  if (point_add== 2 )
      //  {
      //    chassis_route_point_add(&chassis_route_traj,2000.0f,-1400.0f,180.0f,0,0);
			//  	point_add = 0;
      //  }
    //  //测试用添加路径点
//     if (point_add == 1)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_CZ_INIT_X, R_CHASSIS_CZ_INIT_Y, R_CHASSIS_CZ_INIT_W, 0);
//       point_add = 0;
//     }
//     if (point_add == 2)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_1_X, R_CHASSIS_SUDUKU_1_Y, R_CHASSIS_SUDUKU_1_W, 0);
//       point_add = 0;
//     }
//     if (point_add == 3)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_2_X, R_CHASSIS_SUDUKU_2_Y, R_CHASSIS_SUDUKU_2_W, 0);
//       point_add = 0;
//     }
//     if (point_add == 4)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_3_X, R_CHASSIS_SUDUKU_3_Y, R_CHASSIS_SUDUKU_3_W, 0);
//       point_add = 0;
//     }
//     
//     if (point_add1 == 1)
//     {
//       chassis_route_point_add(&chassis_route_traj, 11100.0f, -3000.0f, 90.0f, 0);
//       point_add1 = 0;
//     }
    //上坡测试
//     if (point_add == 1)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SLOPE_START_X, R_CHASSIS_SLOPE_START_Y, R_CHASSIS_SLOPE_START_W, 0);
//       point_add = 0;
//     }
//     if (point_add == 2)
//     {
//       chassis_route_point_add(&chassis_route_traj, R_CHASSIS_CZ_INIT_X, R_CHASSIS_CZ_INIT_Y, 0, 0);
//       point_add = 0;
//     }
// if (point_add == 1)
//      {
//        chassis_route_point_add(&chassis_route_traj, 11500,-4000.0f, 0.0f, 0);
//        point_add = 0;
//      }
//      if (point_add == 2)
//      {
//        chassis_route_point_add(&chassis_route_traj, 11100,-2400.0f, 180.0f, 0);
//        point_add = 0;
//      }
    //  if (point_add == 1)
    //   {
    //     chassis_route_point_add(&chassis_route_traj, 11080,-3055.0f, 90.0f, 0);
    //     point_add = 0;
    //   }
    //三区点位
//    if (point_add == 1)
//      {
//        chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SLOPE_END_X,R_CHASSIS_SLOPE_END_Y, 90.0f, 0,0);
//        point_add = 0;
//      }
//    if (point_add == 2)
//      {
//        chassis_route_point_add(&chassis_route_traj,R_CHASSIS_CZ_CENTER_X,R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W, 0,0);
//        point_add = 0;
//      }if (point_add == 3)
//      {
//        chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_1_X,R_CHASSIS_SUDUKU_1_Y, R_CHASSIS_SUDUKU_1_W, 0,0);
//        point_add = 0;
//      }
//      if (point_add == 4)
//      {
//        chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_1_X,R_CHASSIS_SUDUKU_1_Y+500.0f, R_CHASSIS_SUDUKU_1_W, 0,0);
//        point_add = 0;
//      }
//if(point_add == 1)
//{
//  chassis_route_point_add(&chassis_route_traj, 11400.0f,-2000.0f, 0.0f, 0,0);
//  chassis_route_point_add(&chassis_route_traj, 11400.0f,-1500.0f, -90.0f, 0,0);
//  chassis_route_point_add(&chassis_route_traj, 11400.0f,-2500.0f, 90.0f, 0,1);
//	chassis_route_point_add(&chassis_route_traj, 11400.0f,-1500.0f, -90.0f, 0,0);
//  point_add = 0;
//}
       if (point_add == 1)
      {
        chassis_route_point_add(&chassis_route_traj,arm_target_x,arm_target_y, angle, 0,0);
        point_add = 0;
      }


      // // 滤波后的雷达数据更新
      Radar_Pos.radar_x_filtered = Kalman_Update(&Radar_Kalman_X, Radar_Pos.radar_x);
      Radar_Pos.radar_y_filtered = Kalman_Update(&Radar_Kalman_Y, Radar_Pos.radar_y);
      Radar_Pos.radar_z_filtered = Kalman_Update(&Radar_Kalman_Z, Radar_Pos.radar_z);

      // 一阶低通滤波（与卡尔曼对照）
      if (lowpass_init_flag == 0)
      {
          radar_x_lowpass = Radar_Pos.radar_x;
          radar_y_lowpass = Radar_Pos.radar_y;
          radar_z_lowpass = Radar_Pos.radar_z;
          lowpass_init_flag = 1;
      }

      // 利用电机速度计算底盘实际速度（逆解算）
      omni_chassis_inverse_calculate(&chassis_r2, m3508_fdcan3);

      // 自适应 alpha + 速度前馈预测：静止时深滤波去抖，运动时预测+浅滤波消滞后
      {
          float linear_speed = sqrtf(chassis_r2.real_vx * chassis_r2.real_vx + chassis_r2.real_vy * chassis_r2.real_vy);
          uint8_t is_static = (linear_speed < LOWPASS_SPEED_THRESH) && (abs_float(chassis_r2.real_vw) < LOWPASS_ANGULAR_THRESH);
          float alpha = is_static ? LOWPASS_ALPHA_STATIC : LOWPASS_ALPHA_MOVING;

          if (!is_static)
          {
              // 仅运动时做速度前馈预测（dt=0.001s），静止跳过防止电机速度毛刺注入
              radar_x_lowpass += chassis_r2.real_vx * 0.001f;
              radar_y_lowpass += chassis_r2.real_vy * 0.001f;
          }
          // 测量修正：雷达观测值拉回预测误差
          radar_x_lowpass += alpha * (Radar_Pos.radar_x - radar_x_lowpass);
          radar_y_lowpass += alpha * (Radar_Pos.radar_y - radar_y_lowpass);
          radar_z_lowpass += alpha * (Radar_Pos.radar_z - radar_z_lowpass);
      }

      // 输出可靠位置：静止死区锁定防抖，运动速度前馈无滞后
      {
          float linear_speed = sqrtf(chassis_r2.real_vx * chassis_r2.real_vx + chassis_r2.real_vy * chassis_r2.real_vy);
          uint8_t is_static = (linear_speed < LOWPASS_SPEED_THRESH) && (abs_float(chassis_r2.real_vw) < LOWPASS_ANGULAR_THRESH);

          // 静止且 X/Y 都在死区内 → 锁定，不更新（抑制震荡）
          if (is_static
              && abs_float(radar_x_lowpass - Radar_Pos.reliable_x) < POS_HOLD_DEADBAND
              && abs_float(radar_y_lowpass - Radar_Pos.reliable_y) < POS_HOLD_DEADBAND)
          {
              ;// 真正锁定：什么都不做，维持上次 reliable 值
          }
          else
          {
              Radar_Pos.reliable_x = radar_x_lowpass;
              Radar_Pos.reliable_y = radar_y_lowpass;
              Radar_Pos.reliable_z = radar_z_lowpass;
          }
          Radar_Pos.reliable_yaw = Radar_Pos.hi14_yaw;
      }
 
          
//					 Radar_Pos.reliable_x = Radar_Pos.radar_x;
//           Radar_Pos.reliable_y = Radar_Pos.radar_y;
//           Radar_Pos.reliable_yaw = Radar_Pos.hi14_yaw;
          

      if (traj_init_flag_chassis == 0)
      {
        // 以当前坐标初始化底盘路径系统
        chassis_route_traj_init();
        traj_init_flag_chassis = 1;
      }

      // 多段-单段-坐标系变换-底盘解算 路径执行部分
      if (traj_init_flag_chassis == 1)
      {
        chassis_multi_route_scheduler(&chassis_route_traj, &Radar_Pos);
        chassis_single_route_scheduler(&Radar_Pos, &chassis_route_traj, 1);
        chassis_world_to_robot_velocity(&chassis_r2, chassis_route_traj.traj_vx.out_vel, chassis_route_traj.traj_vy.out_vel, chassis_route_traj.traj_vw.out_vel, Radar_Pos.reliable_yaw);
        if (chassis_r2.chassis_lock_flag)
          chassis_position_lock_execute();   /* 位置环锁止：死守捕获编码器位置 */
        else
          omni_chassis_calculate(&chassis_r2, chassis_r2.vx, chassis_r2.vy, chassis_r2.vw, chassis_r2.chassis_en_flag);
      }
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
        chassis_lift_queue_add_point(&chassis_lift_traj, LIFT_RETRACT_HEIGHT, LIFT_RETRACT_HEIGHT, 0);
        traj_init_flag_lift = 1;
      }
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
      // chassis_lift_excute(&chassis_r2, 0.0f, 0.0f);
      // 7. 抬升轮速度执行
      // chassis_lift_wheel_excute(debug_speed, 2.0f, 0.2f);
      // chassis_lift_wheel_excute(debug_speed, 0.0f, 0.0f);

      // if(debug_flag == 1)
      // {
      //   debug_flag = 0;
      //   chassis_lift_queue_add_point(&chassis_lift_traj, -0.200f, -0.200f, 0);
      // }
      // if(debug_flag == 2)
      // {
      //   debug_flag = 0;
      //   chassis_lift_queue_add_point(&chassis_lift_traj, -0.400f, -0.400f, 0);
      // }
      // if(debug_flag == 3)
      // {
      //   debug_flag = 0;
      //   chassis_lift_queue_add_point(&chassis_lift_traj, LIFT_RETRACT_HEIGHT, LIFT_RETRACT_HEIGHT, 0);
      // }

      // 根据 lift FSM 设置的控制模式执行
      switch (g_lift_ctrl_mode) {
        case LIFT_CTRL_NONE:
        case LIFT_CTRL_POSITION:
            chassis_lift_excute(STEP_UP_BRACE_KP, STEP_UP_BRACE_KD, 1, 1);
            break;
        case LIFT_CTRL_FORCE_FRONT_REAR:
            m8010_torque_control(&m8010_bus_3.motors[0], g_lift_front_torque);
            m8010_torque_control(&m8010_bus_3.motors[1], g_lift_rear_torque);
            break;
        case LIFT_CTRL_FORCE_REAR_ONLY:
            chassis_lift_excute(STEP_UP_BRACE_KP, STEP_UP_BRACE_KD, 1, 0);
            m8010_torque_control(&m8010_bus_3.motors[1], -g_lift_rear_torque);
            break;      
      }
      // switch (g_lift_ctrl_mode) {
      //   case LIFT_CTRL_NONE:
      //   case LIFT_CTRL_POSITION:
      //       chassis_lift_excute(&chassis_r2, 0.0f, 0.0f);
      //       break;
      //   case LIFT_CTRL_FORCE_FRONT_REAR:
      //       m8010_torque_control(&m8010_bus_3.motors[0], 0);
      //       m8010_torque_control(&m8010_bus_3.motors[1], 0);
      //       break;
      //   case LIFT_CTRL_FORCE_REAR_ONLY:
      //       chassis_lift_excute(&chassis_r2, 0.0f, 0.0f);
      //       m8010_torque_control(&m8010_bus_3.motors[1], 0);
      //       break;      
      // }

      // 辅助轮执行
      chassis_lift_wheel_excute(g_lift_wheel_target, g_lift_wheel_kp, g_lift_wheel_kd);

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
  m3508_parameter_init(&m3508_fdcan2[0], 10.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan2[1], 6.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  m3508_parameter_init(&m3508_fdcan2[2], 5.0f, 0.0f, 2.0f, 0.5f, 0.0f, 0.2f);
  // m3508_parameter_init(&m3508_fdcan2[0], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan2[1], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  // m3508_parameter_init(&m3508_fdcan2[2], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

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
     clamp_lift_control(clamp.lift_flag, &m3508_fdcan2[0]);
       //m3508_anglesum_control(&m3508_fdcan2[0],rotate_target_angle,8000);
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

  BT_MASTER_Init_Task();
  mf_planner_ctx.map_color = RED_MAP;

  // task_flag = 1;

  /* Infinite loop */
  for (;;)
  {
//     if(camera_send == 1)
//     {
//       camera_send = 0;
//       MF_Scan_Command_Send(camera_command);
// //			MF_Scan_Command_Send(camera_command);
//     }

  /* 当按下按键时，根据最新地图刷新屏幕 */
  if(key_now != 0)
  {
    uint8_t screen_index = 0;

    if (Keyboard_MF_KeyToCell(key_now, NULL, NULL, &screen_index))
    {
      Merlin_SetBlock(screen_index, keyboard_kfs_screen_info[screen_index]);
    }
  }
  /* 按键启动部分 */
  if (key_now == 'A')
  {
    task_flag = 1;
    key_now = 0;
    key_last = 0;
  }
  else if (key_now == 'B')
  {
    task_flag = 2;
    key_now = 0;
    key_last = 0;
  }
  else if (key_now == 'C')
  {
    task_flag = 3;
    key_now = 0;
    key_last = 0;
  }
  else if (key_now == 'D')
  {
    task_flag = 4;
    key_now = 0;
    key_last = 0;
  }

  /* 按键-雷达偏移部分 */
  if(key_2_now == 1)
  {
    x_offset_by_key_times += 1;
    key_2_now = 0;
    key_2_last = 0;
  }
  else if(key_2_now == 5)
  {
    x_offset_by_key_times -= 1;
    key_2_now = 0;
    key_2_last = 0;
  }
  else if(key_2_now == 9)
  {
    y_offset_by_key_times += 1;
    key_2_now = 0;
    key_2_last = 0;
  }
  else if(key_2_now == 13)
  {
    y_offset_by_key_times -= 1;
    key_2_now = 0;
    key_2_last = 0;
  }

  /* 主赛重试入口：A=MC/全流程，B=MF，C=CZ存一拿一，D=CZ只拿一块。 */




  if (task_flag != 0)
  {
    /* 所有入口均汇入同一个 MASTER，运行期间忽略新的重启键。 */
    if (task_flag == 1 && !BT_MASTER_IsRunning())
    {
      /* A：全流程启动 / MC 区断电重试，只取一个端头。 */
      BT_MASTER_Start_Task();
    }
    else if (task_flag == 2 && !BT_MASTER_IsRunning())
    {
      /* B：操作手完成 MF 地图预输入后，从 MF 区开始全流程。 */
      BT_MASTER_Start_FromMF();
    }
    else if (task_flag == 3 && !BT_MASTER_IsRunning())
    {
      /* C：CZ 重试，车身存一块且机械臂持一块。 */
      BT_MASTER_Start_FromCZ(1U, 1U);
    }
    else if (task_flag == 4)
    {
      /* D 在 MC 等待态用于放行 MF；空闲时用于 CZ 无存块重试。 */
      if (!BT_MASTER_Request_MF_Start() && !BT_MASTER_IsRunning())
        BT_MASTER_Start_FromCZ(0U, 1U);
    }

    task_flag = 0;
  }
    // if(debug_flag == 1)
    // {
    //   debug_flag = 0;
    //   MF_Scan_Command_Send(1);
    // }
    // if(debug_flag == 2)
    // {
    //   debug_flag = 0;
    //   MF_Scan_Command_Send(2);
    // }
    // if(debug_flag == 3)
    // {
    //   debug_flag = 0;
    //   MF_Scan_Command_Send(3);
    // }
    

    BT_MASTER_Tick(); /* 1ms 调用，内部降频到 10ms，串行推进 MC -> MF -> CZ。 */
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
  for (;;)
  {
    osSemaphoreWait(bisem_m8010Handle, osWaitForever); // 等待信号量，确保在主循环中被正确触发

    m8010_bus_poll(&m8010_bus_1);
    // m8010_bus_poll(&m8010_bus_2);
    m8010_bus_poll(&m8010_bus_3);

    // osDelay(1);
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
