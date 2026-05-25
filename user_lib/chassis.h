#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "mycan.h"
#include "myuart.h"
#include "m3508.h"
#include "ht10a.h"
#include "trajectory.h"
#include "radar.h"
#include "pid.h"

#define DISTANCE_WHEEL_CENTER 0.25f //轮子到中心的距离，单位m
#define CHASSIS_V_MAX 2000.0f //底盘最大速度
#define CHASSIS_A_MAX 2000.0f //底盘最大加速度
#define CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD 200.0f //底盘跟踪位置误差阈值，单位mm
#define CHASSIS_TRACKING_ANGLE_ERROR_THRESHOLD 6.0f //底盘跟踪角度误差阈值，单位rad
#define CHASSIS_ROUTE_ARRIVAL_THRESHOLD 10.0f //底盘路径到达判定阈值，单位mm    
#define CHASSIS_ANGLE_ARRIVAL_THRESHOLD 1.0f //底盘路径到达判定阈值，单位度
#define CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD 300.0f //底盘路径最终接近目标判定阈值，单位mm        
#define CHASSIS_ROUTE_FINAL_APPROACH_ANGLE_THRESHOLD 5.0f //底盘路径最终接近目标判定阈值，单位度
/* 底盘控制相关结构体定义 */
typedef struct {
    float vx;           // 目标前向速度 
    float vy;           //目标横向速度 
    float vw;           // 目标旋转速度

    float real_vx;      // 实际底盘前向速度 
    float real_vy;      // 实际底盘横向速度 
    float real_vw;      // 实际底盘旋转速度 
    uint8_t lift_front, lift_back; // 前后升降状态，0代表收回，1代表推出
    int16_t wheel_speed[4]; // 四轮速度，直接对应轮部3508电机的目标速度
} chassis_stu;
/* 底盘轨迹控制相关结构体定义 */
typedef struct{
    Traj_Single_struct traj_vx; //底盘前向速度单轨迹
    Traj_Single_struct traj_vy; //底盘横向速度单轨迹
    Traj_Single_struct traj_vw; //底盘旋转速度单轨迹

    Traj_Queue_struct traj_queue_vx; //底盘前向速度路径队列
    Traj_Queue_struct traj_queue_vy; //底盘横向速度路径队列
    Traj_Queue_struct traj_queue_vw; //底盘旋转速度路径队列

     // ---- 延时控制属性 ----
    uint16_t delay_queue[TRAJ_MAX_WAYPOINTS];  // 与位置队列下标同步，存储每个点到达后的延时时间(ms)
    uint16_t target_delay_ms;                  // 当前正在执行的目标点所要求的延时
    uint16_t wait_counter;                     // 当前目标点到达后，正在倒计时的计数器

    PID_Struct pid_final_vx; //前向速度PID实例      
    PID_Struct pid_final_vy; //横向速度PID实例  
    PID_Struct pid_final_vw; //旋转速度PID实例 
}chassis_route_traj_stu;



extern chassis_stu chassis_r2; // 全局底盘实例
extern chassis_route_traj_stu chassis_route_traj;
void chassis_world_to_robot_velocity(chassis_stu *chassis, float vx_world, float vy_world, float vw_world, float robot_yaw);
void omni_chassis_calculate(chassis_stu *chassis, float vx, float vy, float vw);
void omni_chassis_inverse_calculate(chassis_stu *chassis, M3508_Motor_Struct * m3508_fdcan3);
void chassis_lift_front_control(uint8_t lift_front);
void chassis_lift_back_control(uint8_t lift_back);  
void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag);
void chassis_route_traj_init(void);
void chassis_multi_route_scheduler(chassis_route_traj_stu* chassis_route_traj,RADAR_POSITION * chassis_real_position);
void chassis_route_point_add(chassis_route_traj_stu* chassis_route_traj, float target_vx, float target_vy, float target_vw, uint16_t delay_time_ms);
void chassis_single_route_scheduler(RADAR_POSITION * chassis_real_position, chassis_route_traj_stu* chassis_route_traj,float dt);
#endif /* CHASSIS_H */

