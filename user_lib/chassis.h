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
#define CHASSIS_V_MAX 3500.0f //底盘最大速度,上坡时5500
#define CHASSIS_A_MAX 3500.0f //底盘最大加速度，上坡时5500
#define CHASSIS_SLOPE_V_MAX 6000.0f //上坡时底盘最大速度
#define CHASSIS_SLOPE_A_MAX 6000.0f //上坡时底盘最大加速度
#define CHASSIS_ROTATE_SLOW_V_MAX 1000.0f  //慢速旋转时角速度最大值
#define CHASSIS_ROTATE_SLOW_A_MAX 1000.0f  //慢速旋转时角加速度最大值
#define CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD 200.0f //底盘跟踪位置误差阈值，单位mm
#define CHASSIS_TRACKING_ANGLE_ERROR_THRESHOLD 6.0f //底盘跟踪角度误差阈值，单位rad
#define CHASSIS_ROUTE_ARRIVAL_THRESHOLD 10.0f //底盘路径到达判定阈值，单位mm    
#define CHASSIS_ANGLE_ARRIVAL_THRESHOLD 0.5f //底盘路径到达判定阈值，单位度
#define CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD 250.0f //底盘路径最终接近目标判定阈值，单位mm        
#define CHASSIS_ROUTE_SOFT_BRAKE_THRESHOLD 80.0f    //末端软刹车阈值，不跑PID，线性降速到零
#define CHASSIS_ROUTE_FINAL_APPROACH_ANGLE_THRESHOLD 5.0f //底盘路径最终接近目标判定阈值，单位度
#define CHASSIS_ROUTE_LOOKAHEAD_DIST 350.0f //多段路径预加载距离，距目标小于此值时提前出队下一个点
#define CHASSIS_VX_FILTER_ALPHA 0.2f
#define CHASSIS_VY_FILTER_ALPHA 0.2f
#define CHASSIS_VW_FILTER_ALPHA 0.9f
#define CHASSIS_ZERO_MUTATION_ALPHA 0.01f

#define POS_HOLD_DEADBAND        7.0f     // 静止位置死区 mm，雷达抖动小于此值不更新
/* 底盘控制相关结构体定义 */
typedef struct {
    float vx;           // 目标前向速度 
    float vy;           //目标横向速度 
    float vw;           // 目标旋转速度

    float real_vx;      // 实际底盘前向速度 
    float real_vy;      // 实际底盘横向速度 
    float real_vw;      // 实际底盘旋转速度 
    int16_t wheel_speed[4]; // 四轮速度，直接对应轮部3508电机的目标速度

    uint8_t chassis_en_flag; // 底盘使能标志，1=使能，0=禁用（自由滑行）
    uint8_t chassis_lock_flag; // 底盘锁止标志，1=位置环锁止（仅对杆用）
    int32_t lock_angle_target[4]; // 锁止时四轮目标角度

// 以下为底盘升降相关属性
    float lift_front_target;    // 前端升降目标高度(m)
    float lift_back_target;     // 后端升降目标高度(m)
    float lift_front_current;   // 前端升降当前高度(m)，从电机编码器换算
    float lift_back_current;    // 后端升降当前高度(m)，从电机编码器换算
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
    uint8_t  force_queue[TRAJ_MAX_WAYPOINTS];  // 与位置队列下标同步，1=强制出队该点
    uint16_t target_delay_ms;                  // 当前正在执行的目标点所要求的延时
    uint16_t wait_counter;                     // 当前目标点到达后，正在倒计时的计数器

    PID_Struct pid_final_vx; //前向速度PID实例      
    PID_Struct pid_final_vy; //横向速度PID实例  
    PID_Struct pid_final_vw; //旋转速度PID实例 
    uint8_t slope_mode;       // 上坡模式标志: 0=平地参数, 1=上坡参数(CHASSIS_SLOPE_V/A_MAX)
    uint8_t rotate_slow_mode; // 慢速旋转模式标志: 0=正常, 1=慢速旋转(CHASSIS_ROTATE_SLOW_V/A_MAX)
}chassis_route_traj_stu;

void chassis_route_set_slope_mode(uint8_t enable);
void chassis_route_set_rotate_slow(uint8_t enable);
void chassis_set_lock(uint8_t lock);   /* 锁止底盘：1=位置环锁止，0=恢复 */
void chassis_position_lock_execute(void); /* 位置环锁止执行（1ms调用） */



extern chassis_stu chassis_r2; // 全局底盘实例
extern chassis_route_traj_stu chassis_route_traj;



void chassis_world_to_robot_velocity(chassis_stu *chassis, float vx_world, float vy_world, float vw_world, float robot_yaw);
void omni_chassis_calculate(chassis_stu *chassis, float vx, float vy, float vw, uint8_t en_flag);
void omni_chassis_inverse_calculate(chassis_stu *chassis, M3508_Motor_Struct * m3508_fdcan3);
// void chassis_lift_front_control(uint8_t lift_front);
// void chassis_lift_back_control(uint8_t lift_back);  
// void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag);
float chassis_angle_edge_protection(float target_yaw, float now_yaw);
void chassis_route_traj_init(void);
void chassis_multi_route_scheduler(chassis_route_traj_stu* chassis_route_traj,RADAR_POSITION * chassis_real_position);
void chassis_route_point_add(chassis_route_traj_stu* chassis_route_traj, float target_vx, float target_vy, float target_vw, uint16_t delay_time_ms, uint8_t force_dequeue);
void chassis_single_route_scheduler(RADAR_POSITION * chassis_real_position, chassis_route_traj_stu* chassis_route_traj,float dt);



/* ========================================== 以下为底盘抬升部分 ========================================== */
/* ========================================== 以下为底盘抬升部分 ========================================== */


// 底盘升降速度/加速度参数（可调整）
#define CHASSIS_LIFT_V_MAX   (9.0f)    // 升降最大速度 m/s
#define CHASSIS_LIFT_A_MAX   (6.5f)    // 升降最大加速度 m/s^2
#define CHASSIS_LIFT_TRACKING_ERROR_THRESHOLD  0.04f  // 升降跟踪误差阈值，单位m
#define CHASSIS_LIFT_ARRIVAL_THRESHOLD         0.005f // 升降到达判定阈值，单位m
#define CHASSIS_LIFT_QUEUE_BLEND_RADIUS        0.010f  // 升降队列融合半径，单位m

// #define CHASSIS_LIFT_RATE (0.5f*0.045f) // 升降编码器单位转换为实际高度的比例，单位 m/rad
#define CHASSIS_LIFT_RATE (0.5f*0.025f) // 升降编码器单位转换为实际高度的比例，单位 m/rad（三代车）
#define LIFTER_FRONT_HIGHEST    (0.010f) // 前抬升离地最高位置（单位 m）
#define LIFTER_FRONT_LOWEST     (-0.435f) // 前抬升离地最低位置（单位 m）
#define LIFTER_BACK_HIGHEST     (0.010f) // 后抬升离地最高位置（单位 m）
#define LIFTER_BACK_LOWEST      (-0.435f) // 后抬升离地最低位置（单位 m）

/**
 * @brief 底盘升降轨迹结构体，包含前后升降的单轨迹实例、路径队列实例，以及延时控制属性
*/
typedef struct {
    Traj_Single_struct traj_front;      // 前端升降轨迹
    Traj_Single_struct traj_back;       // 后端升降轨迹

    Traj_Queue_struct traj_queue_front; // 前端升降路径队列
    Traj_Queue_struct traj_queue_back;  // 后端升降路径队列

    uint16_t delay_queue[TRAJ_MAX_WAYPOINTS];  // 延时队列
    uint16_t target_delay_ms;
    uint16_t wait_counter;
} chassis_lift_traj_stu;

extern chassis_lift_traj_stu chassis_lift_traj;

void chassis_lift_fk_update(chassis_stu *chassis);
void chassis_lift_ik_update(chassis_stu *chassis);
void chassis_lift_traj_init(void);
void chassis_lift_linear_interpolation_update(chassis_stu *chassis, chassis_lift_traj_stu *lift_traj, float blend_radius);
void chassis_lift_queue_add_point(chassis_lift_traj_stu *lift_traj, float front_height, float back_height, uint16_t delay_ms);
void chassis_lift_excute(float kp, float kw, uint8_t front_en, uint8_t back_en);
void chassis_lift_wheel_excute(float target_speed, float speed_kp, float speed_kd);

#endif

