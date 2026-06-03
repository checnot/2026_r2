#ifndef __ROBOTIC_ARM_H
#define __ROBOTIC_ARM_H

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "own_lib.h"
#include "mycan.h"
#include "myuart.h"
#include "filter.h"
#include "pid.h"
#include "a1_motor.h"
#include "m3508.h"
#include "trajectory.h"
#include "go_m8010.h"

// 人为设定的“机械臂末端合成上限”
#define GLOBAL_V_MAX 0.5f  // 合成最大速度(m/s)
#define GLOBAL_A_MAX 2.0f  // 合成最大加速度(m/s^2)


/**
 * @brief 机械臂dh参数结构体（Craig 改进 D-H 法参数描述）
 */
typedef struct 
{
    float alpha;    /* 连杆扭角 (Link Twist): 绕 x_(i-1)，从 z_(i-1) 到 z_(i) 旋转的角度 */
    float a;        /* 连杆长度 (Link Length):沿 x_(i-1)，从z_(i-1) 到 z_(i) 的距离 */
    float theta;    /* 关节角 (Joint Angle):    绕 z_(i)，从x_(i-1) 到 x_(i)  旋转的角度 */
    float d;        /* 连杆偏置 (Link Offset):  沿 z_(i)，从x_(i-1) 到 x_(i) 的距离 */
} dh_craig_stu;

/**
 * @brief 机械臂路径点结构体类型，包含三轴的单轨迹和轨迹队列
*/
typedef struct
{
    Traj_Single_struct traj_x; //末端执行器x方向单轨迹
    Traj_Single_struct traj_y; //末端执行器y方向单轨迹
    Traj_Single_struct traj_w; //云台旋转角单轨迹

    Traj_Queue_struct traj_queue_x; //末端执行器x方向路径队列
    Traj_Queue_struct traj_queue_y; //末端执行器y方向路径队列
    Traj_Queue_struct traj_queue_w; //云台旋转角路径队列

    // ---- 延时控制属性 ----
    uint16_t delay_queue[TRAJ_MAX_WAYPOINTS];  // 与位置队列下标同步，存储每个点到达后的延时时间(ms)
    uint16_t target_delay_ms;                  // 当前正在执行的目标点所要求的延时
    uint16_t wait_counter;                     // 当前目标点到达后，正在倒计时的计数器

    // ---- 气泵控制属性 ----
    uint8_t pump_queue[TRAJ_MAX_WAYPOINTS];     // 气泵控制标志，用于控制每个路径点的气泵状态
} robotic_arm_traj_stu;

/**
 * @brief 机械臂结构体类型
*/
typedef struct 
{
    dh_craig_stu joint_parameters[5]; // 机械臂的5个主要关节的D-H参数
    float joint_pitch[3]; //机械臂连杆相对地面的绝对角度([0]为"大臂",[1]为"小臂",[2]为"末端执行器")
    float joint_torque_gc[3]; //用于重力补偿：机械臂关节力矩([0]为"肩膀",[1]为"手肘",[2]为"腕部")
    float x0_now, y0_now;       //目前二连杆末端位置
    float x0_target, y0_target; //目标二连杆末端位置
    float effector_pitch_target;           //目标末端执行器pitch角，平行地面为0，抬起为正
    float theta_target[5];      //目标关节角，[0]为“云台”，[1]为”肩部“，[2]为“手肘”，[3]为”腕部“，[4]为“手腕自转”
} robotic_arm_stu;




/**
 * @brief 机械臂末端位置可达性检测结果枚举类型
*/
typedef enum {
    IK_REACHABLE = 0,    // 解算成功
    IK_LENGTH_OUT_OF_RANGE,   // 长度超出物理限位（不可行）
    IK_ANGLE_OUT_OF_RANGE,   // 关节角超出物理限位（不可行）
    IK_SINGULARITY,    // 奇异点（数学上无解或多解）
    IK_ERROR           // 其他错误
} IK_Reach_Status_enum;

extern robotic_arm_stu arm;    //机械臂实例
extern robotic_arm_traj_stu arm_traj; //机械臂轨迹实例

void arm_joint_parameter_init(robotic_arm_stu *arm);
void arm_joint_state_update(robotic_arm_stu *arm);
// void arm_joint_state_init(robotic_arm_stu *arm);
IK_Reach_Status_enum arm_route_reachable_check(robotic_arm_stu *arm);
void arm_route_point_calculate(robotic_arm_stu *arm);
void arm_route_execute(robotic_arm_stu *arm, float kw, float kp);
void arm_actuator_stable_control(robotic_arm_stu *arm, float target_pitch, float theta4);
void arm_remote_control(void);

void arm_trajectory_init(robotic_arm_traj_stu *arm_traj, robotic_arm_stu *arm);
void arm_queue_add_point(robotic_arm_traj_stu *arm_traj, float target_x, float target_y, float target_w, uint16_t delay_ms, uint8_t air_pump_flag);
void arm_queue_add_point_cartesian(robotic_arm_traj_stu *arm_traj, float target_x, float target_y, float target_z, uint16_t delay_ms, uint8_t air_pump_flag);
void arm_linear_interpolation_update(robotic_arm_stu *arm, robotic_arm_traj_stu *arm_traj, float blend_radius);

void arm_load_test_waypoints_clockwise(void);
void arm_load_test_waypoints_counterclockwise(void);
void arm_load_test_waypoints_rotate(void);
void arm_load_test_waypoints_custom(void);

void arm_gravity_comp(robotic_arm_stu *arm, float torque_preset0, float torque_preset1);

#endif /* __ROBOTIC_ARM_H */
