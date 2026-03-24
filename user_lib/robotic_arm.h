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
 * @brief 机械臂结构体
*/
typedef struct 
{
    dh_craig_stu dh_parameters[5]; // 机械臂的5个主要关节的D-H参数
    float joint_pitch[3]; //用于重力补偿：机械臂连杆相对地面的绝对角度([0]为"肩膀",[1]为"手肘",[2]为"腕部")
    float joint_torque_gc[3]; //用于重力补偿：机械臂关节力矩([0]为"肩膀",[1]为"手肘",[2]为"腕部")
} robotic_arm_stu;

extern robotic_arm_stu arm;    //机械臂实例

void arm_joint_state_init(robotic_arm_stu *arm);
void arm_joint_state_update(robotic_arm_stu *arm);
void arm_gravity_comp(robotic_arm_stu *arm, float torque_preset0, float torque_preset1);
#endif /* __ROBOTIC_ARM_H */
