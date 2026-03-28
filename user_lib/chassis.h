#ifndef CHASSIS_H
#define CHASSIS_H

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "mycan.h"
#include "myuart.h"
#include "m3508.h"
#include "ht10a.h"

/* 底盘控制相关结构体定义 */
typedef struct {
    float vx;           // 前向速度 (m/s)
    float vy;           // 横向速度 (m/s)
    float vw;           // 旋转速度 (rad/s)
    uint8_t lift_front, lift_back; // 前后升降状态，0代表收回，1代表推出
    int16_t wheel_speed[4]; // 四轮速度，直接对应轮部3508电机的目标速度
} chassis_stu;

extern chassis_stu chassis; // 全局底盘实例

void omni_chassis_calculate(chassis_stu *chassis, float vx, float vy, float vw);
void chassis_lift_front_control(uint8_t lift_front);
void chassis_lift_back_control(uint8_t lift_back);  
void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag);

#endif /* CHASSIS_H */

