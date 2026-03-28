#ifndef CLAMP_H
#define CLAMP_H
#include "m3508.h"
#define CLAMP_LIFT_MAX_POSITION -10000000 // 下降最大位置
#define CLAMP_LIFT_SPECIAL_POSITION1 -1600000 // 抓取矛头位置
#define CLAMP_LIFT_SPECIAL_POSITION2 -400000 // 抓取矛头后抬起位置
#define CLAMP_ROTATE_MAX_POSITION 589000 // 旋转最大角度位置，顺时针旋转为正
#define CLAMP_EXTEND_MAX_POSITION -357000// 伸缩最大位置
void clamp_position_init(M3508_Motor_Struct *lift_motor, M3508_Motor_Struct *rotate_motor, M3508_Motor_Struct *extend_motor);
void clamp_lift_control(int32_t target_position, M3508_Motor_Struct * motor) ;
void clamp_rotate_control(int32_t target_angle, M3508_Motor_Struct * motor);
void clamp_extend_control(int32_t target_length, M3508_Motor_Struct * motor);
void clamp_open_control(uint8_t open_flag);

#endif
