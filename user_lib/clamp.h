#ifndef CLAMP_H
#define CLAMP_H

#include "main.h"
#include "pid.h"
#include "cmsis_os.h"
#include "own_lib.h"
#include "m3508.h"

//#define CLAMP_LIFT_MAX_POSITION -895000      // 上升最大位置
#define CLAMP_LIFT_SPECIAL_POSITION1 0 // 暂定对杆位置
#define CLAMP_LIFT_SPECIAL_POSITION2 -413000 //  抓取矛头位置

#define CLAMP_ROTATE_MAX_POSITION -101000    // 旋转最大角度位置，水平放置为0
#define CLAMP_EXTEND_MAX_POSITION -319000    // 伸缩最大位置

typedef struct
{
    int8_t catch_flag; //抓取状态标志，0为松手，1为抓取
    int8_t rotate_flag; //旋转状态标志，0为复位状态，1为倒立完毕
    int8_t retractable_flag; //伸缩状态标志，0为复位状态，1为伸出状态
    int8_t lift_flag; //升降状态标志，0为复位状态，1为抓矛头位置，2为抓矛头后抬起位置，3为暂定对杆位置
} clamp_struct;


extern clamp_struct clamp;//夹爪结构体

void clamp_position_init(M3508_Motor_Struct *lift_motor, M3508_Motor_Struct *rotate_motor, M3508_Motor_Struct *extend_motor);
void clamp_lift_control(uint8_t lift_flag, M3508_Motor_Struct *motor);
void clamp_rotate_control(int32_t target_angle, M3508_Motor_Struct * motor);
void clamp_extend_control(int32_t target_length, M3508_Motor_Struct * motor);
void clamp_open_control(int8_t open_flag);

#endif
