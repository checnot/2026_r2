#ifndef M3508_H
#define M3508_H

#include "main.h"
#include "pid.h"
#include "own_lib.h"
#include "mycan.h"
#include "string.h"

typedef struct
{
	int16_t Now_Speed;
    int16_t Now_Angle;
	int16_t Now_Current;
	int32_t Angle_Sum;
	int16_t Angle_Last;
	
	int32_t Target_Angle;
	int16_t Target_Speed;

	PID_Struct M3508_Speed_PID;
	PID_Struct M3508_Angle_PID;
	int16_t Current_Output;
} M3508_Motor_Struct;//m3508电机结构体

extern M3508_Motor_Struct m3508_fdcan1[8];//定义m3508电机结构体数组，支持最多8个电机
extern M3508_Motor_Struct m3508_fdcan2[8];
extern M3508_Motor_Struct m3508_fdcan3[8];


void m3508_control_speed(M3508_Motor_Struct *motor, int32_t target_speed, int32_t max_output);
void m3508_control_angle(M3508_Motor_Struct *motor, int32_t target_angle, int32_t max_output);
void FDCAN_Send_M3508(FDCAN_HandleTypeDef *hfdcan, M3508_Motor_Struct *Motor);
void M3508_Return_Process(FDCAN_RxHeaderTypeDef* RxHeader, uint8_t Data[], M3508_Motor_Struct *Motor);

#endif /* M3508_H */
