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
	// int32_t Initial_Angle;
	int16_t Initial_flag;

	int32_t Target_Angle;
	int16_t Target_Speed;

	int32_t Start_Angle_Sum;
	uint8_t Change_Start_Angle_Sum;

	PID_Struct M3508_Speed_PID;
	PID_Struct M3508_Angle_PID;
	int16_t Current_Output;
} M3508_Motor_Struct;//m3508电机结构体

extern M3508_Motor_Struct m3508_fdcan1[8];//定义m3508电机结构体数组，支持最多8个电机
extern M3508_Motor_Struct m3508_fdcan2[8];
extern M3508_Motor_Struct m3508_fdcan3[8];


void m3508_speed_control(M3508_Motor_Struct *motor, int16_t target_speed, int16_t max_output);
void m3508_anglesum_control(M3508_Motor_Struct *motor, int32_t target_anglesum, int16_t max_output);
void m3508_anglesum_ETA_control(M3508_Motor_Struct *motor, int32_t start_anglesum, int32_t target_anglesum, float ETA);

void FDCAN_Send_M3508(FDCAN_HandleTypeDef *hfdcan, M3508_Motor_Struct *Motor);
void M3508_Return_Process(FDCAN_RxHeaderTypeDef* RxHeader, uint8_t Data[], M3508_Motor_Struct *Motor);
void m3508_parameter_init(M3508_Motor_Struct *Motor, float K_p_speed, float K_d_speed, float K_p_angle, float K_d_angle);

#endif /* M3508_H */
