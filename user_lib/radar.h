#ifndef __RADAR_H
#define __RADAR_H

#include "main.h"
#include "filter.h"
// #include "chassis.h"
#include "own_lib.h"
#include "hi14.h"
#define RADAR_Filter_Threshold 80.0f //雷达数据滤波的速度阈值
typedef struct {
		float radar_x;
		float radar_y;
		float radar_z;
		float radar_yaw;
		float hi14_yaw;
        float radar_vx;
		float radar_vy;

		float radar_x_filtered;
		float radar_y_filtered;
		float radar_z_filtered;
		float radar_yaw_filtered;

		float reliable_x;
		float reliable_y;
		float reliable_z;
		float reliable_yaw;
}RADAR_POSITION;

typedef struct {
		float chassis_x;
		float chassis_y;
		float chassis_z;
		float chassis_yaw;
}CHASSIS_POSITION;

extern KalmanFilter_stu Radar_Kalman_X;
extern KalmanFilter_stu Radar_Kalman_Y;
extern KalmanFilter_stu Radar_Kalman_Z;
extern KalmanFilter_stu Radar_Kalman_Yaw;
extern float x, y, z, yaw;
extern RADAR_POSITION Radar_Pos;
extern CHASSIS_POSITION Chassis_Pos;
extern uint8_t Rec_Data[65];

void Data_process_String(char *str);
void Calculate_Chassis_Position(void);

#endif
