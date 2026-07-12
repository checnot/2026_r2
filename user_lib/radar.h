#ifndef __RADAR_H
#define __RADAR_H

#include "main.h"
#include "filter.h"
// #include "chassis.h"
#include "own_lib.h"
#include "hi14.h"
#include "keyboard.h"

/* ========== HEU 帧协议常量 ========== */
#define HEU_SYNC1           0x48  /* 'H' */
#define HEU_SYNC2           0x45  /* 'E' */
#define HEU_SYNC3           0x55  /* 'U' */
#define HEU_SRC_RADAR       0x20  /* 雷达遥测 (机器人接收) */
#define HEU_RADAR_PAYLOAD_MAX  80  /* 雷达遥测 ASCII 最大字节数 */

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

/* HEU 帧接收: 从 UART5 字节流中解析 src=0x20 雷达遥测帧 */
void Data_process_String(char *str);

void Radar_ProcessBuffer(uint8_t *data, uint16_t len);


/* 底盘坐标解算 */
void Data_process_String(char *str);
void Calculate_Chassis_Position(void);

#endif

