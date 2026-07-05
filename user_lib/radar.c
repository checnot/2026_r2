#include "radar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis.h"
/**Livox雷达的数据回传处理**/

RADAR_POSITION Radar_Pos;
CHASSIS_POSITION Chassis_Pos;
KalmanFilter_stu Radar_Kalman_X;
KalmanFilter_stu Radar_Kalman_Y;
KalmanFilter_stu Radar_Kalman_Z;
KalmanFilter_stu Radar_Kalman_Yaw;
float x, y, z, yaw;
uint8_t Rec_Data[65];

void Data_process_String(char *str)
{
    // 1. 寻找帧头 "heu"
    char *header_ptr = strstr(str, "heu");
    
    if (header_ptr != NULL)
    {
        float temp_a, temp_b, temp_c, temp_d,temp_e,temp_f  ;
        
        // 2. 尝试解析四个浮点数
        if (sscanf(header_ptr + 3, "%f,%f,%f,%f,%f,%f", &temp_a, &temp_b, &temp_c, &temp_d, &temp_e, &temp_f) >= 6)
        {
            Radar_Pos.radar_x = 1000.0f * temp_a;
            Radar_Pos.radar_y = 1000.0f * temp_b;
            Radar_Pos.radar_z = 1000.0f * temp_c;
            Radar_Pos.radar_yaw = temp_d;
            Radar_Pos.radar_vx = temp_e;
            Radar_Pos.radar_vy = temp_f;
            Radar_Pos.hi14_yaw = hipnuc_raw.hi91.yaw;
          
        
//            if(abs_float(chassis_r2.vx) <20.0f && abs_float(chassis_r2.vy)  <20.0f)
//            {
//            Kalman_Init(&Radar_Kalman_X, 0.0001f, 100.0f, 0.0f, Radar_Pos.radar_x);
//            Kalman_Init(&Radar_Kalman_Y, 0.0001f, 100.0f, 0.0f, Radar_Pos.radar_y);

//            Radar_Pos.radar_x=Kalman_Update(&Radar_Kalman_X, Radar_Pos.radar_x);
//            Radar_Pos.radar_y=Kalman_Update(&Radar_Kalman_Y, Radar_Pos.radar_y);
            
//            }
//            // 解算底盘坐标
            Calculate_Chassis_Position();
					
        }
    }
}

/**
 * @brief  根据雷达坐标解算底盘中心坐标
 * @note   当雷达正着放置在原点时，底盘中心相对于雷达的偏移：
 *         x车 = x雷达 - 385mm
 *         y车 = y雷达 + 225mm
 *         角度直接使用雷达角度（-180°到+180°范围）
 */
void Calculate_Chassis_Position(void)
{
    // 底盘中心相对于雷达的偏移（单位：mm）
    //镜像时记得改Y
    const float OFFSET_X = -290.15f;  // 车体坐标系下，底盘中心相对雷达的x偏移
    const float OFFSET_Y = -152.15f;   // 车体坐标系下，底盘中心相对雷达的y偏移

    // 角度转弧度
    float yaw_rad = Radar_Pos.hi14_yaw * 3.1415926f / 180.0f;
    // 坐标变换（绕雷达yaw旋转）
    float dx = OFFSET_X * cosf(yaw_rad) - OFFSET_Y * sinf(yaw_rad);
    float dy = OFFSET_X * sinf(yaw_rad) + OFFSET_Y * cosf(yaw_rad);

    Radar_Pos.radar_x = Radar_Pos.radar_x + dx;
    Radar_Pos.radar_y = Radar_Pos.radar_y + dy;

}
