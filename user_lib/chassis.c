#include "chassis.h"

chassis_stu chassis;//r2底盘结构体

/**
 * @brief 全向底盘运动学计算函数，根据输入的vx、vy、vw计算底盘各电机的目标速度
 * @param chassis 底盘结构体指针
 * @param vx 前向速度
 * @param vy 横向速度
 * @param vw 旋转速度
 * @warning 该函数全程使用机器人坐标系，并且单位未定义----电机id:右前为id1，右后为id2，左后为id3，左前为id4
*/
void omni_chassis_calculate(chassis_stu *chassis, float vx, float vy, float vw)
{
    chassis->vx = vx;
    chassis->vy = vy;
    chassis->vw = vw;
    //根据全向轮运动学模型计算各轮速度
    chassis->wheel_speed[0] = (int16_t)(-vx - vy - vw);
    chassis->wheel_speed[1] = (int16_t)(-vx + vy - vw);
    chassis->wheel_speed[2] = (int16_t)( vx + vy - vw);
    chassis->wheel_speed[3] = (int16_t)( vx - vy - vw);
    m3508_speed_control(&m3508_fdcan3[0], chassis->wheel_speed[0], 8192);
    m3508_speed_control(&m3508_fdcan3[1], chassis->wheel_speed[1], 8192);
    m3508_speed_control(&m3508_fdcan3[2], chassis->wheel_speed[2], 8192);
    m3508_speed_control(&m3508_fdcan3[3], chassis->wheel_speed[3], 8192);
}


