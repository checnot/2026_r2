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
    m3508_control_speed(&m3508_fdcan3[0], chassis->wheel_speed[0], 8192);
    m3508_control_speed(&m3508_fdcan3[1], chassis->wheel_speed[1], 8192);
    m3508_control_speed(&m3508_fdcan3[2], chassis->wheel_speed[2], 8192);
    m3508_control_speed(&m3508_fdcan3[3], chassis->wheel_speed[3], 8192);
}

/**
 * @brief 底盘前端升降控制函数，根据输入的升降状态控制底盘前端气泵的开合
 * @param lift_front 前端升降状态，0代表收回，1代表推出
 */
void chassis_lift_front_control(uint8_t lift_front)
{
    if (lift_front)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET); // 打开前端气泵
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET); // 关闭前端气泵
    }
   
}
/**
 * @brief 底盘后端升降控制函数，根据输入的升降状态控制底盘后端气泵的开合
 * @param lift_back 后端升降状态，0代表收回，1代表推出
 */
void chassis_lift_back_control(uint8_t lift_back)
{
    if (lift_back)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); // 打开后端气泵
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); // 关闭后端气泵
    }
}   
/**
 * @brief 底盘升降控制函数，根据输入的升降状态控制底盘前后端气泵的开合
 * @param lift_mode 同时升降与前后分开升降两种模式,0代表同时升降，1代表前后分开升降
 * @param lift_front_flag 前端升降状态，0代表收回，1代表推出
 * @param lift_back_flag 后端升降状态，0代表收回，1代表推出
 * @param lift_whole_flag 同时升降模式下的升降状态，0代表收回，1代表推出
 */
void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag)
{
    if (lift_mode == 0) // 同时升降模式
    {
        chassis_lift_front_control(lift_whole_flag);
        chassis_lift_back_control(lift_whole_flag);
    }
    else // 前后分开升降模式
    {
        chassis_lift_front_control(lift_front_flag);
        chassis_lift_back_control(lift_back_flag);
    }
}
