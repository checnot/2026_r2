#include "clamp.h"

clamp_struct clamp;//夹爪结构体
/**
 * @brief 夹爪升降控制函数（比例减速版本）
 * @note 距目标 > 50000 格时全速, 50000 以内速度线性递减, 到位自动归零
 */
void clamp_lift_control(uint8_t lift_flag, M3508_Motor_Struct *motor)
{
    int32_t target_position;
    if (lift_flag == 0)
        target_position = CLAMP_LIFT_SPECIAL_POSITION1;
    else if (lift_flag == 1)
        target_position =  CLAMP_LIFT_SPECIAL_POSITION2;

    int32_t position_error = target_position - motor->Angle_Sum;
    int32_t abs_err = abs_int32(position_error);
    int16_t max_speed = 7000;
    int32_t decel_zone = 50000;  /* 开始减速的距离 */

    if (abs_err < decel_zone)
    {
        /* 比例减速: 误差越小速度越低, 接近0时自然停止 */
        int16_t speed = (int16_t)((int32_t)max_speed * abs_err / decel_zone);
        if (position_error < 0) speed = -speed;
        m3508_speed_control(motor, speed, 8000);
    }
    else
    {
        int16_t speed = (position_error > 0) ? max_speed: -max_speed;
        m3508_speed_control(motor, speed, 8000);
    }
}

/**
 * @brief 夹爪伸缩控制函数（比例减速版本）
 */
void clamp_extend_control(int32_t target_length, M3508_Motor_Struct *motor)
{
    int32_t position_error = target_length - motor->Angle_Sum;
    int32_t abs_err = abs_int32(position_error);
    int16_t max_speed = 8000;
    int32_t decel_zone = 60000;

    if (abs_err < decel_zone)
    {
        int16_t speed = (int16_t)((int32_t)max_speed * abs_err / decel_zone);
        if (position_error < 0) speed = -speed;
        m3508_speed_control(motor, speed, 3000);
    }
    else
    {
        int16_t speed = (position_error > 0) ? max_speed : -max_speed;
        m3508_speed_control(motor, speed, 8000);
    }
}

/**
 * @brief 夹爪旋转控制函数（比例减速版本）
 */
void clamp_rotate_control(int32_t target_angle, M3508_Motor_Struct *motor)
{
    int32_t position_error = target_angle - motor->Angle_Sum;
    int32_t abs_err = abs_int32(position_error);
    int16_t max_speed = 6000;
    int32_t decel_zone = 30000;

    if (abs_err < decel_zone)
    {
        int16_t speed = (int16_t)((int32_t)max_speed * abs_err / decel_zone);
        if (position_error < 0) speed = -speed;
        m3508_speed_control(motor, speed, 3000);
    }
    else
    {
        int16_t speed = (position_error > 0) ? max_speed : -max_speed;
        m3508_speed_control(motor, speed, 8000);
    }
}

/**
 * @brief 夹爪开合控制函数，1为打开夹爪，0为关闭夹爪
 */
void clamp_open_control(int8_t open_flag)
{
    if (open_flag)
    {
        HAL_GPIO_WritePin(clamp_catch_GPIO_Port, clamp_catch_Pin, GPIO_PIN_SET); // 打开夹爪
    }
    else
    {
        HAL_GPIO_WritePin(clamp_catch_GPIO_Port, clamp_catch_Pin, GPIO_PIN_RESET); // 关闭夹爪
    }
   
}
