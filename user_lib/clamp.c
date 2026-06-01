#include "clamp.h"

clamp_struct clamp;//夹爪结构体

/**
 * @brief 夹爪升降控制函数，当距离误差过大时，先以较大速度快速接近目标位置，当距离误差较小时，使用位置PID进行精准控制
 * @param target_position 目标位置，单位为电机位置计数值
 */
void clamp_lift_control(uint8_t lift_flag, M3508_Motor_Struct *motor)
{
    int32_t target_position;
    if (lift_flag == 0)
    {
        target_position = 0; // 复位位置
    }
    else if (lift_flag == 1)
    {
        target_position = CLAMP_LIFT_SPECIAL_POSITION1; // 抓矛头位置
    }
    else if (lift_flag == 2)
    {
        target_position = CLAMP_LIFT_SPECIAL_POSITION2; // 抓矛头后抬起位置
    }
    else if (lift_flag == 3)
    {
        target_position = CLAMP_LIFT_SPECIAL_POSITION3; // 暂定对杆位置
    }
    float position_error = target_position - motor->Angle_Sum; // 计算位置误差
    if (abs_float(position_error) < 10000.f)                   // 位置pid精准控制
    {
        m3508_anglesum_control(motor, target_position, 8192); // 使用位置PID控制
    }
    else // 位置误差较大时，先快速接近目标位置
    {
        int16_t speed = (position_error > 0) ? 5000 : -5000; // 大幅度控制
        m3508_speed_control(motor, speed, 8192);             // 使用速度PID控制
    }
}

/**
 * @brief 夹爪旋转控制函数
 */
void clamp_rotate_control(int32_t target_angle, M3508_Motor_Struct *motor)
{
    float angle_error = target_angle - motor->Angle_Sum; // 计算角度误差
    if (abs_float(angle_error) < 5000.f)                // 角度pid精准控制
    {
        m3508_anglesum_control(motor, target_angle, 8192); // 使用位置PID控制
    }
    else // 角度误差较大时，先快速接近目标角度
    {
        int16_t speed = (angle_error > 0) ? 4000 : -4000; // 大幅度控制
        m3508_speed_control(motor, speed, 8192);          // 使用速度PID控制
    }
}

/**
 * @brief 夹爪伸缩控制函数
 */
void clamp_extend_control(int32_t target_length, M3508_Motor_Struct *motor)
{
    float length_error = target_length - motor->Angle_Sum; // 计算长度误差
    if (abs_float(length_error) < 5000.f)                 // 长度pid精准控制
    {
        m3508_anglesum_control(motor, target_length, 8192); // 使用位置PID控制
    }
    else // 长度误差较大时，先快速接近目标长度
    {
        int16_t speed = (length_error > 0) ? 5000 : -5000; // 大幅度控制
        m3508_speed_control(motor, speed, 8192);           // 使用速度PID控制
    }
}

/**
 * @brief 夹爪开合控制函数，1为打开夹爪，0为关闭夹爪
 */
void clamp_open_control(int8_t open_flag)
{
    if (open_flag)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET); // 打开夹爪
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET); // 关闭夹爪
    }
   
}
