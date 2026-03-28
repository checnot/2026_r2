#include "main.h"
#include "clamp.h"
#include "pid.h"
#include "cmsis_os.h"
#include "own_lib.h"

/**
 * @brief 夹爪初始化函数
 */
void clamp_position_init(M3508_Motor_Struct *lift_motor, M3508_Motor_Struct *rotate_motor, M3508_Motor_Struct *extend_motor)
{
    // // 先向下运动一段距离，再向上运动到头
    // for (uint16_t i = 0; i < 40; i++)
    // {
    //     m3508_speed_control(lift_motor, -5000, 10000); // 向下运动
    //     HAL_Delay(15);
    // }
    // m3508_speed_control(lift_motor, 0, 10000);
    // HAL_Delay(20);

    uint8_t init_flag = 1;
    while (init_flag)
    {
        m3508_speed_control(lift_motor, 5000, 10000); // 以较大速度向上移动
        HAL_Delay(15);
        if (lift_motor->Now_Speed < 100)
        {
            init_flag = 0;
        }
    }
    HAL_Delay(2);

    lift_motor->Angle_Sum = 0;   // 重置升降电机位置计数器
    rotate_motor->Angle_Sum = 0; // 重置旋转电机位置计数器
    extend_motor->Angle_Sum = 0; // 重置伸缩电机位置计数器
}
/**
 * @brief 夹爪升降控制函数
 */

void clamp_lift_control(int32_t target_position, M3508_Motor_Struct *motor)
{
    float position_error = target_position - motor->Angle_Sum; // 计算位置误差
    if (abs_float(position_error) < 10000.f)                   // 位置pid精准控制
    {
        m3508_anglesum_control(motor, target_position, 8000); // 使用位置PID控制
    }
    else // 位置误差较大时，先快速接近目标位置
    {
        int16_t speed = (position_error > 0) ? 5000 : -5000; // 大幅度控制
        m3508_speed_control(motor, speed, 10000);             // 使用速度PID控制
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
        m3508_anglesum_control(motor, target_angle, 8000); // 使用位置PID控制
    }
    else // 角度误差较大时，先快速接近目标角度
    {
        int16_t speed = (angle_error > 0) ? 5000 : -5000; // 大幅度控制
        m3508_speed_control(motor, speed, 8000);          // 使用速度PID控制
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
        m3508_anglesum_control(motor, target_length, 8000); // 使用位置PID控制
    }
    else // 长度误差较大时，先快速接近目标长度
    {
        int16_t speed = (length_error > 0) ? 4000 : -4000; // 大幅度控制
        m3508_speed_control(motor, speed, 8000);           // 使用速度PID控制
    }
}
/**
 * @brief 夹爪开合控制函数
 */
void clamp_open_control(uint8_t open_flag)
{
    if (open_flag)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET); // 打开夹爪
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET); // 关闭夹爪
    }
   
}
