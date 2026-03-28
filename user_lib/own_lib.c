#include "own_lib.h"

/**
 * @brief 角度制转弧度制
*/
float deg2rad(float deg)
{
    return deg * (PI / 180.0f);
}

/**
 * @brief 弧度制转角度制
*/
float rad2deg(float rad)
{
    return rad * (180.0f / PI);
}

/**
 * @brief 简单绝对值函数(int16_t类型)
 */
int16_t abs_int16(int16_t value)
{
    return (value < 0) ? -value : value;
}

/**
 * @brief 简单绝对值函数(int32_t类型)
*/
int32_t abs_int32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/**
 * @brief 简单绝对值函数(float类型)
*/
float abs_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

/**
 * @brief 简单限幅函数(float类型)
*/
float limit_float(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}