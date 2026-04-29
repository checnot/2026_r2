#include "route.h"
#include "pid.h"
#include "math.h"
#include "cmsis_os.h"
#include "Chassis.h"
#include "FreeRTOS.h"
#include "task.h"
#include "radar.h"
#include "chassis.h"
#include <stdint.h>
#include "arm_math.h"
#include "main.h"
/*========================================================================================================================
                                                代码经验
1.在cube会刷的文件里面不写注释，cube刷配置的时候会换编码方式，注释全部变乱码（所以main和RTOS里面只调用，和下面提到的相呼应）
2.代码模块化、接口化，优点 -> 方便多人协作（不需要理解代码内核实现，直接调用接口）
                            在main和RTOS里面直接调用函数即可，修改的时候直接定位函数接口修改方便改参数和修Bug
3.嵌入式软件开发里面因为涉及到闭环控制，所以很多时候需要将函数卡到一个while里面，通过不断地判断来实现闭环控制
                                    （一定记得while里面要写osDelay，这是RTOS跳转到其他任务的接口）（单纯的软件开发不会涉及这些）
4.不同机构之间一定要用RTOS来写到不同的RTOS任务里面，然后利用全局变量标志位作为接口钥匙，来实现不同代码模块和结构模块的通讯
                                    （一个机构一个任务，因为同一时间同一机构不能执行不同动作）
一个机构一个RTOS任务，因为同一个机构的不同函数不可能同时执行，所以不用担心卡while的情况
                    优点 -> 方便多人协作，可以合理分工完成不同机构代码模块的编写，然后约定全局变量标志位通讯即可
                            不分开写到不同任务里面会导致在一个闭环控制任务里面While卡死，整个程序无法继续推进
5.在嵌入式软件开发中，几乎所有的参数全部都要写成活的，且在闭环控制判断的时候判据要留有余量
                                    （因为嵌入式软件开发不完全是代码世界，涉及现实世界硬件机械误差等等）
6.所有的解算一定要分级分层！分层的思想特别重要！（这个个人认为适用于所有软件开发涉及解算的部分）
7.分层和模块化的思想可以体现在项目列表的文件夹里，现在全放到一个文件夹里较难找到
8.头文件相当于程序的接口，体现了文件对外的作用，源文件相当于接口功能的实现，所以头文件中东西能少尽量少，变量，结构体，内部函数全写私有的
9.static将函数和变量的作用域限制在当前文件内，其他文件没有办法通过extern来访问，在函数里面用可以延长变量的生命周期，每次调用时不初始化
10.在使用VESC上位机的时候，注意把CAN线拔了
11.以后如果会涉及结构设计方向，一定记得余量和限位思想。电控调不坏的机构，方便检修的机构才是好机构
12.涉及角度闭环控制一定注意加入速度环，目的是给予限流，且参数能够适配不同角度差
13.在进行线性映射的时候注意和现实世界的物理量结合，能映射到现实物理量在程序设计debug上会方便很多
14.在碰到从一个协议更换到另一个协议的时候，可以选择不动原来协议，将新协议映射到旧协议里面（映射两次，虽然增加代码量，但是方便修改）
15.在数据波形不好的时候采用滤波器来处理数据，部分滤波器虽然有滞后效应，但传输足够快可以忽略
16.回去有空可以用matlab仿真来完成日常控制器的设计

                                                                                                                      孔维彬
                                                                                                               2025年6月16日
                                                                                                                  写于哈尔滨
==========================================================================================================================*/

/*===================================================================================================================
                                                操作手须知
1.有关初始位置，运球机构的初始位置在右侧铝管与齿条边缘重合处，扳机初始位置在距离白色打印件最远处
2.在运球预选赛的时候打开自动路径即开启自动跑全程
=====================================================================================================================*/
// 角度制转换为弧度制系数
#define CHANGE_TO_RADIAN (0.01745329251994f)
// 弧度制转换为角度制系数
#define CHANGE_TO_ANGLE (57.29577951308232f)
// 角度限制处理函数，这个是在江阴路径角度出现问题，更改时添加的
#define PI 3.14159265358979f
#define ANGLE_MIN_AREA_PROTECTION 0.01f // 角度区间最小保护值（度）
#define VW_MAX_LIMIT 2000.0f            // Vw最大限制值
#define FITTING_INPUT_MIN 0.0f          // 插值函数输入最小值
#define FITTING_INPUT_MAX 1.0f          // 插值函数输入最大值
#define SMALL_ANGLE_THRESHOLD 1.0f      // 小角度阈值（度），小于此值时使用保守参数

Coordinate_Position_Struct World_Coordinate_System_NowPos; // 世界坐标系下当前位置
Coordinate_Speed_Struct World_Coordinate_System_Target_V;  // 世界坐标系下目标速度
Coordinate_Speed_Struct Robot_Coordinate_System_Target_V;  // 车身坐标系下目标速度
Coordinate_Speed_Struct Robot_Coordinate_System_V;         // 路径计算出的车身速度，用于车轮速度分配
Coordinate_Position_Struct Zero_Point = {0, 0, 0};         // 世界坐标系零点
Coordinate_Position_Struct Now_World_Positon;              // 进行投篮时获得一个实时更新的全局坐标，不用雷达数据的原因是数据类型不统一

Route_STU Route_Status; // 路径结构体

PID_Struct Line_AdjustPID;       // 直线法向pid调节
PID_Struct Line_StopPID;         // 直线停止区切向pid调节
PID_Struct Line_AnglePID;        // 直线停止区角度pid调节
PID_Struct Cir_AdjustPID;        // 圆弧法向pid调节
PID_Struct Cir_StopPID;          // 圆弧停止区切向pid调节
PID_Struct Cir_AnglePID;         // 圆弧角度pid调节
PID_Struct Keep_X_PID;           // 保持x方向位置
PID_Struct Keep_Y_PID;           // 保持y方向位置
PID_Struct Keep_W_PID;           // 保持角度
PID_Struct Automatic_Aiming_PID; // 自瞄pid

float Basketball_Angle; // 球框的目标角度，设置为全局变量方便观察
float Angle_Aim = 0;    // 目标角度值（绝对角度），中间变量
// float Angle_Offset = 0; // 删除：剩余要转的角度，改用 Status->Parameter.Abs_Angle_Error
float Line_Angle = 0; // 线坐标系与世界坐标系夹角
// float Angle_Dis = 0;    // 删除：角度转动距离，改用 Status->Parameter.Angle_Distance_Traveled

// extern Disk_Encoder_Struct Disk_Encoder;

/*===================================================================================================================
                                            此部分往下为float保护部分
=====================================================================================================================*/
/*********************************************************************************
 * @name   Safe_fabs
 * @brief  安全的绝对值函数，防止异常输入
 * @param  value: 输入值
 * @retval 安全的绝对值
 *********************************************************************************/
static inline float Safe_fabs(float value)
{
    // 检查输入是否为有效数值
    if (isnan(value) || isinf(value))
    {
        return 0.0f;
    }
    return fabsf(value);
}

/*********************************************************************************
 * @name   Clamp_Float
 * @brief  浮点数限幅函数
 * @param  value: 输入值
 * @param  min_val: 最小值
 * @param  max_val: 最大值
 * @retval 限幅后的值
 *********************************************************************************/
static inline float Clamp_Float(float value, float min_val, float max_val)
{
    if (isnan(value) || isinf(value))
    {
        return 0.0f;
    }
    if (value < min_val)
        return min_val;
    if (value > max_val)
        return max_val;
    return value;
}

/*********************************************************************************
 * @name   Safe_Division
 * @brief  安全除法，防止除零和极小值
 * @param  numerator: 分子
 * @param  denominator: 分母
 * @param  protection_value: 保护值，当分母过小时使用
 * @retval 安全的除法结果
 *********************************************************************************/
static inline float Safe_Division(float numerator, float denominator, float protection_value)
{
    float abs_denominator = Safe_fabs(denominator);

    // 如果分母过小，使用保护值
    if (abs_denominator < protection_value)
    {
        abs_denominator = protection_value;
    }

    return numerator / abs_denominator;
}

/*********************************************************************************
 * @name   Enhanced_Polynomial_Fitting_Function
 * @brief  增强的5次多项式插值函数 - 更平滑的加减速曲线
 * @param  x: 输入比例 (0.0 到 1.0)
 * @retval 插值结果 (0.0 到 1.0)
 *
 * 使用5次多项式: f(x) = 6x^5 - 15x^4 + 10x^3
 * 特点：在0和1处一阶、二阶导数都为0，保证平滑过渡
 *********************************************************************************/
float Enhanced_Polynomial_Fitting_Function(float x)
{
    // 输入范围保护
    x = Clamp_Float(x, FITTING_INPUT_MIN, FITTING_INPUT_MAX);

    // 5次多项式插值: 6x^5 - 15x^4 + 10x^3
    float x2 = x * x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;

    float result = 6.0f * x5 - 15.0f * x4 + 10.0f * x3;

    // 输出范围保护
    return Clamp_Float(result, 0.0f, 1.0f);
}

/*********************************************************************************
 * @name   Power_S_Curve_Fitting_Function
 * @brief  幂次S曲线插值函数 - 可调节的平滑度
 * @param  x: 输入比例 (0.0 到 1.0)
 * @param  power: 幂次参数 (推荐2.0-4.0)
 * @retval 插值结果 (0.0 到 1.0)
 *********************************************************************************/
float Power_S_Curve_Fitting_Function(float x, float power)
{
    // 输入范围保护
    x = Clamp_Float(x, FITTING_INPUT_MIN, FITTING_INPUT_MAX);
    power = Clamp_Float(power, 1.0f, 6.0f); // 限制幂次范围

    // 幂次S曲线: x^power / (x^power + (1-x)^power)
    float x_power = powf(x, power);
    float one_minus_x_power = powf(1.0f - x, power);

    // 防止除零
    float denominator = x_power + one_minus_x_power;
    if (denominator < 1e-6f)
    {
        return x; // 退化为线性
    }

    float result = x_power / denominator;
    return Clamp_Float(result, 0.0f, 1.0f);
}

/*********************************************************************************
 * @name   Safe_Angle_Area_Calculate
 * @brief  安全的角度区间计算，防止除零和极小值问题
 * @param  angle_rotate_sum: 总旋转角度
 * @param  area_rate: 区间比例 (0.0 到 1.0)
 * @param  min_protection: 最小保护值
 * @retval 安全的角度区间大小
 *********************************************************************************/
float Safe_Angle_Area_Calculate(float angle_rotate_sum, float area_rate, float min_protection)
{
    // 输入参数保护
    float abs_angle = Safe_fabs(angle_rotate_sum);
    area_rate = Clamp_Float(area_rate, 0.0f, 1.0f);

    // 计算区间大小
    float area_size = area_rate * abs_angle;

    // 应用最小保护值
    if (area_size < min_protection)
    {
        area_size = min_protection;
    }

    return area_size;
}

/*********************************************************************************
 * @name   Adaptive_Angle_Parameters
 * @brief  自适应角度参数调整 - 根据角度大小动态调整参数
 * @param  angle_rotate_sum: 总旋转角度
 * @param  speedup_rate: 原始加速区比例
 * @param  slow_rate: 原始减速区比例
 * @param  adjusted_speedup_rate: 调整后的加速区比例 (输出)
 * @param  adjusted_slow_rate: 调整后的减速区比例 (输出)
 *********************************************************************************/
void Adaptive_Angle_Parameters(float angle_rotate_sum,
                               float speedup_rate,
                               float slow_rate,
                               float *adjusted_speedup_rate,
                               float *adjusted_slow_rate)
{
    float abs_angle = Safe_fabs(angle_rotate_sum);

    // 小角度时使用更保守的参数
    if (abs_angle < SMALL_ANGLE_THRESHOLD)
    {
        *adjusted_speedup_rate = speedup_rate * 0.5f; // 减少加速区
        *adjusted_slow_rate = slow_rate * 0.5f;       // 减少减速区
    }
    // 中等角度
    else if (abs_angle < 10.0f)
    {
        *adjusted_speedup_rate = speedup_rate * 0.8f;
        *adjusted_slow_rate = slow_rate * 0.8f;
    }
    // 大角度时使用原参数
    else
    {
        *adjusted_speedup_rate = speedup_rate;
        *adjusted_slow_rate = slow_rate;
    }

    // 确保调整后的参数合理
    *adjusted_speedup_rate = Clamp_Float(*adjusted_speedup_rate, 0.01f, 0.5f);
    *adjusted_slow_rate = Clamp_Float(*adjusted_slow_rate, 0.01f, 0.5f);

    // 确保加速区+减速区不超过100%
    float total_rate = *adjusted_speedup_rate + *adjusted_slow_rate;
    if (total_rate > 1.0f)
    {
        float scale = 0.9f / total_rate; // 留10%给恒速区
        *adjusted_speedup_rate *= scale;
        *adjusted_slow_rate *= scale;
    }
}

/*********************************************************************************
 * @name   Safe_Vw_Calculate
 * @brief  安全的角速度计算 - 集成所有保护机制
 * @param  angle_rotate_sum: 总旋转角度
 * @param  angle_dis: 已转过的角度
 * @param  angle_offset: 剩余角度
 * @param  start_w_speed: 起始角速度
 * @param  max_w_speed: 最大角速度
 * @param  speedup_rate: 加速区比例
 * @param  slow_rate: 减速区比例
 * @param  angle_stop_area: 角度停止区
 * @retval 安全限幅后的角速度Vw
 *********************************************************************************/
float Safe_Vw_Calculate(float angle_rotate_sum,
                        float angle_dis,
                        float angle_offset,
                        float start_w_speed,
                        float max_w_speed,
                        float speedup_rate,
                        float slow_rate,
                        float angle_stop_area)
{
    float vw = 0.0f;

    // 输入参数验证
    if (Safe_fabs(angle_rotate_sum) < 0.001f)
    {
        return 0.0f; // 无需旋转
    }

    // 自适应参数调整
    float adjusted_speedup_rate, adjusted_slow_rate;
    Adaptive_Angle_Parameters(angle_rotate_sum, speedup_rate, slow_rate,
                              &adjusted_speedup_rate, &adjusted_slow_rate);

    // 计算安全的角度区间
    float angle_speedup_area = Safe_Angle_Area_Calculate(angle_rotate_sum,
                                                         adjusted_speedup_rate,
                                                         ANGLE_MIN_AREA_PROTECTION);
    float angle_slow_area = Safe_Angle_Area_Calculate(angle_rotate_sum,
                                                      adjusted_slow_rate,
                                                      ANGLE_MIN_AREA_PROTECTION);

    // 确保停止区不会过大
    angle_stop_area = Clamp_Float(angle_stop_area, 0.1f, angle_slow_area * 0.5f);

    // 角速度计算 - 使用增强的插值函数
    if (angle_dis < angle_speedup_area)
    {
        // 加速区
        float ratio = Safe_Division(angle_dis, angle_speedup_area, ANGLE_MIN_AREA_PROTECTION);
        float fitting_value = Enhanced_Polynomial_Fitting_Function(ratio);
        vw = start_w_speed + fitting_value * (max_w_speed - start_w_speed);
    }
    else if (angle_dis >= angle_speedup_area &&
             angle_offset > angle_slow_area &&
             angle_offset <= Safe_fabs(angle_rotate_sum))
    {
        // 恒速区
        vw = max_w_speed;
    }
    else if (angle_offset <= angle_slow_area && angle_offset > angle_stop_area)
    {
        // 减速区
        float ratio = Safe_Division(angle_offset, angle_slow_area, ANGLE_MIN_AREA_PROTECTION);
        float fitting_value = Enhanced_Polynomial_Fitting_Function(ratio);
        vw = fitting_value * max_w_speed;
    }
    else if (angle_offset <= angle_stop_area)
    {
        // 停止区 - 这里应该用PID控制，返回一个小的期望值供PID处理
        vw = 0.0f; // 由外部PID控制器处理
    }

    // 考虑旋转方向
    if (angle_rotate_sum < 0)
    {
        vw *= -1.0f;
    }

    // 最终限幅保护
    vw = Clamp_Float(vw, -VW_MAX_LIMIT, VW_MAX_LIMIT);

    return vw;
}

/*********************************************************************************
 * @name 	ABS
 * @brief	安全的绝对值函数
 * @param	a: 输入值
 * @retval	绝对值
 *********************************************************************************/
static float ABS(float a)
{
    return a > 0 ? a : -a;
}
/*===================================================================================================================
                                            此部分往上为float保护部分
=====================================================================================================================*/

/*********************************************************************************
 * @name 	Calculate_Line_Angle
 * @brief	计算世界坐标系下起始点到终止点连线和X轴的夹角
 * @param	Start_Point:起始点：
 * @param	End_Point:终止点;
 * @retval	两点矢量方向角度 -180~180
 *********************************************************************************/
float Calculate_Line_Angle(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point)
{
    float a = 0.0f;
    float b = 0.0f;

    a = End_Point.Y - Start_Point.Y;
    b = End_Point.X - Start_Point.X;
    // atan2f范围可以包含-180到180
    return (atan2f(a, b) * CHANGE_TO_ANGLE);
}

/*********************************************************************************
 * @name 	Calculate_Line_Distance
 * @brief	计算点到点的距离
 * @param	Start_Point 起始点
 * @param	End_Point 结束点
 * @retval   返回两点之间的距离
 *********************************************************************************/
float Calculate_Line_Distance(Coordinate_Position_Struct Start_Point, Coordinate_Position_Struct End_Point)
{
    float Distant;
    Distant = sqrt((Start_Point.X - End_Point.X) * (Start_Point.X - End_Point.X) + (Start_Point.Y - End_Point.Y) * (Start_Point.Y - End_Point.Y));
    return Distant;
}

/*********************************************************************************
 * @name 	Position_Coordinate_Transformation
 * @brief: 位置坐标系旋转变换
 * @param  Coordinate_To_Convert 要转换的坐标系
 * @param  CoordinateSystem 新的坐标系的原点
 * @param  angle 坐标系夹角，逆时针旋转为正
 * @retval 转换后的坐标系
 *********************************************************************************/
Coordinate_Position_Struct Position_Coordinate_Transformation(Coordinate_Position_Struct *Coordinate_To_Convert, Coordinate_Position_Struct *CoordinateSystem, float angle)
{
    float rad = (float)angle * CHANGE_TO_RADIAN;
    Coordinate_Position_Struct Coordinate_Result;
    Coordinate_Result.Y = +(Coordinate_To_Convert->Y - CoordinateSystem->Y) * arm_cos_f32(rad) - (Coordinate_To_Convert->X - CoordinateSystem->X) * arm_sin_f32(rad);
    Coordinate_Result.X = +(Coordinate_To_Convert->Y - CoordinateSystem->Y) * arm_sin_f32(rad) + (Coordinate_To_Convert->X - CoordinateSystem->X) * arm_cos_f32(rad);
    Coordinate_Result.W = Coordinate_To_Convert->W;
    return Coordinate_Result;
}
/*********************************************************************************
 * @name 	Speed_Coordinate_Transformation
 * @brief: 速度坐标系旋转变换
 * @param  Coordinate_To_Convert 要转换的坐标系
 * @param  CoordinateSystem 新的坐标系的原点
 * @param  angle 坐标系夹角，逆时针旋转为正
 * @retval 转换后的坐标系
 *********************************************************************************/
Coordinate_Speed_Struct Speed_Coordinate_Transformation(Coordinate_Speed_Struct *Coordinate_To_Convert, Coordinate_Speed_Struct *CoordinateSystem, float angle)
{
    double rad = angle * CHANGE_TO_RADIAN;
    Coordinate_Speed_Struct Coordinate_Result;
    Coordinate_Result.Vy = +(Coordinate_To_Convert->Vy - CoordinateSystem->Vy) * arm_cos_f32(rad) - (Coordinate_To_Convert->Vx - CoordinateSystem->Vx) * arm_sin_f32(rad);
    Coordinate_Result.Vx = +(Coordinate_To_Convert->Vy - CoordinateSystem->Vy) * arm_sin_f32(rad) + (Coordinate_To_Convert->Vx - CoordinateSystem->Vx) * arm_cos_f32(rad);
    Coordinate_Result.Vw = Coordinate_To_Convert->Vw;
    return Coordinate_Result;
}
/*********************************************************************************
 * @name 	Fitting_Function
 * @brief: 加速区和减速区的拟合函数
 * @param  x 传入加速区所占的比例，如果是减速区的话传入(1-x),(传入的数在0到1之间)
 *********************************************************************************/
float Fitting_Function(float x)
{
    float V_Rate;
    V_Rate = (sinf(PI * (x - 0.5f)) + 1.0f) / 2.0f;
    return V_Rate;
}

/*********************************************************************************
 * @name 	Stop_Area_Slide_PID_Calculate
 * @brief: 停止区滑行PID计算（使用多项式插值）
 * @param  PID PID结构体指针
 * @param  Target 目标位置
 * @param  Measure 当前测量位置
 * @param  Start 起始位置
 * @param  Kp_Parameter Kp参数基准值
 * @param  Kd_Parameter Kd参数基准值
 * @retval 返回计算后的PID结构体
 *********************************************************************************/
PID_Struct Stop_Area_Slide_PID_Calculate(PID_Struct *PID, float Target, float Measure, float Start, float Kp_Parameter, float Kd_Parameter)
{
    float Distance = Target - Start;        // 计算总距离
    float Rest_Distance = Target - Measure; // 计算剩余距离

    float Parameter = 1 - fabsf(Rest_Distance / Distance); // 计算剩余距离占总距离的比例,越近越大
    Parameter = Clamp_Float(Parameter, 0.0f, 1.0f);        // 确保参数在合理范围内

    // 使用3次多项式插值替代线性插值
    // 多项式函数: f(x) = 2x3 - 3x2 + 1 (当x=1时f(x)=0, 当x=0时f(x)=1)
    // 这样越接近目标点(Parameter越大)，PID参数越小
    float polynomial_factor = 2.0f * Parameter * Parameter * Parameter - 3.0f * Parameter * Parameter + 1.0f;

    // 确保多项式因子在合理范围内
    polynomial_factor = Clamp_Float(polynomial_factor, 0.0f, 1.0f);

    // 使用多项式插值计算PID参数
    PID->Kp = polynomial_factor * Kp_Parameter; // 基础值10保证至少有值
    PID->Ki = 0;                                     // 积分项保持为0
    PID->Kd = 2 * PID->Kp ;                                     // 微分项也使用多项式插值
    return *PID;
}
/*********************************************************************************
 * @name   Normalize_Angle
 * @brief  角度归一化到 [-180, 180] 度范围（适配雷达回传范围）
 * @param  angle: 输入角度
 * @retval 归一化后的角度
 *********************************************************************************/
static float Normalize_Angle(float angle)
{
    // 处理NaN和无穷大
    if (isnan(angle) || isinf(angle))
    {
        return 0.0f;
    }

    // 将角度归一化到 [-180, 180] 范围
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}

/*********************************************************************************
 * @name   Calculate_Angle_Difference
 * @brief  计算角度差（考虑最短路径，处理-180/+180突变）
 * @param  target: 目标角度
 * @param  current: 当前角度
 * @retval 角度差（带符号，正值表示逆时针）
 *********************************************************************************/
// static float Calculate_Angle_Difference(float target, float current)
// {
//     // 确保输入角度在有效范围内
//     target = Normalize_Angle(target);
//     current = Normalize_Angle(current);

//     float diff = target - current;

//     // 处理跨越-180/+180边界的情况，选择最短路径
//     if (diff > 180.0f)
//     {
//         diff -= 360.0f; // 选择顺时针方向
//     }
//     else if (diff < -180.0f)
//     {
//         diff += 360.0f; // 选择逆时针方向
//     }

//     return diff;
// }

/*********************************************************************************
 * @name   Calculate_Absolute_Angle_Distance
 * @brief  计算绝对角度距离（用于区域判断，处理突变问题）
 * @param  angle1: 角度1
 * @param  angle2: 角度2
 * @retval 绝对角度距离
 *********************************************************************************/
// static float Calculate_Absolute_Angle_Distance(float angle1, float angle2)
// {
//     return fabsf(Calculate_Angle_Difference(angle1, angle2));
// }

/*********************************************************************************
 * @name   Safe_Angle_Normalization
 * @brief  安全的角度归一化，包含输入验证
 * @param  angle: 输入角度
 * @retval 归一化后的安全角度
 *********************************************************************************/
static float Safe_Angle_Normalization(float angle)
{
    // 检查输入是否为有效数值
    if (isnan(angle) || isinf(angle))
    {
        return 0.0f;
    }

    // 限制输入范围，防止极值计算
    if (angle > 36000.0f || angle < -36000.0f)
    {
        // 如果角度超出合理范围，取模后归一化
        angle = fmodf(angle, 360.0f);
    }

    return Normalize_Angle(angle);
}

/*********************************************************************************
 * @name   Calculate_Angle_Error_With_Direction
 * @brief  计算角度误差，始终选择最短路径，返回带方向的误差
 * @param  target: 目标角度 [-180, 180]
 * @param  current: 当前角度 [-180, 180]
 * @retval 角度误差（正值：需要逆时针，负值：需要顺时针）
 *********************************************************************************/
static float Calculate_Angle_Error_With_Direction(float target, float current)
{
    // 归一化到 [-180, 180]
    target = Normalize_Angle(target);
    current = Normalize_Angle(current);

    float error = target - current;

    // 选择最短路径
    if (error > 180.0f)
    {
        error -= 360.0f; // 实际应该顺时针转
    }
    else if (error < -180.0f)
    {
        error += 360.0f; // 实际应该逆时针转
    }

    return error;
}

/*********************************************************************************
 * @name   Calculate_Angle_Progress
 * @brief  计算角度旋转的进度 [0, 1]
 * @param  start: 起始角度
 * @param  target: 目标角度
 * @param  current: 当前角度
 * @retval 进度值 [0, 1]，0表示刚开始，1表示已到达
 *********************************************************************************/
static float Calculate_Angle_Progress(float start, float target, float current)
{
    // 计算总的角度变化量（最短路径）
    float total_change = Calculate_Angle_Error_With_Direction(target, start);

    // 如果目标角度变化很小，认为已经完成
    if (fabsf(total_change) < 0.1f)
    {
        return 1.0f;
    }

    // 计算当前已完成的角度变化量
    float current_change = Calculate_Angle_Error_With_Direction(current, start);

    // 确保方向一致
    if ((total_change > 0 && current_change < 0) || (total_change < 0 && current_change > 0))
    {
        // 方向相反，说明还没开始正确的旋转
        return 0.0f;
    }

    // 计算进度比例
    float progress = current_change / total_change;

    // 限制在 [0, 1.2] 范围内（允许少量超调）
    return Clamp_Float(progress, 0.0f, 1.2f);
}

/*********************************************************************************
 * @name   Advanced_Angle_Control
 * @brief  改进的角度控制算法
 * @param  Status: 路径状态结构体
 * @param  start_angle: 起始角度
 * @param  target_angle: 目标角度
 * @param  current_angle: 当前角度
 * @retval 计算得到的角速度
 *********************************************************************************/
static float Advanced_Angle_Control(Route_STU *Status, float start_angle, float target_angle, float current_angle)
{
    // 计算当前角度误差（带方向）
    Status->Parameter.Current_Angle_Error = Calculate_Angle_Error_With_Direction(target_angle, current_angle);
    Status->Parameter.Abs_Angle_Error = fabsf(Status->Parameter.Current_Angle_Error);

    // 计算旋转进度
    float progress = Calculate_Angle_Progress(start_angle, target_angle, current_angle);

    // 计算总的角度变化量（用于设置区域大小）
    float total_angle_change = fabsf(Calculate_Angle_Error_With_Direction(target_angle, start_angle));
    Status->Parameter.Total_Angle_Change = total_angle_change;

    // 动态计算区域大小（基于总角度变化量）
    float dynamic_stop_area = fminf(Status->Parameter.Angle_Stop_Area, total_angle_change * 0.1f);
    float dynamic_slow_area = fminf(total_angle_change * Status->Parameter.Slow_W_Rate, total_angle_change * 0.5f);
    float dynamic_speedup_area = total_angle_change * Status->Parameter.Speedup_W_Rate;

    // 更新到结构体中（用于调试）
    Status->Parameter.Angle_Distance_Traveled = progress * total_angle_change;

    float target_w_speed = 0.0f;

    // 停止区：使用PID精确控制
    if (Status->Parameter.Abs_Angle_Error <= dynamic_stop_area)
    {
        PID_Calculate(&Line_AnglePID, Status->Parameter.Current_Angle_Error, 0, 1000.0f);
        target_w_speed = Clamp_Float(Line_AnglePID.Output, -1000.0f, 1000.0f);
    }
    // 减速区：根据剩余角度减速
    else if (Status->Parameter.Abs_Angle_Error <= dynamic_slow_area)
    {
        float slow_ratio = Status->Parameter.Abs_Angle_Error / dynamic_slow_area;
        slow_ratio = Clamp_Float(slow_ratio, 0.0f, 1.0f);

        float speed_magnitude = Fitting_Function(slow_ratio) * Status->Parameter.Max_W_Speed;
        target_w_speed = (Status->Parameter.Current_Angle_Error >= 0) ? speed_magnitude : -speed_magnitude;
    }
    // 加速区：根据进度加速
    else if (progress < (dynamic_speedup_area / total_angle_change))
    {
        float speedup_ratio = progress / (dynamic_speedup_area / total_angle_change);
        speedup_ratio = Clamp_Float(speedup_ratio, 0.0f, 1.0f);

        float speed_magnitude = Status->Parameter.Start_W_Speed +
                                Fitting_Function(speedup_ratio) * (Status->Parameter.Max_W_Speed - Status->Parameter.Start_W_Speed);
        target_w_speed = (Status->Parameter.Current_Angle_Error >= 0) ? speed_magnitude : -speed_magnitude;
    }
    // 恒速区：最大速度
    else
    {
        target_w_speed = (Status->Parameter.Current_Angle_Error >= 0) ? Status->Parameter.Max_W_Speed : -Status->Parameter.Max_W_Speed;
    }

    // 错误检测：如果进度超过120%，认为控制有误
    if (progress > 1.2f)
    {
        Status->Flag.W_Error = ENABLE;
        // 使用PID拉回
        PID_Calculate(&Line_AnglePID, Status->Parameter.Current_Angle_Error, 0, 1000.0f);
        target_w_speed = Clamp_Float(Line_AnglePID.Output, -1000.0f, 1000.0f);
    }
    else
    {
        Status->Flag.W_Error = DISABLE;
    }

    // 最终限幅
    return Clamp_Float(target_w_speed, -VW_MAX_LIMIT, VW_MAX_LIMIT);
}

/*********************************************************************************
 * @name 	Line
 * @brief: 直线路径规划，使用的时候不调用它，调用Chassis_Line_Route
 * @param  Status 路径结构体
 * @details 该函数实现基于绝对角度的直线路径规划，包含：
 *          1. 线坐标系转换和路径参数计算
 *          2. 基于绝对角度的角速度控制（加速区/恒速区/减速区/停止区）
 *          3. 线速度控制和法向调节
 *          4. 安全限幅和路径完成判断
 *********************************************************************************/
void Line(Route_STU *Status)
{
    // 局部变量，每次进函数都置为0，为了每次刷新Line_StopPID的值为初始值
    // 名词解释：线坐标系：以起始点为原点，起始点和目标点连线为X轴的坐标系

    /*===================================================================================================================
                                                路径初始化参数计算部分（一次性执行）
    =====================================================================================================================*/
    // 计算初始参数，一段路径只运行一次
    // 算两点连线与X轴的夹角
    Line_Angle = Calculate_Line_Angle(Status->Coordinate_System.Start_Position, Status->Coordinate_System.Target_Position);

    // 计算线坐标系下的目标位置 ，这个Start_Position是线坐标系下的原点
    Status->Coordinate_System.Line_Target_Position = Position_Coordinate_Transformation(&Status->Coordinate_System.Target_Position, &Status->Coordinate_System.Start_Position, Line_Angle);

    // 此时已经换到线坐标系下，计算到目标点的距离就是X坐标
    Status->Parameter.Distance = Status->Coordinate_System.Line_Target_Position.X;

    // 安全地归一化起始角度和目标角度到 [-180, 180] 范围
    Status->Coordinate_System.Start_Position.W = Normalize_Angle(Status->Coordinate_System.Start_Position.W);
    Status->Coordinate_System.Target_Position.W = Normalize_Angle(Status->Coordinate_System.Target_Position.W);

    // 目标角度已经是绝对角度，直接使用（雷达直接回传绝对角度）
    Angle_Aim = Status->Coordinate_System.Target_Position.W;

    // 计算路径加速区和减速区大小 (停止区直接给参数)
    Status->Parameter.Line_Route_Slow_Area = Status->Parameter.Slow_Rate * Status->Parameter.Distance;
    Status->Parameter.Line_Route_Speedup_Area = Status->Parameter.Speedup_Rate * Status->Parameter.Distance;

    // 清pid避免上一段路径的误差积累
    PID_Clear(&Line_AdjustPID);
    PID_Clear(&Line_StopPID);
    PID_Clear(&Line_AnglePID);
    Status->Flag.Work_Start = ENABLE;

    /*===================================================================================================================
                                                路径执行循环部分
    =====================================================================================================================*/
    while (Status->Flag.Work_Start)
    {
        /*===============================================================================================================
                                                坐标系更新部分
        ===============================================================================================================*/
        // 更新世界坐标系下坐标，读雷达回传后处理过的值
        World_Coordinate_System_NowPos.X = Radar_Pos.radar_x;
        World_Coordinate_System_NowPos.Y = Radar_Pos.radar_y;
        World_Coordinate_System_NowPos.W = Normalize_Angle(Radar_Pos.radar_yaw); // 确保在 [-180, 180] 范围内

        // 更新线坐标系下坐标
        Status->Coordinate_System.Line_Now_Position = Position_Coordinate_Transformation(&World_Coordinate_System_NowPos, &Status->Coordinate_System.Start_Position, Line_Angle);

        /*===============================================================================================================
                                                角速度计算部分（使用改进的角度控制算法）
        ===============================================================================================================*/
        Status->Coordinate_System.Line_Target_V.Vw = Advanced_Angle_Control(
            Status,
            Status->Coordinate_System.Start_Position.W,
            Angle_Aim,
            World_Coordinate_System_NowPos.W);

        /*===============================================================================================================
                                                线速度计算部分（保持原有逻辑）
        ===============================================================================================================*/
        // 加速区内线速度计算 - 路径起始段平滑加速
        if ((Status->Coordinate_System.Line_Now_Position.X) < Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Line_Target_V.Vx =
                Status->Parameter.Start_Speed +
                Fitting_Function(Status->Coordinate_System.Line_Now_Position.X / Status->Parameter.Line_Route_Speedup_Area) *
                    (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
        }
        // 在最大速度区 - 路径中间段匀速运行
        else if ((Status->Coordinate_System.Line_Now_Position.X) >= Status->Parameter.Line_Route_Speedup_Area &&
                 (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area))
        {
            Status->Coordinate_System.Line_Target_V.Vx = Status->Parameter.Max_Speed;
        }
        // 减速区内线速度计算 - 接近目标点时平滑减速
        else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Slow_Area) &&
                 (Status->Coordinate_System.Line_Now_Position.X) <= (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
        {
            Status->Coordinate_System.Line_Target_V.Vx = // 期望速度
                Fitting_Function(((Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X) / Status->Parameter.Line_Route_Slow_Area)) *
                    (Status->Parameter.Max_Speed) +
                Status->Parameter.End_Speed;
        }
        // 在停止区用PID拉到指定位置 - 精确位置控制
        else if ((Status->Coordinate_System.Line_Now_Position.X) > (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area))
        {
            // 实现滑动PID控制 - 根据剩余距离动态调整PID参数
            Line_StopPID = Stop_Area_Slide_PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Target_Position.X,
                                                         Status->Coordinate_System.Line_Now_Position.X,
                                                         (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area),
                                                         Status->Parameter.Kp, Status->Parameter.Kd);
            PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Now_Position.X, Status->Coordinate_System.Line_Target_Position.X, 800.0f);
            Status->Coordinate_System.Line_Target_V.Vx = Line_StopPID.Output;
        }

        /*---------------------------------------------------------------------------------------------------------------
                                                法向速度控制（保持直线）
        ---------------------------------------------------------------------------------------------------------------*/
        // 法向保持直线 - 修正路径偏移
        if (ABS(Status->Coordinate_System.Line_Now_Position.X - Status->Coordinate_System.Line_Target_Position.X) > 100.0f) // 快到直线结尾时不调节法向速度，避免出现轮子抖动
        {
            if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 10.0f)
            {
                PID_Calculate(&Line_AdjustPID, Status->Coordinate_System.Line_Now_Position.Y, 0, 1000.0f);

                // 前馈调节，差太多的话给一个大力拽回来
                if (ABS(Status->Coordinate_System.Line_Now_Position.Y - Status->Coordinate_System.Line_Target_Position.Y) > 30.0f)
                {
                    if (Line_AdjustPID.Output > 0)
                        Line_AdjustPID.Output += 3;
                    else if (Line_AdjustPID.Output < 0)
                        Line_AdjustPID.Output -= 3;
                }
                Status->Coordinate_System.Line_Target_V.Vy = Line_AdjustPID.Output;
            }
        }
        else
            Status->Coordinate_System.Line_Target_V.Vy = 0;

        /*===============================================================================================================
                                                坐标系转换部分
        ===============================================================================================================*/
        // 将线坐标系下的速度转换成世界坐标系下速度
        Status->Coordinate_System.World_Coordinate_System_Target_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.Line_Target_V, &Status->Coordinate_System.Zero_Speed, -Line_Angle);

        // 将世界坐标系下速度转换成车身坐标系下速度
        Status->Coordinate_System.Robot_Coordinate_System_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.World_Coordinate_System_Target_V, &Status->Coordinate_System.Zero_Speed, Radar_Pos.radar_yaw);

        // 输出速度到底盘电机
        omni_chassis_calculate(&chassis_r2, Status->Coordinate_System.Robot_Coordinate_System_V.Vx, Status->Coordinate_System.Robot_Coordinate_System_V.Vy, Status->Coordinate_System.Robot_Coordinate_System_V.Vw);

        /*===============================================================================================================
                                                路径完成判断部分
        ===============================================================================================================*/
        // 判断路径是否走完
        // 同时判断位置误差、角度误差和电机转速，确保真正停稳
        if (Status->Coordinate_System.Line_Target_Position.X - Status->Coordinate_System.Line_Now_Position.X < 50.0f &&
            Status->Parameter.Abs_Angle_Error < 3.0f)
        {
            Status->Flag.Work_Start = DISABLE;
            Status->Coordinate_System.Line_Target_V.Vx = 0;
            Status->Coordinate_System.Line_Target_V.Vy = 0;
            Status->Coordinate_System.Line_Target_V.Vw = 0;
            omni_chassis_calculate(&chassis_r2, 0, 0, 0); // 停止底盘
        }

        osDelay(1);
    }
}

/*********************************************************************************
 * @name 	Chassis_Line_Route
 * @brief: 直线路径规划
 * @param  Up_Rate 加速区间比例(输入一个0到1的数)
 * @param  Down_Rate 减速区间比例(输入一个0到1的数)
 * @param  Stop_Area 停止区间大小(正常输入大小)
 *********************************************************************************/
void Chassis_Line_Route(float Target_X, float Target_Y, float Target_W, float Start_Speed, float Max_Speed, float End_Speed, float Start_w, float Max_w, float Up_Rate, float Down_Rate, float Stop_Area, float Kp, float Kd)
{
    // Order_To_Subcontroller.Wheel_Break = 0; // 移除，如果不需要
    Route_Status.Coordinate_System.Start_Position.X = Radar_Pos.radar_x;
    Route_Status.Coordinate_System.Start_Position.Y = Radar_Pos.radar_y;
    Route_Status.Coordinate_System.Start_Position.W = Safe_Angle_Normalization(Radar_Pos.radar_yaw);
    // 这里注释掉了码盘的回传改用雷达的回传
    // Route_Status.Coordinate_System.Start_Position.X = Disk_Encoder.Cod.Chassis_Position_From_Disk.X;
    // Route_Status.Coordinate_System.Start_Position.Y = Disk_Encoder.Cod.Chassis_Position_From_Disk.Y;
    // Route_Status.Coordinate_System.Start_Position.W = Disk_Encoder.Yaw.World_Rotation_Angle;
    Route_Status.Coordinate_System.Target_Position.X = Target_X;
    Route_Status.Coordinate_System.Target_Position.Y = Target_Y;
    Route_Status.Coordinate_System.Target_Position.W = Safe_Angle_Normalization(Target_W); // 直接使用绝对角度，确保在有效范围内

    Route_Status.Parameter.Start_Speed = Start_Speed;
    Route_Status.Parameter.Max_Speed = Max_Speed;
    Route_Status.Parameter.End_Speed = End_Speed;
    Route_Status.Parameter.Start_W_Speed = Start_w;
    Route_Status.Parameter.Max_W_Speed = Max_w;

    Route_Status.Parameter.Speedup_Rate = Up_Rate;
    Route_Status.Parameter.Slow_Rate = Down_Rate;
    Route_Status.Parameter.Speedup_W_Rate = 0.1;
    Route_Status.Parameter.Slow_W_Rate = 0.4;
    Route_Status.Parameter.Line_Route_Stop_Area = Stop_Area;
    Route_Status.Parameter.Angle_Stop_Area = 15.0f;

    Route_Status.Parameter.Kp = Kp;
    Route_Status.Parameter.Kd = Kd;
    Line(&Route_Status);
}

/*********************************************************************************
 * @name 	Circle
 * @brief: 圆弧路径规划，使用的时候不调用它，调用Chassis_Circle_Route
 * @param  Status 路径结构体
 *********************************************************************************/
void Circle(Route_STU *Status)
{
    static float Slope_angle;                            // 极坐标速度坐标系与世界坐标系夹角
    static float Angle_last;                             // 机器人在目标线坐标系下走过的角度、上一时刻的角度
    static float angle_temp_1, angle_temp_2, angle_temp; // 一些计算角度的临时变量
    static float Arc_Remain;                             // 与目标点的距离差值，这个单位是弧长
    static float Arc_Dis;                                // 已经走的弧长
    // 计算初始参数,一段路径只运行一次
    // 更新起始角度
    Status->Coordinate_System.Start_Position.W = Status->Coordinate_System.Target_Position.W;
    Status->Parameter.Distance = ABS(Status->Parameter.Target_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R;
    // 计算减速区大小
    Status->Parameter.Line_Route_Slow_Area = Status->Parameter.Slow_Rate * Status->Parameter.Distance;
    // 计算加速区大小
    Status->Parameter.Line_Route_Speedup_Area = Status->Parameter.Speedup_Rate * Status->Parameter.Distance;
    // PID误差归零
    PID_Clear(&Cir_AdjustPID);
    PID_Clear(&Cir_StopPID);
    PID_Clear(&Cir_AnglePID);
    // 计算路径起始点在极坐标系下的坐标
    Status->Coordinate_System.Polar_NowPos.Rho = Calculate_Line_Distance(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
    Status->Coordinate_System.Polar_NowPos.Theta = Calculate_Line_Angle(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
    // 更新Angle_last
    Angle_last = Status->Coordinate_System.Polar_NowPos.Theta;
    Status->Parameter.Now_Angle_Sum = 0;
    Status->Flag.Work_Start = ENABLE;
    while (Status->Flag.Work_Start)
    {
        // 更新世界坐标系坐标
        World_Coordinate_System_NowPos.X = Radar_Pos.radar_x;
        World_Coordinate_System_NowPos.Y = Radar_Pos.radar_y;
        // 更新极坐标
        Status->Coordinate_System.Polar_NowPos.Rho = Calculate_Line_Distance(Status->Coordinate_System.World_HeartPos, Status->Coordinate_System.Start_Position);
        Status->Coordinate_System.Polar_NowPos.Theta = Calculate_Line_Angle(Status->Coordinate_System.World_HeartPos, World_Coordinate_System_NowPos);
        // 首先计算与目标点的角度差值，计算方法借鉴了M3508无刷电机计算角度的方法
        if (Angle_last < Status->Coordinate_System.Polar_NowPos.Theta) // 一种情况
        {
            angle_temp_1 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last; // 逆时针
            angle_temp_2 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last - 2 * PI;
        }
        else
        {
            angle_temp_1 = Status->Coordinate_System.Polar_NowPos.Theta - Angle_last; // 顺时针
            angle_temp_2 = 2 * PI + Status->Coordinate_System.Polar_NowPos.Theta - Angle_last;
        }
        // 无论顺时针转还是逆时针转，都是取小的那个角度
        angle_temp = (ABS(angle_temp_1)) > (ABS(angle_temp_2)) ? angle_temp_2 : angle_temp_1;
        Status->Parameter.Now_Angle_Sum += angle_temp;
        Angle_last = Status->Coordinate_System.Polar_NowPos.Theta; // 当前所在位置极角变为last
        // 再计算与目标点的距离差值
        Arc_Dis = ABS(Status->Parameter.Now_Angle_Sum) * Status->Parameter.R;
        Arc_Remain = ABS((Status->Parameter.Target_Angle_Sum - Status->Parameter.Now_Angle_Sum) * CHANGE_TO_RADIAN * Status->Parameter.R);
        // 如果在加速区
        if (Arc_Dis < Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = // 期望速度
                Status->Parameter.Start_Speed +
                Fitting_Function(Arc_Dis / Status->Parameter.Line_Route_Speedup_Area) *
                    (Status->Parameter.Max_Speed - Status->Parameter.Start_Speed);
        }
        // 如果没在加速区也没在减速区，切向速度为最大速度
        else if (Arc_Remain >= Status->Parameter.Line_Route_Slow_Area && Arc_Dis >= Status->Parameter.Line_Route_Speedup_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = Status->Parameter.Max_Speed;
        }
        // 如果在减速区内，停止区外
        else if (Arc_Remain >= Status->Parameter.Line_Route_Stop_Area && Arc_Remain < Status->Parameter.Line_Route_Slow_Area)
        {
            Status->Coordinate_System.Polar_Target_V.Vx = // 期望速度
                Fitting_Function(Arc_Remain / Status->Parameter.Line_Route_Slow_Area) *
                (Status->Parameter.Max_Speed);
        }
        // 如果在停止区内，用PID拉到指定位置
        else if (Arc_Remain < Status->Parameter.Line_Route_Stop_Area)
        {
            // 实现滑动PID控制
            Line_StopPID = Stop_Area_Slide_PID_Calculate(&Line_StopPID, Status->Coordinate_System.Line_Target_Position.X,
                                                         Status->Coordinate_System.Line_Now_Position.X,
                                                         (Status->Coordinate_System.Line_Target_Position.X - Status->Parameter.Line_Route_Stop_Area),
                                                         Status->Parameter.Kp, Status->Parameter.Kd);
            PID_Calculate(&Cir_StopPID, Arc_Remain, 0, 2.0f);
            Status->Coordinate_System.Polar_Target_V.Vx = -Cir_StopPID.Output;
        }
        // 顺时针旋转则切向速度取反(还没测试不知道需不需要)
        if (Status->Flag.Circle_Type == Robot_LineType_Cycle_Clockwise)
        {
            Status->Coordinate_System.Polar_Target_V.Vx *= -1;
        }
        // 如果沿切线，时时刻刻改变车头方向
        if (Status->Flag.Robot_Direction)
        {
            Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W + Status->Parameter.Now_Angle_Sum;
        }
        // 如果不沿切线，车头方向不变
        else
        {
            Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W;
        }
        // PID保持车身角度
        PID_Calculate(&Cir_AnglePID, Radar_Pos.radar_yaw, Status->Coordinate_System.Target_Position.W, 3000.0f);
        //////Robot_Coordinate_System_V.Vw = Cir_AnglePID.Output;
        // 法向用PID保持在圆上
        PID_Calculate(&Cir_AdjustPID, Status->Coordinate_System.Polar_NowPos.Rho, Status->Parameter.R, 3000.0f);
        Status->Coordinate_System.Polar_Target_V.Vy = -Cir_AdjustPID.Output;
        // 当极坐标速度坐标系法向方向偏差过大时可以尝试去减小切向方向速度(待测试)
        // 计算极坐标速度坐标系与世界坐标系夹角，原先是以y轴为基准，现在以x轴为基准
        Slope_angle = Status->Coordinate_System.Polar_NowPos.Theta - 90.0f;
        // 将极坐标速度坐标系的速度转换成世界坐标系下速度
        World_Coordinate_System_Target_V = Speed_Coordinate_Transformation(&Status->Coordinate_System.Polar_Target_V, &Status->Coordinate_System.Zero_Speed, -Slope_angle);
        // 将世界坐标系下速度转换成车身坐标系下速度
        Robot_Coordinate_System_V = Speed_Coordinate_Transformation(&World_Coordinate_System_Target_V, &Status->Coordinate_System.Zero_Speed, Radar_Pos.radar_yaw);
        // 判断路径是否走完
        if (Arc_Remain < 5)
        {
            if (Status->Flag.Robot_Direction)
                Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W + Status->Parameter.Target_Angle_Sum;
            else
                Status->Coordinate_System.Target_Position.W = Status->Coordinate_System.Start_Position.W;
            Status->Flag.Work_Start = DISABLE;
        }
        // Robot_Wheel_Control(Robot_Coordinate_System_V.Vx, Robot_Coordinate_System_V.Vy, Robot_Coordinate_System_V.Vw);
        osDelay(2);
    }
}

/*********************************************************************************
 * @name 	Chassis_Circle_Route
 * @brief: 圆弧路径规划
 * @param  Target_X 目标X坐标
 * @param  Target_Y 目标Y坐标
 * @param  Angle 圆心角(顺时针为正，逆时针为负)(单位：度)
 * @param  Start_V 起始速度
 * @param  End_V 结束速度
 * @param  V_Max 最大速度
 * @param  Yanqie 是否沿切线(enable为沿切线,disable为方向不变)
 * @param  Up_Rate 加速区间比例(输入一个0到1的数)
 * @param  Down_Rate 减速区间比例(输入一个0到1的数)
 *********************************************************************************/
void Chassis_Circle_Route(float Target_X, float Target_Y, float Angle, int Start_V, int End_V, int V_Max, FunctionalState Yanqie, float Up_Rate, float Down_Rate, float Kp, float Kd)
{
    float Gamma, R, Epsilon, D, C_X, C_Y;
    float Rad = Angle * CHANGE_TO_RADIAN;
    // 这个D是当前点到目标点的距离
    D = sqrt((Target_X - Radar_Pos.radar_x) * (Target_X - Radar_Pos.radar_x) +
             (Target_Y - Radar_Pos.radar_y) * (Target_Y - Radar_Pos.radar_y));
    // 这个R是圆的半径
    R = D / 2.0f / sinf(Rad / 2.0f);
    // 计算圆心坐标
    Gamma = acosf((Target_X - Radar_Pos.radar_x) / D);
    if ((Target_Y - Radar_Pos.radar_y) < 0)
    {
        Gamma = 2 * PI - Gamma;
    }
    if (Angle >= 0) // 顺时针
    {
        Epsilon = Gamma - (PI / 2 - Rad / 2);
        Route_Status.Flag.Circle_Type = Robot_LineType_Cycle_Clockwise; // 顺时针
    }
    else // 逆时针
    {
        Epsilon = Gamma + (PI / 2 - Rad / 2);
        Route_Status.Flag.Circle_Type = Robot_LineType_Cycle_Anticlockwise; // 逆时针
    }
    // 这个C_X和C_Y是圆心坐标
    C_X = Radar_Pos.radar_x + R * cos(Epsilon);
    C_Y = Radar_Pos.radar_y + R * sin(Epsilon);
    // 起始位置替换为上一段路径的目标角度
    Route_Status.Coordinate_System.Start_Position.X = Route_Status.Coordinate_System.Target_Position.X;
    Route_Status.Coordinate_System.Start_Position.Y = Route_Status.Coordinate_System.Target_Position.Y;
    Route_Status.Coordinate_System.Target_Position.X = Target_X; // 目标位置
    Route_Status.Coordinate_System.Target_Position.Y = Target_Y;

    Route_Status.Parameter.Start_Speed = Start_V; // 开始速度
    Route_Status.Parameter.Max_Speed = V_Max;     // 最大速度
    Route_Status.Parameter.End_Speed = End_V;     // 路径末端速度

    Route_Status.Parameter.Speedup_Rate = Up_Rate;         // 加速区间比例
    Route_Status.Parameter.Slow_Rate = Down_Rate;          // 减速区间比例
    Route_Status.Parameter.Line_Route_Stop_Area = 20;      // 停止区长度
    Route_Status.Flag.Robot_Direction = Yanqie;            // 是否沿切线
    Route_Status.Parameter.Target_Angle_Sum = Angle;       // 圆心角
    Route_Status.Parameter.R = R;                          // 半径
    Route_Status.Coordinate_System.World_HeartPos.X = C_X; // 圆心
    Route_Status.Coordinate_System.World_HeartPos.Y = C_Y;

    Route_Status.Parameter.Kp = Kp;
    Route_Status.Parameter.Kd = Kd;
    Circle(&Route_Status);
}

void Route_Init(void)
{
    PID_Set(&Line_AdjustPID, 8, 0, 10);
    PID_Set(&Line_StopPID, 10, 0, 13);
    PID_Set(&Line_AnglePID, -80, 0, 30);
    PID_Set(&Cir_AdjustPID, 0.5f, 0.0f, 0.0f);
    PID_Set(&Cir_StopPID, 0.5f, 0.0f, 0.0f);
    PID_Set(&Cir_AnglePID, 0.5f, 0.0f, 0.0f);
    PID_Set(&Keep_X_PID, 0.5f, 0.0f, 0.0f);
    PID_Set(&Keep_Y_PID, 0.5f, 0.0f, 0.0f);
    PID_Set(&Keep_W_PID, 0.5f, 0.0f, 0.0f);
}

/*===================================================================================================================
                                              运球预选赛相关
=====================================================================================================================*/
// 定义所有路径点常量
Coordinate_Position_Struct Last_Point;
uint8_t Last_Route = 0;
const RoutePoint PATH_POINTS[] = {
    // Route_Type_0: 起始点到S&E点，长路径
    {2458, -2204, 0, 1500, 10000, 1700, 50, 2000, 0.01f, 0.6f, 100, true, 25, 10},
    // Route_Type_1: S&E点到第二个点，短路径
    {1730, -2976, 0, 1500, 4000, 500, 50, 2000, 0.01f, 0.4f, 200, true, 10, 15},
    // Route_Type_2: 第二个点到第三个点，中路径
    {1741, -4705, 0, 1500, 5000, 500, 50, 2000, 0.01f, 0.4f, 100, true, 10, 13},
    // Route_Type_3: 第三个点到第四个点，中路径
    {3765, -4666, 0, 1500, 6000, 800, 50, 2000, 0.01f, 0.5f, 200, true, 10, 15},
    // Route_Type_4: 第四个点到第五个点，中路径
    {5777, -4648, 0, 1500, 6000, 800, 50, 2000, 0.01f, 0.5f, 200, true, 10, 15},
    // Route_Type_5: 第五个点到第六个点，中路径
    {5745, -2920, 0, 1500, 5000, 500, 50, 2000, 0.01f, 0.4f, 100, true, 10, 13},
    // Route_Type_6: 第六个点到第七个点，短路径
    {4972, -2175, 0, 1500, 4000, 1000, 50, 2000, 0.01f, 0.4f, 200, true, 8, 15},
    // Route_Type_7: 第七个点到S&E点左侧，不运球
    {1000, -2175, 0, 1500, 12000, 1000, 50, 2000, 0.01f, 0.7f, 100, false, 25, 15},
    // Route_Type_8: S&E点左侧到出发点，不运球
    {200, -200, 0, 1500, 5000, 1500, 50, 2000, 0.01f, 0.4f, 100, false, 15, 15}};

/*********************************************************************************
 * @name 	Variable_Element_Route
 * @brief   根据目标点与当前位置的距离和其他条件，动态计算并设置路径参数（如速度、加速度等），实现更平滑和高效的导航。
 * @param   Target_Point 目标坐标点
 *********************************************************************************/
void Variable_Element_Route(Coordinate_Position_Struct *Target_Point)
{
    float Line_Distance = sqrtf(pow(Radar_Pos.radar_x - Target_Point->X, 2) +
                                pow(Radar_Pos.radar_y - Target_Point->Y, 2));
    Parameter_Of_Route_With_Automatic_Aiming Route_Params;
    // 根据距离调整速度参数
    if (Line_Distance > 3000)
    {
        Route_Params.Max_Speed = 10000;
        Route_Params.End_Speed = 1700;
        Route_Params.Down_Rate = 0.6;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 25;
        Route_Params.Kd = 10;
    }
    else if (Line_Distance > 2000)
    {
        Route_Params.Max_Speed = 6000;
        Route_Params.End_Speed = 800;
        Route_Params.Down_Rate = 0.5;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 200;
        Route_Params.Kp = 10;
        Route_Params.Kd = 15;
    }
    else if (Line_Distance > 1000)
    {
        Route_Params.Max_Speed = 5000;
        Route_Params.End_Speed = 500;
        Route_Params.Down_Rate = 0.4;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 10;
        Route_Params.Kd = 13;
    }
    else
    {
        Route_Params.Max_Speed = 4000;
        Route_Params.End_Speed = 500;
        Route_Params.Down_Rate = 0.4;
        Route_Params.Max_w = 800;
        Route_Params.Up_Rate = 0.01;
        Route_Params.Stop_Area = 100;
        Route_Params.Kp = 10;
        Route_Params.Kd = 15;
    }
    // 执行路径规划
    Chassis_Line_Route(Target_Point->X, Target_Point->Y, Target_Point->W,
                       1200, Route_Params.Max_Speed, Route_Params.End_Speed,
                       50, Route_Params.Max_w, Route_Params.Up_Rate,
                       Route_Params.Down_Rate, Route_Params.Stop_Area,
                       Route_Params.Kp, Route_Params.Kd);
}


/*===================================================================================================================
                                        多段自动路径执行相关
=====================================================================================================================*/

// 多段路径全局标志位
uint8_t g_multi_route_segment = 0;     // 当前路径段数 (0-2)
uint8_t g_multi_route_is_running = 0;  // 路径是否正在执行 (1-执行中, 0-空闲)

// 定义3段路径的参数结构
typedef struct
{
    float target_x;      // 目标X坐标
    float target_y;      // 目标Y坐标
    float target_w;      // 目标角度
    float start_speed;   // 起始速度
    float max_speed;     // 最大速度
    float end_speed;     // 结束速度
    float start_w;       // 起始角速度
    float max_w;         // 最大角速度
    float up_rate;       // 加速区比例
    float down_rate;     // 减速区比例
    float stop_area;     // 停止区大小
    float kp;            // Kp参数
    float kd;            // Kd参数
} Multi_Route_Params;

// 定义3段路径参数
const Multi_Route_Params g_route_segments[4] = {
    // 第一段路径：从起点到中间点
    {
        .target_x = 810.0f,    // 目标X坐标 (mm)
        .target_y = 930.0f,    // 目标Y坐标
        .target_w = 0.0f,       // 目标角度
        .start_speed = 600.0f,    // 起始速度
        .max_speed = 1500.0f,   // 最大速度
        .end_speed = 0.0f,    // 结束速度
        .start_w = 100.0f,        // 起始角速度
        .max_w = 800.0f,          // 最大角速度
        .up_rate = 0.1f,        // 加速区比例
        .down_rate = 0.4f,      // 减速区比例
        .stop_area = 100.0f,    // 停止区大小
        .kp = 7.5f,            // Kp参数
        .kd = 15.0f              // Kd参数
    },
    // 第二段路径：从中间点到第二个目标
    {
        .target_x = 690.0f,    // 目标X坐标
        .target_y = 50.0f,    // 目标Y坐标
        .target_w = 0.0f,      // 目标角度
        .start_speed = 600.0f,
        .max_speed = 1500.0f,
        .end_speed = 00.0f,
        .start_w = 100.0f,
        .max_w = 800.0f,
        .up_rate = 0.1f,
        .down_rate = 0.4f,
        .stop_area = 100.0f,
        .kp = 7.5f,
        .kd = 15.0f
    },
    // 第三段路径：
    {
        .target_x = 1100.0f,       // 返回起点X坐标
        .target_y = 815.0f,       // 返回起点Y坐标
        .target_w = 175.0f,       // 返回起始角度
        .start_speed = 600.0f,
        .max_speed = 1500.0f,
        .end_speed = 0.0f,
        .start_w =600.0f,
        .max_w = 1000.0f,
        .up_rate = 0.1f,
        .down_rate = 0.4f,
        .stop_area = 100.0f,
        .kp = 7.5f,
        .kd = 15.0f
    },
    // 第四段路径：
    {
        .target_x = 2610.0f,       // 返回起点X坐标
        .target_y = 2510.0f,       // 返回起点Y坐标
        .target_w = 0.0f,       // 返回起始角度
        .start_speed = 600.0f,
        .max_speed = 2000.0f,
        .end_speed = 0.0f,
        .start_w =600.0f,
        .max_w = 1000.0f,
        .up_rate = 0.1f,
        .down_rate = 0.4f,
        .stop_area = 100.0f,
        .kp = 7.5f,
        .kd = 15.0f
    }		
};

/*********************************************************************************
 * @name   Chassis_Multi_Route_Segments
 * @brief  自动执行多段直线路径，一段完成后自动跑下一段
 * @param  None
 * @retval None
 * @details 使用全局标志位g_multi_route_segment来追踪当前段数
 *          调用此函数会自动检查当前路径是否完成，完成后自动+1切换到下一段
 *          当完成第3段后，标志位会重置为0
 *********************************************************************************/
void Chassis_Multi_Route_Segments(void)
{
    // 检查当前段数是否有效
    if (g_multi_route_segment >= 4)
    {
        g_multi_route_segment = 0;      // 重置为第0段
        g_multi_route_is_running = 0;   // 停止运行标志
        return;
    }

    // 获取当前段的路径参数
    const Multi_Route_Params *current_route = &g_route_segments[g_multi_route_segment];

    // 执行当前段的路径
    Chassis_Line_Route(
        current_route->target_x,
        current_route->target_y,
        current_route->target_w,
        current_route->start_speed,
        current_route->max_speed,
        current_route->end_speed,
        current_route->start_w,
        current_route->max_w,
        current_route->up_rate,
        current_route->down_rate,
        current_route->stop_area,
        current_route->kp,
        current_route->kd
    );

    // 路径执行完成，移到下一段
    g_multi_route_segment++;
}

/*********************************************************************************
 * @name   Chassis_Multi_Route_Start
 * @brief  启动多段路径执行，从第0段开始
 * @param  None
 * @retval None
 *********************************************************************************/
void Chassis_Multi_Route_Start(void)
{
    g_multi_route_segment = 0;
    g_multi_route_is_running = 1;
}

/*********************************************************************************
 * @name   Chassis_Multi_Route_Stop
 * @brief  停止多段路径执行
 * @param  None
 * @retval None
 *********************************************************************************/
void Chassis_Multi_Route_Stop(void)
{
    g_multi_route_segment = 0;
    g_multi_route_is_running = 0;
    omni_chassis_calculate(&chassis_r2, 0, 0, 0); // 停止底盘
}

/*********************************************************************************
 * @name   Chassis_Multi_Route_Get_Segment
 * @brief  获取当前路径段数
 * @param  None
 * @retval 当前段数 (0-2，3表示已完成)
 *********************************************************************************/
uint8_t Chassis_Multi_Route_Get_Segment(void)
{
    return g_multi_route_segment;
}

/*********************************************************************************
 * @name   Chassis_Multi_Route_Is_Running
 * @brief  查询多段路径是否正在运行
 * @param  None
 * @retval 1-运行中, 0-空闲或已完成
 *********************************************************************************/
uint8_t Chassis_Multi_Route_Is_Running(void)
{
    return g_multi_route_is_running;
}

/*********************************************************************************
 * @name   Chassis_Multi_Route_Update_Params
 * @brief  在运行时更新指定段的路径参数
 * @param  segment: 段数 (0-2)
 * @param  params: 新的路径参数
 * @retval None
 *********************************************************************************/
void Chassis_Multi_Route_Update_Params(uint8_t segment, const Multi_Route_Params *params)
{
    if (segment < 3 && params != NULL)
    {
        // 注意：这里不能直接修改const的数组，如需修改参数，请定义可修改的副本
        // 可选方案：定义全局的可修改参数数组副本
    }
}

