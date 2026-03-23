#include "robotic_arm.h"

robotic_arm_stu arm;    //机械臂实例

/**
 * @brief 机械臂初始化函数，在机械臂任务开始时调用，进行机械臂固定参数的初始化
*/
void arm_joint_state_init(robotic_arm_stu *arm)
{
    arm->dh_parameters[0].alpha = 0.0f;
    arm->dh_parameters[0].a = 0.0f;
    arm->dh_parameters[0].d = 0.15f;

    arm->dh_parameters[1].alpha = PI/2.0f;
    arm->dh_parameters[1].a = 0.0f;
    arm->dh_parameters[1].d = 0.0f;

    arm->dh_parameters[2].alpha = 0.0f;
    arm->dh_parameters[2].a = 0.55f;
    arm->dh_parameters[2].d = 0.0f;

    arm->dh_parameters[3].alpha = 0.0f;
    arm->dh_parameters[3].a = 0.45f;
    arm->dh_parameters[3].d = 0.0f;
}

/**
 * @brief 机械臂关节状态更新函数，在机械臂任务中调用，更新机械臂的关节状态
*/
void arm_joint_state_update(robotic_arm_stu *arm)
{
    arm->dh_parameters[0].theta = A1_debug_485_2[0].measure.Pos;        //云台朝向角
    arm->dh_parameters[1].theta = -A1_debug_485_2[1].measure.Pos + PI;  //大臂与云台平面的夹角
    arm->dh_parameters[2].theta = m3508_fdcan1[0].Angle_Sum/100000.0f*PI/2.0f + PI/12.0f;
    arm->joint_pitch[0] = arm->dh_parameters[1].theta;
    arm->joint_pitch[1] = -PI + arm->joint_pitch[0] + arm->dh_parameters[2].theta;

//TODO末端机构解算
    // arm->dh_parameters[3].theta = m3508_fdcan1[1].Angle_Sum;
    // arm->dh_parameters[3].theta = m3508_fdcan1[2].Angle_Sum;
}

/**
 * @brief 通过预设力矩值计算单一连杆所需的重力补偿力矩
 * @param arm 机械臂结构体指针
 * @param torque_preset 预设的重力补偿力矩值，需要根据测试进行调整，单位不定
 * @return 返回重力补偿力矩值，单位Nm
 * @warning 预设力矩值的大小需要实验得出，不同的电机其力矩描述也不同，所以单位不定，但是可以通过手册数据推算国际单位下的力矩值
*/
float arm_gravity_compensation_calculate(float pitch, float torque_preset)
{
    return torque_preset * arm_cos_f32(pitch);
}


