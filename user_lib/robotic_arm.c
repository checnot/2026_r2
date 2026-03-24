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
 * @param pitch 连杆相对于地面的绝对角度，单位rad
 * @param torque_preset 预设的重力补偿力矩值，需要根据测试进行调整，单位不定
 * @return 返回重力补偿力矩值，单位Nm
 * @warning 预设力矩值的大小需要实验得出，不同的电机其力矩描述也不同，所以单位不定，但是可以通过手册数据推算国际单位下的力矩值
*/
float gravity_comp_calc(float pitch, float torque_preset)
{
    return torque_preset * arm_cos_f32(pitch);
}

/**
 * @brief 机械臂整体的重力补偿函数，在机械臂任务中调用，用robotic_arm_stu中的jointpitch计算的joint_torque_gc
 * @param torque_preset0 预设的肩部重力补偿力矩值，对应A1电机输出轴力矩
 * @param torque_preset1 预设的手肘重力补偿力矩值，对应m3508电机输出轴力矩
 * @warning 仅用于目前的机械臂设计，且预设力矩值需要根据测试进行调整，但保留了可拓展性
*/
void arm_gravity_comp(robotic_arm_stu *arm, float torque_preset0, float torque_preset1)
{
    //手肘处电机输出轴力矩(Nm)
    arm->joint_torque_gc[1] = gravity_comp_calc(arm->joint_pitch[1], torque_preset1)/8192.0f*10.0f*0.3f;
    //肩部处电机输出轴力矩(Nm)，由于肩部需要带动手肘，所以肩部的重力补偿不仅要补偿肩部自身的重力，还要补偿手肘的重力（通过手肘处的电机力矩反推）
    arm->joint_torque_gc[0] = ((1.006f-0.254f*arm_cos_f32(arm->dh_parameters[2].theta))*(-arm_cos_f32(arm->joint_pitch[0])))*9.1f;

}
