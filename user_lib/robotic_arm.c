#include "robotic_arm.h"

robotic_arm_stu arm;    //机械臂实例

/**
 * @brief 机械臂参数初始化函数，在机械臂任务开始时调用，进行机械臂固定参数的初始化
*/
void arm_joint_parameter_init(robotic_arm_stu *arm)
{
    arm->joint_parameters[0].alpha = 0.0f;
    arm->joint_parameters[0].a = 0.0f;
    arm->joint_parameters[0].d = 0.15f;

    arm->joint_parameters[1].alpha = PI/2.0f;
    arm->joint_parameters[1].a = 0.0f;
    arm->joint_parameters[1].d = 0.0f;

    arm->joint_parameters[2].alpha = 0.0f;
    arm->joint_parameters[2].a = 0.55f;
    arm->joint_parameters[2].d = 0.0f;

    arm->joint_parameters[3].alpha = 0.0f;
    arm->joint_parameters[3].a = 0.45f;
    arm->joint_parameters[3].d = 0.0f;

    arm->joint_parameters[4].alpha = 0.0f;
    arm->joint_parameters[4].a = 0.05f;
    arm->joint_parameters[4].d = 0.0f;

}

/**
 * @brief 机械臂关节状态更新函数，在机械臂任务中调用，更新机械臂的关节状态
*/
void arm_joint_state_update(robotic_arm_stu *arm)
{
//云台朝向角    
    arm->joint_parameters[0].theta = A1_debug_485_2[0].measure.Pos;        
//根据各关节处电机的测量位置更新机械臂的关节角状态，对二连杆末端位置进行正解算
    arm->joint_parameters[1].theta = -A1_debug_485_2[1].measure.Pos + PI;  //大臂与云台平面的夹角
    arm->joint_parameters[2].theta = m3508_fdcan1[0].Angle_Sum/100000.0f*PI/2.0f + PI/12.0f;
    arm->joint_pitch[0] = arm->joint_parameters[1].theta;
    arm->joint_pitch[1] = -PI + arm->joint_pitch[0] + arm->joint_parameters[2].theta;

    arm->x0_now = 0.55f*arm_cos_f32(arm->joint_pitch[0]) + 0.45f*arm_cos_f32(arm->joint_pitch[1]);
    arm->y0_now = 0.55f*arm_sin_f32(arm->joint_pitch[0]) + 0.45f*arm_sin_f32(arm->joint_pitch[1]);

//末端机构解算
    arm->joint_parameters[3].theta = PI*7.0f/12.0f + (float)(m3508_fdcan1[1].Angle_Sum - m3508_fdcan1[2].Angle_Sum)/2.0f/1272.0f/180.0f*PI;
    //joint_parameters[4].theta实际上是前部吸盘自转角度，对着吸盘逆时针为正
    arm->joint_parameters[4].theta = -1.0f*(float)(m3508_fdcan1[1].Angle_Sum + m3508_fdcan1[2].Angle_Sum)/2.0f/1222.0f/180.0f*PI;
    
    arm->joint_pitch[2] = arm->joint_pitch[1] + arm->joint_parameters[3].theta;
}

/**
 * @brief 机械臂复位函数，在机械臂任务开始时调用，进行机械臂的复位操作，并对各电机的参数也同时设置
 * @warning 在使用之前要手动将A1结构体中的Change_Start_Position置1，以便进行初始位置的更新
*/
void arm_joint_state_init(robotic_arm_stu *arm)
{
    // arm_joint_state_update(arm);
    // if(A1_debug_485_2[1].target.Change_Start_Position == 1)
    // {  
    //     A1_debug_485_2[1].target.Change_Start_Position = 0;
    //     A1_debug_485_2[1].target.Start_Position = A1_debug_485_2[1].measure.Pos;
    // }
    // A1_Motor_Position_Improved_Control(&A1_debug_485_2[1], 3.0f, (PI-A1_debug_485_2[1].measure.Pos), A1_debug_485_2[1].target.Start_Position, PI/2.0f, 3.0f);

    A1_Motor_Position_Control(&A1_debug_485_2[1], PI/2.0f, 20.0f, 0.015f);
    m3508_parameter_init(&m3508_fdcan1[0], 2.0f, 10.0f, 0.5f, 2.0f);
    m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)(0.0f/(PI/2.0f)*100000.0f), 8192);
}

/**
 * @brief 用于检测目标路径点是否可到达，推荐在目标点解算前以及完毕后调用
*/
IK_Reach_Status_enum arm_route_reachable_check(robotic_arm_stu *arm)
{
    if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) > 0.95f || sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) < 0.2f) 
    {
        return IK_OUT_OF_RANGE;//末端位置超出机械臂工作范围
    }
    else if(arm->theta_target[0]<0.0f || arm->theta_target[0]>PI || arm->theta_target[1]<PI/12.0f || arm->theta_target[1]>PI)
    {
        return IK_OUT_OF_RANGE;//关节角超出物理限位
    }
    else
    {
        return IK_REACHABLE;//末端位置在机械臂工作范围内
    }
}

/**
 * @brief 机械臂路径点计算函数，输入目标末端位置，输出目标关节角
 * @warning 目前仅支持二连杆末端位置的计算
*/
void arm_route_point_calculate(robotic_arm_stu *arm)
{
    float x = arm->x0_target;
    float y = arm->y0_target;
    float l1 = arm->joint_parameters[2].a;
    float l2 = arm->joint_parameters[3].a;

    float l0 = sqrtf(x*x + y*y);

    // 确保 l0 和 l1 不为 0，防止除以 0
    if(l0 > 0.2f)
    {
        float temp_cos_angle0, temp_cos_angle1;
        temp_cos_angle0 = (l1*l1 + l0*l0 - l2*l2)/(2.0f*l1*l0);
        temp_cos_angle1 = (l1*l1 + l2*l2 - l0*l0)/(2.0f*l1*l2);
        if(temp_cos_angle0 > 1.0f) temp_cos_angle0 = 1.0f;
        else if(temp_cos_angle0 < -1.0f) temp_cos_angle0 = -1.0f;
        if(temp_cos_angle1 > 1.0f) temp_cos_angle1 = 1.0f;
        else if(temp_cos_angle1 < -1.0f) temp_cos_angle1 = -1.0f;

        arm->theta_target[0] = atan2f(y, x) + acosf(temp_cos_angle0);
        arm->theta_target[1] = acosf(temp_cos_angle1);
    }
}

/**
 * @brief 利用arm_route_point_calculate计算的关节夹角来执行各关节电机的位置环控制，输入关节夹角(rad)，经过内置单位转换后输出到电机
 * @warning A1位置环参数需要根据测试进行调整
*/
void arm_route_execute(robotic_arm_stu *arm)
{
    A1_Motor_Position_Control(&A1_debug_485_2[1], PI - arm->theta_target[0], 20.0f, 0.025f);
    m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)((arm->theta_target[1]-PI/12.0f)/(PI/2.0f)*100000.0f), 8192);
}

/**
 * @brief 机械臂末端执行器稳定控制函数，输入末端执行器的目标pitch与自转角来控制末端执行器的姿态稳定，输入单位为rad
 * @param target_pitch 末端执行器的目标pitch角，平行地面为0，抬起为正
 * @param theta4 末端执行器的自转角，面对吸盘逆时针为正
*/
void arm_actuator_stable_control(robotic_arm_stu *arm, float target_pitch, float theta4)
{
// 计算为了让 pitch[2] 达到 PI/2 所需的 joint_parameters[3].theta (即 theta3)
      float target_theta3 = target_pitch - arm->joint_pitch[1];
      // 差速控制电机 1 和 2 达到该 theta3 
      target_theta3 = limit_float(target_theta3, -PI*7.0f/12.0f, PI*7.0f/12.0f);
      float target_delta_m = (target_theta3 - PI*7.0f/12.0f) / PI * 180.0f * 1272.0f * 2.0f;
      // 结合 debug_theta4 (翻转/自转角) 给出绝对目标
      // 注意：不要在目标值里减去当前的 arm.joint_pitch[2]
      m3508_anglesum_control(&m3508_fdcan1[1], (target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * 1222.0f), 8192);
      m3508_anglesum_control(&m3508_fdcan1[2], (-target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * 1222.0f), 8192);
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
    arm->joint_torque_gc[0] = ((1.006f-0.254f*arm_cos_f32(arm->joint_parameters[2].theta))*(-arm_cos_f32(arm->joint_pitch[0])))*9.1f;

}


