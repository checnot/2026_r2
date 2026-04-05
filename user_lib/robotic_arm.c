#include "robotic_arm.h"

robotic_arm_stu arm;    //机械臂实例

robotic_arm_path_point_stu arm_path_debug; //测试用机械臂路径点数组

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
    float rate = 2666.0f;

//云台朝向角    
    arm->joint_parameters[0].theta = A1_debug_485_2[0].measure.Pos;        
//根据各关节处电机的测量位置更新机械臂的关节角状态，对二连杆末端位置进行正解算
    arm->joint_parameters[1].theta = -A1_debug_485_2[1].measure.Pos + PI *11.0f/12.0f;  //大臂与云台平面的夹角
    arm->joint_parameters[2].theta = m3508_fdcan1[0].Angle_Sum/100000.0f*PI/2.0f + PI/12.0f;
    arm->joint_pitch[0] = arm->joint_parameters[1].theta;
    arm->joint_pitch[1] = -PI + arm->joint_pitch[0] + arm->joint_parameters[2].theta;

    arm->x0_now = 0.55f*arm_cos_f32(arm->joint_pitch[0]) + 0.45f*arm_cos_f32(arm->joint_pitch[1]);
    arm->y0_now = 0.55f*arm_sin_f32(arm->joint_pitch[0]) + 0.45f*arm_sin_f32(arm->joint_pitch[1]);

//末端机构解算
    arm->joint_parameters[3].theta = PI*7.0f/12.0f + (float)(m3508_fdcan1[1].Angle_Sum - m3508_fdcan1[2].Angle_Sum)/2.0f/rate/180.0f*PI;
    //joint_parameters[4].theta实际上是前部吸盘自转角度，对着吸盘逆时针为正
    arm->joint_parameters[4].theta = -1.0f*(float)(m3508_fdcan1[1].Angle_Sum + m3508_fdcan1[2].Angle_Sum)/2.0f/rate/180.0f*PI;
    
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

    A1_Motor_Position_Control(&A1_debug_485_2[1], PI*5.0f/12.0f, 20.0f, 0.015f);
    m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)(0.0f/(PI/2.0f)*100000.0f), 8192);
}

/**
 * @brief 用于检测目标路径点是否可到达，推荐在目标点解算前以及完毕后调用
*/
IK_Reach_Status_enum arm_route_reachable_check(robotic_arm_stu *arm)
{
    if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) > 0.95f || sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) < 0.15f) 
    {
        return IK_LENGTH_OUT_OF_RANGE;//末端位置超出机械臂工作范围
    }
    else if(arm->theta_target[1]<0.0f || arm->theta_target[1]>PI || arm->theta_target[2]<PI/12.0f || arm->theta_target[2]>PI)
    {
        return IK_ANGLE_OUT_OF_RANGE;//关节角超出物理限位
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
    if(l0 > 0.15f)
    {
        float temp_cos_angle0, temp_cos_angle1;
        temp_cos_angle0 = (l1*l1 + l0*l0 - l2*l2)/(2.0f*l1*l0);
        temp_cos_angle1 = (l1*l1 + l2*l2 - l0*l0)/(2.0f*l1*l2);
        if(temp_cos_angle0 > 1.0f) temp_cos_angle0 = 1.0f;
        else if(temp_cos_angle0 < -1.0f) temp_cos_angle0 = -1.0f;
        if(temp_cos_angle1 > 1.0f) temp_cos_angle1 = 1.0f;
        else if(temp_cos_angle1 < -1.0f) temp_cos_angle1 = -1.0f;

        arm->theta_target[1] = atan2f(y, x) + acosf(temp_cos_angle0);
        arm->theta_target[2] = acosf(temp_cos_angle1);
    }
}

/**
 * @brief 将关节夹角控制至指定位置，利用arm_route_point_calculate计算的关节夹角来执行各关节电机的位置环控制，输入关节夹角(rad)，经过内置单位转换后输出到电机
 * @warning A1位置环参数需要根据测试进行调整
*/
void arm_route_execute(robotic_arm_stu *arm, float a1_kw, float a1_kp)
{
    A1_Motor_Position_Control(&A1_debug_485_2[0], arm->theta_target[0], a1_kw/3.0f, a1_kp/3.0f);                    //云台(由于云台旋转负载小，所以参数调小一些)
    A1_Motor_Position_Control(&A1_debug_485_2[1], PI*11.0f/12.0f - arm->theta_target[1], a1_kw, a1_kp);             //大臂
    m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)((arm->theta_target[2]-PI/12.0f)/(PI/2.0f)*100000.0f), 12000);//小臂
}

/**
 * @brief 机械臂路径点修复，当输入超出结构物理限制的路径点时，进行修复以达到可行的路径点；在内部将超出限额的函数重新赋值，采用的方法是直接将
 * @warning 暂时未使用，待后续测试
*/
void arm_route_repair(robotic_arm_stu *arm)
{
// 1. 长度修复
    if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) > 0.95f) 
    {
        float angle = atan2f(arm->y0_target, arm->x0_target);
        arm->x0_target = 0.95f * arm_cos_f32(angle);
        arm->y0_target = 0.95f * arm_sin_f32(angle);
    }
    else if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) < 0.15f)
    {
        float angle = atan2f(arm->y0_target, arm->x0_target);
        arm->x0_target = 0.15f * arm_cos_f32(angle);
        arm->y0_target = 0.15f * arm_sin_f32(angle);
    }

    arm_route_point_calculate(arm); // 位置修复后重新计算关节角，以便进行后续的角度修复

    uint8_t angle_repaired = 0; // 标记角度是否被修复过
// 2. 关节角度软限位约束 (大臂)
    if(arm->theta_target[1] < 0.0f)
    {
        arm->theta_target[1] = 0.0f;
        angle_repaired = 1;
    }
    else if(arm->theta_target[1] > PI*11.0f/12.0f)
    {
        arm->theta_target[1] = PI*11.0f/12.0f;
        angle_repaired = 1;
    }
    
// 3. 关节角度软限位约束 (小臂，基于你 arm_route_reachable_check 里面的约束条件)
    if(arm->theta_target[2] < PI/12.0f)
    {
        arm->theta_target[2] = PI/12.0f;
        angle_repaired = 1;
    }
    else if(arm->theta_target[2] > PI)
    {
        arm->theta_target[2] = PI;
        angle_repaired = 1;
    }

// 正运动学坐标回算：如果角度被强行切断改变了，就把 target_x/y 更新到切断后的实际位置
// 算法依据: arm_joint_state_update() 内部的坐标变换公式
    if(angle_repaired)
    {
        float target_pitch_0 = arm->theta_target[1];
        float target_pitch_1 = -PI + target_pitch_0 + arm->theta_target[2];
        
        arm->x0_target = 0.55f * arm_cos_f32(target_pitch_0) + 0.45f * arm_cos_f32(target_pitch_1);
        arm->y0_target = 0.55f * arm_sin_f32(target_pitch_0) + 0.45f * arm_sin_f32(target_pitch_1);
    }
    
    arm_route_point_calculate(arm); // 角度修复后重新计算关节角

}

/**
 * @brief 机械臂预设路径函数，输入预设路径点，检测当前位置，在内部输出关节角度
 * @todo 单/多段路径兼容
*/
void arm_route_multipoint(robotic_arm_stu *arm, robotic_arm_path_point_stu *path)
{
    arm_joint_state_update(arm);
    if(path->act_flag == 0)
    {
        arm->x0_target = path->x_target[path->position_flag];
        arm->y0_target = path->y_target[path->position_flag];
        arm->theta_target[0] = path->theta0_target[path->position_flag];
    }
    else if(path->act_flag == 1)
    {
        arm_route_point_calculate(arm);
        if (arm_route_reachable_check(arm) == IK_REACHABLE)
        {
            arm_route_execute(arm, 12.0f, 0.037f);
        }
        
    }
}

/**
 * @brief 机械臂末端执行器稳定控制函数，输入末端执行器的目标pitch与自转角来控制末端执行器的姿态稳定，输入单位为rad
 * @param target_pitch 末端执行器的目标pitch角，平行地面为0，抬起为正
 * @param theta4 末端执行器的自转角，面对吸盘逆时针为正
*/
void arm_actuator_stable_control(robotic_arm_stu *arm, float target_pitch, float theta4)
{
//该值为实验测得的电机与机构之间的角度比例
    float rate = 2666.0f; 

// 计算为了让 pitch[2] 达到 PI/2 所需的 joint_parameters[3].theta (即 theta3)
    arm->theta_target[3] = target_pitch - arm->joint_pitch[1];
    // 差速控制电机 1 和 2 达到该 theta3 
    arm->theta_target[3] = limit_float(arm->theta_target[3], -PI*7.0f/12.0f, PI*7.0f/12.0f);
    float target_delta_m = (arm->theta_target[3] - PI*7.0f/12.0f) / PI * 180.0f * rate * 2.0f;
    // 结合 debug_theta4 (翻转/自转角) 给出绝对目标
    m3508_anglesum_control(&m3508_fdcan1[1], (-target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * rate), 10000);
    m3508_anglesum_control(&m3508_fdcan1[2], (target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * rate), 10000);
}

/**
 * @brief 使用ht10a遥控器进行机械臂控制的函数，内含判断逻辑，仅用于精简代码
*/
void arm_remote_control(void)
{
    //遥控器控制执行器预期位置，测试用
    // if(HT10A.right_x !=0 || HT10A.right_y != 0 || HT10A.left_x != 0 || HT10A.switch_a != 0 || HT10A.knob_a != HT10A.last_status.knob_a_last
    //      )
    // {
      //执行器位置部分
      if(HT10A.right_x !=0 || HT10A.right_y != 0)
      {
        float last_x0_target = arm.x0_target;
        float last_y0_target = arm.y0_target;
        arm.x0_target += 0.0001f * arm_cos_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
        arm.y0_target += 0.0001f * arm_sin_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
        arm.x0_target = limit_float(arm.x0_target, 0.1f, 0.95f);
        arm.y0_target = limit_float(arm.y0_target, -0.4f, 0.95f);
        if(arm_route_reachable_check(&arm) == IK_LENGTH_OUT_OF_RANGE)
        {
          arm.x0_target = last_x0_target;
          arm.y0_target = last_y0_target;
        }
      }
      //云台部分
      if(HT10A.left_x != 0)
      {
        if(HT10A.left_x > 0){arm.theta_target[0] -= 0.0003f;}
        else{arm.theta_target[0] += 0.0003f;}
      }

      //执行器转角部分
      arm.effector_pitch_target += -0.0004f * (float)HT10A.switch_a;
      if(arm.effector_pitch_target > PI*7.0f/12.0f) arm.effector_pitch_target = PI*7.0f/12.0f;
      else if(arm.effector_pitch_target < -PI*7.0f/12.0f) arm.effector_pitch_target = -PI*7.0f/12.0f;
      
      arm.theta_target[4] = (float)(HT10A.knob_a+8000)/16000.0f*PI;
      if(arm.theta_target[4] > PI) arm.theta_target[4] = PI;
      else if(arm.theta_target[4] < -PI) arm.theta_target[4] = -PI;
    // }

    if(HT10A.switch_b == 1 )
    {
        //吸块，关节力矩需调大
        HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_SET);
        if(arm_route_reachable_check(&arm) == IK_REACHABLE)
        {
        arm_route_execute(&arm, 12.0f, 0.037f);
        }
    }
    else
    {
        //不吸，关节力矩需偏小
        HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_RESET);
        if(arm_route_reachable_check(&arm) == IK_REACHABLE)
        {
        arm_route_execute(&arm, 12.0f, 0.030f);
        }
    }
}

//-------------------------------------------以下为暂时弃用的重力补偿函数--------------------------------------------

//example:

    // arm_gravity_comp(&arm, 0.60f, 3000.0f);
    // //此处输出表示为输出轴电流(力矩)
    // if(m3508_fdcan1[0].Initial_flag == 200 && A1_debug_485_2[1].Pos_init_flag == 200)
    // {
    //   // m3508_fdcan1[0].Current_Output = (int16_t)(arm.joint_torque_gc[1]/0.3f/10.0f*8192.0f);
    //   // m3508_fdcan1[0].Current_Output = 0;
    //   // A1_debug_485_2[1].target.T = arm.joint_torque_gc[0] / 9.1f;
    // }

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


