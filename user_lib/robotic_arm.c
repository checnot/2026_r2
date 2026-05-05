#include "robotic_arm.h"

robotic_arm_stu arm;    //机械臂实例
robotic_arm_traj_stu arm_traj; //机械臂轨迹实例

/**
 * @brief 机械臂参数初始化函数，在机械臂任务开始时调用，进行机械臂物理参数的初始化
 * @note 这里的参数是根据机械臂的实际结构测量得到的，修改机械臂结构后需要重新测量并更新这些参数
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
    float rate = 2666.0f; //实验测得3508电机的转子和机械臂角度的转化比

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
    // m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)(0.0f/(PI/2.0f)*100000.0f), 8192);
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
    else if(arm->theta_target[1]<0.0f || arm->theta_target[1]>PI || arm->theta_target[2]<PI/36.0f || arm->theta_target[2]>PI)
    {
        return IK_ANGLE_OUT_OF_RANGE;//关节角超出物理限位
    }
    else
    {
        return IK_REACHABLE;//末端位置在机械臂工作范围内
    }
}

/**
 * @brief 机械臂点逆解算函数，输入目标末端位置，输出目标关节角
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


// ============================================ 以下为轨迹规划部分 ============================================


/**
 * @brief 机械臂轨迹初始化函数，主要用于初始化各类参数
 * @note 目前固定参数0.5m/s，2.0m/s^2，0.05m // PI/2rad/s，PIrad/s^2，PI/36rad
*/
void arm_trajectory_init(robotic_arm_traj_stu *arm_traj, robotic_arm_stu *arm)
{
    // 假设当前机械臂停在 arm->x0_now, arm->y0_now, arm->theta_target[0]
    // 参数:(实例, 起点位置, 目标位置, 两轴最高速度 0.5m/s, 加速度 2.0m/s^2, 弹簧死区 0.05m)

    // 机械臂平面x方向
    Traj_Adaptive_Init(&arm_traj->traj_x, arm->x0_now, arm->x0_now, 0.5f, 2.0f, 0.05f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_x, 0.025f); // 2cm 的路径融合半径
    // 机械臂平面y方向
    Traj_Adaptive_Init(&arm_traj->traj_y, arm->y0_now, arm->y0_now, 0.5f, 2.0f, 0.05f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_y, 0.025f); 
    // 机械臂云台旋转角
    Traj_Adaptive_Init(&arm_traj->traj_w, A1_debug_485_2[0].measure.Pos, A1_debug_485_2[0].measure.Pos, PI/2.0f, PI, PI/36.0f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_w, PI/72.0f); 

    //机械臂延时控制参数初始化
    arm_traj->target_delay_ms = 0;
    arm_traj->wait_counter = 0;

}


/**
 * @brief 直线插补多段路径调度器，包括了机械臂路径计算，路径点的联合抛出，以及机械臂到达路径点后的延时控制和气泵控制
 * @note 周期1ms，如果强行放在周期不一致的任务中会导致延时bug
 * @note 必须放在 Traj_Adaptive_Update(x) 和 Traj_Adaptive_Update(y) 之前调用
 */
void arm_linear_interpolation_update(robotic_arm_stu *arm, robotic_arm_traj_stu *arm_traj, float blend_radius) 
{ 
    // 最小融合半径限制，防止过于苛刻的判定条件导致死锁
    if(blend_radius < 0.001f) blend_radius = 0.001f;

    // 如果任意一个轴的队列为空，说明点位不齐，不进行出队（保证X/Y/W同步出队）
    if(arm_traj->traj_queue_x.count == 0 || arm_traj->traj_queue_y.count == 0 || arm_traj->traj_queue_w.count == 0) return;

    // // 如果当前这个点要求了延停，我们必须强制要求 blend = 0，使得机械臂 is_finished == 1 才能触发！
    // float current_blend = (arm_traj->target_delay_ms > 0) ? 0.0f : blend_radius;

    // === 1. 判敛条件（三轴必须同时准备好） ===
    float dist_act_x = fabsf(arm_traj->traj_x.target_pos - arm->x0_now);
    float dist_out_x = fabsf(arm_traj->traj_x.target_pos - arm_traj->traj_x.out_pos);
    uint8_t ready_x = arm_traj->traj_x.is_finished || 
                     (blend_radius > 0.001f && dist_act_x <= blend_radius && dist_out_x <= blend_radius);

    float dist_act_y = fabsf(arm_traj->traj_y.target_pos - arm->y0_now);
    float dist_out_y = fabsf(arm_traj->traj_y.target_pos - arm_traj->traj_y.out_pos);
    uint8_t ready_y = arm_traj->traj_y.is_finished || 
                     (blend_radius > 0.001f && dist_act_y <= blend_radius && dist_out_y <= blend_radius);

    // 云台W轴的判敛（角度的融合偏差可以适当放宽，或者按照弧度值判定）
    float dist_act_w = fabsf(arm_traj->traj_w.target_pos - arm->joint_parameters[0].theta);
    float dist_out_w = fabsf(arm_traj->traj_w.target_pos - arm_traj->traj_w.out_pos);
    uint8_t ready_w = arm_traj->traj_w.is_finished || 
                     (blend_radius > 0.001f && dist_act_w <= (blend_radius * 2.0f) && dist_out_w <= (blend_radius * 2.0f));


    // 新增延时逻辑：如果还有延时没等完，就原地return，不下发下一个点
    if (arm_traj->wait_counter > 0)
    {
        arm_traj->wait_counter--;
        return; 
    }
    // === 2. 联合抛点与参数动态重分配 ===
    if (ready_x && ready_y && ready_w) 
    {
        // // 新增延时逻辑：如果还有延时没等完，就原地return，不下发下一个点
        // if (arm_traj->wait_counter > 0)
        // {
        //     arm_traj->wait_counter--;
        //     return; 
        // }
        // 新增气泵逻辑
        uint8_t next_pump_action = arm_traj->pump_queue[arm_traj->traj_queue_x.tail];
        if (next_pump_action == 1) {
            HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_SET);   // 吸货
        } 
        else if (next_pump_action == 0) {
            HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_RESET); // 松货
        }

        float next_x = arm_traj->traj_queue_x.points[arm_traj->traj_queue_x.tail];
        float next_y = arm_traj->traj_queue_y.points[arm_traj->traj_queue_y.tail];
        float next_w = arm_traj->traj_queue_w.points[arm_traj->traj_queue_w.tail]; // 取出W轴目标
        // 提取该点的预设延时，并赋给计数器
        uint16_t next_delay = arm_traj->delay_queue[arm_traj->traj_queue_x.tail];
        arm_traj->target_delay_ms = next_delay;
        arm_traj->wait_counter = next_delay; // 为到达这个点后的停顿上满发条！

        // 同步弹出点位
        arm_traj->traj_queue_x.tail = (arm_traj->traj_queue_x.tail + 1) % TRAJ_MAX_WAYPOINTS;
        arm_traj->traj_queue_x.count--;
        arm_traj->traj_queue_y.tail = (arm_traj->traj_queue_y.tail + 1) % TRAJ_MAX_WAYPOINTS;
        arm_traj->traj_queue_y.count--;
        arm_traj->traj_queue_w.tail = (arm_traj->traj_queue_w.tail + 1) % TRAJ_MAX_WAYPOINTS;
        arm_traj->traj_queue_w.count--;
        
        // 计算各轴偏差
        float delta_x = fabsf(next_x - arm->x0_now);
        float delta_y = fabsf(next_y - arm->y0_now);
        float delta_w = fabsf(next_w - arm->joint_parameters[0].theta);
        
        // 综合距离：为了让角度也参与时间同步，将角度转化为权重系数(0.3)
        float distance_xy = sqrtf(delta_x*delta_x + delta_y*delta_y);
        float distance_total = distance_xy + (delta_w * 0.3f); 
        
        if (distance_total > 0.002f) 
        {
            // 计算各轴投影比例
            float px = delta_x / distance_total;
            float py = delta_y / distance_total;
            float pw = (delta_w * 0.3f) / distance_total; 
            
            // 下发X轴
            arm_traj->traj_x.v_max = GLOBAL_V_MAX * px;
            arm_traj->traj_x.a_max = GLOBAL_A_MAX * px;
            if(arm_traj->traj_x.v_max < 0.001f) arm_traj->traj_x.v_max = 0.001f;
            if(arm_traj->traj_x.a_max < 0.001f) arm_traj->traj_x.a_max = 0.001f;
            arm_traj->traj_x.target_pos = next_x;
            arm_traj->traj_x.is_finished = 0;

            // 下发Y轴
            arm_traj->traj_y.v_max = GLOBAL_V_MAX * py;
            arm_traj->traj_y.a_max = GLOBAL_A_MAX * py;
            if(arm_traj->traj_y.v_max < 0.001f) arm_traj->traj_y.v_max = 0.001f;
            if(arm_traj->traj_y.a_max < 0.001f) arm_traj->traj_y.a_max = 0.001f;
            arm_traj->traj_y.target_pos = next_y;
            arm_traj->traj_y.is_finished = 0;

            // 下发W轴 (角速度/角加速度根据力臂反算)
            arm_traj->traj_w.v_max = (GLOBAL_V_MAX / 0.3f) * pw;
            arm_traj->traj_w.a_max = (GLOBAL_A_MAX / 0.3f) * pw;
            if(arm_traj->traj_w.v_max < 0.002f) arm_traj->traj_w.v_max = 0.002f;
            if(arm_traj->traj_w.a_max < 0.002f) arm_traj->traj_w.a_max = 0.002f;
            arm_traj->traj_w.target_pos = next_w;
            arm_traj->traj_w.is_finished = 0;
        }
        
        return; 
    }
}


/**
 * @brief 机械臂添加单个路径点，坐标采用机械臂下的柱坐标，带有自动边界吸附保护功能
 * @param target_x 柱坐标系下期望的 X 坐标 (向前为正，向后为负)
 * @param target_y 柱坐标系下期望的 Y 坐标 (向上为正，向下为负)
 * @param target_w 柱坐标系下期望的云台旋转角 W (从上方看，逆时针为正，顺时针为负)
 * @param delay_ms 目标点到达后的延时时间(ms)
 * @param air_pump_flag 气泵控制标志，0为不控制，1为吸，2为吹（预留接口，目前未实现）
 */
void arm_queue_add_point(robotic_arm_traj_stu *arm_traj, float target_x, float target_y, float target_w,
                         uint16_t delay_time_ms, uint8_t air_pump_flag)
{
    float max_radius = 0.95f; // 最大物理伸展半径
    float min_radius = 0.15f; // 最小物理收缩半径 (防机械臂打脸)
    
    // 计算圆心（云台原点0,0）到该点的直线距离（半径L）
    float L = sqrtf(target_x * target_x + target_y * target_y);
    // 当发生超限时，将坐标等比例缩放至最大/最小圆周上
    if (L > max_radius && L > 0.001f)
    {
        target_x = target_x * (max_radius / L);
        target_y = target_y * (max_radius / L);
    }
    else if (L < min_radius && L > 0.001f)
    {
        target_x = target_x * (min_radius / L);
        target_y = target_y * (min_radius / L);
    }
    
    if(target_y < -0.3f) target_y = -0.3f; // 额外的边界限制，防止机械臂过度向下伸展
    else if(target_y > 0.95f) target_y = 0.95f; // 上限边界限制

    if(delay_time_ms > 3000) delay_time_ms = 3000; // 最大延时限制，防止过长时间占用队列资源
    if (air_pump_flag > 2) air_pump_flag = 0; // 无效的气泵标志重置为不控制

    // 将x的head作为其他元素的索引，进行延时和气泵标志的存储
    if (arm_traj->traj_queue_x.count >= TRAJ_MAX_WAYPOINTS) return;
    // 将延时信息存储到轨迹实例中，和点位队列下标同步
    arm_traj->delay_queue[arm_traj->traj_queue_x.head] = delay_time_ms;
    // 将气泵控制标志存储到轨迹实例中，和点位队列下标同步
    arm_traj->pump_queue[arm_traj->traj_queue_x.head] = air_pump_flag;

    // 将合法化后的安全坐标压入各自队列
    Traj_Path_Queue_AddPoint(&arm_traj->traj_queue_w, target_w);
    Traj_Path_Queue_AddPoint(&arm_traj->traj_queue_x, target_x);
    Traj_Path_Queue_AddPoint(&arm_traj->traj_queue_y, target_y);
}

/**
 * @brief 机械臂添加单个路径点，坐标使用车身下的直角坐标系
 * @note 输入车身坐标系下的目标点，输出柱坐标目标点给内部arm_queue_add_point使用
 * @param target_x 车身坐标系下的 X 坐标 (向前为正，向后为负)
 * @param target_y 车身坐标系下的 Y 坐标 (向左为正，向右为负)
 * @param target_z 车身坐标系下的 Z 坐标 (向上为正，向下为负)
 * @param delay_time_ms 目标点到达后的延时时间(ms)
 * @param air_pump_flag 气泵控制标志
 */
void arm_queue_add_point_cartesian(robotic_arm_traj_stu *arm_traj, float target_x, float target_y, float target_z, uint16_t delay_time_ms, uint8_t air_pump_flag)
{
    //将车身坐标系下的目标点转换柱坐标，机械臂坐标系以云台为原点，向前为X轴正向，向上为Y轴正向
    float l0 = sqrtf(target_x*target_x + target_y*target_y);
    float x_arm = l0;
    float y_arm = target_z;
    float w_arm = atan2f(target_y, target_x);
    uint16_t delay_ms_arm = delay_time_ms;

    arm_queue_add_point(arm_traj, x_arm, y_arm, w_arm, delay_ms_arm, air_pump_flag);
}


/**
 * @brief 预设机械臂轨迹测试函数，向轨迹队列中添加一系列点位，形成一个预设的轨迹，供测试使用
*/
void arm_load_test_waypoints_clockwise(void)
{
    //起始点
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 0, 0);

    arm_queue_add_point(&arm_traj, 0.3f, 0.6f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.6f, 0.6f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.6f, 0.3f, 0.0f, 0, 0);

    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 0, 0);
}

/**
 * @brief 预设机械臂轨迹测试函数，向轨迹队列中添加一系列点位，形成一个预设的轨迹，供测试使用
*/
void arm_load_test_waypoints_counterclockwise(void)
{
    //起始点
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 0, 0);

    arm_queue_add_point(&arm_traj, 0.6f, 0.3f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.6f, 0.6f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.3f, 0.6f, 0.0f, 0, 0);

    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 0, 0);
}

/**
 * @brief 预设机械臂轨迹测试函数，向轨迹队列中添加一系列点位，形成一个预设的轨迹，供测试使用
*/
void arm_load_test_waypoints_rotate(void)
{
    //起始点
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 1000, 0);

    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, PI/6.0f, 1000, 0);
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, -PI/6.0f, 1000, 0);
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, PI/6.0f, 1000, 0);
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, -PI/6.0f, 1000, 0);

    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 1000, 0);
}

/**
 * @brief 自定义机械臂轨迹
*/
void arm_load_test_waypoints_custom(void)
{
    arm_queue_add_point(&arm_traj, 0.3f, 0.3f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.3f, 0.45f, 0.0f, 0, 0);
    arm_queue_add_point(&arm_traj, 0.80f, 0.45f, 0.0f, 500, 0);
    arm_queue_add_point(&arm_traj, 0.80f, 0.36f, 0.0f, 1500, 1);
    arm_queue_add_point(&arm_traj, 0.4f, 0.36f, 0.0f, 500, 1);
    arm_queue_add_point(&arm_traj, 0.4f, 0.5f, 0.0f, 500, 1);

}


//================================ 遥控器手动控制 ======================================================

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

//================================ 遥控器手动控制 ======================================================

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


