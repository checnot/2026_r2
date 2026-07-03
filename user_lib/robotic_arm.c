#include "robotic_arm.h"

robotic_arm_stu arm;    //机械臂实例
robotic_arm_traj_stu arm_traj; //机械臂轨迹实例
robotic_arm_trans_traj_stu arm_trans_traj; //机械臂过渡轨迹实例

// 实际测试从最上掰到最下转子经过了105800单位，末端在机械上实际经过240deg，即pi*4/3 rad，所以转化比约为 105800/(PI*4/3) = 25258
// 为实机测量角度映射关系，该数值代表转子转过rate个单位，机械臂末端转过1rad
static float rate = 25258.0f;


/**
 * @brief 机械臂参数初始化函数，在机械臂任务开始时调用，进行机械臂物理参数的初始化
 * @note 这里的参数是根据机械臂的实际结构测量得到的，修改机械臂结构后需要重新测量并更新这些参数
*/
void arm_joint_parameter_init(robotic_arm_stu *arm)
{

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
 * @brief 机械臂正解算函数，在机械臂任务中调用，使用关节电机数据来更新机械臂的关节各状态
*/
void arm_joint_state_update(robotic_arm_stu *arm)
{  
//     float rate = 2666.0f; //实验测得3508电机的转子和机械臂角度的转化比
// //根据各关节处电机的测量位置更新机械臂的关节角状态，对二连杆末端位置进行正解算
//     arm->joint_parameters[2].theta = m3508_fdcan1[0].Angle_Sum/100000.0f*PI/2.0f + PI/12.0f;
//     arm->joint_pitch[0] = arm->joint_parameters[1].theta;
//     arm->joint_pitch[1] = -PI + arm->joint_pitch[0] + arm->joint_parameters[2].theta;
//     arm->x0_now = 0.55f*arm_cos_f32(arm->joint_pitch[0]) + 0.45f*arm_cos_f32(arm->joint_pitch[1]);
//     arm->y0_now = 0.55f*arm_sin_f32(arm->joint_pitch[0]) + 0.45f*arm_sin_f32(arm->joint_pitch[1]);
// //末端机构解算
//     arm->joint_parameters[3].theta = PI*7.0f/12.0f + (float)(m3508_fdcan1[1].Angle_Sum - m3508_fdcan1[2].Angle_Sum)/2.0f/rate/180.0f*PI;
//     //joint_parameters[4].theta实际上是前部吸盘自转角度，对着吸盘逆时针为正
//     arm->joint_parameters[4].theta = -1.0f*(float)(m3508_fdcan1[1].Angle_Sum + m3508_fdcan1[2].Angle_Sum)/2.0f/rate/180.0f*PI;
//     arm->joint_pitch[2] = arm->joint_pitch[1] + arm->joint_parameters[3].theta;


// 根据各关节处电机的测量位置更新机械臂的关节角状态，对二连杆末端位置进行正解算
    arm->joint_parameters[2].theta = m8010_bus_1.motors[2].measure.Pos + FOREARM_OFFSET_DEG/180.0f*PI;  // 小臂与大臂的夹角
    arm->joint_pitch[0] = arm->joint_parameters[1].theta;
    arm->joint_pitch[1] = -PI + arm->joint_pitch[0] + arm->joint_parameters[2].theta;
    arm->x0_now = 0.55f*arm_cos_f32(arm->joint_pitch[0]) + 0.45f*arm_cos_f32(arm->joint_pitch[1]);
    arm->y0_now = 0.55f*arm_sin_f32(arm->joint_pitch[0]) + 0.45f*arm_sin_f32(arm->joint_pitch[1]);
// 末端机构解算

    arm->joint_parameters[3].theta = -m3508_fdcan1[0].Angle_Sum/rate + WRIST_OFFSET_DEG/180.0f*PI;   //（实测56.4deg）末端与小臂的夹角
    arm->joint_pitch[2] = arm->joint_pitch[1] + arm->joint_parameters[3].theta;
}


/**
 * @brief 用于检测目标路径点是否可到达（前面/常规工作区模式），推荐在目标点解算前以及完毕后调用
*/
IK_Reach_Status_enum arm_route_reachable_check_front(robotic_arm_stu *arm)
{
    if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) > 0.95f || sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) < 0.15f) 
    {
        return IK_LENGTH_OUT_OF_RANGE;//末端位置超出机械臂工作范围
    }
    else if(arm->theta_target[1]<UPPERARM_LIMIT_DEG_MIN/180.0f*PI || arm->theta_target[1]>UPPERARM_LIMIT_DEG_MAX/180.0f*PI || arm->theta_target[2]<FOREARM_LIMIT_DEG_MIN/180.0f*PI || arm->theta_target[2]>FOREARM_LIMIT_DEG_MAX/180.0f*PI)
    {
        return IK_ANGLE_OUT_OF_RANGE;//关节角超出物理限位
    }
    else
    {
        return IK_REACHABLE;//末端位置在机械臂工作范围内
    }
}

/**
 * @brief 用于检测目标路径点是否可到达（背后/存块工作区模式），推荐在目标点解算前以及完毕后调用
*/
IK_Reach_Status_enum arm_route_reachable_check_back(robotic_arm_stu *arm)
{
    if(sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) > 0.95f || sqrtf(arm->x0_target*arm->x0_target + arm->y0_target*arm->y0_target) < 0.15f) 
    {
        return IK_LENGTH_OUT_OF_RANGE;//末端位置超出机械臂工作范围
    }
    // 大臂：30°~170° (向后工作区)
    if(arm->theta_target[1] < UPPERARM_LIMIT_DEG_MIN/180.0f*PI || arm->theta_target[1] > UPPERARM_LIMIT_DEG_MAX/180.0f*PI)
        return IK_ANGLE_OUT_OF_RANGE;

    // 小臂：180°~330°，即 [PI, 11*PI/6]（排除±30°死区）
    if(arm->theta_target[2] < FOREARM_LIMIT_DEG_MIN/180.0f*PI || arm->theta_target[2] > FOREARM_LIMIT_DEG_MAX/180.0f*PI)
        return IK_ANGLE_OUT_OF_RANGE;

    return IK_REACHABLE;
}

/**
 * @brief 机械臂点逆解算函数，输入目标末端位置，输出目标关节角
 * @warning 目前仅支持二连杆末端位置的计算
*/
void arm_route_point_calculate_front(robotic_arm_stu *arm)
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
 * @brief 机械臂点逆解算函数，输入目标末端位置，输出目标关节角（背后/存块工作区模式，从正常构型到此处需要经过位置环过渡）
 * @note 该函数与 arm_route_point_calculate 的区别在于肘部朝下构型（theta1更靠后/更高，theta2夹角更大），适用于机械臂背后/存块工作区的情况
*/
void arm_route_point_calculate_back(robotic_arm_stu *arm)
{
    float x = arm->x0_target;
    float y = arm->y0_target;
    float l1 = arm->joint_parameters[2].a; // 0.55
    float l2 = arm->joint_parameters[3].a; // 0.45
    float l0 = sqrtf(x*x + y*y);

    if(l0 > 0.15f)
    {
        float temp_cos_angle0, temp_cos_angle1;
        temp_cos_angle0 = (l1*l1 + l0*l0 - l2*l2) / (2.0f*l1*l0);
        temp_cos_angle1 = (l1*l1 + l2*l2 - l0*l0) / (2.0f*l1*l2);
        if(temp_cos_angle0 > 1.0f) temp_cos_angle0 = 1.0f;
        else if(temp_cos_angle0 < -1.0f) temp_cos_angle0 = -1.0f;
        if(temp_cos_angle1 > 1.0f) temp_cos_angle1 = 1.0f;
        else if(temp_cos_angle1 < -1.0f) temp_cos_angle1 = -1.0f;

        // 肘部朝下构型：
        // theta1 = atan2(y,x) - acos(...)  （大臂更靠后/更高）
        // theta2 = 2*PI - acos(...)        （小臂夹角 > 180°，电机返回正值）
        arm->theta_target[1] = atan2f(y, x) - acosf(temp_cos_angle0);
        arm->theta_target[2] = 2.0f * PI - acosf(temp_cos_angle1);
    }
}


/**
 * @brief 将关节夹角控制至指定位置，利用arm_route_point_calculate计算的关节夹角来执行各关节电机的位置环控制，输入关节夹角(rad)，经过内置单位转换后输出到电机
 * @warning a1, m8010 位置环参数需要根据测试进行调整
*/
void arm_route_execute(robotic_arm_stu *arm, float m8010_kp, float m8010_kw, float a1_kp, float a1_kw)
{
    // A1_Motor_Position_Control(&A1_debug_485_2[1], PI*11.0f/12.0f - arm->theta_target[1], a1_kw, a1_kp);             //大臂
    // m3508_anglesum_control(&m3508_fdcan1[0], (int32_t)((arm->theta_target[2]-PI/12.0f)/(PI/2.0f)*100000.0f), 12000);//小臂
    // m8010_position_control(&m8010_bus_1.motors[1], (UPPERARM_OFFSET_DEG/180.0f*PI - arm->theta_target[1]), m8010_kp, m8010_kw);             //大臂
    // m8010_position_control(&m8010_bus_1.motors[2], (arm->theta_target[2] - FOREARM_OFFSET_DEG/180.0f*PI), m8010_kp, m8010_kw);             //小臂

    A1_Motor_Position_Control(&A1_debug_485_2[0], (UPPERARM_OFFSET_DEG/180.0f*PI - arm->theta_target[1]), a1_kw, a1_kp);
    m8010_position_control(&m8010_bus_1.motors[2], (arm->theta_target[2] - FOREARM_OFFSET_DEG/180.0f*PI), m8010_kp, m8010_kw);
}

/**
 * @brief 机械臂气泵应用函数，根据气泵状态标志控制气泵的吸取/释放动作
*/
void arm_pump_apply(robotic_arm_stu *arm)
{
    if (arm->pump_state == ARM_PUMP_SUCK) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_SET);
    } else if (arm->pump_state == ARM_PUMP_RELEASE) {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);
    }
}
/**
 * @brief 机械臂气泵控制函数，输入气泵控制命令来设置气泵的吸取/释放状态标志
 * @param cmd 气泵控制命令，0为保持当前状态，1为吸取，2为释放
*/
void arm_pump_set(robotic_arm_stu *arm, uint8_t cmd)
{
    if (cmd == ARM_PUMP_SUCK) {
        arm->pump_state = ARM_PUMP_SUCK;
    } else if (cmd == ARM_PUMP_RELEASE) {
        arm->pump_state = ARM_PUMP_RELEASE;
    }

    arm_pump_apply(arm);
}


/**
 * @brief 检查末端执行器的pitch角是否在可达范围内
 * @param arm 机械臂实例
 * @param target_pitch 目标pitch角
 * @return 1表示可达，0表示不可达
 */
uint8_t arm_effector_pitch_reachable_check(robotic_arm_stu *arm, float target_pitch)
{
    float theta3_cmd = target_pitch - arm->joint_pitch[1];

    float theta3_min = WRIST_LIMIT_DEG_MIN / 180.0f * PI;
    float theta3_max = WRIST_LIMIT_DEG_MAX / 180.0f * PI;

    if (theta3_cmd < theta3_min || theta3_cmd > theta3_max) {
        return 0;
    }

    return 1;
}

/**
 * @brief 将末端执行器绝对pitch目标限制到当前小臂姿态下腕部可达范围内
 */
static float arm_effector_pitch_clamp(robotic_arm_stu *arm, float target_pitch)
{
    float theta3_min = WRIST_LIMIT_DEG_MIN / 180.0f * PI;
    float theta3_max = WRIST_LIMIT_DEG_MAX / 180.0f * PI;

    float pitch_min = arm->joint_pitch[1] + theta3_min;
    float pitch_max = arm->joint_pitch[1] + theta3_max;

    return limit_float(target_pitch, pitch_min, pitch_max);
}

/**
 * @brief 机械臂末端执行器稳定控制函数，输入末端执行器的目标pitch与自转角来控制末端执行器的姿态稳定，输入单位为rad
 * @param target_pitch 末端执行器的目标pitch角，单位为rad，平行地面为0，抬起为正
 * @param theta4 末端执行器的自转角，面对吸盘逆时针为正
*/
void arm_actuator_stable_control(robotic_arm_stu *arm, float target_pitch, float theta4)
{
// //该值为实验测得的电机与机构之间的角度比例
//     float rate = 2666.0f; 
// // 计算为了让 pitch[2] 达到 PI/2 所需的 joint_parameters[3].theta (即 theta3)
//     arm->theta_target[3] = target_pitch - arm->joint_pitch[1];
//     // 差速控制电机 1 和 2 达到该 theta3 
//     arm->theta_target[3] = limit_float(arm->theta_target[3], -PI*7.0f/12.0f, PI*7.0f/12.0f);
//     float target_delta_m = (arm->theta_target[3] - PI*7.0f/12.0f) / PI * 180.0f * rate * 2.0f;
//     // 结合 debug_theta4 (翻转/自转角) 给出绝对目标
//     m3508_anglesum_control(&m3508_fdcan1[1], (-target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * rate), 10000);
//     m3508_anglesum_control(&m3508_fdcan1[2], (target_delta_m / 2.0f) + (-1.0f * theta4 / PI * 180.0f * rate), 10000);


// 计算为了让 pitch[2] 达到 PI/2 所需的 joint_parameters[3].theta (即 theta3)
    arm->theta_target[3] = target_pitch - arm->joint_pitch[1];
// 末端执行器的自转角控制
// 根据实际机械限位修改
    arm->theta_target[3] = limit_float(arm->theta_target[3], WRIST_LIMIT_DEG_MIN/180.0f*PI, WRIST_LIMIT_DEG_MAX/180.0f*PI);
    m3508_anglesum_control(&m3508_fdcan1[0], (-arm->theta_target[3] + WRIST_OFFSET_DEG/180.0f*PI) * rate, 8192);
}


// ============================================ 以下为轨迹规划部分 ============================================
// ============================================ 以下为轨迹规划部分 ============================================
// ============================================ 以下为轨迹规划部分 ============================================


/**
 * @brief 机械臂轨迹初始化函数，主要用于初始化各类参数
 * @note 目前固定参数0.5m/s，2.0m/s^2，0.05m // PI/2rad/s，PIrad/s^2，PI/36rad
*/
void arm_trajectory_init(robotic_arm_traj_stu *arm_traj, robotic_arm_stu *arm)
{
    // 参数:(实例, 起点位置, 目标位置, 两轴最高速度 0.5m/s, 加速度 2.0m/s^2, 弹簧死区 0.05m)

    // 机械臂平面x方向
    Traj_Adaptive_Init(&arm_traj->traj_x, arm->x0_now, arm->x0_now, GLOBAL_V_MAX, GLOBAL_A_MAX, 0.05f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_x, 0.025f); // 2cm 的路径融合半径
    // 机械臂平面y方向
    Traj_Adaptive_Init(&arm_traj->traj_y, arm->y0_now, arm->y0_now, GLOBAL_V_MAX, GLOBAL_A_MAX, 0.05f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_y, 0.025f); 
    //     //机械臂延时控制参数初始化
    arm_traj->target_delay_ms = 0;
    arm_traj->wait_counter = 0;

    Traj_Adaptive_Init(&arm_traj->traj_pitch, arm->joint_pitch[2],
                       arm->effector_pitch_target,
                        PITCH_V_MAX, PITCH_A_MAX, PI/6.0f); // 角速度/角加速度/死区

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

    // 如果任意一个轴的队列为空，说明点位不齐，不进行出队（保证 X/Y 同步出队）
    if(arm_traj->traj_queue_x.count == 0 || arm_traj->traj_queue_y.count == 0) return;

    // // 如果当前这个点要求了延停，我们必须强制要求 blend = 0，使得机械臂 is_finished == 1 才能触发！
    // float current_blend = (arm_traj->target_delay_ms > 0) ? 0.0f : blend_radius;

    // === 1. 判敛条件（X/Y 两轴必须同时准备好） ===
    float dist_act_x = fabsf(arm_traj->traj_x.target_pos - arm->x0_now);
    float dist_out_x = fabsf(arm_traj->traj_x.target_pos - arm_traj->traj_x.out_pos);
    uint8_t ready_x = arm_traj->traj_x.is_finished || 
                     (blend_radius > 0.001f && dist_act_x <= blend_radius && dist_out_x <= blend_radius);

    float dist_act_y = fabsf(arm_traj->traj_y.target_pos - arm->y0_now);
    float dist_out_y = fabsf(arm_traj->traj_y.target_pos - arm_traj->traj_y.out_pos);
    uint8_t ready_y = arm_traj->traj_y.is_finished || 
                     (blend_radius > 0.001f && dist_act_y <= blend_radius && dist_out_y <= blend_radius);



    // 新增延时逻辑：如果还有延时没等完，就原地return，不下发下一个点
    if (arm_traj->wait_counter > 0)
    {
        arm_traj->wait_counter--;
        return; 
    }
    // === 2. 联合抛点与参数动态重分配 ===
    if (ready_x && ready_y) 
    {
        // // 新增延时逻辑：如果还有延时没等完，就原地return，不下发下一个点
        // if (arm_traj->wait_counter > 0)
        // {
        //     arm_traj->wait_counter--;
        //     return; 
        // }
        // 新增气泵逻辑
        uint8_t next_pump_action = arm_traj->pump_queue[arm_traj->traj_queue_x.tail];
        if (next_pump_action == ARM_PUMP_SUCK) {
            arm_pump_set(arm, ARM_PUMP_SUCK);
        }
        else if (next_pump_action == ARM_PUMP_RELEASE) {
            arm_pump_set(arm, ARM_PUMP_RELEASE);
        }

        float next_x = arm_traj->traj_queue_x.points[arm_traj->traj_queue_x.tail];
        float next_y = arm_traj->traj_queue_y.points[arm_traj->traj_queue_y.tail];
        // 提取该点的预设延时，并赋给计数器
        uint16_t next_delay = arm_traj->delay_queue[arm_traj->traj_queue_x.tail];
        arm_traj->target_delay_ms = next_delay;
        arm_traj->wait_counter = next_delay; // 为到达这个点后的停顿上满发条！

        // === 新增：取出对应 pitch ===
        float next_pitch = arm_traj->pitch_queue[arm_traj->traj_queue_x.tail];

        // 同步弹出点位
        arm_traj->traj_queue_x.tail = (arm_traj->traj_queue_x.tail + 1) % TRAJ_MAX_WAYPOINTS;
        arm_traj->traj_queue_x.count--;
        arm_traj->traj_queue_y.tail = (arm_traj->traj_queue_y.tail + 1) % TRAJ_MAX_WAYPOINTS;
        arm_traj->traj_queue_y.count--;        
        // 计算各轴偏差
        float delta_x = fabsf(next_x - arm->x0_now);
        float delta_y = fabsf(next_y - arm->y0_now);        
        // 综合距离：按 X/Y 平面距离同步分配速度
        float distance_xy = sqrtf(delta_x*delta_x + delta_y*delta_y);

        if (distance_xy > 0.002f) 
        {
            // 计算各轴投影比例
            float px = delta_x / distance_xy;
            float py = delta_y / distance_xy;
            
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


            float delta_pitch = fabsf(next_pitch - arm->joint_pitch[2]);
            if(delta_pitch > 0.02f)
            {
                float pp = delta_pitch / distance_xy;
                arm_traj->traj_pitch.v_max = GLOBAL_V_MAX * pp * PI; // 角速度映射系数
                arm_traj->traj_pitch.a_max = GLOBAL_A_MAX * pp * 0.3f * PI;
                if (arm_traj->traj_pitch.v_max < 0.01f)  arm_traj->traj_pitch.v_max  = 0.01f;
                if (arm_traj->traj_pitch.a_max < 0.01f)  arm_traj->traj_pitch.a_max  = 0.01f;
                arm_traj->traj_pitch.target_pos = next_pitch;
                arm_traj->traj_pitch.is_finished = 0;
            } else {
                // 距离太小或 pitch 变化太小：直接跳到位
                arm_traj->traj_pitch.target_pos = next_pitch;
                arm_traj->traj_pitch.is_finished = 0;
            }
        }
        
        return; 
    }
}


/**
 * @brief 机械臂添加单个路径点，坐标采用机械臂下的柱坐标，带有自动边界吸附保护功能
 * @param target_x 柱坐标系下期望的 X 坐标 (向前为正，向后为负)
 * @param target_y 柱坐标系下期望的 Y 坐标 (向上为正，向下为负)
 * @param delay_ms 目标点到达后的延时时间(ms)
 * @param pump_state 气泵控制标志，0为不控制，1为吸，2为松开
 * @param target_pitch 末端执行器的目标pitch角，单位为rad
 */
void arm_queue_add_point(robotic_arm_traj_stu *arm_traj, float target_x, float target_y,
                         uint16_t delay_time_ms, uint8_t pump_state, float target_pitch)
{
    float max_radius = 0.95f; // 最大物理伸展半径
    float min_radius = 0.30f; // 最小物理收缩半径 (机械结构限制)
    
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
    
    if(target_y < -0.175f) target_y = -0.175f; // 额外的边界限制，防止机械臂过度向下伸展
    else if(target_y > 0.95f) target_y = 0.95f; // 上限边界限制

    if(delay_time_ms > 3000) delay_time_ms = 3000; // 最大延时限制，防止过长时间占用队列资源
    if (pump_state > 3) pump_state = 0; // 无效的气泵标志重置为不控制

    // 将x的head作为其他元素的索引，进行延时和气泵标志的存储
    if (arm_traj->traj_queue_x.count >= TRAJ_MAX_WAYPOINTS) return;
    // 将延时信息存储到轨迹实例中，和点位队列下标同步
    arm_traj->delay_queue[arm_traj->traj_queue_x.head] = delay_time_ms;
    // 将气泵控制标志存储到轨迹实例中，和点位队列下标同步
    arm_traj->pump_queue[arm_traj->traj_queue_x.head] = pump_state;
    // === 新增：pitch 角度入队（与 delay/pump 同步，用当前 head） ===
    arm_traj->pitch_queue[arm_traj->traj_queue_x.head] = target_pitch;

    // 将合法化后的安全坐标压入各自队列
    Traj_Path_Queue_AddPoint(&arm_traj->traj_queue_x, target_x);
    Traj_Path_Queue_AddPoint(&arm_traj->traj_queue_y, target_y);
}




/* ============================================= 以下为工作区切换部分 ============================================= */

/**
 * @brief 将机械臂模式切换到过渡模式
 * @param dir 过渡方向
 * @note 用于发起过渡
 */
void arm_mode_switch_to_transition(robotic_arm_stu *arm, robotic_arm_traj_stu *arm_traj, robotic_arm_trans_traj_stu *arm_trans_traj, ArmTransDir_enum dir)
{
    // 1. 清空所有轨迹队列（丢弃未完成的路径点）
    Traj_Path_Queue_Init(&arm_traj->traj_queue_x, 0.025f);
    Traj_Path_Queue_Init(&arm_traj->traj_queue_y, 0.025f);
        // 2. 重置延时/pitch状态
    arm_traj->target_delay_ms = 0;
    arm_traj->wait_counter = 0;
    arm->pump_state = ARM_PUMP_KEEP; // 过渡阶段保持当前气泵状态
    

    // 3. 设定过渡方向
    arm->trans_dir = dir;
    
    // 4. 根据方向设定过渡目标关节角
    if (dir == ARM_TRANS_FRONT_TO_BACK) {
        arm->trans_target_theta1 = TRANS_F2B_THETA1_TARGET;
        arm->trans_target_theta2 = TRANS_F2B_THETA2_TARGET;
        arm->trans_target_pitch = TRANS_F2B_PITCH_TARGET;
    } else if (dir == ARM_TRANS_BACK_TO_FRONT) {
        arm->trans_target_theta1 = TRANS_B2F_THETA1_TARGET;
        arm->trans_target_theta2 = TRANS_B2F_THETA2_TARGET;
        arm->trans_target_pitch = TRANS_B2F_PITCH_TARGET;
    }
    
    if (!arm_effector_pitch_reachable_check(arm, arm->trans_target_pitch)) {
        /*
        * 这里有两种处理方式：
        * 1. 直接 return，不进入过渡
        * 2. 将 trans_pitch_target 修正成当前 pitch，保证腕部不乱动
        *
        * 调试阶段建议先用第 2 种，更安全。
        */
        // arm->trans_target_pitch = arm->joint_pitch[2];
    }

    // 初始化过渡轨迹
    Traj_Adaptive_Init(
        &arm_trans_traj->traj_theta1,
        arm->joint_parameters[1].theta,
        arm->trans_target_theta1,
        TRANS_V_MAX,          // 最大速度：2pi rad/s，可根据实机调
        TRANS_A_MAX,          // 最大加速度：3pi rad/s^2
        PI / 24.0f            // 最大跟随误差：10 deg
    );
    Traj_Adaptive_Init(
        &arm_trans_traj->traj_theta2,
        arm->joint_parameters[2].theta,
        arm->trans_target_theta2,
        TRANS_V_MAX * 1.25f,  // 小臂速度可以更快一些
        TRANS_A_MAX * 1.25f,  // 小臂加速度也可以更快一些
        PI / 15.0f           // 小臂跟随误差可以更大一些
    );
    Traj_Adaptive_Init(
        &arm_trans_traj->traj_pitch,
        arm->joint_pitch[2],
        arm->trans_target_pitch,
        PITCH_V_MAX,    // pitch 速度可以稍微慢一些，防止运动过猛块飞了
        PITCH_A_MAX,    // pitch 加速度也可以稍微慢一些
        PI / 9.0f
    );

    
    // 6. 切换到过渡模式
    arm->mode = ARM_MODE_TRANSITION;
}


/**
 * @brief 更新机械臂工作模式，根据当前模式执行相应的控制逻辑，包括前向工作模式、过渡模式和后向工作模式
 * @note m8010参数推荐：kp=3, kw=0.3
 * @note a1参数推荐：kp=0.25, kw=7.5
 * @note ==============================================
 * @note 这是你放在 1ms 任务中最核心的调度函数
 * @note m8010和a1的参数大小关系不同
 * @note 过渡模式的位置和参数需要根据实际测试进行调整
 * @warning 代替了原有 task 中最后的 arm_route_execute 和 arm_route_point_calculate 和 arm_effector_control 的调用
 */
void arm_mode_update_execute(robotic_arm_stu *arm, robotic_arm_traj_stu *arm_traj, robotic_arm_trans_traj_stu *arm_trans_traj, float m8010_kp, float m8010_kw, float a1_kp, float a1_kw)
{
    switch (arm->mode) 
    {
        
    // ========== 前向工作模式 ==========
    case ARM_MODE_FRONT:
    {
        // 正常流程：轨迹规划 → 逆解算 → 可到达检查 → 执行
        arm_route_point_calculate_front(arm);
        if (arm_route_reachable_check_front(arm) == IK_REACHABLE) {
            // 轨迹更新后，用逆解算结果驱动位置环
            arm_route_execute(arm, m8010_kp, m8010_kw, a1_kp, a1_kw);
        }
        // 注意：从 FRONT 切到 TRANSITION 由上层（行为树/手动指令）触发

        // === 新增：pitch 轨迹推进 + 执行 ===
        Traj_Adaptive_Update(&arm_traj->traj_pitch, arm->joint_pitch[2], 0.001f);
        float safe_pitch = arm_effector_pitch_clamp(arm, arm_traj->traj_pitch.out_pos);
        if (arm_traj->traj_pitch.is_finished) {
            safe_pitch = arm_effector_pitch_clamp(arm, arm_traj->traj_pitch.target_pos);
        }
        arm_actuator_stable_control(arm, safe_pitch, arm->theta_target[4]);
        if (arm_traj->traj_pitch.is_finished) {
            arm->effector_pitch_target = arm_traj->traj_pitch.target_pos;
        }
        break;
    }
    // ========== 过渡模式 ==========
    case ARM_MODE_TRANSITION:
    {
        // 1. 保持吸盘当前状态
        arm_pump_apply(arm);

        // 2. 推演三条过渡轨迹
        Traj_Adaptive_Update(&arm_trans_traj->traj_theta1, arm->joint_parameters[1].theta, 0.001f);
        Traj_Adaptive_Update(&arm_trans_traj->traj_theta2, arm->joint_parameters[2].theta, 0.001f);
        Traj_Adaptive_Update(&arm_trans_traj->traj_pitch, arm->joint_pitch[2], 0.001f);

        // 3. 下发大臂、小臂轨迹输出
        arm->theta_target[1] = arm_trans_traj->traj_theta1.out_pos;
        arm->theta_target[2] = arm_trans_traj->traj_theta2.out_pos;

        // 4. 将 pitch 轨迹输出钳制到当前小臂姿态下腕部可达范围内，保证末端姿态命令连续
        float safe_pitch = arm_effector_pitch_clamp(arm, arm_trans_traj->traj_pitch.out_pos);
        arm_actuator_stable_control(arm,
                                    safe_pitch,
                                    arm->theta_target[4]);

        // 5. 执行大臂、小臂位置环
        arm_route_execute(arm, m8010_kp, m8010_kw, a1_kp, a1_kw);

        // 6. 三条轨迹都完成，才允许切换模式
        if (arm_trans_traj->traj_theta1.is_finished &&
            arm_trans_traj->traj_theta2.is_finished &&
            arm_trans_traj->traj_pitch.is_finished) 
        {
            arm->effector_pitch_target = arm->trans_target_pitch; // 更新末端执行器目标 pitch（过渡结束后保持这个值不变）
            if (arm->trans_dir == ARM_TRANS_FRONT_TO_BACK) {
                arm->mode = ARM_MODE_BACK;
            } else {
                arm->mode = ARM_MODE_FRONT;
            }

            // 过渡完成后以当前位置重新初始化笛卡尔轨迹规划器
            arm_trajectory_init(arm_traj, arm);
        }

        break;
    }
        
    // ========== 后向工作模式 ==========
    case ARM_MODE_BACK:
    {
        // 反向逆解算 → 可到达检查 → 轨迹规划 → 执行
        arm_route_point_calculate_back(arm);
        if (arm_route_reachable_check_back(arm) == IK_REACHABLE) {
            arm_route_execute(arm, m8010_kp, m8010_kw, a1_kp, a1_kw);
        }

        // === 新增：pitch 轨迹推进 + 执行（与 FRONT 相同） ===
        Traj_Adaptive_Update(&arm_traj->traj_pitch, arm->joint_pitch[2], 0.001f);
        float safe_pitch = arm_effector_pitch_clamp(arm, arm_traj->traj_pitch.out_pos);
        if (arm_traj->traj_pitch.is_finished) {
            safe_pitch = arm_effector_pitch_clamp(arm, arm_traj->traj_pitch.target_pos);
        }
        arm_actuator_stable_control(arm, safe_pitch, arm->theta_target[4]);
        if (arm_traj->traj_pitch.is_finished) {
            arm->effector_pitch_target = arm_traj->traj_pitch.target_pos;
        }
        break;
    }
    }
}








/* ============================================= 以下为弃用部分 ============================================= */
/* ============================================= 以下为弃用部分 ============================================= */
/* ============================================= 以下为弃用部分 ============================================= */
/* ============================================= 以下为弃用部分 ============================================= */
/* ============================================= 以下为弃用部分 ============================================= */


//================================ 遥控器手动控制 ======================================================
/**
 * @brief 使用ht10a遥控器进行机械臂控制的函数，内含判断逻辑，仅用于精简代码
*/
// void arm_remote_control(void)
// {
//     //遥控器控制执行器预期位置，测试用
//     // if(HT10A.right_x !=0 || HT10A.right_y != 0 || HT10A.left_x != 0 || HT10A.switch_a != 0 || HT10A.knob_a != HT10A.last_status.knob_a_last
//     //      )
//     // {
//       //执行器位置部分
//       if(HT10A.right_x !=0 || HT10A.right_y != 0)
//       {
//         float last_x0_target = arm.x0_target;
//         float last_y0_target = arm.y0_target;
//         arm.x0_target += 0.0001f * arm_cos_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
//         arm.y0_target += 0.0001f * arm_sin_f32(atan2f((float)HT10A.right_y, (float)HT10A.right_x));
//         arm.x0_target = limit_float(arm.x0_target, 0.1f, 0.95f);
//         arm.y0_target = limit_float(arm.y0_target, -0.4f, 0.95f);
//         if(arm_route_reachable_check(&arm) == IK_LENGTH_OUT_OF_RANGE)
//         {
//           arm.x0_target = last_x0_target;
//           arm.y0_target = last_y0_target;
//         }
//       }
//       if(HT10A.left_x != 0)
//       {
//       }
//       //执行器转角部分
//       arm.effector_pitch_target += -0.0004f * (float)HT10A.switch_a;
//       if(arm.effector_pitch_target > PI*7.0f/12.0f) arm.effector_pitch_target = PI*7.0f/12.0f;
//       else if(arm.effector_pitch_target < -PI*7.0f/12.0f) arm.effector_pitch_target = -PI*7.0f/12.0f;
//       arm.theta_target[4] = (float)(HT10A.knob_a+8000)/16000.0f*PI;
//       if(arm.theta_target[4] > PI) arm.theta_target[4] = PI;
//       else if(arm.theta_target[4] < -PI) arm.theta_target[4] = -PI;
//     // }
//     if(HT10A.switch_b == 1 )
//     {
//         //吸块，关节力矩需调大
//         // HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_SET);
//         if(arm_route_reachable_check(&arm) == IK_REACHABLE)
//         {
//         arm_route_execute(&arm, 12.0f, 0.037f);
//         }
//     }
//     else
//     {
//         //不吸，关节力矩需偏小
//         // HAL_GPIO_WritePin(ARM_AIR_PUMP_GPIO_Port, ARM_AIR_PUMP_Pin, GPIO_PIN_RESET);
//         if(arm_route_reachable_check(&arm) == IK_REACHABLE)
//         {
//         arm_route_execute(&arm, 12.0f, 0.030f);
//         }
//     }
// }
//================================ 遥控器手动控制 ======================================================


//-------------------------------------------以下为暂时弃用的重力补偿函数--------------------------------------------
//-------------------------------------------以下为暂时弃用的重力补偿函数--------------------------------------------
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
// /**
//  * @brief 通过预设力矩值计算单一连杆所需的重力补偿力矩
//  * @param pitch 连杆相对于地面的绝对角度，单位rad
//  * @param torque_preset 预设的重力补偿力矩值，需要根据测试进行调整，单位不定
//  * @return 返回重力补偿力矩值，单位Nm
//  * @warning 预设力矩值的大小需要实验得出，不同的电机其力矩描述也不同，所以单位不定，但是可以通过手册数据推算国际单位下的力矩值
// */
// float gravity_comp_calc(float pitch, float torque_preset)
// {
//     return torque_preset * arm_cos_f32(pitch);
// }
// /**
//  * @brief 机械臂整体的重力补偿函数，在机械臂任务中调用，用robotic_arm_stu中的jointpitch计算的joint_torque_gc
//  * @param torque_preset0 预设的肩部重力补偿力矩值
//  * @param torque_preset1 预设的手肘重力补偿力矩值
//  * @warning 仅用于目前的机械臂设计，且预设力矩值需要根据测试进行调整，但保留了可拓展性
// */
// void arm_gravity_comp(robotic_arm_stu *arm, float torque_preset0, float torque_preset1)
// {
//     //手肘处电机输出轴力矩(Nm)
//     arm->joint_torque_gc[1] = gravity_comp_calc(arm->joint_pitch[1], torque_preset1);
//     //肩部处电机输出轴力矩(Nm)，由于肩部需要带动手肘，所以肩部的重力补偿不仅要补偿肩部自身的重力，还要补偿手肘的重力（通过手肘处的电机力矩反推）
//     arm->joint_torque_gc[0] = ((1.006f-0.254f*arm_cos_f32(arm->joint_parameters[2].theta))*(-arm_cos_f32(arm->joint_pitch[0])))*9.1f;
// }

//------------------------------------------- 重力补偿函数 --------------------------------------------
//------------------------------------------- 重力补偿函数 --------------------------------------------
//------------------------------------------- 重力补偿函数 --------------------------------------------

/**
 * @brief 机械臂重力补偿函数（适配 M8010 电机）
 * @note 
 *   肩部电机 = m8010_bus_1.motors[1]（大臂）
 *   肘部电机 = m8010_bus_1.motors[2]（小臂）
 * 
 *   target.T 的单位是 Nm（电机输出轴力矩），范围 -127.99 ~ 127.99
 * 
 *   标定方法：
 *     1. 将大臂手动摆到水平位置（joint_pitch[0] ≈ 0），小臂折叠到最小力矩姿态
 *     2. 逐渐增大 shoulder_torque_horizontal 直到大臂刚好不下坠，记录该值
 *     3. 将小臂手动摆到水平（joint_pitch[1] ≈ 0），大臂保持不动
 *     4. 逐渐增大 elbow_torque_horizontal 直到小臂刚好不下坠，记录该值
 *     5. 将这两个值填入下面的宏定义
 * 
 * @warning 该函数必须在 m8010_position_control() 之后调用，
 *          因为 m8010_position_control 内部会清零 target.T！
 */
 
/* === 实测标定值（单位：Nm，M8010 输出轴力矩）=== */
/* 实测后填入! */
#define SHOULDER_TORQUE_MAX   0.915f   // 大臂水平时肩部电机所需力矩 (Nm)
#define ELBOW_TORQUE_MAX      0.5f   // 小臂水平时肘部电机所需力矩 (Nm)

/**
 * @brief 对 M8010 电机进行重力补偿
 * @note 必须在 m8010_position_control() 之后调用
 */
void arm_gravity_comp(void)
{

    // --- 肘部电机（小臂，motors[2]）---
    // 只取决于小臂相对地面的绝对角度 joint_pitch[1]
    // joint_pitch[1] = 0 时（小臂水平），力矩最大
    // joint_pitch[1] = ±PI/2 时（小臂竖直），力矩≈0
    float elbow_torque = ELBOW_TORQUE_MAX * arm_cos_f32(arm.joint_pitch[1]);
    m8010_bus_1.motors[2].target.T = elbow_torque;

    // --- 肩部电机（大臂，motors[1]）---
    // 大臂自身的重力分量 + 小臂重力通过大臂传递的分量
    // joint_pitch[0] = 0 时（大臂水平），自身重力力矩最大
    float shoulder_self = SHOULDER_TORQUE_MAX * arm_cos_f32(arm.joint_pitch[0]);
    // 小臂重力也会作用在肩部关节上，取决于小臂绝对角度
    float shoulder_from_elbow = ELBOW_TORQUE_MAX * arm_cos_f32(arm.joint_pitch[1]);
    m8010_bus_1.motors[1].target.T = shoulder_self + shoulder_from_elbow;
}
