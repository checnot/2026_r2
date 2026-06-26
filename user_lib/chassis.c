#include "chassis.h"

chassis_stu chassis_r2;//r2底盘结构体
chassis_route_traj_stu chassis_route_traj;

/**
 * @brief 轨迹控制中的符号函数，返回输入值的符号
 * @param val: 输入值
 * @return: 符号 (1.0f, -1.0f, 或 0.0f)
 */
static inline float traj_sign2(float val) {
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}
/**
 * @brief 一阶低通滤波函数（用于速度平滑）
 * @param current_value 当前值（PID输出）
 * @param last_filtered_value 上次滤波后的值
 * @param alpha 滤波系数 (0-1)
 * @return 滤波后的值
 */
float chassis_velocity_lpf(float current_value, float last_filtered_value, float alpha)
{
    float filter_speed = 0.0f;
    if (last_filtered_value == 0.0f) filter_speed = CHASSIS_ZERO_MUTATION_ALPHA * current_value + (1.0f - CHASSIS_ZERO_MUTATION_ALPHA) * last_filtered_value;
    else filter_speed = alpha * current_value + (1.0f - alpha) * last_filtered_value;
//    filter_speed = alpha * current_value + (1.0f - alpha) * last_filtered_value;
    
    return filter_speed;
}
/**
 * @brief 底盘世界坐标系速度转换函数，根据输入的世界坐标系速度计算底盘坐标系速度
 * @param chassis 底盘结构体指针
 * @param vx_world 世界坐标系前向速度   
 * @param vy_world 世界坐标系横向速度
 * @param vw_world 世界坐标系旋转速度
 * @param robot_yaw 机器人当前朝向角，单位度
 * @warning 该函数在底盘运动学计算前调用，确保底盘坐标系与世界坐标系的转换正确
 */
void chassis_world_to_robot_velocity(chassis_stu *chassis, float vx_world, float vy_world, float vw_world, float robot_yaw)
{
    float robot_yaw_rad = robot_yaw * PI / 180.0f; // 将朝向角转换为弧度    
    // 计算底盘坐标系速度
    chassis->vx = vx_world * arm_cos_f32(robot_yaw_rad) + vy_world * arm_sin_f32(robot_yaw_rad);
    chassis->vy = -vx_world * arm_sin_f32(robot_yaw_rad) + vy_world * arm_cos_f32(robot_yaw_rad);
    chassis->vw = vw_world; // 旋转速度不受坐标系影响
}

/**
 * @brief 全向底盘运动学计算函数，根据输入的vx、vy、vw计算底盘各电机的目标速度
 * @param chassis 底盘结构体指针
 * @param vx 前向速度
 * @param vy 横向速度
 * @param vw 旋转速度
 * @warning 该函数全程使用机器人坐标系，并且单位未定义----电机id:右前为id1，右后为id2，左后为id3，左前为id4
*/
void omni_chassis_calculate(chassis_stu *chassis, float vx, float vy, float vw)
{
   
    chassis->vx = vx;
    chassis->vy = vy;
    chassis->vw = vw;
    //根据全向轮运动学模型计算各轮速度
    chassis->wheel_speed[0] = (int16_t)(vx - vy - vw);
    chassis->wheel_speed[1] = (int16_t)(-vx - vy - vw);
    chassis->wheel_speed[2] = (int16_t)( -vx + vy - vw);
    chassis->wheel_speed[3] = (int16_t)( vx + vy - vw);

    m3508_speed_control(&m3508_fdcan3[0], chassis->wheel_speed[0], 8192);
    m3508_speed_control(&m3508_fdcan3[1], chassis->wheel_speed[1], 8192);
    m3508_speed_control(&m3508_fdcan3[2], chassis->wheel_speed[2], 8192);
    m3508_speed_control(&m3508_fdcan3[3], chassis->wheel_speed[3], 8192);
}
/*
*@brief 底盘速度逆解算函数，根据输入的各轮速度计算底盘的vx、vy、vw
*@param chassis 底盘结构体指针  
*@param m3508_fdcan3 底盘电机结构体数组指针，包含四个电机的当前速度信息
*/
void omni_chassis_inverse_calculate(chassis_stu *chassis, M3508_Motor_Struct * m3508_fdcan3)
{
    //根据全向轮运动学模型计算底盘速度
    chassis->real_vx = -(m3508_fdcan3[0].Now_Speed + m3508_fdcan3[1].Now_Speed - m3508_fdcan3[2].Now_Speed - m3508_fdcan3[3].Now_Speed) / 4.0f;
    chassis->real_vy = (-m3508_fdcan3[0].Now_Speed + m3508_fdcan3[1].Now_Speed + m3508_fdcan3[2].Now_Speed - m3508_fdcan3[3].Now_Speed) / 4.0f;
    chassis->real_vw = -(m3508_fdcan3[0].Now_Speed + m3508_fdcan3[1].Now_Speed + m3508_fdcan3[2].Now_Speed + m3508_fdcan3[3].Now_Speed) / 4.0f;
}

// /**
//  * @brief 底盘前端升降控制函数，根据输入的升降状态控制底盘前端气缸的开合
//  * @param lift_front 前端升降状态，0代表收回，1代表推出
//  */
// void chassis_lift_front_control(uint8_t lift_front)
// {
//     if (lift_front)
//     {
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); // 打开前端气缸
//     }
//     else
//     {
//         HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); // 关闭前端气缸
//     }
// }
// /**
//  * @brief 底盘后端升降控制函数，根据输入的升降状态控制底盘后端气缸的开合
//  * @param lift_back 后端升降状态，0代表收回，1代表推出
//  */
// void chassis_lift_back_control(uint8_t lift_back)
// {
//     if (lift_back)
//     {
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); // 打开后端气缸
//     }
//     else
//     {
//         HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // 关闭后端气缸
//     }
// }   
// /**
//  * @brief 底盘升降控制函数，根据输入的升降状态控制底盘前后端气缸的开合
//  * @param lift_mode 同时升降与前后分开升降两种模式,0代表同时升降，1代表前后分开升降
//  * @param lift_front_flag 前端升降状态，0代表收回，1代表推出
//  * @param lift_back_flag 后端升降状态，0代表收回，1代表推出
//  * @param lift_whole_flag 同时升降模式下的升降状态，0代表收回，1代表推出
//  */
// void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag)
// {
//     if (lift_mode == 0) // 同时升降模式
//     {
//         chassis_lift_front_control(lift_whole_flag);
//         chassis_lift_back_control(lift_whole_flag);
//     }
//     else // 前后分开升降模式
//     {
//         chassis_lift_front_control(lift_front_flag);
//         chassis_lift_back_control(lift_back_flag);
//     }
// }


/**
 * @brief 底盘路径轨迹实例初始化
 * 
 */
void chassis_route_traj_init(void)
{
    Traj_Adaptive_Init(&chassis_route_traj.traj_vx, Radar_Pos.reliable_x, Radar_Pos.reliable_x, CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD);
    Traj_Adaptive_Init(&chassis_route_traj.traj_vy, Radar_Pos.reliable_y, Radar_Pos.reliable_y, CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD);
    Traj_Adaptive_Init(&chassis_route_traj.traj_vw, Radar_Pos.reliable_yaw , Radar_Pos.reliable_yaw , CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_ANGLE_ERROR_THRESHOLD);
    
    // PID参数初始化，针对最终接近目标时的微调，输出限制在较小范围内    
    PID_Set(&chassis_route_traj.pid_final_vx,12.0f, 0.0f, 5.0f);      
    PID_Set(&chassis_route_traj.pid_final_vy,12.0f, 0.0f, 5.0f);
    PID_Set(&chassis_route_traj.pid_final_vw, 110.0f, 0.0f, 5.0f);
}
/*
 *@brief 角度控制边界保护函数，确保在角度接近±180度时，误差计算能够正确处理环绕问题
 *@param target_yaw: 目标角度，单位度
 *@param now_yaw: 当前角度，单位度 
 *@return: 计算得到的误差值，单位度，范围在[-180, 180]之间  
 */
float chassis_angle_edge_protection(float target_yaw,float now_yaw)
{
    float error1= target_yaw - now_yaw;
    float error2 = error1 > 0 ? error1 - 360.0f : error1 + 360.0f; // 计算环绕后的误差  
    float error = fabsf(error1) < fabsf(error2) ? error1 : error2; // 选择绝对值较小的误差作为最终误差  
    
    return error;
}
/*
*@brief 路径完成判定与锁定函数(由于x、y轴)
*@param traj: 轨迹控制实例
*@param actual_pos: 真实物理位置
*@note 当判定完成时，强制锁定目标位置并刹停，防止微小误差引起的抖动
*/
void chassis_single_route_completion_check(Traj_Single_struct * traj, float actual_pos)
{
    float error = fabsf(traj->target_pos - actual_pos);
    if (error < CHASSIS_ROUTE_ARRIVAL_THRESHOLD) // 误差小于误差阈值则判定完成
    {
        traj->is_finished = 1;
        traj->out_pos = traj->target_pos; // 锁定目标位置
        traj->out_vel = 0.0f; // 刹停
    }
    else
    {
        traj->is_finished = 0;
    }
    
}
/*
*@brief 路径完成判定与锁定函数(由于w轴)
*@param traj: 轨迹控制实例
*@param actual_angle: 真实物理位置
*@note 当判定完成时，强制锁定目标位置并刹停，防止微小误差引起的抖动
*/
void chassis_single_route_completion_check_angle(Traj_Single_struct * traj, float actual_angle)
{
    float error = fabsf(chassis_angle_edge_protection(traj->target_pos, actual_angle));     
    // float error = fabsf(traj->target_pos - actual_angle);
    if (error < CHASSIS_ANGLE_ARRIVAL_THRESHOLD) // 误差小于误差阈值则判定完成
    {
        traj->is_finished = 1;
        // traj->out_pos = traj->target_pos; // 锁定目标位置
        traj->out_vel = 0.0f; // 刹停
    }
     else
    {
        traj->is_finished = 0;
    }
}
/*
*@brief 扰动自适应与补救策略
* @param traj:        轨迹控制实例
* @param actual_pos:  真实物理位置
*/
void chassis_disturbance_adaptive(Traj_Single_struct * traj, float actual_pos)
{
    // 计算当前跟踪误差
    float error = fabsf(traj->out_pos - actual_pos);
    
    // 检测是否超过最大跟踪误差阈值，进入补救模式
    if (error > traj->max_tracking_error )
    {   traj->is_remedy_mode = 1; 
        float dir_of_error = traj_sign2(traj->out_pos - actual_pos);
        traj->out_pos = actual_pos + dir_of_error * (traj->max_tracking_error * 0.9f); 
        traj->out_vel = 0.0f; // 完全刹停，等待实体
        return;               
    } else {
        traj->is_remedy_mode = 0;
    }
}
/**
*@brief 各轴最大速度与最大加速度分配函数
*@param traj:轨迹控制实例 
*@param px:路程占比映射速度分配，取值范围0-1
*@note 该函数需要在单段路径调度器（chassis_single_route_scheduler）中计算完距离占比后调用，以动态调整每个轴的速度和加速度限制
*/
void chassis_velocity_acceleration_allocation(Traj_Single_struct * traj, float px, uint8_t slope_mode)
{
    traj->v_max = slope_mode ? CHASSIS_SLOPE_V_MAX : CHASSIS_V_MAX;
    traj->a_max = (slope_mode ? CHASSIS_SLOPE_A_MAX : CHASSIS_A_MAX) * px;
            if(traj->v_max < 0.001f) traj->v_max = 0.001f;
            if(traj->a_max < 0.001f) traj->a_max = 0.001f;
}

/**
 * @brief 设置底盘路径上坡模式（切换最大速度/加速度参数）
 * @param enable 1=上坡参数(CHASSIS_SLOPE_V/A_MAX), 0=恢复平地参数
 */
void chassis_route_set_slope_mode(uint8_t enable)
{
    chassis_route_traj.slope_mode = enable;
}

/*
*@brief 底盘单段路径速度推演函数
* @param traj: 轨迹控制实例
* @param now_position: 当前位置
* @param dt: 调用周期时长 (如 0.001f 代表 1ms)
*/
void chassis_single_route_velocity_update(Traj_Single_struct * traj,float now_position, float dt)
{
    float dist_to_target = fabsf(traj->target_pos-now_position);
    float dir = traj_sign2(traj->target_pos - traj->out_pos);
    float dir_final = traj_sign2(traj->target_pos-now_position);
    float expected_vel =0; 
    // // [新增] 方向保护：如果正在反向运行（上位机突然反向更改点位），必须先减速到 0
    // if (traj->out_vel * dir < -0.001f) {
    //     // 让当前速度按照最大减速度逼近 0
    //     expected_vel = fabsf(traj->out_vel) - traj->a_max * dt;
    //     if (expected_vel < 0.0f) expected_vel = 0.0f; 
    //     // 维持之前的方向极性，只是把速度降下来
    //     traj->out_vel = expected_vel * traj_sign2(traj->out_vel); 
    // }
    // else {
    //     // 方向一致，可以正常执行加速/匀减速逻辑
    //     // 距离过于极小则不再计算 (防平方根附近计算奇点抖动)
    //     if (dist_to_target < (traj->v_max * dt * 0.1f)) {
    //         traj->out_vel = 0.0f;
    //         traj->out_pos = traj->target_pos; // 顺带对齐防止微小偏差
    //         return;
    //     }
    // }
    float safe_decel_vel = sqrtf(2.0f * traj->a_max * dist_to_target);
        
        // 我们渴望本周期的速度 (上一周期速度 + 匀加速)
        float desire_v_delta = traj->a_max * dt;
        expected_vel = fabsf(traj->out_vel) + desire_v_delta;
        
        // 限制1：不能超过给定的总最大速度 
        if (expected_vel > traj->v_max) {
            expected_vel = traj->v_max;
        }
        
        // 限制2：不能超过到达终点的安全刹车速度 
        if (expected_vel > safe_decel_vel) {
            expected_vel = safe_decel_vel;
        }
       
    // 把推算出的速度附加方向符号，得到真正输出速度
        traj->out_vel = expected_vel * dir_final;
}

/**
 * @brief 底盘多段路径调度器
 * @note 该函数需要在单段路径调度器（chassis_single_route_scheduler）之前调用   
 */
void chassis_multi_route_scheduler(chassis_route_traj_stu* chassis_route_traj,RADAR_POSITION * chassis_real_position)
{
    // 如果任意一个轴的队列为空，说明点位不齐，不进行出队（保证X/Y/W同步出队）
    if(chassis_route_traj->traj_queue_vx.count == 0 || chassis_route_traj->traj_queue_vy.count == 0 || chassis_route_traj->traj_queue_vw.count == 0) return;

// 新增延时逻辑：如果还有延时没等完，就原地return，不下发下一个点
    if (chassis_route_traj->wait_counter > 0)
    {
        chassis_route_traj->wait_counter--;
        return;  
    }
    // 如果当前轨迹已经完成，并且队列中还有待执行的轨迹点，则切换到下一个轨迹点
    if (chassis_route_traj->traj_vx.is_finished && chassis_route_traj->traj_vy.is_finished && chassis_route_traj->traj_vw.is_finished) 
    {
        // 从队列头部取出下一个目标点
        chassis_route_traj->traj_vx.target_pos = chassis_route_traj->traj_queue_vx.points[chassis_route_traj->traj_queue_vx.tail];
        chassis_route_traj->traj_vy.target_pos = chassis_route_traj->traj_queue_vy.points[chassis_route_traj->traj_queue_vy.tail];
        chassis_route_traj->traj_vw.target_pos = chassis_route_traj->traj_queue_vw.points[chassis_route_traj->traj_queue_vw.tail];
        // 提取该点的预设延时，并赋给计数器
        uint16_t next_delay = chassis_route_traj->delay_queue[chassis_route_traj->traj_queue_vx.tail];
        chassis_route_traj->target_delay_ms = next_delay;
        chassis_route_traj->wait_counter = next_delay; // 为到达这个点后的停顿上满发条！
        
        // 队列出队，注意这里是循环队列
        chassis_route_traj->traj_queue_vx.tail = (chassis_route_traj->traj_queue_vx.tail + 1) % TRAJ_MAX_WAYPOINTS;
        chassis_route_traj->traj_queue_vy.tail = (chassis_route_traj->traj_queue_vy.tail + 1) % TRAJ_MAX_WAYPOINTS;
        chassis_route_traj->traj_queue_vw.tail = (chassis_route_traj->traj_queue_vw.tail + 1) % TRAJ_MAX_WAYPOINTS;
        chassis_route_traj->traj_queue_vx.count--;
        chassis_route_traj->traj_queue_vy.count--;
        chassis_route_traj->traj_queue_vw.count--;

        chassis_route_traj->traj_vx.is_finished = 0; // 切换到新目标后重置完成标志  
        chassis_route_traj->traj_vy.is_finished = 0;        
        chassis_route_traj->traj_vw.is_finished = 0;

        // // 复位 PID 状态，防止上次累积的 I 项引发跳变
        // PID_Reset(&chassis_route_traj->pid_final_vx);
        // PID_Reset(&chassis_route_traj->pid_final_vy);
        // PID_Reset(&chassis_route_traj->pid_final_vw);
    }
    return;
}
/**
 * @brief 底盘路径点添加函数
 * @param chassis_route_traj: 底盘路径轨迹实例指针
 * @param target_vx: 前向目标点
 * @param target_vy: 横向目标点
 * @param target_vw: 旋转角度目标点  
 * @param delay_time_ms: 到达该目标点后需要等待的时间，单位毫秒
 * @note 该函数会将新的目标点添加到路径队列中，等待调度器切换执行
 */
 void chassis_route_point_add(chassis_route_traj_stu* chassis_route_traj, float target_vx, float target_vy, float target_vw, uint16_t delay_time_ms)
 {
      
      // 将x的head作为其他元素的索引，进行延时和气泵标志的存储
    if (chassis_route_traj->traj_queue_vx.count >= TRAJ_MAX_WAYPOINTS) return;
    // 将延时信息存储到轨迹实例中，和点位队列下标同步
    chassis_route_traj->delay_queue[chassis_route_traj->traj_queue_vx.head] = delay_time_ms;
    // 将坐标压入各自队列
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vx, target_vx);
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vy, target_vy);
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vw, target_vw);
 }  
 
/*
*@brief 角度控制PID，为解决±180度边界问题，专门设计了一个角度PID计算函数，在误差计算时调用chassis_angle_edge_protection进行边界保护
*@param pid: PID实例指针
*@param measure: 当前测量值，单位度 
*@param target: 目标值，单位度  
*@param max_output: PID输出限制
*@return: PID计算结果
*/
float chassis_angle_pid_calculate(PID_Struct * PID, float measure, float target, float max_output)
{
    float error = chassis_angle_edge_protection(target, measure);
    PID->Measure = measure;
    PID->Target = target;
    PID->Error = error;
    
    PID->P_out = PID->Kp * PID->Error;
	PID->I_out += PID->Ki * ((PID->Error + PID->LastError)/2);//梯形积分
	PID->I_out = PID->I_out > (800 ? 800 : PID->I_out);//积分限幅
	PID->D_out = PID->Kd * (PID->LastMeasure - PID->Measure);//微分项使用测量值的变化率，减少微分环节对目标值突变的敏感性
	PID->Output = PID->P_out + PID->I_out + PID->D_out;

	PID->LastMeasure = PID->Measure;
	PID->LastLastError = PID->LastError;
	
	PID->LastError = PID->Error;

	if(PID->Output > max_output)
		{PID->Output = max_output;}
	else if(PID->Output < -max_output)
		{PID->Output = -max_output;}

	return PID->Output;
}
/**
 * @brief 直线插补单段路径调度器
 * @note 必须放在 Traj_Adaptive_Update(x)、Traj_Adaptive_Update(y)、Traj_Adaptive_Update(w) 之前调用
 */
void chassis_single_route_scheduler(RADAR_POSITION * chassis_real_position, chassis_route_traj_stu* chassis_route_traj,float dt)
{
    
    // === 1. 计算当前目标距离 ===
    float dist_to_actual_vx = fabsf(chassis_route_traj->traj_vx.target_pos - chassis_real_position->reliable_x);
    float dist_to_out_vx = fabsf(chassis_route_traj->traj_vx.target_pos - chassis_route_traj->traj_vx.out_pos);
    float dist_to_actual_vy = fabsf(chassis_route_traj->traj_vy.target_pos - chassis_real_position->reliable_y);
    float dist_to_out_vy = fabsf(chassis_route_traj->traj_vy.target_pos - chassis_route_traj->traj_vy.out_pos);
    // float dist_to_actual_vw = fabsf(chassis_route_traj->traj_vw.target_pos - chassis_real_position->reliable_yaw);
    // float dist_to_actual_vw = fabsf(chassis_angle_edge_protection(chassis_route_traj->traj_vw.target_pos, chassis_real_position->reliable_yaw));
    // float dist_to_out_vw = fabsf(chassis_route_traj->traj_vw.target_pos - chassis_route_traj->traj_vw.out_pos);
    // uint8_t x_integral_flag = 1;// 积分使能标志，初始为1，表示允许速度积分
    // uint8_t y_integral_flag = 1;    
    // uint8_t w_integral_flag = 1;
    // === 最终接近：2D合成距离统一门控，确保两轴同时进入PID ===
    float dist_2d_out = sqrtf(dist_to_out_vx * dist_to_out_vx + dist_to_out_vy * dist_to_out_vy);
    if (dist_2d_out < CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD &&
        (chassis_route_traj->traj_vx.is_finished == 0 || chassis_route_traj->traj_vy.is_finished == 0))
    {
        PID_Calculate(&chassis_route_traj->pid_final_vx, chassis_real_position->reliable_x, 
                      chassis_route_traj->traj_vx.target_pos, CHASSIS_V_MAX / 3.0f);
        PID_Calculate(&chassis_route_traj->pid_final_vy, chassis_real_position->reliable_y, 
                      chassis_route_traj->traj_vy.target_pos, CHASSIS_V_MAX / 3.0f);

        // 一阶低通滤波平滑 PID 输出
        chassis_route_traj->traj_vx.out_vel = chassis_velocity_lpf(
            chassis_route_traj->pid_final_vx.Output,
            chassis_route_traj->traj_vx.out_vel,
            CHASSIS_VX_FILTER_ALPHA);
        chassis_route_traj->traj_vy.out_vel = chassis_velocity_lpf(
            chassis_route_traj->pid_final_vy.Output,
            chassis_route_traj->traj_vy.out_vel,
            CHASSIS_VY_FILTER_ALPHA);
    }
    if (chassis_route_traj->traj_vw.is_finished == 0) // 角度纯靠PID调节
    {
            chassis_angle_pid_calculate(&chassis_route_traj->pid_final_vw, chassis_real_position->reliable_yaw, chassis_route_traj->traj_vw.target_pos, CHASSIS_V_MAX/2.0f); // 计算PID输出，限制在较小范围内   
         //chassis_route_traj->traj_vw.out_vel = chassis_route_traj->pid_final_vw.Output; 
        //  使用一阶低通滤波平滑PID输出
    chassis_route_traj->traj_vw.out_vel = chassis_velocity_lpf(
        chassis_route_traj->pid_final_vw.Output,
        chassis_route_traj->traj_vw.out_vel,
        CHASSIS_VW_FILTER_ALPHA
    );
    }
    // if(dist_to_out_vw<CHASSIS_ROUTE_FINAL_APPROACH_ANGLE_THRESHOLD) 
    // {
    //     chassis_angle_pid_calculate(&chassis_route_traj->pid_final_vw, chassis_real_position->reliable_yaw, chassis_route_traj->traj_vw.target_pos, CHASSIS_V_MAX/5.0f); // 计算PID输出，限制在较小范围内   
    //     // PID_Calculate(&chassis_route_traj->pid_final_vw, chassis_real_position->reliable_yaw, chassis_route_traj->traj_vw.target_pos, CHASSIS_V_MAX/5.0f); // 计算PID输出，限制在较小范围内
    //     chassis_route_traj->traj_vw.out_vel = chassis_route_traj->pid_final_vw.Output; // 用PID输出修正速度，进行微调
    // }
     // === 2. 如果已经判定到达，强制锁定目标防止晃动 ===
    chassis_single_route_completion_check(&chassis_route_traj->traj_vx, chassis_real_position->reliable_x);
    chassis_single_route_completion_check(&chassis_route_traj->traj_vy, chassis_real_position->reliable_y);
    chassis_single_route_completion_check_angle(&chassis_route_traj->traj_vw, chassis_real_position->reliable_yaw);

    if(chassis_route_traj->traj_vx.is_finished && chassis_route_traj->traj_vy.is_finished && chassis_route_traj->traj_vw.is_finished) 
    {
        return;
    } 
    // ===3.扰动自适应与补救策略（如果跟踪误差过大，先让目标影子回到合理范围内，等待实体追上来）===
    if(dist_to_out_vx>CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD)chassis_disturbance_adaptive(&chassis_route_traj->traj_vx, chassis_real_position->reliable_x);
    if(dist_to_out_vy>CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD) chassis_disturbance_adaptive(&chassis_route_traj->traj_vy, chassis_real_position->reliable_y);
    // chassis_disturbance_adaptive(&chassis_route_traj->traj_vw, chassis_real_position->reliable_yaw);
    // === 4. 速度推演 ===
    // float dis_total =   dist_to_actual_vx + dist_to_actual_vy + dist_to_actual_vw * DISTANCE_WHEEL_CENTER*1000.0f; 
    // float rate_vx = (dist_to_actual_vx + 0.001f) / (dis_total + 0.003f); // 加小常数避免除零
    // float rate_vy = (dist_to_actual_vy + 0.001f) / (dis_total + 0.003f);
    // float rate_vw = (dist_to_actual_vw * DISTANCE_WHEEL_CENTER*1000.0f + 0.001f) / (dis_total + 0.003f);
    float dis_total =  sqrtf(dist_to_actual_vx * dist_to_actual_vx + dist_to_actual_vy * dist_to_actual_vy);
    float rate_vx = (dist_to_actual_vx + 0.001f) / (dis_total + 0.002f); // 加小常数避免除零
    float rate_vy = (dist_to_actual_vy + 0.001f) / (dis_total + 0.002f);

    // ===4. 统一2D直线速度规划 ===
    // 不再对X/Y独立规划，而是基于2D合成距离做统一速度推演，再按比例分解
    // 保证 vx:vy ≡ dx:dy，路径始终为直线
    if (chassis_route_traj->traj_vx.is_finished == 0 || chassis_route_traj->traj_vy.is_finished == 0)
    {
        // 2D合成"影子"到目标距离（用于门控）
        float dist_2d_out = sqrtf(dist_to_out_vx * dist_to_out_vx + dist_to_out_vy * dist_to_out_vy);

        if (dist_2d_out >= CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD)
        {
            float v_max_2d = chassis_route_traj->slope_mode ? CHASSIS_SLOPE_V_MAX : CHASSIS_V_MAX;
            float a_max_2d = chassis_route_traj->slope_mode ? CHASSIS_SLOPE_A_MAX : CHASSIS_A_MAX;

            // 方向
            float dir_x = traj_sign2(chassis_route_traj->traj_vx.target_pos - chassis_route_traj->traj_vx.out_pos);
            float dir_y = traj_sign2(chassis_route_traj->traj_vy.target_pos - chassis_route_traj->traj_vy.out_pos);

            // 安全刹车速度（基于实际2D距离）
            float safe_decel_vel = sqrtf(2.0f * a_max_2d * dis_total);
            if (safe_decel_vel < 0.001f) safe_decel_vel = 0.001f;

            // 当前合成速度标量
            float cur_speed_2d = sqrtf(chassis_route_traj->traj_vx.out_vel * chassis_route_traj->traj_vx.out_vel +
                                       chassis_route_traj->traj_vy.out_vel * chassis_route_traj->traj_vy.out_vel);

            // 期望速度 = 当前速度 + 匀加速增量
            float expected_vel = cur_speed_2d + a_max_2d * dt;

            if (expected_vel > v_max_2d)      expected_vel = v_max_2d;
            if (expected_vel > safe_decel_vel) expected_vel = safe_decel_vel;

            // 按距离比例分解到X/Y
            chassis_route_traj->traj_vx.out_vel = expected_vel * rate_vx * dir_x;
            chassis_route_traj->traj_vy.out_vel = expected_vel * rate_vy * dir_y;
        }
    }
    // ===5.速度积分===
      chassis_route_traj->traj_vx.out_pos = chassis_route_traj->traj_vx.out_pos + chassis_route_traj->traj_vx.out_vel * 0.001f;       
     chassis_route_traj->traj_vy.out_pos = chassis_route_traj->traj_vy.out_pos + chassis_route_traj->traj_vy.out_vel * 0.001f;
    // if(w_integral_flag) chassis_route_traj->traj_vw.out_pos = chassis_route_traj->traj_vw.out_pos + chassis_route_traj->traj_vw.out_vel / DISTANCE_WHEEL_CENTER * 0.001f;   
}







/* ========================================== 以下为底盘抬升部分 ========================================== */
/* ========================================== 以下为底盘抬升部分 ========================================== */
/* ========================================== 以下为底盘抬升部分 ========================================== */



// 底盘升降轨迹实例
chassis_lift_traj_stu chassis_lift_traj;

/**
 * @brief 底盘升降正运动学更新函数，根据电机位置计算当前升降位置
 * @param chassis 底盘结构体指针，包含升降位置变量
*/
void chassis_lift_fk_update(chassis_stu *chassis)
{
    // 这里的正负号和机械结构有关，可能需要根据实际情况调整
    chassis->lift_front_current = -m8010_bus_3.motors[0].measure.Pos*CHASSIS_LIFT_RATE;
    chassis->lift_back_current = m8010_bus_3.motors[1].measure.Pos*CHASSIS_LIFT_RATE;
}

/**
 * @brief 底盘升降逆运动学更新函数，根据目标升降位置计算需要下发的电机位置
 * @param chassis 底盘结构体指针，包含当前升降位置和目标升降位置变量
*/
void chassis_lift_ik_update(chassis_stu *chassis)
{
    m8010_bus_3.motors[0].target.Pos = -chassis->lift_front_target/CHASSIS_LIFT_RATE;
    m8010_bus_3.motors[1].target.Pos = chassis->lift_back_target/CHASSIS_LIFT_RATE;
}

/**
 * @brief 底盘升降轨迹初始化（入口处调用一次）
 * @note 调用前需要确保电机编码器已初始化，chassis->lift_front_current/lift_back_current 已从电机读取
 */
void chassis_lift_traj_init(void)
{
    // 假设 lift_xxx_current 已经在状态更新中赋值
    Traj_Adaptive_Init(&chassis_lift_traj.traj_front, 
        chassis_r2.lift_front_current, chassis_r2.lift_front_current,
        CHASSIS_LIFT_V_MAX, CHASSIS_LIFT_A_MAX, CHASSIS_LIFT_TRACKING_ERROR_THRESHOLD);
    Traj_Path_Queue_Init(&chassis_lift_traj.traj_queue_front, CHASSIS_LIFT_QUEUE_BLEND_RADIUS);

    Traj_Adaptive_Init(&chassis_lift_traj.traj_back, 
        chassis_r2.lift_back_current, chassis_r2.lift_back_current,
        CHASSIS_LIFT_V_MAX, CHASSIS_LIFT_A_MAX, CHASSIS_LIFT_TRACKING_ERROR_THRESHOLD);
    Traj_Path_Queue_Init(&chassis_lift_traj.traj_queue_back, CHASSIS_LIFT_QUEUE_BLEND_RADIUS);

    chassis_lift_traj.target_delay_ms = 0;
    chassis_lift_traj.wait_counter = 0;
}

/**
 * @brief 底盘升降直线插补多段路径调度器（核心）
 * @note 放在 Traj_Adaptive_Update 之前调用，和 arm_linear_interpolation_update 逻辑一致
 * @note 1ms 周期调用
 */
void chassis_lift_linear_interpolation_update(chassis_stu *chassis, chassis_lift_traj_stu *lift_traj, float blend_radius)
{
    if (blend_radius < 0.001f) blend_radius = 0.001f;

    // 队列为空则不处理
    if (lift_traj->traj_queue_front.count == 0 || lift_traj->traj_queue_back.count == 0) return;

    // === 延时倒计时 ===
    if (lift_traj->wait_counter > 0) {
        lift_traj->wait_counter--;
        return;
    }

    // === 两轴判敛 ===
    float dist_act_f = fabsf(lift_traj->traj_front.target_pos - chassis->lift_front_current);
    float dist_out_f = fabsf(lift_traj->traj_front.target_pos - lift_traj->traj_front.out_pos);
    uint8_t ready_f = lift_traj->traj_front.is_finished || 
        (blend_radius > 0.001f && dist_act_f <= blend_radius && dist_out_f <= blend_radius);

    float dist_act_b = fabsf(lift_traj->traj_back.target_pos - chassis->lift_back_current);
    float dist_out_b = fabsf(lift_traj->traj_back.target_pos - lift_traj->traj_back.out_pos);
    uint8_t ready_b = lift_traj->traj_back.is_finished || 
        (blend_radius > 0.001f && dist_act_b <= blend_radius && dist_out_b <= blend_radius);

    // === 联合抛点 ===
    if (ready_f && ready_b) {
        float next_f = lift_traj->traj_queue_front.points[lift_traj->traj_queue_front.tail];
        float next_b = lift_traj->traj_queue_back.points[lift_traj->traj_queue_back.tail];

        uint16_t next_delay = lift_traj->delay_queue[lift_traj->traj_queue_front.tail];
        lift_traj->target_delay_ms = next_delay;
        lift_traj->wait_counter = next_delay;

        // 出队
        lift_traj->traj_queue_front.tail = (lift_traj->traj_queue_front.tail + 1) % TRAJ_MAX_WAYPOINTS;
        lift_traj->traj_queue_front.count--;
        lift_traj->traj_queue_back.tail = (lift_traj->traj_queue_back.tail + 1) % TRAJ_MAX_WAYPOINTS;
        lift_traj->traj_queue_back.count--;

        // ========== 同步到达时间速度分配 ==========
        float delta_f = fabsf(next_f - chassis->lift_front_current);
        float delta_b = fabsf(next_b - chassis->lift_back_current);
        // 防除零
        if (delta_f < 0.0001f) delta_f = 0.0001f;
        if (delta_b < 0.0001f) delta_b = 0.0001f;
        // 以行程较长的轴为基准，短轴按比例减速
        float dist_max = (delta_f > delta_b) ? delta_f : delta_b;
        float pf = delta_f / dist_max;  // ≤ 1
        float pb = delta_b / dist_max;  // ≤ 1

        // 下发前端
        lift_traj->traj_front.v_max = CHASSIS_LIFT_V_MAX * pf;
        lift_traj->traj_front.a_max = CHASSIS_LIFT_A_MAX * pf;
        lift_traj->traj_front.max_tracking_error = CHASSIS_LIFT_TRACKING_ERROR_THRESHOLD * pf;
        if (lift_traj->traj_front.v_max < 0.001f) lift_traj->traj_front.v_max = 0.001f;
        if (lift_traj->traj_front.a_max < 0.001f) lift_traj->traj_front.a_max = 0.001f;
        if (lift_traj->traj_front.max_tracking_error < 0.001f) lift_traj->traj_front.max_tracking_error = 0.001f;
        lift_traj->traj_front.target_pos = next_f;
        lift_traj->traj_front.is_finished = 0;
        // 下发后端
        lift_traj->traj_back.v_max = CHASSIS_LIFT_V_MAX * pb;
        lift_traj->traj_back.a_max = CHASSIS_LIFT_A_MAX * pb;
        lift_traj->traj_back.max_tracking_error = CHASSIS_LIFT_TRACKING_ERROR_THRESHOLD * pb;
        if (lift_traj->traj_back.v_max < 0.001f) lift_traj->traj_back.v_max = 0.001f;
        if (lift_traj->traj_back.a_max < 0.001f) lift_traj->traj_back.a_max = 0.001f;
        if (lift_traj->traj_back.max_tracking_error < 0.001f) lift_traj->traj_back.max_tracking_error = 0.001f;
        lift_traj->traj_back.target_pos = next_b;
        lift_traj->traj_back.is_finished = 0;
    
    }

}

/**
 * @brief 底盘升降添加路径点
 * @param front_height 前端目标高度（m）
 * @param back_height  后端目标高度（m）
 * @param delay_ms     到达后延时（ms）
 * @note 带有边界保护，限制在机械行程范围内
 * @note 前抬升最高 0.405m，最低 -0.029m；后抬升最高 0.087m，最低 -0.440m
 */
void chassis_lift_queue_add_point(chassis_lift_traj_stu *lift_traj, float front_height, float back_height, uint16_t delay_ms)
{
    // 边界保护--参数待测试
    if (front_height < LIFTER_FRONT_LOWEST) front_height = LIFTER_FRONT_LOWEST;// 前抬升最低 -0.029m
    if (front_height > LIFTER_FRONT_HIGHEST) front_height = LIFTER_FRONT_HIGHEST; // 前抬升最高 0.405m
    if (back_height < LIFTER_BACK_LOWEST) back_height = LIFTER_BACK_LOWEST;  // 后抬升最低 -0.440m
    if (back_height > LIFTER_BACK_HIGHEST) back_height = LIFTER_BACK_HIGHEST;   // 后抬升最高 0.087m
    if (delay_ms > 3000) delay_ms = 3000;

    if (lift_traj->traj_queue_front.count >= TRAJ_MAX_WAYPOINTS) return;

    lift_traj->delay_queue[lift_traj->traj_queue_front.head] = delay_ms;

    Traj_Path_Queue_AddPoint(&lift_traj->traj_queue_front, front_height);
    Traj_Path_Queue_AddPoint(&lift_traj->traj_queue_back, back_height);
}

/**
 * @brief 底盘抬升执行函数，并下发电机位置环目各参数
 * @warning 位置环目标位置应由 chassis_lift_ik_update 计算，该函数仅负责执行控制，确保调用顺序正确以避免数据不一致
*/
void chassis_lift_excute(chassis_stu *chassis, float kp, float kw)
{
    m8010_position_control(&m8010_bus_3.motors[0], m8010_bus_3.motors[0].target.Pos, kp, kw);
    m8010_position_control(&m8010_bus_3.motors[1], m8010_bus_3.motors[1].target.Pos, kp, kw);
}

/**
 * @brief 底盘升降轮速控制函数，提供给路径调度器调用以实现速度控制
*/
void chassis_lift_wheel_excute(float target_speed, float speed_kp, float speed_kd)
{
    m3508_parameter_init(&m3508_fdcan1[1], speed_kp, 0.0f, speed_kd, 0.0f, 0.0f, 0.0f);
    m3508_parameter_init(&m3508_fdcan1[2], speed_kp, 0.0f, speed_kd, 0.0f, 0.0f, 0.0f);
    m3508_speed_control(&m3508_fdcan1[1], (int16_t)target_speed, 3000);
    m3508_speed_control(&m3508_fdcan1[2], (int16_t)(-target_speed), 3000);
}

