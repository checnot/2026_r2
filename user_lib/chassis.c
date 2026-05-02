#include "chassis.h"

chassis_stu chassis_r2;//r2底盘结构体
chassis_route_traj_stu chassis_route_traj;
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
    chassis->wheel_speed[0] = (int16_t)(-vx - vy - vw);
    chassis->wheel_speed[1] = (int16_t)(-vx + vy - vw);
    chassis->wheel_speed[2] = (int16_t)( vx + vy - vw);
    chassis->wheel_speed[3] = (int16_t)( vx - vy - vw);
    m3508_speed_control(&m3508_fdcan3[0], chassis->wheel_speed[0], 8192);
    m3508_speed_control(&m3508_fdcan3[1], chassis->wheel_speed[1], 8192);
    m3508_speed_control(&m3508_fdcan3[2], chassis->wheel_speed[2], 8192);
    m3508_speed_control(&m3508_fdcan3[3], chassis->wheel_speed[3], 8192);
}

/**
 * @brief 底盘前端升降控制函数，根据输入的升降状态控制底盘前端气缸的开合
 * @param lift_front 前端升降状态，0代表收回，1代表推出
 */
void chassis_lift_front_control(uint8_t lift_front)
{
    if (lift_front)
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET); // 打开前端气缸
    }
    else
    {
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET); // 关闭前端气缸
    }
   
}
/**
 * @brief 底盘后端升降控制函数，根据输入的升降状态控制底盘后端气缸的开合
 * @param lift_back 后端升降状态，0代表收回，1代表推出
 */
void chassis_lift_back_control(uint8_t lift_back)
{
    if (lift_back)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET); // 打开后端气缸
    }
    else
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET); // 关闭后端气缸
    }
}   
/**
 * @brief 底盘升降控制函数，根据输入的升降状态控制底盘前后端气缸的开合
 * @param lift_mode 同时升降与前后分开升降两种模式,0代表同时升降，1代表前后分开升降
 * @param lift_front_flag 前端升降状态，0代表收回，1代表推出
 * @param lift_back_flag 后端升降状态，0代表收回，1代表推出
 * @param lift_whole_flag 同时升降模式下的升降状态，0代表收回，1代表推出
 */
void chassis_lift_control(uint8_t lift_mode, uint8_t lift_front_flag, uint8_t lift_back_flag, uint8_t lift_whole_flag)
{
    if (lift_mode == 0) // 同时升降模式
    {
        chassis_lift_front_control(lift_whole_flag);
        chassis_lift_back_control(lift_whole_flag);
    }
    else // 前后分开升降模式
    {
        chassis_lift_front_control(lift_front_flag);
        chassis_lift_back_control(lift_back_flag);
    }
}

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
 * @brief 底盘路径轨迹实例初始化
 * 
 */
void chassis_route_traj_init(void)
{
    Traj_Adaptive_Init(&chassis_route_traj.traj_vx, Radar_Pos.radar_x, 300.0f, CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD);
    Traj_Adaptive_Init(&chassis_route_traj.traj_vy, Radar_Pos.radar_y, 915.0f, CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD);
    Traj_Adaptive_Init(&chassis_route_traj.traj_vw, Radar_Pos.radar_yaw , 0.0f, CHASSIS_V_MAX, CHASSIS_A_MAX, CHASSIS_TRACKING_ANGLE_ERROR_THRESHOLD);
    
    // PID参数初始化，针对最终接近目标时的微调，输出限制在较小范围内    
    PID_Set(&chassis_route_traj.pid_final_vx,8.0f, 0.0f, 2.0f);      
    PID_Set(&chassis_route_traj.pid_final_vy,8.0f, 0.0f, 2.0f);
    PID_Set(&chassis_route_traj.pid_final_vw, 100.0f, 0.0f, 1.0f);
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
    float error = fabsf(traj->target_pos - actual_angle);
    if (error < CHASSIS_ANGLE_ARRIVAL_THRESHOLD) // 误差小于误差阈值则判定完成
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
void chassis_velocity_acceleration_allocation(Traj_Single_struct * traj, float px)
{
    traj->v_max = CHASSIS_V_MAX ;
    traj->a_max = CHASSIS_A_MAX * px;
            if(traj->v_max < 0.001f) traj->v_max = 0.001f;
            if(traj->a_max < 0.001f) traj->a_max = 0.001f;
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
        // 限制3：如果速度过小，强制提升速度，确保能够克服静摩擦并开始移动
        // while(expected_vel < CHASSIS_ADJUSE_MIN_SPEED) {
        //     expected_vel = expected_vel * 1.5f; // 以指数方式提升速度，快速脱离低速区
        // }
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

    // 如果当前轨迹已经完成，并且队列中还有待执行的轨迹点，则切换到下一个轨迹点
    if (chassis_route_traj->traj_vx.is_finished && chassis_route_traj->traj_vy.is_finished && chassis_route_traj->traj_vw.is_finished) 
    {
        // 从队列头部取出下一个目标点
        chassis_route_traj->traj_vx.target_pos = chassis_route_traj->traj_queue_vx.points[chassis_route_traj->traj_queue_vx.tail];
        chassis_route_traj->traj_vy.target_pos = chassis_route_traj->traj_queue_vy.points[chassis_route_traj->traj_queue_vy.tail];
        chassis_route_traj->traj_vw.target_pos = chassis_route_traj->traj_queue_vw.points[chassis_route_traj->traj_queue_vw.tail];
        
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
    }
    return;
}
/**
 * @brief 底盘路径点添加函数
 * @param chassis_route_traj: 底盘路径轨迹实例指针
 * @param target_vx: 前向速度目标点
 * @param target_vy: 横向速度目标点
 * @param target_vw: 旋转速度目标点  
 * @note 该函数会将新的目标点添加到路径队列中，等待调度器切换执行
 */
 void chassis_route_point_add(chassis_route_traj_stu* chassis_route_traj, float target_vx, float target_vy, float target_vw)
 {
      // 将坐标压入各自队列
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vx, target_vx);
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vy, target_vy);
    Traj_Path_Queue_AddPoint(&chassis_route_traj->traj_queue_vw, target_vw);
 }  
/**
 * @brief 直线插补单段路径调度器
 * @note 必须放在 Traj_Adaptive_Update(x)、Traj_Adaptive_Update(y)、Traj_Adaptive_Update(w) 之前调用
 */
void chassis_single_route_scheduler(RADAR_POSITION * chassis_real_position, chassis_route_traj_stu* chassis_route_traj,float dt)
{
    
    // === 1. 计算当前目标距离 ===
    float dist_to_actual_vx = fabsf(chassis_route_traj->traj_vx.target_pos - chassis_real_position->radar_x);
    float dist_to_out_vx = fabsf(chassis_route_traj->traj_vx.target_pos - chassis_route_traj->traj_vx.out_pos);
    float dist_to_actual_vy = fabsf(chassis_route_traj->traj_vy.target_pos - chassis_real_position->radar_y);
    float dist_to_out_vy = fabsf(chassis_route_traj->traj_vy.target_pos - chassis_route_traj->traj_vy.out_pos);
    float dist_to_actual_vw = fabsf(chassis_route_traj->traj_vw.target_pos - chassis_real_position->radar_yaw);
    float dist_to_out_vw = fabsf(chassis_route_traj->traj_vw.target_pos - chassis_route_traj->traj_vw.out_pos);
    uint8_t x_integral_flag = 1;// 积分使能标志，初始为1，表示允许速度积分
    uint8_t y_integral_flag = 1;    
    uint8_t w_integral_flag = 1;
    if(dist_to_out_vx<CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD) 
    {
        
        PID_Calculate(&chassis_route_traj->pid_final_vx, chassis_real_position->radar_x, chassis_route_traj->traj_vx.target_pos, CHASSIS_V_MAX/5.0f); // 计算PID输出，限制在较小范围内
        chassis_route_traj->traj_vx.out_vel = chassis_route_traj->pid_final_vx.Output; // 用PID输出修正速度，进行微调
    }
    if(dist_to_out_vy<CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD) 
    {
        
        PID_Calculate(&chassis_route_traj->pid_final_vy, chassis_real_position->radar_y, chassis_route_traj->traj_vy.target_pos, CHASSIS_V_MAX/5.0f); // 计算PID输出，限制在较小范围内
        chassis_route_traj->traj_vy.out_vel = chassis_route_traj->pid_final_vy.Output; // 用PID输出修正速度，进行微调
    }
    if(dist_to_out_vw<CHASSIS_ROUTE_FINAL_APPROACH_ANGLE_THRESHOLD) 
    {
        
        PID_Calculate(&chassis_route_traj->pid_final_vw, chassis_real_position->radar_yaw, chassis_route_traj->traj_vw.target_pos, CHASSIS_V_MAX/5.0f); // 计算PID输出，限制在较小范围内
        chassis_route_traj->traj_vw.out_vel = chassis_route_traj->pid_final_vw.Output; // 用PID输出修正速度，进行微调
    }
     // === 2. 如果已经判定到达，强制锁定目标防止晃动 ===
    chassis_single_route_completion_check(&chassis_route_traj->traj_vx, chassis_real_position->radar_x);
    chassis_single_route_completion_check(&chassis_route_traj->traj_vy, chassis_real_position->radar_y);
    chassis_single_route_completion_check_angle(&chassis_route_traj->traj_vw, chassis_real_position->radar_yaw);

    if(chassis_route_traj->traj_vx.is_finished && chassis_route_traj->traj_vy.is_finished && chassis_route_traj->traj_vw.is_finished) 
    {
        return;
    } 
    // ===3.扰动自适应与补救策略（如果跟踪误差过大，先让目标影子回到合理范围内，等待实体追上来）===
    chassis_disturbance_adaptive(&chassis_route_traj->traj_vx, chassis_real_position->radar_x);
    chassis_disturbance_adaptive(&chassis_route_traj->traj_vy, chassis_real_position->radar_y);
    chassis_disturbance_adaptive(&chassis_route_traj->traj_vw, chassis_real_position->radar_yaw);
    // === 4. 速度推演 ===
    float dis_total =   dist_to_actual_vx + dist_to_actual_vy + dist_to_actual_vw * DISTANCE_WHEEL_CENTER*1000.0f; 
    float rate_vx = (dist_to_actual_vx + 0.001f) / (dis_total + 0.003f); // 加小常数避免除零
    float rate_vy = (dist_to_actual_vy + 0.001f) / (dis_total + 0.003f);
    float rate_vw = (dist_to_actual_vw * DISTANCE_WHEEL_CENTER*1000.0f + 0.001f) / (dis_total + 0.003f);

    chassis_velocity_acceleration_allocation(&chassis_route_traj->traj_vx,rate_vx);
    chassis_velocity_acceleration_allocation(&chassis_route_traj->traj_vy,rate_vy);
    chassis_velocity_acceleration_allocation(&chassis_route_traj->traj_vw,rate_vw);     
    if(chassis_route_traj->traj_vx.is_finished == 0 && dist_to_out_vx >= CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD) chassis_single_route_velocity_update(&chassis_route_traj->traj_vx, chassis_real_position->radar_x, dt);
    if(chassis_route_traj->traj_vy.is_finished == 0 && dist_to_out_vy >= CHASSIS_ROUTE_FINAL_APPROACH_THRESHOLD) chassis_single_route_velocity_update(&chassis_route_traj->traj_vy, chassis_real_position->radar_y, dt);
    if(chassis_route_traj->traj_vw.is_finished == 0 && dist_to_out_vw >= CHASSIS_ROUTE_FINAL_APPROACH_ANGLE_THRESHOLD) chassis_single_route_velocity_update(&chassis_route_traj->traj_vw, chassis_real_position->radar_yaw, dt);  
    // ===5.速度积分===
    if(x_integral_flag) chassis_route_traj->traj_vx.out_pos = chassis_route_traj->traj_vx.out_pos + chassis_route_traj->traj_vx.out_vel * 0.001f;       
    if(y_integral_flag) chassis_route_traj->traj_vy.out_pos = chassis_route_traj->traj_vy.out_pos + chassis_route_traj->traj_vy.out_vel * 0.001f;
    if(w_integral_flag) chassis_route_traj->traj_vw.out_pos = chassis_route_traj->traj_vw.out_pos + chassis_route_traj->traj_vw.out_vel / DISTANCE_WHEEL_CENTER * 0.001f;   
}
