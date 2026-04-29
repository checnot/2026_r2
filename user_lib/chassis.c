#include "chassis.h"

chassis_stu chassis_r2;//r2底盘结构体
chassis_route_traj_stu chassis_route_traj;
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
/*
*@brief 路径完成判定与锁定函数(由于x、y轴)
*@param traj: 轨迹控制实例
*@param actual_pos: 真实物理位置
*@note 当判定完成时，强制锁定目标位置并刹停，防止微小误差引起的抖动
*/
void chassis_single_route_completion_check(Traj_Single_struct * traj, float actual_pos)
{
    float error = fabsf(traj->target_pos - actual_pos);
    if (error < CHASSIS_TRACKING_POSITION_ERROR_THRESHOLD) // 误差小于误差阈值则判定完成
    {
        traj->is_finished = 1;
        traj->out_pos = traj->target_pos; // 锁定目标位置
        traj->out_vel = 0.0f; // 刹停
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
    if (error < CHASSIS_TRACKING_ANGLE_ERROR_THRESHOLD) // 误差小于误差阈值则判定完成
    {
        traj->is_finished = 1;
        traj->out_pos = traj->target_pos; // 锁定目标位置
        traj->out_vel = 0.0f; // 刹停
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
    traj->v_max = CHASSIS_V_MAX * px;
    traj->a_max = CHASSIS_A_MAX * px;
            if(traj->v_max < 0.001f) traj->v_max = 0.001f;
            if(traj->a_max < 0.001f) traj->a_max = 0.001f;
}

/*
*@brief 底盘单段路径速度推演函数
* @param traj: 轨迹控制实例
* @param dist_to_target: 当前目标距离
* @param dt: 调用周期时长 (如 0.001f 代表 1ms)
*/
void chassis_single_route_velocity_update(Traj_Single_struct * traj,float dist_to_target, float dt)
{

    float expected_vel = 0.0f;
    float dir = traj_sign2(traj->target_pos - traj->out_pos);
    
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
        traj->out_vel = expected_vel * dir;
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
    chassis_single_route_velocity_update(&chassis_route_traj->traj_vx, dist_to_actual_vx, dt);
    chassis_single_route_velocity_update(&chassis_route_traj->traj_vy, dist_to_actual_vy, dt);
    chassis_single_route_velocity_update(&chassis_route_traj->traj_vw, dist_to_actual_vw, dt);  
    // ===5.速度积分===
    chassis_route_traj->traj_vx.out_pos = chassis_route_traj->traj_vx.out_pos + chassis_route_traj->traj_vx.out_vel * dt;
    chassis_route_traj->traj_vy.out_pos = chassis_route_traj->traj_vy.out_pos + chassis_route_traj->traj_vy.out_vel * dt;
    chassis_route_traj->traj_vw.out_pos = chassis_route_traj->traj_vw.out_pos + chassis_route_traj->traj_vw.out_vel / DISTANCE_WHEEL_CENTER * dt;
}
