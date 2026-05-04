#include "trajectory.h"

int32_t debug_count; //测试用计数器

Traj_Single_struct debug_traj_a1;    //测试用单段轨迹结构体
Traj_Queue_struct debug_traj_queue_a1; //测试用轨迹队列结构体

Traj_Single_struct debug_traj_m3508;
Traj_Queue_struct debug_traj_queue_m3508;

/**
 * @brief 轨迹控制中的符号函数，返回输入值的符号
 * @param val: 输入值
 * @return: 符号 (1.0f, -1.0f, 或 0.0f)
 */
static inline float traj_sign(float val) {
    if (val > 0.0f) return 1.0f;
    if (val < 0.0f) return -1.0f;
    return 0.0f;
}

/* ============================== 单段路径执行实现 ===================================== */

/**
 * @brief 初始化关节自适应负载轨迹
 * @param traj:        轨迹控制实例
 * @param start_pos:   当前物理位置
 * @param target_pos:  终点目标位置
 * @param v_max:       最大允许规划速度（单位与pos单位一致）
 * @param a_max:       最大允许规划加速度（单位与pos单位一致）
 * @param max_error:   允许的最大跟随误差阈值 (当实际位置与规划位置差值超过此值，系统开始补救)
 */
void Traj_Adaptive_Init(Traj_Single_struct *traj, float start_pos, float target_pos, 
                        float v_max, float a_max, float max_error)
{
    // 输入保护
    if (v_max <= 0.001f) v_max = 0.001f;
    if (a_max <= 0.001f) a_max = 0.001f;
    if (max_error <= 0.001f) max_error = 0.001f;

    traj->start_pos = start_pos;
    traj->target_pos = target_pos;
    traj->v_max = v_max;
    traj->a_max = a_max;
    traj->max_tracking_error = max_error;
    
    // 初始状态清零
    traj->out_pos = start_pos;
    traj->out_vel = 0.0f;
    traj->is_remedy_mode = 0;
    traj->is_finished = 0;
}

/**
 * @brief 单段路径执行器 (在定时器/task中高频调用)
 * @param traj:        轨迹控制实例
 * @param actual_pos:  关节编码器反馈的真实物理位置！(非常重要)
 * @param dt:          调用周期时长 (如 0.001f 代表 1ms)
 */
void Traj_Adaptive_Update(Traj_Single_struct *traj, float actual_pos, float dt)
{
    // === 1. 如果已经判定到达，强制锁定目标防止晃动 ===
    if (traj->is_finished) {
        traj->out_pos = traj->target_pos;
        traj->out_vel = 0.0f;
        return;
    }

    // === 2. 计算当前目标距离 ===
    float err_to_target = traj->target_pos - traj->out_pos;
    float dist_to_target = fabsf(err_to_target);
    
    // 柔性判敛容差：取最大允许误差的 1/10 作为到达判定条件，避免魔数限制
    // 注意：如果 max_tracking_error 设得非常巨大（例如 PI/2 = 1.57），
    // 那么 finish_window 就会高达 0.157！这依然是一个非常松垮的容差。
    // 限制：到达容差绝对不能超过一个极其微小的合理死区（比如 0.02 rad，约1度），
    // 否则当实际位置离目标还有十几二十度时，系统可能会直接判定到达(is_finished=1)，从而提前触发下一点。
    float finish_window = traj->max_tracking_error * 0.1f; 
    if (finish_window > 0.02f) finish_window = 0.02f; // 强制钳制最大到位死区
    
    // 判定条件：规划影子抵达终点容差内，且实体也追到了容差内
    // 注意！不能仅仅满足误差条件就立刻判断到达（is_finished），
    // 由于刚起步时 dist_to_target = fabsf(err) 也很小（如果当前刚好处于目标点附件但还没出发），
    // 更致命的是，目标发生变化瞬间也可能瞬态触发。
    // 我们必须保证当前期望速度也已经非常微小（即系统真的“停稳”了）。
    
    if (dist_to_target < finish_window && fabsf(traj->target_pos - actual_pos) < finish_window && fabsf(traj->out_vel) < 0.1f) {
        traj->is_finished = 1;
        // 把实际位置对齐到终极目标
        traj->out_pos = traj->target_pos;
        traj->out_vel = 0.0f;
        traj->is_remedy_mode = 0;
        return;
    }
    
    // === 3. 负载自适应与补救策略 ===
    float tracking_error = fabsf(traj->out_pos - actual_pos);
    
    if (tracking_error > traj->max_tracking_error) {
        traj->is_remedy_mode = 1; 
        float dir_of_error = traj_sign(traj->out_pos - actual_pos);
        traj->out_pos = actual_pos + dir_of_error * (traj->max_tracking_error * 0.9f); 
        traj->out_vel = 0.0f; // 完全刹停，等待实体
        return;               
    } else {
        traj->is_remedy_mode = 0;
    }

    // === 4. 常规：带有完整反向保护的离散速度推演 ===
    float expected_vel = 0.0f;
    float dir = traj_sign(err_to_target);
    
    // [新增] 方向保护：如果正在反向运行（上位机突然反向更改点位），必须先减速到 0
    if (traj->out_vel * dir < -0.001f) {
        // 让当前速度按照最大减速度逼近 0
        expected_vel = fabsf(traj->out_vel) - traj->a_max * dt;
        if (expected_vel < 0.0f) expected_vel = 0.0f; 
        
        // 维持之前的方向极性，只是把速度降下来
        traj->out_vel = expected_vel * traj_sign(traj->out_vel); 
    }
    else {
        // 方向一致，可以正常执行加速/匀减速逻辑
        // 距离过于极小则不再计算 (防平方根附近计算奇点抖动)
        if (dist_to_target < (traj->v_max * dt * 0.1f)) {
            traj->out_vel = 0.0f;
            traj->out_pos = traj->target_pos; // 顺带对齐防止微小偏差
            return;
        }

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
    
    // === 5. 位置积分 ===
    traj->out_pos += traj->out_vel * dt;
}
/* ============================== 单段路径执行实现 ===================================== */


/* =============================== 多段队列管理器实现 =============================== */

/**
 * @brief 初始化多段路径的途径点队列
 * @param queue:        队列句柄
 * @param blend_radius: 融合倒角半径。
 *                      如果设为 0，机械臂将在每个点完全刹停后再去下一点。
 *                      如果设为 5.0f，它会在距离目标 5 单位(rad/deg)时无缝平滑切入下一点。
 */
void Traj_Path_Queue_Init(Traj_Queue_struct *queue, float blend_radius) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->blend_radius = blend_radius;
}

/**
 * @brief 动态向队列打入新的路径点 (上位机可随时随地调用此函数塞任务)；
 *        往路径队尾追加一个新的途经点 (返回1成功，0失败满载)
 * @param target_pos: 目标位置（单位与单段轨迹一致）
 */
uint8_t Traj_Path_Queue_AddPoint(Traj_Queue_struct *queue, float target_pos) {
    if (queue->count >= TRAJ_MAX_WAYPOINTS) return 0; // 队列满了
    
    queue->points[queue->head] = target_pos;
    queue->head = (queue->head + 1) % TRAJ_MAX_WAYPOINTS;
    queue->count++;
    return 1;
}

/**
 * @brief 多段路径规划器；用于向单段路径执行器发送连续的目标点；
 *        在 1ms 定时器中拦截预读目标；
 *        将此函数放在你的 Traj_Adaptive_Update 之前运行；
 */
void Traj_Path_Queue_Update(Traj_Queue_struct *queue, Traj_Single_struct *traj, float actual_pos) {

    // !!!如果清单空了，就啥也不做（路径规划器不对电机进行实际控制），同时单段发生器会自动让电机在终点平稳停下
    if (queue->count == 0) return;
    
    // 算出实际位置距离它目前目标还有多远
    float dist_to_actual = fabsf(traj->target_pos - actual_pos);    
    // 我们必须保证：1. 实际跑到了；2. 虚拟目标影子也跑到了，二者才能切点。
    float dist_to_out = fabsf(traj->target_pos - traj->out_pos);

    // 切换条件1：彻底停稳（用于 blend_radius == 0 时的精准到达切换）
    uint8_t condition_arrived = traj->is_finished;
    
    // 切换条件2：进入了融合圆角以内（且启用融合模式）
    uint8_t condition_blend = (queue->blend_radius > 0.001f) && 
                              (dist_to_actual <= queue->blend_radius) && 
                              (dist_to_out <= queue->blend_radius);

    if (condition_arrived || condition_blend) {
        
        float next_target = traj->target_pos; // 兜底
        
        // 循环出队：目的是剔除恰好和当前位置重合的“废点/重复点”
        while (queue->count > 0) {
            //由于先赋值，再执行，所以在 debug_traj_queue_a1.points[debug_traj_queue_a1.tail] 中实际上显示的是下一个路径点
            next_target = queue->points[queue->tail];

            queue->tail = (queue->tail + 1) % TRAJ_MAX_WAYPOINTS;
            queue->count--;
            debug_count++; // 测试用：记录已经切换了\几个点了
            
            // 检查这个新目标，如果距离我们真实的物理位置只有不到 0.02 (约1度)
            // 说明这基本是个原地重复点 (比如用户把第一段的终点，又作为目标重新推入了一次队列)
            // 如果不剔除，下一毫秒 queue update 时判断距离依然为 0，从而引发两次甚至多次连跳 (Double Pop)
            if (fabsf(next_target - actual_pos) > traj->max_tracking_error/30.0f) {
                break; // 是一个有意义的远方新点，跳出剔除循环
            }
        }
        
        // 把有效的新点装弹，重置到达标记，原有的速度和方向保持不变！！
        traj->target_pos = next_target;
        traj->is_finished = 0;
        
        // 修改目标位置后必须立即 return 打断当前周期
        return; 
    }
}
/* =============================== 多段队列管理器实现 =============================== */
