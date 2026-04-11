#include "trajectory.h"

Traj_Single_struct debug_traj;    //测试用单段轨迹结构体
Traj_Queue_struct debug_traj_queue; //测试用轨迹队列结构体

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

/**
 * @brief 初始化关节自适应负载轨迹
 * @param traj:        轨迹控制实例
 * @param start_pos:   机械臂当前物理位置 (不要传入虚拟位置)
 * @param target_pos:  机械臂的终点目标位置
 * @param v_max:       最大允许规划速度
 * @param a_max:       最大允许规划加速度
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


/* ===================== 新增：多段队列管理器实现 ===================== */

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
    
    // ==== 核心融合逻辑 ====
    // 如果已经触发完美停车(is_finished)，或者“即将到达”并且开启了融合半径预读(blend)
    // 修复：同时要求虚拟影子 out_pos 也跑到了终点附近（不再只看电机物理位置），
    // 否则当电机卡住导致实际位置落后于影子时，只要一进圆圈，影子就会瞬间被弹跃，
    // 把下一个目标点赋值给当前由于保护处于满载滞后的系统。

    // 算出实际位置距离它目前目标还有多远
    float dist_to_actual = fabsf(traj->target_pos - actual_pos);    
    // 我们必须保证：1. 实际跑到了；2. 虚拟目标影子也跑到了，二者才能切点。
    float dist_to_out = fabsf(traj->target_pos - traj->out_pos);

    if (traj->is_finished || (dist_to_actual <= queue->blend_radius && dist_to_out <= queue->blend_radius)) {
        
        // 取出下一个路点！
        float next_target = queue->points[queue->tail];
        queue->tail = (queue->tail + 1) % TRAJ_MAX_WAYPOINTS;
        queue->count--;
        
        // 暴力篡改目标点，并重置到达标记，原有的速度和方向保持不变！！
        traj->target_pos = next_target;
        traj->is_finished = 0;
        
        // 由于目标位置发生改变，我们必须马上通知单段轨迹控制器：“现在误差已经变了，不是原来跑到终点附近极小的误差了！”
        // 否则下一个毫秒单段发生器中的 dir (err 的符号) 会沿用老点周围的异常符号，导致方向反向保护误触发。
        return; 
    }
}
