#include "BT_customized.h"

static ArmPickState pick_state = PICK_IDLE; /* 抓取动作的状态机当前状态 */
static uint32_t pick_timer_ms = 0; /* 以 BT tick (示例为10ms) 计数，或在 Tick1ms 中累加 */


/*================================ 辅助函数 ================================*/

/**
 * @brief 判断机械臂末端是否在 (tx,ty) 附近且轨迹队列为空（已稳定）
*/
static inline uint8_t arm_if_arrived_xy(float tx, float ty)
{
    float dx = fabsf(arm.x0_now - tx);
    float dy = fabsf(arm.y0_now - ty);
    return (dx < ARRIVE_TOL && dy < ARRIVE_TOL && arm_traj.traj_queue_x.count == 0);
}

/*================================ 条件 节点 ================================*/

/**
 * @brief 条件节点：判断是否有一个机械臂取放块请求
*/
static BT_Status Cond_HasPickRequest(void)
{
    return bb.req_pick ? BT_SUCCESS : BT_FAILURE;
}

/*================================ 行为/动作 节点 ================================*/

/**
 * @brief 机械臂抓取状态机，根据机械臂运行状态和计时，自动推进抓取动作的各个阶段
 * @note - 上层BT把控制流交到这个节点后，它会在内部按状态机推进。
 * @note - 每次被 tick 时，FSM 只推进有限步并返回当前状态。
 * @return 返回 BT_RUNNING / BT_SUCCESS / BT_FAILURE
*/
static BT_Status Act_Arm_Pick_FSM(void)
{
    /* 该函数由 BT_Customized_Tick1ms 每 10ms 调用一次（示例），
       pick_timer_ms 的单位与 Tick 调用频率对应（本示例按 10ms 计数）。 */

    switch (pick_state) {       //根据当前状态执行对应的动作，并根据条件推进状态机

    case PICK_IDLE:
        /* 第一次进入：回归到 HOME 位置，并进入 APPROACH */
        taskENTER_CRITICAL(); /* 保护轨迹队列写入，或使用 mutex */
        arm_queue_add_point(&arm_traj, ARM_HOME_X, ARM_HOME_Y, arm.theta_target[0], 0, 0);
        taskEXIT_CRITICAL();
        pick_state = PICK_APPROACH_1;
        pick_timer_ms = 0;
        return BT_RUNNING;

    case PICK_APPROACH_1:
        /* 等待到达预抓位置 */
        if (arm_if_arrived_xy(ARM_HOME_X, ARM_HOME_Y)) {
            /* 到位后下发预抓取点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_APPROACH_X_1, ARM_PICK_APPROACH_Y_1, arm.theta_target[0], 0, 0);
            taskEXIT_CRITICAL();
            pick_state = PICK_APPROACH_2;
            pick_timer_ms = 0;
        } else {
            /* 超时判定 */
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) { /* *10 假设Tick10ms */
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_APPROACH_2:
        /* 等待到达第二预抓位置（更近） */
        if (arm_if_arrived_xy(ARM_PICK_APPROACH_X_1, ARM_PICK_APPROACH_Y_1)) {
            /* 到位后下发真正抓取点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_APPROACH_X_2, ARM_PICK_APPROACH_Y_2, arm.theta_target[0], 0, 0);
            taskEXIT_CRITICAL();
            pick_state = PICK_LOWER;
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_LOWER:
        /* 等待到达第二预抓位置（更近） */
        if (arm_if_arrived_xy(ARM_PICK_APPROACH_X_2, ARM_PICK_APPROACH_Y_2)) {
            /* 到位后下发真正抓取点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_TARGET_X, ARM_PICK_TARGET_Y, arm.theta_target[0], ARM_PICK_TARGET_TIME, ARM_SUCK_ON);
            taskEXIT_CRITICAL();
            pick_state = PICK_LIFT; /* 直接进入抬升阶段，等待轨迹完成后再判断是否成功 */
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_LIFT:
        /* 等待吸泵建立真空（防止立即抬起导致掉落） */
        if (pick_timer_ms * 10 >= ARM_PICK_TARGET_TIME) {
            /* 吸牢后抬升到安全高度 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_SAFE_LIFT_X, ARM_SAFE_LIFT_Y, arm.theta_target[0], ARM_SAFE_LIFT_TIME, ARM_SUCK_ON);
            taskEXIT_CRITICAL();
            pick_state = PICK_STORE;/* 抬升到安全高度后进入存储阶段 */
            pick_timer_ms = 0;
            return BT_RUNNING;
        }
        pick_timer_ms++;
        return BT_RUNNING;

    case PICK_STORE:
        /* 等待抬升完成 */
        if (arm_if_arrived_xy(ARM_SAFE_LIFT_X, ARM_SAFE_LIFT_Y)) {
            /* 到位后下发临时存储点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_STORE_X, ARM_STORE_Y, arm.theta_target[0], 0, ARM_SUCK_ON);
            taskEXIT_CRITICAL();
            pick_state = PICK_COMPLETE; /* 进入完成阶段，等待轨迹完成后再判断是否成功 */
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_COMPLETE:
        /* 等待到达存储点 */
        if (arm_if_arrived_xy(ARM_STORE_X, ARM_STORE_Y)) {
            /* 到位时返回success */
            pick_state = PICK_IDLE; /* 进入完成阶段，等待轨迹完成后再判断是否成功 */
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_FAILED:
    default:
        /* 失败时复位并告诉上层失败，供上层 Selector 做重试或回退 */
        arm_queue_add_point(&arm_traj, ARM_HOME_X, ARM_HOME_Y, arm.theta_target[0], 0, ARM_SUCK_OFF);
        pick_state = PICK_IDLE;
        pick_timer_ms = 0;

        return BT_FAILURE;
    }

}

/**
 * @brief 抓取完成后的收尾清理动作，重置黑板状态和 Sequence 记忆
*/
static BT_Status Act_FinishCleanup(void)
{
    bb.req_pick = 0;
    bb.retry_count = 0;
    seq_mem_idx = 0;    /* 重置 Sequence 记忆，方便下一次请求 */
    return BT_SUCCESS;
}


/*================================ 上层行为树根（简化，用 Memory Sequence 保持进度） ================================*/

/**
 * @brief 简单的机械臂取放行为树根节点，使用 Memory Sequence 来保持状态机进度
*/
static BT_Status BT_SimplePickTree_Root(void)
{
    /* 按顺序执行：
     *  1) 有请求？
     *  2) 执行下层抓取 FSM（可能多 Tick 返回 RUNNING）
     *  3) 收尾清理
     */
    static const BT_NodeFunc seq[] = {
        Cond_HasPickRequest,
        Act_Arm_Pick_FSM,
        Act_FinishCleanup
    };

    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &seq_mem_idx);
}


/*=============================== 外部接口 ===============================*/
/* 在 BT_customized.h 中声明这些函数，并在 freertos 的初始化处调用 Init，在 cmd_task 中定期调用 Tick */

/**
 * @brief BT 客制化模块初始化，重置黑板和状态机
*/
void BT_Customized_Init(void)
{
    bb.req_pick = 0;
    bb.retry_count = 0;
    seq_mem_idx = 0;
    pick_state = PICK_IDLE;
    pick_timer_ms = 0;
}

/**
 * @brief 外部触发器（例如遥控上升沿）调用此函数发起一次取放请求
 */
void BT_RequestSimplePick(void)
{
    bb.req_pick = 1;
}


/*=============================== tick ===============================*/

/**
 * @brief 行为树tick，每 1ms 调用一次；内部降频到 10ms 用于BT树调度，避免过于频繁遍历
*/
void BT_Customized_Tick1ms(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; /* 每 10ms 执行一次 BT Tick */
    div10 = 0;

    (void)BT_SimplePickTree_Root();
}

