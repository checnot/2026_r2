#include "BT_customized.h"

ArmPickState pick_state = PICK_IDLE; /* 单独开发时使用抓取动作的状态机当前状态 */
ChassisRouteState chassis_state = CHASSIS_ROUTE_IDLE;/* 单独开发时使用底盘路径状态机当前状态 */

uint32_t pick_timer_ms = 0; /* 以 BT tick (10ms为周期) 计数，或在 Tick1ms 中累加 */
uint32_t chassis_timer_ms = 0 ;

/* 保存 Sequence 的记忆索引（Memory Sequence 使用） */
uint8_t seq_mem_idx = 0;
uint8_t seq_mem_idx_combined = 0;

/* ------------ 黑板 (行为树状态共享区) ------------ */
struct {
    uint8_t req_pick;     /* 发起一次自动取放请求 (1 = 有请求) */
    uint8_t req_chassis_route; /* 发起一次底盘路径请求 (1 = 有请求) */
    uint8_t retry_count;  /* 重试计数（示例） */
} bb;
/* ------------ 黑板 (行为树状态共享区) ------------ */








/*=========================================================== 机械臂行为树测试用 ===============================================================*/
/*=========================================================== 机械臂行为树测试用 ===============================================================*/
/*=========================================================== 机械臂行为树测试用 ===============================================================*/














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
static BT_Status Cond_Has_Arm_PickRequest(void)
{
    return bb.req_pick ? BT_SUCCESS : BT_FAILURE;
}

/*================================ 行为/动作 节点 ================================*/

/**
 * @brief 机械臂抓取状态机（高200），根据机械臂运行状态和计时，自动推进抓取动作的各个阶段
 * @return 返回 BT_RUNNING / BT_SUCCESS / BT_FAILURE
 * @note - 上层BT把控制流交到这个节点后，它会在内部按状态机推进。
 * @note - 每次被 tick 时，FSM 只推进有限步并返回当前状态。
*/
static BT_Status Act_Arm_Pick_FSM_P200(void)
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
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_APPROACH_X_1, ARM_PICK_P200_APPROACH_Y_1, arm.theta_target[0], 0, 0);
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
        if (arm_if_arrived_xy(ARM_PICK_P200_APPROACH_X_1, ARM_PICK_P200_APPROACH_Y_1)) {
            /* 到位后下发真正抓取点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_APPROACH_X_2, ARM_PICK_P200_APPROACH_Y_2, arm.theta_target[0], 0, 0);
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
        if (arm_if_arrived_xy(ARM_PICK_P200_APPROACH_X_2, ARM_PICK_P200_APPROACH_Y_2)) {
            /* 到位后下发真正抓取点 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_TARGET_X, ARM_PICK_P200_TARGET_Y, arm.theta_target[0], ARM_PICK_P200_TARGET_TIME, ARM_SUCK_ON);
            taskEXIT_CRITICAL();
            pick_state = PICK_STORE; /* 直接进入存储阶段，等待轨迹完成后再判断是否成功 */
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    // case PICK_LIFT:
    //     /* 等待吸盘到达抓取位置 */
    //     if (arm_if_arrived_xy(ARM_PICK_P200_TARGET_X, ARM_PICK_P200_TARGET_Y) && pick_timer_ms * 10 >= ARM_PICK_P200_TARGET_TIME) {
    //         /* 到位后下发真正抓取点 */
    //         taskENTER_CRITICAL();
    //         arm_queue_add_point(&arm_traj, ARM_SAFE_P200_LIFT_X, ARM_SAFE_P200_LIFT_Y, arm.theta_target[0], ARM_SAFE_P200_LIFT_TIME, ARM_SUCK_ON);
    //         taskEXIT_CRITICAL();
    //         pick_state = PICK_STORE; /* 直接进入存储阶段 */
    //         pick_timer_ms = 0;
    //     } else {
    //         if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
    //             pick_state = PICK_FAILED;
    //         }
    //     }
    //     return BT_RUNNING;

    case PICK_STORE:
        /* 等待抬升完成 */
        if (arm_if_arrived_xy(ARM_PICK_P200_TARGET_X, ARM_PICK_P200_TARGET_Y)) {
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
            return BT_SUCCESS;
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
        bb.req_pick = 0; /* 失败时清除请求，等待下一次触发 */

        return BT_FAILURE;
    }

}
/**
 * @brief 抓取完成后的收尾清理动作，重置黑板状态和 Sequence 记忆
*/
static BT_Status Act_Arm_Pick_FinishCleanup(void)
{
    bb.req_pick = 0;
    // bb.retry_count = 0;
    // seq_mem_idx = 0;    /* 重置 Sequence 记忆，方便下一次请求 */
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
        Cond_Has_Arm_PickRequest,
        Act_Arm_Pick_FSM_P200,
        Act_Arm_Pick_FinishCleanup
    };

    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &seq_mem_idx);
}


/*=============================== 外部接口 ===============================*/
/* 在 BT_customized.h 中声明这些函数，并在 freertos 的初始化处调用 Init，在 cmd_task 中定期调用 Tick */

/**
 * @brief BT 客制化模块初始化，重置黑板和状态机
*/
void BT_Customized_Init_arm_test(void)
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
void BT_Customized_Tick1ms_arm_test(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; /* 每 10ms 执行一次 BT Tick */

    //清空计时，遍历行为树
    div10 = 0;
     (void)BT_SimplePickTree_Root();
   
}








/*=========================================================== 底盘路径行为树测试用 ===============================================================*/
/*=========================================================== 底盘路径行为树测试用 ===============================================================*/
/*=========================================================== 底盘路径行为树测试用 ===============================================================*/









static BT_Status Act_Chassis_Route_FSM(void)
{
    /* 该函数由 BT_Customized_Tick1ms 每 10ms 调用一次（示例），
       route_timer_ms 的单位与 Tick 调用频率对应（本示例按 10ms 计数）。 */

    switch (chassis_state) {       //根据当前状态执行对应的动作，并根据条件推进状态机

    case CHASSIS_ROUTE_IDLE:
        /* 第一次进入：回归到 HOME 位置，并进入 Position1 */
        taskENTER_CRITICAL(); /* 保护轨迹队列写入，或使用 mutex */
        chassis_route_point_add(&chassis_route_traj, 400.0f, 1400.0f, 0.0f,1000);
        taskEXIT_CRITICAL();
        chassis_state = CHASSIS_ROUTE_1;
        chassis_timer_ms = 0;
        return BT_RUNNING;
    case CHASSIS_ROUTE_1:
        /* 等待到达抓取矛头位置 */
        if (chassis_route_traj.traj_vx.is_finished && chassis_route_traj.traj_vy.is_finished && chassis_route_traj.traj_vw.is_finished) {
            /* 到位后下发下一个点 */
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, 400.0f, 600.0f, 0.0f,1000);
            taskEXIT_CRITICAL();
            chassis_state = CHASSIS_ROUTE_2;
            chassis_timer_ms = 0;
        }else if(++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
        {
            chassis_state = CHASSIS_FAILED;
        }
        return BT_RUNNING;
    case CHASSIS_ROUTE_2:
        /* 等待到达抓取矛头位置 */
        if (chassis_route_traj.traj_vx.is_finished && chassis_route_traj.traj_vy.is_finished && chassis_route_traj.traj_vw.is_finished) {
            /* 到位后下发下一个点 */
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, 1200.0f, 1400.0f, 0.0f,1000);   //对接位置
            taskEXIT_CRITICAL();
            chassis_state = CHASSIS_ROUTE_3;
            chassis_timer_ms = 0;
        }else if(++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
        {
            chassis_state = CHASSIS_FAILED;
        }
        return BT_RUNNING;
    case CHASSIS_ROUTE_3:
        /* 等待到达对矛头位置 */
        if (chassis_route_traj.traj_vx.is_finished && chassis_route_traj.traj_vy.is_finished && chassis_route_traj.traj_vw.is_finished) {
            /* 到位时返回success */
            chassis_state = CHASSIS_ROUTE_IDLE; /* 进入完成阶段，等待轨迹完成后再判断是否成功 */
            chassis_timer_ms = 0;
            return BT_SUCCESS;  
        } else {
            if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                chassis_state = CHASSIS_FAILED;
            }
        }
        return BT_RUNNING;  
    case CHASSIS_FAILED:
    default:
        /* 失败时复位并告诉上层失败，供上层 Selector 做重试或回退 */
        chassis_route_point_add(&chassis_route_traj, 400.0f, 1400.0f, 0.0f,1000);
        chassis_state = CHASSIS_ROUTE_IDLE;
        chassis_timer_ms = 0;
        bb.req_chassis_route = 0; /* 失败时清除请求，等待下一次触发 */
        return BT_FAILURE;
    }
}
static BT_Status Cond_Has_Chassis_Request(void)
{
    return bb.req_chassis_route ? BT_SUCCESS : BT_FAILURE;
}
static BT_Status Act_Chassis_Finish_Cleanup(void)
{
    bb.req_chassis_route = 0;
    chassis_timer_ms = 0;
    return BT_SUCCESS;
}
static BT_Status BT_Chassis_Route_Tree_Root(void)
{
    /* 按顺序执行：
     *  1) 有请求？
     *  2) 执行底盘路径 FSM
     *  3) 收尾清理
     */
    static const BT_NodeFunc seq[] = {
        Cond_Has_Chassis_Request,
        Act_Chassis_Route_FSM,
        Act_Chassis_Finish_Cleanup
    };

    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &seq_mem_idx);
}

void BT_Customized_Tick1ms_chassis_test(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; /* 每 10ms 执行一次 BT Tick */

    //清空计时，遍历行为树
    div10 = 0;
  
    (void)BT_Chassis_Route_Tree_Root();
}












/*=========================================================== 组合行为树测试用 ===============================================================*/
/*=========================================================== 组合行为树测试用 ===============================================================*/
/*=========================================================== 组合行为树测试用 ===============================================================*/










/*=========================================================== 组合行为树任务(底盘+机械臂协同) ===============================================================*/
/* 
 * 【架构说明】(方案: 解耦式协同)
 * 1. 核心思想：动作的具体执行（电机发流、轨迹验算）依然由各自底层 FSM 负责；
 * 2. 上层行为树（BT）只作为一个“宏观调度总监”，负责按顺序调用下层模块；
 * 3. 实现了高内聚低耦合：底盘不需要知道机械臂怎么动，机械臂也不需要知道底盘在哪（待定，其实可以优化）。
 */

/* ================= 1. 综合任务专属状态区 ================= */

// 底盘状态
CombinedChassisState task_chassis_state = CHASSIS_IDLE;

uint8_t task_request_flag = 0;   // 仅用于启动综合任务 (1=有请求)


/* ================= 2. 行为树节点实现 (条件节点与动作节点) ================= */

/**
 * @brief [条件节点] 检查是否收到了组合任务的执行请求
 * @return BT_SUCCESS (收到了，继续往下执行) / BT_FAILURE (没收到，打断执行树)
 */
static BT_Status Cond_Has_Combined_Task_Request(void)
{
    return task_request_flag ? BT_SUCCESS : BT_FAILURE;
}

/**
 * @brief [动作节点] 描述
 * @note  这是一个“耗时动作”，因此在移动过程中我们会一直返回 BT_RUNNING（挂起）。
 *        上层记忆序列节点（BT_Sequence_Memory）收到 RUNNING 后，下一帧会再次调用本节点，直到返回 SUCCESS。
 */
static BT_Status Act_Chassis_Move_FSM_MC(void)
{
    switch (task_chassis_state) {
        
        case CHASSIS_IDLE:
            /* 刚进入本节点，下发前往HOME的指令 */
            taskENTER_CRITICAL(); // 下发轨迹队列涉及到共享资源，最好进入临界区保护
            chassis_route_point_add(&chassis_route_traj, CHASSIS_HOME_MC_X, CHASSIS_HOME_MC_Y, CHASSIS_HOME_MC_W, 2000);
            taskEXIT_CRITICAL();
            
            task_chassis_state = CHASSIS_TO_RACK;
            chassis_timer_ms = 0; // 重置底盘任务计时器
            return BT_RUNNING;    // 告诉上层 BT 树：“我这个节点还没做完，你需要继续调用我”

        case CHASSIS_TO_RACK:
            /* 检查是否到达HOME点 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                /* HOME点到了，立马前往RACK点 */
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, CHASSIS_RACK_6_X, CHASSIS_RACK_6_Y, CHASSIS_RACK_6_W, 15000); 
                taskEXIT_CRITICAL();
                
                task_chassis_state = CHASSIS_TO_DOCKING;
                chassis_timer_ms = 0;
            } 
            /* 判定动作是否超时卡死 */
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE; // 超时，返回失败。上方的 Sequence 会直接中断整个综合任务
            }
            return BT_RUNNING;

        case CHASSIS_TO_DOCKING:
            /* 检查是否完成取矛头 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                /* 暂定取矛头瞬间成功，直接前往对接点 */
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, CHASSIS_DOCKING_X, CHASSIS_DOCKING_Y, CHASSIS_DOCKING_W, 10000);
                taskEXIT_CRITICAL();

                task_chassis_state = CHASSIS_TO_SCANNING; // 状态推进，防止下次误入
                chassis_timer_ms = 0;
                return BT_RUNNING;   
            } 
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        
        case CHASSIS_TO_SCANNING:
            /* 检查是否完成对接 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                /* 暂定对接瞬间成功，直接前往扫描点 */
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, CHASSIS_SCANNING_X, CHASSIS_SCANNING_Y, CHASSIS_SCANNING_W, 10000); 
                taskEXIT_CRITICAL();

                task_chassis_state = CHASSIS_TO_PICK; // 状态推进，防止下次误入
                chassis_timer_ms = 0;
                return BT_RUNNING;   
            } 
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        
        case CHASSIS_TO_PICK:
            /* 检查是否扫描完毕 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                /* 假设扫描瞬间成功前往拾取点 */
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, CHASSIS_ENTRY_2_X, CHASSIS_ENTRY_2_Y, CHASSIS_ENTRY_2_W, 10000); 
                taskEXIT_CRITICAL();

                /* 当到达拾取点之后，返回bt_success，将行为树往下推进，控制权给到机械臂部分 */
                task_chassis_state = CHASSIS_TO_PICK_DONE; // 状态推进，防止下次误入
                chassis_timer_ms = 0;
                /* 目前期望的拾取点到了，向行为树返回 SUCCESS，行为树将调用下一个节点(机械臂拾取) */
                return BT_RUNNING;   
            } 
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE;
            }
            return BT_RUNNING;
                    
        case CHASSIS_TO_PICK_DONE:
            /* 检测是否到达拾取点 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                /* 在到达拾取点后返回SUCCESS，BT下一步调用机械臂节点 */
                chassis_timer_ms = 0;
                return BT_SUCCESS; 
            }
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE;
            }
            return BT_RUNNING;

        // 默认未知情况返回FAILURE，保护行为树不被卡死
        default:
            return BT_FAILURE;
    }
}

/**
 * @brief [动作节点] 阶段二：操控机械臂去拾取方块，并包含容错重试和最终跳过机制
 * @note 【核心逻辑剖析】：
 *       行为树中的默认机制：一旦子节点返回 BT_FAILURE，作为父节点的 Sequence 就会彻底崩溃并中断。
 *       为了实现“失败重试并在2次失败后依然能去待机区”，我们需要在这个中间层实施“善意的欺骗”：
 *       1. 截获底层 FSM 的 BT_FAILURE。
 *       2. 如果还能重试：拦截FAILURE，向行为树上报 BT_RUNNING（继续假装自己还在干活），
 *          借由底层FSM已自我归位 PICK_IDLE，下一次进入必定会重新启动抓取；
 *       3. 如果无药可救：拦截FAILURE，向行为树上报 BT_SUCCESS（假装自己成功了），
 *          让行为树认为“抓完了”，从而顺畅地走向下一步，也就是回到待机点。
 */
static BT_Status Act_Arm_Pick_With_Retry(void)
{
    /* 调用独立分离好的底层机械臂 FSM，底层 FSM 自己本身并不带重试逻辑，它只会干干脆脆地返回 SUCCESS 或 FAILURE */
    // 暂定使用取高200的 FSM 作为示例
    BT_Status status = Act_Arm_Pick_FSM_P200(); 

    if (status == BT_SUCCESS) {
        /* 如果一次就成功了，皆大欢喜，重置计数并放行 */
        bb.retry_count = 0;     
        return BT_SUCCESS;      
    } 
    else if (status == BT_FAILURE) {
        /* 如果底层 FSM 返回了抓取失败（例如吸盘没戏住、位置超差超时） */
        bb.retry_count++;
        if (bb.retry_count < 2) { 
            /* 【还未达到最大失败次数，启动重试】
             * 底层抓取 FSM 在返回 FAILURE 前其实已经自己重置 pick_state = PICK_IDLE 了。
             * 我们故意给上面的行为树返回 RUNNING，不让 Sequence 断掉。 
             */
            return BT_RUNNING;
        } else {
            /* 【超过2次失败，强行放弃抓取】
             * 按照需求，“如果还是失败的话就跳过拾取动作，直接底盘运行到待机位置”。
             * 所以我们向 Sequence 节点返回 SUCCESS (善意假行骗)，这样下一个节点(底盘去待机区)就能得到执行！
             */
            bb.retry_count = 0;
            return BT_SUCCESS; 
        }
    }

    return BT_RUNNING; // 如果底层既不是 SUCCESS 也不是 FAILURE，说明依然在干活中
}

/**
 * @brief [动作节点] 阶段三：底盘无论成败，最后走向待机（休眠基地）位置
 */
static BT_Status Act_Chassis_MoveToStandby(void)
{
    switch (task_chassis_state) {
        case CHASSIS_TO_PICK_DONE:
            /* 从抓取完成跳入本状态机后，下发待机点指令 */
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, CHASSIS_HOME_MC_X, CHASSIS_HOME_MC_Y, CHASSIS_HOME_MC_W, 2000);
            taskEXIT_CRITICAL();
            
            task_chassis_state = CHASSIS_TO_STANDBY;
            chassis_timer_ms = 0;
            return BT_RUNNING;

        case CHASSIS_TO_STANDBY:
            /* 等待到达最终待机处 */
            if (chassis_route_traj.traj_vx.is_finished && 
                chassis_route_traj.traj_vy.is_finished && 
                chassis_route_traj.traj_vw.is_finished) 
            {
                task_chassis_state = CHASSIS_DONE;
                /* 所有旅程终结，返回成功 */
                return BT_SUCCESS; 
            } 
            else if (++chassis_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE;
            }
            return BT_RUNNING;

        case CHASSIS_DONE:
            return BT_SUCCESS;

        default:
            return BT_FAILURE;
    }
}

/**
 * @brief [动作节点] 阶段四：清道夫节点，综合任务全部落地后负责状态量归位
 */
static BT_Status Act_Combined_Task_Finish_Cleanup(void)
{
    task_chassis_state = CHASSIS_IDLE; // 复位自己的任务状态机，方便下一次唤醒
    task_request_flag = 0;              // 任务结束，清空任务总起请求标志
    
    // 因为抓取状态机目前使用的是之前写的测试版本，所以这里需要复位之前的抓取状态机状态和黑板重试计数
    bb.req_pick = 0;    
    bb.retry_count = 0;
    /* 返回 SUCCESS 意味着这个行为树的最后一步走完了，整个 Sequence 完美收官 */
    return BT_SUCCESS;
}


/* ================= 3. 顶层行为树统筹 (Root Node) ================= */

/**
 * @brief 综合工作流行为树 Root 根节点
 * @note 使用 Memory Sequence (带有记忆功能的顺序执行器)。
 *       其特点是：它会按数组顺序逐个执行节点。如果某节点返回 RUNNING，它就会记住当前位置，下一次 Tick 时直接从该节点继续，
 *       而不用从头开始重查前面的请求。这对于这种长耗时（数秒才能到位）的工作流来说至关重要。
 */
static BT_Status BT_Combined_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_Combined_Task_Request,   // 1. 判断是否被触发启动
        Act_Chassis_Move_FSM_MC,   // 2. 底盘分两段走向拾取区
        Act_Arm_Pick_With_Retry,          // 3. 机械臂拾取（带“包皮” Wrapper 层，掩盖失败重试跳过逻辑）
        Act_Chassis_MoveToStandby,        // 4. 底盘走回待命区
        Act_Combined_Task_Finish_Cleanup  // 5. 将任务状态位全部重置清除
    };

    /* (uint8_t)(sizeof(seq) / sizeof(seq[0])) = 5 (一共有 5 个子节点) */
    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &seq_mem_idx_combined);
}


/* ================= 4. 外部调用接口 ================= */

/**
 * @brief 外部函数：将启动任务标志位置为1 （可被按键中断、通信协议等调用）
 */
void BT_Start_Combined_Task(void)
{
    task_request_flag = 1;
    bb.retry_count = 0; // 每次新任务开始时重置重试计数
}

/**
 * @brief 外部函数：重置/初始化综合任务状态（初始化或者紧急状态下调用）
*/
void BT_Init_Combined_Task(void)
{
    task_request_flag = 0;
    task_chassis_state = CHASSIS_IDLE;
    bb.req_pick = 0;    
    bb.retry_count = 0;
    seq_mem_idx_combined = 0;
}

/**
 * @brief BT 挂置于 FreeRTOS 任务调度器的主 Tick 执行入口 (放假设定的 1ms 任务中周期调用)
 */
void BT_Customized_Tick1ms_Combined(void)
{
    /* 使用 10 ms 的时间片来调度行为树以节省 CPU 算力资源 */
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; 
    div10 = 0;
    
    /* 运行顶层的行为树控制器 */
    BT_Status status = BT_Combined_Task_Tree_Root();
    
    /* 
     * [异常自愈机制] : 
     * 若在底盘寻路时不幸卡墙，导致底盘模块在第2步底盘或第4步直接抛出了纯粹的 BT_FAILURE 给最外层，
     * Memory Sequence 树由于抛出 FAILURE 会被彻底击碎打断。为了避免卡死再也起不来，
     * 我们须立刻手动执行状态清理，并干掉记忆位置 `seq_mem_idx_combined = 0`。
     */
    if (status == BT_FAILURE) {
        Act_Combined_Task_Finish_Cleanup();
        seq_mem_idx_combined = 0; 
    }
}
