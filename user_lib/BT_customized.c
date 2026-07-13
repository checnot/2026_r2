#include "BT_customized.h"

#include "keyboard.h"

/* ================= 万能小变�?================= */
uint8_t debug_if_reach_MF_phase2_scan = 0; // 用于调试是否达到 MF 区第二次扫描的条件，实际比赛时请删除或替换为 BuildPhase2 调用

/* ================= 万能小变�?================= */


ArmPickState pick_state = PICK_IDLE; /* 单独开发时使用抓取动作的状态机当前状�?*/


uint32_t pick_timer_ms = 0; /* �?BT tick (10ms为周�? 计数，或�?Tick1ms 中累�?*/
uint32_t chassis_timer_ms = 0 ;

/* 保存 Sequence 的记忆索引（Memory Sequence 使用�?*/
uint8_t seq_mem_idx = 0;
uint8_t seq_mem_idx_combined = 0;

/* ------------ 黑板 (行为树状态共享区) ------------ */
struct {
    uint8_t req_pick;     /* 发起一次自动取放请�?(1 = 有请�? */
    uint8_t req_chassis_route; /* 发起一次底盘路径请�?(1 = 有请�? */
    uint8_t retry_count;  /* 重试计数（示例） */
} bb;
/* ------------ 黑板 (行为树状态共享区) ------------ */








// /*=========================================================== 机械�?/ 底盘 行为树测试用 ===============================================================*/
// /*=========================================================== 机械�?/ 底盘 行为树测试用 ===============================================================*/
// /*=========================================================== 机械�?/ 底盘 行为树测试用 ===============================================================*/










// /* ========================================================== 底盘部分 ========================================================== */




// /*================================ 辅助函数 ================================*/

/**
 * @brief 判断机械臂末端是否在 (tx,ty) 附近且轨迹队列为空（已稳定）
*/
static inline uint8_t arm_if_arrived_xy(float tx, float ty)
{
    float dx = fabsf(arm.x0_now - tx);
    float dy = fabsf(arm.y0_now - ty);
    return (dx < ARRIVE_TOL && dy < ARRIVE_TOL
        && arm_traj.traj_queue_x.count == 0
        && arm_traj.traj_queue_y.count == 0
        && arm_traj.wait_counter == 0);
}
/**
 * @brief 判断底盘是否达到目标位置附近且轨迹列表已空（已稳定）
 */
static inline uint8_t chassis_if_arrived(float tx, float ty, float tw)
{
    float dx = fabsf(Radar_Pos.reliable_x - tx);
    float dy = fabsf(Radar_Pos.reliable_y - ty);
    float dw = fabsf(chassis_angle_edge_protection(tw, Radar_Pos.reliable_yaw));
    return ((dx < CHASSIS_ROUTE_ARRIVAL_THRESHOLD )
    && (dy < CHASSIS_ROUTE_ARRIVAL_THRESHOLD )
    && (dw < CHASSIS_ANGLE_ARRIVAL_THRESHOLD)
    && (chassis_route_traj.traj_queue_vx.count == 0)
    && (chassis_route_traj.traj_queue_vy.count == 0)
    && (chassis_route_traj.traj_queue_vw.count == 0));
}
/*================================ 条件 节点 ================================*/

/**
 * @brief 条件节点：判断是否有一个机械臂取放块请�?
*/
static BT_Status Cond_Has_Arm_PickRequest(void)
{
    return bb.req_pick ? BT_SUCCESS : BT_FAILURE;
}

/*================================ 行为/动作 节点 ================================*/

/**
 * @brief 机械臂抓取状态机（高200），根据机械臂运行状态和计时，自动推进抓取动作的各个阶段
 * @return 返回 BT_RUNNING / BT_SUCCESS / BT_FAILURE
 * @note - 上层BT把控制流交到这个节点后，它会在内部按状态机推进�?
 * @note - 每次�?tick 时，FSM 只推进有限步并返回当前状态�?
*/
static BT_Status Act_Arm_Pick_FSM_P200(void)
{
    /* 该函数由 BT_Customized_Tick1ms �?10ms 调用一次（示例），
       pick_timer_ms 的单位与 Tick 调用频率对应（本示例�?10ms 计数）�?*/

    switch (pick_state) {       //根据当前状态执行对应的动作，并根据条件推进状态机

    case PICK_IDLE:
        /* 第一次进入：回归�?HOME 位置，并进入 APPROACH */
        taskENTER_CRITICAL(); /* 保护轨迹队列写入，或使用 mutex */
        arm_queue_add_point(&arm_traj, ARM_HOME_X, ARM_HOME_Y, 0, 0, 0);
        taskEXIT_CRITICAL();
        pick_state = PICK_APPROACH_1;
        pick_timer_ms = 0;
        return BT_RUNNING;

    case PICK_APPROACH_1:
        /* 等待到达预抓位置 */
        if (arm_if_arrived_xy(ARM_HOME_X, ARM_HOME_Y)) {
            /* 到位后下发预抓取�?*/
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_APPROACH_X_1, ARM_PICK_P200_APPROACH_Y_1, 0, 0, 0);
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
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_APPROACH_X_2, ARM_PICK_P200_APPROACH_Y_2, 0, 0, 0);
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
            arm_queue_add_point(&arm_traj, ARM_PICK_P200_TARGET_X, ARM_PICK_P200_TARGET_Y, ARM_PICK_P200_TARGET_TIME, ARM_PUMP_SUCK, 0);
            taskEXIT_CRITICAL();
            pick_state = PICK_STORE; /* 直接进入存储阶段，等待轨迹完成后再判断是否成�?*/
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
    //         arm_queue_add_point(&arm_traj, ARM_SAFE_P200_LIFT_X, ARM_SAFE_P200_LIFT_Y, ARM_SAFE_P200_LIFT_TIME, ARM_SUCK_ON);
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
            arm_queue_add_point(&arm_traj, ARM_STORE_X, ARM_STORE_Y, 0, ARM_PUMP_SUCK, 0);
            taskEXIT_CRITICAL();
            pick_state = PICK_COMPLETE; /* 进入完成阶段，等待轨迹完成后再判断是否成�?*/
            pick_timer_ms = 0;
        } else {
            if (++pick_timer_ms * 10 >= ACTION_TIMEOUT_MS) {
                pick_state = PICK_FAILED;
            }
        }
        return BT_RUNNING;

    case PICK_COMPLETE:
        /* 等待到达存储�?*/
        if (arm_if_arrived_xy(ARM_STORE_X, ARM_STORE_Y)) {
            /* 到位时返回success */
            pick_state = PICK_IDLE; /* 进入完成阶段，等待轨迹完成后再判断是否成�?*/
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
        arm_queue_add_point(&arm_traj, ARM_HOME_X, ARM_HOME_Y, 0, ARM_PUMP_RELEASE, 0);
        pick_state = PICK_IDLE;
        pick_timer_ms = 0;
        bb.req_pick = 0; /* 失败时清除请求，等待下一次触�?*/

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
    // seq_mem_idx = 0;    /* 重置 Sequence 记忆，方便下一次请�?*/
    return BT_SUCCESS;
}


/*================================ 上层行为树根（简化，�?Memory Sequence 保持进度�?================================*/

/**
 * @brief 简单的机械臂取放行为树根节点，使用 Memory Sequence 来保持状态机进度
*/
static BT_Status BT_SimplePickTree_Root(void)
{
    /* 按顺序执行：
     *  1) 有请求？
     *  2) 执行下层抓取 FSM（可能多 Tick 返回 RUNNING�?
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
/* �?BT_customized.h 中声明这些函数，并在 freertos 的初始化处调�?Init，在 cmd_task 中定期调�?Tick */

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
 * @brief 外部触发器（例如遥控上升沿）调用此函数发起一次取放请�?
 */
void BT_RequestSimplePick(void)
{
    bb.req_pick = 1;
}


/*=============================== tick ===============================*/

/**
 * @brief 行为树tick，每 1ms 调用一次；内部降频�?10ms 用于BT树调度，避免过于频繁遍历
*/
void BT_Customized_Tick1ms_arm_test(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; /* �?10ms 执行一�?BT Tick */

    //清空计时，遍历行为树
    div10 = 0;
     (void)BT_SimplePickTree_Root();
   
}


/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */



/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */
/* ================================================ 以上弃用 ================================================ */







/* ================================================ MC 规划行为�?================================================ */
/* ================================================ MC 规划行为�?================================================ */
/* ================================================ MC 规划行为�?================================================ */



MC_Blackboard MC_bb; //一区行为树的黑板，存放状态量                     


/**
 * @brief [条件节点]检查是否收到了一区任务的执行请求
 * @return BT_SUCCESS (收到了，继续往下执�? / BT_FAILURE (没收到，打断执行�?
*/
static BT_Status Cond_Has_R2_MC_Mission_Request(void)
{
    return MC_bb.req_pick ? BT_SUCCESS : BT_FAILURE;
}


/**
 * @brief [动作节点]一区任务执�?
 * @return "耗时动作"，因此在执行过程中我们会一直返�?BT_RUNNING（挂起）。上层记忆序列节点（BT_Sequence_Memory）收�?RUNNING 后，下一帧会再次调用本节点，直到返回 SUCCESS�?
*/
static BT_Status Act_R2_MC_Mission(void)
{
    static const struct { float x, y, w; } mc_rack[6] = {
        {R_CHASSIS_RACK_3_X, R_CHASSIS_RACK_Y,   R_CHASSIS_RACK_W},
        {R_CHASSIS_RACK_2_X, R_CHASSIS_RACK_Y,   R_CHASSIS_RACK_W},
        {R_CHASSIS_RACK_1_X, R_CHASSIS_RACK_Y_1, R_CHASSIS_RACK_W_1},
        {R_CHASSIS_RACK_4_X, R_CHASSIS_RACK_Y,   R_CHASSIS_RACK_W},
        {R_CHASSIS_RACK_5_X, R_CHASSIS_RACK_Y,   R_CHASSIS_RACK_W},
        {R_CHASSIS_RACK_6_X, R_CHASSIS_RACK_Y,   R_CHASSIS_RACK_W},
    };
    switch (MC_bb.mc_mission_state) {

        case MC_IDLE:
        {
            //  空任务状态，等待触发进入MC任务流程，进入后直接推进状态机到第一个阶�?

            MC_bb.mc_mission_state = CLAMP_INIT; 
            MC_bb.R2_MC_timer_ms = 0; // 重置计时�?
            return BT_RUNNING;
        }
        case CLAMP_INIT:
        {
            //夹爪位置初始化指�?
            clamp.lift_flag = 1;
            clamp.rotate_flag = 1;
            MC_bb.mc_mission_state = NIVEGATE_TO_RACK;
            MC_bb.R2_MC_timer_ms = 0;
            return BT_RUNNING;
        }
        case NIVEGATE_TO_RACK:
        {
            /* 表驱动：按 mc_rack 顺序从索引0开始取矛 */
            MC_bb.rack_retry_count = 0;
            taskENTER_CRITICAL(); 
            chassis_route_point_add(&chassis_route_traj, mc_rack[0].x, mc_rack[0].y, mc_rack[0].w, 0, 1);
            taskEXIT_CRITICAL();

            MC_bb.mc_mission_state = RACK_PICK; 
            MC_bb.R2_MC_timer_ms = 0;
            MC_bb.RACK_POS_STABLE_TIMES = 0;
            MC_bb.ext_jam_retries = 0;
            MC_bb.ext_timer_ms = 0;
            MC_bb.last_ext_angle = 0;
            return BT_RUNNING;
        }
        case RACK_PICK:
        {
            /* 表驱动取矛头：rack_retry_count 索引 mc_rack 表 */
            uint8_t r = MC_bb.rack_retry_count;
            if (chassis_if_arrived(mc_rack[r].x, mc_rack[r].y, mc_rack[r].w))
            {
                MC_bb.RACK_POS_STABLE_TIMES++;
                if (MC_bb.RACK_POS_STABLE_TIMES > R_RACK_STABLE_TIMES_CONDITION)
                {
                    clamp.retractable_flag = 1;
                    MC_bb.last_ext_angle = m3508_fdcan2[1].Angle_Sum; /* 记录伸出起始角度 */
                   
                    {
                        if (abs_int32(m3508_fdcan2[1].Angle_Sum - CLAMP_EXTEND_MAX_POSITION) < 8000)
                        {
                            /* 伸到位 → 抓取 */
                            clamp.catch_flag = 1;
                            osDelay(400);
                            if (MC_bb.RACK_PICK_SUCCESS == 1)
                            {
                                clamp.rotate_flag = 0;
                                MC_bb.mc_mission_state = DOCKING;
                                MC_bb.R2_MC_timer_ms = 0;
                                return BT_RUNNING; /* 防止穿透到堵转检测 */
                            }
                            else
                            {
                                /* 抓取失败 → 换下一个矛架 */
                                clamp.catch_flag = 0;
                                clamp.retractable_flag = 0;
                                osDelay(300);
                                MC_bb.rack_retry_count++;
                                if (MC_bb.rack_retry_count >= 6)
                                {
                                    MC_bb.rack_retry_count = 0;
                                }
                                taskENTER_CRITICAL();
                                chassis_route_point_add(&chassis_route_traj, mc_rack[MC_bb.rack_retry_count].x, mc_rack[MC_bb.rack_retry_count].y, mc_rack[MC_bb.rack_retry_count].w, 0, 0);
                                taskEXIT_CRITICAL();
                                MC_bb.ext_jam_retries = 0;
                                MC_bb.ext_timer_ms = 0;
                                MC_bb.RACK_POS_STABLE_TIMES = 0;
                                MC_bb.R2_MC_timer_ms = 0;
                            }
                        }
                    }
                }
            }
            if (clamp.retractable_flag == 1)
            {
                /*
                 * 夹爪伸出中，若电机Angle_Sum连续不变（堵转/撞歪），
                 * 计时3秒后缩回重试，每个位置仅1次重试机会。
                 * 若电机恢复转动则清零计时。
                 */
                int32_t cur_angle = m3508_fdcan2[1].Angle_Sum;
                if (abs_int32(cur_angle - MC_bb.last_ext_angle) < 3000)
                {
                    
                     
                    

                    /* 本Tick电机基本没动，累计堵转计时 */
                    if (++MC_bb.ext_timer_ms * 10 >= R_EXT_TIMEOUT_MS)
                    {
                            clamp.catch_flag = 1;
                            osDelay(400);
                            if (MC_bb.RACK_PICK_SUCCESS == 1)
                            {
                                clamp.rotate_flag = 0;
                                MC_bb.mc_mission_state = DOCKING;
                                MC_bb.R2_MC_timer_ms = 0;
                                return BT_RUNNING; /* 防止穿透到堵转检测 */
                            }
                            else
                            {
                                clamp.catch_flag = 0;
                            }
                        MC_bb.ext_timer_ms = 0;
                        if (MC_bb.ext_jam_retries >= R_EXT_MAX_RETRIES )
                        {
                            /* 已重试过1次，放弃此位置，换下一个矛架 */
                            MC_bb.ext_jam_retries = 0;
                            MC_bb.ext_timer_ms = 0;
                            MC_bb.RACK_POS_STABLE_TIMES = 0;
                            clamp.catch_flag = 0;
                            clamp.retractable_flag = 0;
                            MC_bb.rack_retry_count++;
                            if (MC_bb.rack_retry_count >= 6)
                            {
                                MC_bb.rack_retry_count = 0;
                            }
                            taskENTER_CRITICAL();
                            chassis_route_point_add(&chassis_route_traj, mc_rack[MC_bb.rack_retry_count].x, mc_rack[MC_bb.rack_retry_count].y, mc_rack[MC_bb.rack_retry_count].w, 0, 0);
                            taskEXIT_CRITICAL();
                            osDelay(300);
                        }
                        else
                        {
                            MC_bb.ext_jam_retries++;       /* 记录本次重试 */
                            clamp.retractable_flag = 0;    /* 缩回重试 */
                            MC_bb.ext_timer_ms = 0;
                            MC_bb.RACK_POS_STABLE_TIMES = 0; /* 重试时重新等待稳定 */
                        }
                    }
                }
                else
                {
                    /* 电机仍在转动，重置堵转计时 */
                    MC_bb.ext_timer_ms = 0;
                }
                MC_bb.last_ext_angle = cur_angle; /* 始终更新，比较相邻Tick差值 */
            }

            return BT_RUNNING;
        }
        case DOCKING:
        {
            /* 进入这个状态后,执行寻路到达对接点的动作*/
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_DOCKING_X, R_CHASSIS_DOCKING_Y, R_CHASSIS_DOCKING_W, 0, 0);
            taskEXIT_CRITICAL();
           

            MC_bb.mc_mission_state = WAIT_DOCKING_COMPLETE;
            MC_bb.R2_MC_timer_ms = 0;
            return BT_RUNNING;
        }
        case WAIT_DOCKING_COMPLETE:
        {
            if(chassis_if_arrived(R_CHASSIS_DOCKING_X,R_CHASSIS_DOCKING_Y,R_CHASSIS_DOCKING_W))
            {
                clamp.lift_flag = 0;
                chassis_set_lock(1);  /* 到达对接点，四轮锁止 */
            }
        
            /* 进入这个状态后,等待对接完成的动作*/
            if(MC_bb.DOCKING_SUCCESS == 1)
            {
                clamp.catch_flag = 0;
                osDelay(500);
                clamp.rotate_flag = 1;
                clamp.retractable_flag = 0;
                chassis_set_lock(0);  /* 解锁，恢复底盘控制 */
                MC_bb.mc_mission_state = WAIT_MF_INSTRUCTION;
                MC_bb.R2_MC_timer_ms = 0;
            }
            else if(++MC_bb.R2_MC_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS*2) {
                return BT_FAILURE; 
            }
            return BT_RUNNING;
        }
        case WAIT_MF_INSTRUCTION:
        {
            /* 进入这个状态后,等待R1发出进入MF的指令，暂定为一个flag */
            if(MC_bb.MF_ENTRY_INSTRUCTION == 1)
            {
                //发送去扫描点的任务
              taskENTER_CRITICAL();
              chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SCANNING_X, R_CHASSIS_SCANNING_Y, R_CHASSIS_SCANNING_W, 0, 0);
              taskEXIT_CRITICAL();
              clamp.retractable_flag = 0;
              clamp.rotate_flag = 0; //旋转夹爪，避免干涉
			  MC_bb.mc_mission_state = CAMERA_SCAN;
              MC_bb.R2_MC_timer_ms = 0;
              
            }
            else if(++MC_bb.R2_MC_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE; 
            }
            return BT_RUNNING;  
        }
        case CAMERA_SCAN:
        {
            if (chassis_if_arrived(R_CHASSIS_SCANNING_X, R_CHASSIS_SCANNING_Y, R_CHASSIS_SCANNING_W))
            {
                /* 暂定立马完成扫描*/
                MC_bb.mc_mission_state = MF_TASK_FLAG;
                MC_bb.R2_MC_timer_ms = 0;
				

            }
            else if(++MC_bb.R2_MC_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
                return BT_FAILURE; 
            }
            return BT_RUNNING;
        }
        case MF_TASK_FLAG:
        {
            MC_bb.MF_MISSION_Req = 1;
            MC_bb.mc_mission_state = MC_IDLE; //任务完成，回到初始状态等待下一次触�?
            MC_bb.R2_MC_timer_ms = 0;
            return BT_SUCCESS;

        }

        default:
        {
            return BT_FAILURE;
        }
    }
}
 
/**
 * @brief [动作节点]一区任务完成后的收尾清理动作，重置黑板状态和 Sequence 记忆
*/
static BT_Status Act_R2_MC_FinishCleanup(void)
{
    MC_bb.req_pick = 0; //清除任务请求标志，等待下一次触�? 
    MC_bb.RACK_POS_STABLE_TIMES = 0; //重置矛头位置稳定计数�?  
    MC_bb.mc_mission_state = MC_IDLE; //重置任务状态机状态，准备下一次任务流�? 
    return BT_SUCCESS;
}

/**
 * @brief 综合工作流行为树 Root 根节�?
 * @note 使用 Memory Sequence (带有记忆功能的顺序执行器)。其特点是：它会按数组顺序逐个执行节点�?
*/
static BT_Status BT_R2_MC_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_R2_MC_Mission_Request, // 1. 判断是否被触发启动
        Act_R2_MC_Mission,              // 2. 执行一区任务的主体流程（包含寻路、夹取、对接等多个阶段）
        Act_R2_MC_FinishCleanup         // 3. 任务完成后的状态清除
    };

    /* (uint8_t)(sizeof(seq) / sizeof(seq[0])) = 3 (一共有 3 个子节点) */
    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &MC_bb.seq_mem_idx_combined);
}

/**
 * @brief MC任务的外部初始化函数
*/
void BT_Init_R2_MC_Mission(void)
{
    MC_bb.req_pick = 0; // 初始化时没有任务请求
    MC_bb.mc_mission_state = MC_IDLE; // 任务状态机回到初始状态
    MC_bb.seq_mem_idx_combined = 0; // 初始化行为树记忆索引
    MC_bb.R2_MC_timer_ms = 0; // 初始化任务计时器   
    MC_bb.RACK_POS_STABLE_TIMES = 0; // 初始化矛头位置稳定计数器
    MC_bb.RACK_PICK_SUCCESS = 0; // 初始化夹取成功标志
    MC_bb.DOCKING_SUCCESS = 0; // 初始化对接成功标志   
    MC_bb.MF_ENTRY_INSTRUCTION = 0; // 初始化等待MF指令标志 
    MC_bb.MF_MISSION_Req = 0; // 初始化MF任务重置标志
    MC_bb.ext_jam_retries = 0;
    MC_bb.ext_timer_ms = 0;
    MC_bb.last_ext_angle = 0;
    MC_bb.rack_retry_count = 0; // 初始化矛架索引
}

/**
 * @brief 武馆任务外部函数：将启动一区任务标志位置为1 
*/
void BT_Start_R2_MC_Mission(void)
{
    MC_bb.req_pick = 1;
}

/**
 * @brief 武馆任务 
 * @note 挂置�?FreeRTOS 任务调度器的�?Tick 执行入口 (设定 1ms 任务中周期调�?
*/
void BT_Customized_Tick1ms_R2_MC(void)
{
    /* 使用 10 ms 的时间片来调度行为树以节�?CPU 算力资源 */
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return; 
    div10 = 0;
    
    /* 运行顶层的行为树控制�?*/
    BT_Status status = BT_R2_MC_Tree_Root();
    
    /* [异常自愈机制] : 若在执行过程中不幸卡死在某个阶段，导致底�?FSM 抛出了纯粹的 BT_FAILURE 给最外层，Memory Sequence 树由于抛�?FAILURE 会被彻底击碎打断。为了避免卡死再也起不来，我们须立刻手动执行状态清理，并干掉记忆位�?`seq_mem_idx_r2_mc = 0`�?*/
    if (status == BT_FAILURE) {
        Act_R2_MC_FinishCleanup();
        MC_bb.seq_mem_idx_combined = 0; 
    }
}                   







/* ================================================ MF 规划行为�?================================================ */
/* ================================================ MF 规划行为�?================================================ */
/* ================================================ MF 规划行为�?================================================ */




/*
 * 【架构说明�?
 * 1. MF 规划�?mf_planner_ctx)一次性生成完整动作列�?
 * 2. 行为树节�?Act_MF_Execute_Action 内部�?FSM 逐个消费动作
 * 3. 每个 MF_Action 映射到对应的底层 task 动作标志�?动作调用
 * 4. �?MC 组合任务一样，�?Memory Sequence 保持进度
 */

/* ================= 1. MF 任务专属状态区 ================= */

MF_Blackboard mf_bb;                   /* MF 任务黑板，存放状态量 */

MF_ExecState mf_exec_state = MF_EXEC_IDLE;  /* MF 动作执行状态机 */
uint32_t mf_exec_timer_ms = 0;              /* MF 执行计时�?*/
uint8_t mf_task_request_flag = 0;           /* MF 任务请求标志 (1=有请�? */
uint8_t mf_phase_flag = 0;                  /* 规划阶段标记: 0=未规�? 1=Phase1完成, 2=全部规划完成 */

uint8_t mf_picked_phase1 = 0;               /* Phase1 已拾�?R2KFS 数量 */
uint8_t mf_picked_kfs_count = 0;            /* 已执行的 PICK_R2 数量：0=下一块存储，1=下一块手持 */
static uint8_t mf_r2_kfs_all_need = R2_KFS_ALL_NEED;
static uint8_t mf_finish_start_cz = 1;

#define MF_PREINPUT_START_ROW 0
#define MF_PREINPUT_START_COL 1

uint8_t arm_if_send_point = 0; /* 机械臂是否已经下发轨迹点标志位，避免重复下发 */

typedef enum {
    ARM_FRONT_DISCARD_IDLE = 0,
    ARM_FRONT_DISCARD_PICK_SENT,
    ARM_FRONT_DISCARD_RELEASE_SENT
} ArmFrontDiscardState;

static ArmFrontDiscardState arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;

uint8_t arm_back_request = 0;     /* 机械臂后向服务请求标志 */
uint8_t arm_back_busy = 0;        /* 机械臂后向服务占用机械臂标志，后台收尾期间仍为 1 */
uint8_t arm_back_payload_done = 0;/* 后向服务完整结束，MF 可以推进 */
uint8_t arm_back_failed = 0;      /* 机械臂后向服务失败标志 */
uint8_t arm_back_slot = 0;        /* 本次存储使用的槽位: 0=A, 1=B, 2=C */

ArmBackState arm_back_state = ARM_BACK_IDLE;   /* 机械臂后向服务状态机 */
uint32_t arm_back_timer_ms = 0;                 /* 机械臂后向服务计时器 */
ArmBackMode arm_back_mode = ARM_BACK_MODE_STORE;/* 后向服务模式: 存储/丢弃 */

/**
 * @brief �?MF 方向转对应的底盘朝向角度(yaw)
 */
static float mf_dir_to_yaw(MF_Dir dir)
{
    switch (dir) {
        case MF_DIR_NORTH: return MF_YAW_NORTH;
        case MF_DIR_EAST:  return MF_YAW_EAST;
        case MF_DIR_SOUTH: return MF_YAW_SOUTH;
        case MF_DIR_WEST:  return MF_YAW_WEST;
        default:           return 0.0f;
    }
}

/**
 * @brief MF 栅格中心坐标 (row, col) 转换为世界坐标系 (x, y)
 * @note 坐标原点 MF_ORIGIN_WORLD_X_BLUE/Y 对应 row0,col0 的中�?
 * @note row 向北递增, col 向东递增
 */
static void mf_cell_to_world(uint8_t row, uint8_t col, float* out_x, float* out_y, MF_MapColor map_color)
{
    if (map_color == BLUE_MAP) {
        *out_x = MF_ORIGIN_WORLD_X_BLUE + (float)row * MF_CELL_SIZE_MM;
        *out_y = MF_ORIGIN_WORLD_Y_BLUE - (float)col * MF_CELL_SIZE_MM;
    } else {
        *out_x = MF_ORIGIN_WORLD_X_RED + (float)row * MF_CELL_SIZE_MM;
        *out_y = MF_ORIGIN_WORLD_Y_RED - (float)col * MF_CELL_SIZE_MM;
    }
}

static void mf_scan_stand_position(const MF_Action* act, float* out_x, float* out_y, float* out_yaw)
{
    if (act->height_pick_diff == MF_SCAN_STAGE_ROW1) {
        mf_cell_to_world(0, 1, out_x, out_y, mf_planner_ctx.map_color);
        *out_x -= MF_SCAN_ROW1_X_OFFSET_MM;
        *out_y -= MF_SCAN_ROW1_Y_OFFSET_MM;
        *out_yaw = MF_YAW_NORTH;
        return;
    }

    if (act->height_pick_diff == MF_SCAN_STAGE_ROW2) {
        mf_cell_to_world(0, 1, out_x, out_y, mf_planner_ctx.map_color);
        *out_x += MF_SCAN_ROW2_X_OFFSET_MM;
        *out_y -= MF_SCAN_ROW2_Y_OFFSET_MM;
        *out_yaw = MF_YAW_NORTH;
        return;
    }

    if (act->height_pick_diff == MF_SCAN_STAGE_ROW34) {
        mf_cell_to_world(1, 1, out_x, out_y, mf_planner_ctx.map_color);
        *out_x += MF_SCAN_ROW34_X_OFFSET_MM;
        *out_y -= MF_SCAN_ROW34_Y_OFFSET_MM;
        *out_yaw = MF_YAW_NORTH;
        return;
    }
    // mf_cell_to_world(act->row, act->col, out_x, out_y, mf_planner_ctx.map_color);
    // *out_yaw = mf_dir_to_yaw(act->dir);
}

// /*
//  * 🟡【需实测】获取当�?R2 所在格的理论高�?(mm)
//  * @note 用于底盘升降控制——知道目标格的高度才能决定升/降多�?
//  */
// static uint16_t mf_get_current_height(void)
// {
//     /*
//      * 🟡【需实测】这里需要返�?R2 当前的底盘高度�?
//      * 方案1：从 MF 规划器内部追踪（需要在 MF_Planner 中加一�?current_row/col 字段�?
//      * 方案2：用底盘升降的实际编码器位置反算
//      * 方案3（推荐）：在 MF_Planner 中新�?MFPlanner_GetCurrentHeight() 函数
//      *
//      * 当前返回 0 作为占位，需替换为实际实现�?
//      */
//     return 0;  /* 🟡 占位，需实测替换 */
// }

/**
 * @brief �?KFS 坐标和拾取方向，反推 R2 应站立的世界坐标
 * @param kfs_row    KFS 所在行
 * @param kfs_col    KFS 所在列
 * @param pick_dir   拾取方向（R2 面对 KFS 的方向）
 * @param out_x      输出：R2 应站立的世界 X
 * @param out_y      输出：R2 应站立的世界 Y
 * @note R2 站在 KFS 的相邻格（反方向一格），然后向 KFS 偏移 500mm
 * @note 坐标系：�?= +X, �?= +Y
 */
static void mf_pick_stand_position(uint8_t kfs_row, uint8_t kfs_col,
                                    MF_Dir pick_dir,
                                    float* out_x, float* out_y)
{
    /*
     * 第一步：确定 R2 站在哪个格子
     * 方向 N �?R2 在南�?(row-1)
     * 方向 S �?R2 在北�?(row+1)
     * 方向 E �?R2 在西�?(col+1, 因为�?+Y)
     * 方向 W �?R2 在东�?(col-1)
     */
    int8_t stand_r = (int8_t)kfs_row;
    int8_t stand_c = (int8_t)kfs_col;

    switch (pick_dir) {
        case MF_DIR_NORTH: stand_r -= 1; break;  /* 面朝北，R2 在南�?*/
        case MF_DIR_SOUTH: stand_r += 1; break;  /* 面朝南，R2 在北�?*/
        case MF_DIR_EAST:  stand_c -= 1; break;  /* 面朝东，R2 在西�?*/
        case MF_DIR_WEST:  stand_c += 1; break;  /* 面朝西，R2 在东�?*/
        default: break;
    }

    /* 第二步：计算站立格中心的世界坐标 */
    float center_x, center_y;
    mf_cell_to_world((uint8_t)stand_r, (uint8_t)stand_c, &center_x, &center_y, mf_planner_ctx.map_color);

    /*
     * 第三步：从格中心�?KFS 方向偏移 200mm
     * MF_CELL_SIZE_MM 是格边长（如 1200mm�?
     * 格中心到格边界的距离 = MF_CELL_SIZE_MM / 2
     * 再向 KFS 靠近，使底盘(600*600)前缘距离台阶边界 100mm
     *
     *
     * 🟡【需实测�?00mm 这个值需要根据底盘实际尺寸调�?
     */
    float offset = 200.0f; /* 站立点距离格中心的偏移量，需实测调整 */

    switch (pick_dir) {
        case MF_DIR_NORTH: *out_x = center_x + offset; *out_y = center_y; break;
        case MF_DIR_SOUTH: *out_x = center_x - offset; *out_y = center_y; break;
        case MF_DIR_EAST:  *out_x = center_x; *out_y = center_y - offset; break;
        case MF_DIR_WEST:  *out_x = center_x; *out_y = center_y + offset; break;
        default: *out_x = center_x; *out_y = center_y; break;
    }
}

/**
 * @brief 从跨越台阶的目标格子和方向，反推 R2 在原有格子时应站立的世界坐标
*/
static void mf_step_stand_position(uint8_t step_row, uint8_t step_col,
                                    MF_Dir step_dir,
                                    float* out_x, float* out_y)
{
    /*
     * 第一步：确定 R2 站在哪个格子
     * 方向 N �?R2 在南�?(row-1)
     * 方向 S �?R2 在北�?(row+1)
     * 方向 E �?R2 在西�?(col-1)
     * 方向 W �?R2 在东�?(col+1)
     */
    int8_t stand_r = (int8_t)step_row;
    int8_t stand_c = (int8_t)step_col;

    switch (step_dir) {
        case MF_DIR_NORTH: stand_r -= 1; break;  /* 面朝北，R2 在南�?*/
        case MF_DIR_SOUTH: stand_r += 1; break;  /* 面朝南，R2 在北�?*/
        case MF_DIR_EAST:  stand_c -= 1; break;  /* 面朝东，R2 在西�?*/
        case MF_DIR_WEST:  stand_c += 1; break;  /* 面朝西，R2 在东�?*/
        default: break;
    }

    /* 第二步：计算站立格中心的世界坐标 */
    float center_x, center_y;
    mf_cell_to_world((uint8_t)stand_r, (uint8_t)stand_c, &center_x, &center_y, mf_planner_ctx.map_color);

    *out_x = center_x;
    *out_y = center_y;

    // float offset = 200.0f; /* 站立点距离格中心的偏移量，需实测调整 */
    // switch (step_dir) {
    //     case MF_DIR_NORTH: *out_x = center_x + offset; *out_y = center_y; break;
    //     case MF_DIR_SOUTH: *out_x = center_x - offset; *out_y = center_y; break;
    //     case MF_DIR_EAST:  *out_x = center_x; *out_y = center_y - offset; break;
    //     case MF_DIR_WEST:  *out_x = center_x; *out_y = center_y + offset; break;
    //     default: *out_x = center_x; *out_y = center_y; break;
    // }
}

/**
 * @brief 统一下发前向机械臂拾取轨迹（取 R2KFS，存储和丢弃共用）
 * @param height_pick_diff 高度差 (-200/0/+200/+400)
 * @note 在临界区内操作，保护轨迹队列写入
 */
static void arm_send_pick_trajectory(int16_t height_pick_diff)
{
    taskENTER_CRITICAL();
    if (height_pick_diff == -200) {
        arm_queue_add_point(&arm_traj, PICK_N200_APR_X_1, PICK_N200_APR_Y_1, 0, ARM_PUMP_SUCK, -90.0f/180.0f*PI);
        arm_queue_add_point(&arm_traj, PICK_N200_TARGET_X, PICK_N200_TARGET_Y, PICK_TIME, ARM_PUMP_SUCK, -90/180.0f*PI);
        arm_queue_add_point(&arm_traj, PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y, 0, ARM_PUMP_SUCK, -90.0f/180.0f*PI);
    } else if (height_pick_diff == 200) {
        arm_queue_add_point(&arm_traj, PICK_P200_APR_X_1, PICK_P200_APR_Y_1, 0, ARM_PUMP_SUCK, 0.0f);
        arm_queue_add_point(&arm_traj, PICK_P200_TARGET_X, PICK_P200_TARGET_Y, PICK_TIME, ARM_PUMP_SUCK, 0.0f);
        arm_queue_add_point(&arm_traj, PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y, 0, ARM_PUMP_SUCK, 0.0f);
    } else if (height_pick_diff == 400) {
        arm_queue_add_point(&arm_traj, PICK_P400_APR_X_1, PICK_P400_APR_Y_1, 0, ARM_PUMP_SUCK, 0.0f);
        arm_queue_add_point(&arm_traj, PICK_P400_TARGET_X, PICK_P400_TARGET_Y, PICK_TIME, ARM_PUMP_SUCK, 0.0f);
        arm_queue_add_point(&arm_traj, PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y, 0, ARM_PUMP_SUCK, 0.0f);
    } else { /* height_pick_diff == 0 */
        arm_queue_add_point(&arm_traj, PICK_0_APR_X_1, PICK_0_APR_Y_1, 0, ARM_PUMP_SUCK, -90.0f/180.0f*PI);
        arm_queue_add_point(&arm_traj, PICK_0_TARGET_X, PICK_0_TARGET_Y, PICK_TIME, ARM_PUMP_SUCK, -90.0f/180.0f*PI);
        arm_queue_add_point(&arm_traj, PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y, 0, ARM_PUMP_SUCK, 0.0f);
    }
    taskEXIT_CRITICAL();
}

static void arm_send_front_discard_trajectory(void)
{
    taskENTER_CRITICAL();
    arm_queue_add_point(&arm_traj, FRONT_DISCARD_X, FRONT_DISCARD_Y, 0, ARM_PUMP_KEEP, FRONT_DISCARD_THETA);
    arm_queue_add_point(&arm_traj, FRONT_DISCARD_X, FRONT_DISCARD_Y, DISCARD_TIME, ARM_PUMP_RELEASE, FRONT_DISCARD_THETA);
    arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, ARM_PUMP_RELEASE, 0.0f);
    taskEXIT_CRITICAL();
}

static ChassisStepType mf_step_type_from_action(const MF_Action *act)
{
    if (act->type == MF_ACT_STEP_UP) {
        if (act->height_pick_diff == 400) {
            return STEP_UP_400;
        }
        if (act->height_pick_diff == 0 || act->height_pick_diff == 200) {
            return STEP_UP_200;
        }
    } else if (act->type == MF_ACT_STEP_DOWN) {
        if (act->height_pick_diff == -400) {
            return STEP_DOWN_400;
        }
        if (act->height_pick_diff == 0 || act->height_pick_diff == -200) {
            return STEP_DOWN_200;
        }
    }
    return STEP_NONE;
}

/* ================= 统一后向服务 ================= */

static void Arm_Path_Queue_Clear(robotic_arm_traj_stu *traj)
{
    uint16_t i;

    Traj_Path_Queue_Init(&traj->traj_queue_x, 0.025f);
    Traj_Path_Queue_Init(&traj->traj_queue_y, 0.025f);
    traj->target_delay_ms = 0;
    traj->wait_counter = 0;

    for (i = 0; i < TRAJ_MAX_WAYPOINTS; i++) {
        traj->traj_queue_x.points[i] = 0.0f;
        traj->traj_queue_y.points[i] = 0.0f;
        traj->delay_queue[i] = 0;
        traj->pump_queue[i] = 0;
        traj->pitch_queue[i] = 0.0f;
    }
}

/**
 * @brief 统一下发后向路径（存储/丢弃共用）
 * @return 1 表示成功下发
 */
void Arm_Back_SendPath(void)
{
    taskENTER_CRITICAL();

    Arm_Path_Queue_Clear(&arm_traj);

    if (arm_back_mode == ARM_BACK_MODE_STORE) {
        /* 存储模式：保持吸附 → 到达目标 → 释放 → 过渡 */
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);
        arm_queue_add_point(&arm_traj, STORE_PRE_X, STORE_PRE_Y, 0, ARM_PUMP_KEEP, STORE_THETA);
        arm_queue_add_point(&arm_traj, STORE_X, STORE_Y, STORE_TIME, ARM_PUMP_KEEP, STORE_THETA);
        arm_queue_add_point(&arm_traj, STORE_X, STORE_Y, STORE_TIME/4, ARM_PUMP_RELEASE, STORE_THETA);
        arm_queue_add_point(&arm_traj, STORE_PRE_TRANS_X, STORE_PRE_TRANS_Y, 0, ARM_PUMP_RELEASE, STORE_PRE_TRANS_THETA);
    } else {
        /* ！！！弃用！！！ */
        /* 丢弃模式：移动到位 → 直接释放 → 过渡 */
        arm_queue_add_point(&arm_traj, DISCARD_PRE_X, DISCARD_PRE_Y, 0, ARM_PUMP_KEEP, DISCARD_THETA);
        arm_queue_add_point(&arm_traj, DISCARD_X, DISCARD_Y, DISCARD_TIME, ARM_PUMP_RELEASE, DISCARD_THETA);
        arm_queue_add_point(&arm_traj, DISCARD_PRE_TRANS_X, DISCARD_PRE_TRANS_Y, 0, ARM_PUMP_RELEASE, DISCARD_PRE_TRANS_THETA);
    }

    taskEXIT_CRITICAL();
}

/**
 * @brief 机械臂后向服务（统一存储/丢弃），负责 后向切换 → 操作 → 返回前向
 * @note 上层通过 arm_back_request/busy/payload_done/done/failed 与本服务通信
 * @note 调用前需设置 arm_back_mode（模式）
 */
static void Arm_Back_Service_Tick(void)
{
    switch (arm_back_state) {
    case ARM_BACK_IDLE:
        if (arm_back_request) {
            arm_back_request = 0;
            arm_back_busy = 1;
            arm_back_payload_done = 0;
            arm_back_failed = 0;
            arm_back_state = ARM_BACK_REQ_F2B;
            arm_back_timer_ms = 0;
        }
        break;

    case ARM_BACK_REQ_F2B:
        taskENTER_CRITICAL();
        arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_FRONT_TO_BACK);
        taskEXIT_CRITICAL();
        arm_back_state = ARM_BACK_WAIT_BACK;
        arm_back_timer_ms = 0;
        break;

    case ARM_BACK_WAIT_BACK:
        if (arm.mode == ARM_MODE_BACK) {
            arm_back_state = ARM_BACK_SEND_PATH;
            arm_back_timer_ms = 0;
        } else if (++arm_back_timer_ms * 10 >=
                   ((arm_back_mode == ARM_BACK_MODE_DISCARD) ? ARM_DISCARD_TRANS_TIMEOUT_MS : ARM_STORE_TRANS_TIMEOUT_MS)) {
            arm_back_state = ARM_BACK_FAILED;
        }
        break;

    case ARM_BACK_SEND_PATH:
            arm_trajectory_init(&arm_traj, &arm);
            Arm_Back_SendPath();
            arm_back_state = ARM_BACK_WAIT_DONE;
            arm_back_timer_ms = 0;

        break;

    case ARM_BACK_WAIT_DONE:
        if ((arm_back_mode == ARM_BACK_MODE_DISCARD &&
             arm_if_arrived_xy(DISCARD_PRE_TRANS_X, DISCARD_PRE_TRANS_Y)) ||
            (arm_back_mode == ARM_BACK_MODE_STORE &&
             arm_if_arrived_xy(STORE_PRE_TRANS_X, STORE_PRE_TRANS_Y))) {

            // if (arm_back_payload_done)
            // {
                // arm_back_payload_done = 0;
            //     arm_if_send_point = 0;

                if (arm_back_mode == ARM_BACK_MODE_STORE)
                {
                    mf_bb.kfs_stored++;
                    mf_picked_kfs_count++;
                }
                mf_exec_timer_ms = 0;
                MFPlanner_AdvanceAction(&mf_planner_ctx);
                mf_exec_state = MF_EXEC_IDLE;
            //     mf_exec_timer_ms = 0;
            // }
            // else if (arm_back_failed)
            // {
            //     arm_back_failed = 0;
            //     mf_exec_state = MF_EXEC_FAILED;
            // }

            arm_back_state = ARM_BACK_REQ_B2F;
            arm_back_timer_ms = 0;
        } else if (++arm_back_timer_ms * 10 >=
                   ((arm_back_mode == ARM_BACK_MODE_DISCARD) ? ARM_DISCARD_PATH_TIMEOUT_MS : ARM_STORE_PATH_TIMEOUT_MS)) {
            arm_back_state = ARM_BACK_FAILED;
        }
        break;

    case ARM_BACK_REQ_B2F:
        taskENTER_CRITICAL();
        arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_BACK_TO_FRONT);
        taskEXIT_CRITICAL();
        arm_back_state = ARM_BACK_WAIT_FRONT;
        arm_back_timer_ms = 0;
        break;

    case ARM_BACK_WAIT_FRONT:
        if (arm.mode == ARM_MODE_FRONT) {
            arm_back_state = ARM_BACK_SEND_REST;
            arm_back_timer_ms = 0;
        } else if (++arm_back_timer_ms * 10 >=
                   ((arm_back_mode == ARM_BACK_MODE_DISCARD) ? ARM_DISCARD_TRANS_TIMEOUT_MS : ARM_STORE_TRANS_TIMEOUT_MS)) {
            arm_back_state = ARM_BACK_FAILED;
        }
        break;

    case ARM_BACK_SEND_REST:
        taskENTER_CRITICAL();
        arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, ARM_PUMP_KEEP, 0.0f);
        taskEXIT_CRITICAL();
        arm_back_state = ARM_BACK_WAIT_REST;
        arm_back_timer_ms = 0;
        break;

    case ARM_BACK_WAIT_REST:
        if (arm_if_arrived_xy(PICK_REST_X, PICK_REST_Y)) {
            arm_back_busy = 0;
            arm_back_payload_done = 0;
            arm_if_send_point = 0;
            arm_back_state = ARM_BACK_IDLE;
        } else if (++arm_back_timer_ms * 10 >=
                   ((arm_back_mode == ARM_BACK_MODE_DISCARD) ? ARM_DISCARD_REST_TIMEOUT_MS : ARM_STORE_REST_TIMEOUT_MS)) {
            arm_back_state = ARM_BACK_FAILED;
        }
        break;

    case ARM_BACK_FAILED:
    default:
        arm_back_busy = 0;
        arm_back_failed = 1;
        break;
    }
}

/* ================= 2. 条件节点 ================= */

/**
 * @brief [条件节点] 检查是否有 MF 任务请求
 */
static BT_Status Cond_Has_MF_Request(void)
{
    MC_bb.MF_MISSION_Req = 1;
    return (MC_bb.MF_MISSION_Req || mf_task_request_flag) ? BT_SUCCESS : BT_FAILURE;
}

/* ================= 3. 动作节点 ================= */

/**
 * @brief 初始化 MF，并使用新版预输入地图做一次性全局规划。
 * @note 新版规划器不再下发 SCAN 动作，也不再依赖 MF_EXEC_SCAN 中的 Phase1/Phase2。
 * @note 当前使用 UART8 矩阵键盘维护的预输入地图，不再载入固定调试地图。
 */
static BT_Status Act_MF_Init_And_Plan(void)
{
    // if (mf_phase_flag >= 2) {
    //     /* 已经规划完成，直接返回成功 */
    //     return BT_SUCCESS;
    // }

    /* 使用键盘预输入地图装载 row1~row4 的 KFS 分布。 */
    if (!MFPlanner_PreInputLoadCells(&mf_planner_ctx, BLUE_MAP,
                                     (const MF_CellType (*)[MF_COLS])keyboard_mf_cell)) {
        mf_exec_state = MF_EXEC_FAILED;
        mf_exec_timer_ms = 0;
        mf_phase_flag = 0;
        return BT_FAILURE;
    }

    mf_bb.kfs_stored = 0;
    mf_picked_kfs_count = 0;
    mf_picked_phase1 = 0;

    if (!MFPlanner_PreInputBuildFullPlan(&mf_planner_ctx,
                                         MF_PREINPUT_START_ROW,
                                         MF_PREINPUT_START_COL,
                                         mf_r2_kfs_all_need)) {
        mf_exec_state = MF_EXEC_FAILED;
        mf_exec_timer_ms = 0;
        mf_phase_flag = 0;
        return BT_FAILURE;
    }
    mf_phase_flag = 2;

    /* 初始化执行状态 */
    mf_exec_state = MF_EXEC_IDLE;
    mf_exec_timer_ms = 0;

    return BT_SUCCESS;
}

/**
 * @brief [动作节点] 阶段二：逐个执行 MF 动作列表
 * @note 这是核心节点。内部用 FSM 消费 mf_planner_ctx.actions[]
 * @note 每次 Tick 推进一步，返回 BT_RUNNING 直到全部完成
 *
 * 🟡🟡🟡【重要】以下每�?case 中标注了 🟡【需实测】的地方�?
 *     需要你根据实际硬件（底盘、机械臂、传感器）填入真实的底层调用�?
 */
static BT_Status Act_MF_Execute_Action(void)
{

    // 如果当前没有正在执行的动作，尝试�?mf_planner_ctx �?actions 列表中取出下一个动�?
    MF_Action act;
    if (!MFPlanner_PeekAction(&mf_planner_ctx, &act)) {
        /* 没有更多动作�?*/
        if (arm_back_failed) {
            arm_back_failed = 0;
            return BT_FAILURE;
        }
        if (arm_back_busy || arm_back_request || arm_back_state != ARM_BACK_IDLE) {
            return BT_RUNNING;
        }
        return BT_SUCCESS;
    }

    // 根据当前动作类型和执行状态，执行对应的底层指令，并推进状态机
    switch (mf_exec_state) {

    /* ============================================================ */
    /* IDLE 状态：读取当前动作，下发对应的底层指令                  */
    /* ============================================================ */
    case MF_EXEC_IDLE:

        // 过滤�?动作（无效动作）
        if (act.type == MF_ACT_MOVE_FLAT && act.row == 0 && act.col == 0 && act.height_pick_diff == 0 && act.dir == MF_DIR_NORTH) {
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            return BT_RUNNING;  /* mf_exec_state 保持 IDLE */
        }


        /* ======================== 根据动作类型下发指令 ======================== */
        switch (act.type) {

        /* ---------- 平移 ---------- */
        case MF_ACT_MOVE_FLAT: {
            /*
                * 🟡【需实测】将 (row,col) + 方向 转为世界坐标，下发给底盘
                * 你需要做的事�?
                *   1. 调用 mf_cell_to_world(act.row, act.col, &tx, &ty) 得到目标坐标
                *   2. 调用 mf_dir_to_yaw(act.dir) 得到目标朝向
                *   3. 调用 chassis_route_point_add(&chassis_route_traj, tx, ty, yaw, 0, 0)
                *      下发给底盘轨迹队�?
                *
                * 下层 task 的标志位/动作�?
                *   - 底盘 task (entry_chassis) 会轮�?chassis_route_traj 队列自动执行
                *   - 无需额外的标志位，chassis_route_point_add 即可触发
                */
            float tx, ty;
            mf_cell_to_world(act.row, act.col, &tx, &ty, mf_planner_ctx.map_color);
            // float yaw;
            // mf_dir_to_yaw(act.dir);

            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, tx, ty, Radar_Pos.reliable_yaw, 0, 1);
            taskEXIT_CRITICAL();

            mf_exec_state = MF_EXEC_MOVE_FLAT;
            mf_exec_timer_ms = 0;
            break;
        }

        /* ---------- 上台�?---------- */
        case MF_ACT_STEP_UP: {
            // /*
            // * 启动底盘上台�?FSM�?
            // * 当前 MF_Action 结构中暂不携带台阶高�?200/400)信息�?
            // * 默认使用 STEP_UP_200；如需区分 400mm，可复用 act.height_pick_diff 字段�?
            // *
            // * 台阶 FSM 会自行管理：
            // *   - 支撑柱升降（通过 chassis_lift_queue_add_point �?freertos 位控消费�?
            // *   - 辅助轮驱动（通过 g_lift_wheel_target �?freertos 消费�?
            // *   - 微动开关检�?�?决定何时收回�?后柱
            // */
            // ChassisStepType step_type = STEP_UP_200;  // 默认 200mm
            // // 如果 MF_Action 携带了高度差信息，用 400mm
            // // if (act.height_pick_diff == 400) step_type = STEP_UP_400;
            // chassis_step_fsm_start(&g_step_fsm, step_type);
            // mf_exec_state = MF_EXEC_STEP_UP;
            // mf_exec_timer_ms = 0;
            // break;
            /*
            * 第一步：先让底盘转向台阶方向，面�?act.dir�?
            * 目标点为当前格中心坐标，朝向与动作方向一致�?
            */
            // float cur_cell_x, cur_cell_y;
            // mf_cell_to_world(act.row, act.col, &cur_cell_x, &cur_cell_y, mf_planner_ctx.map_color);
            float stand_x, stand_y;
            mf_step_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            /* 目标朝向 = 拾取方向对应�?yaw */
            float step_yaw;
            step_yaw = mf_dir_to_yaw(act.dir);
            
            taskENTER_CRITICAL();
            /* 复用 */
            chassis_route_point_add(&chassis_route_traj, stand_x, stand_y, step_yaw, 0, 1);
            taskEXIT_CRITICAL();

            mf_exec_state = MF_EXEC_STEP_UP;
            mf_exec_timer_ms = 0;
            break;
        }

        /* ---------- 下台�?---------- */
        case MF_ACT_STEP_DOWN: {

            float stand_x, stand_y;
            mf_step_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            /* 目标朝向 = 拾取方向对应�?yaw */
            float step_yaw;
            step_yaw = mf_dir_to_yaw(act.dir);
            
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, stand_x, stand_y, step_yaw, 0, 1);
            taskEXIT_CRITICAL();

            mf_exec_state = MF_EXEC_STEP_DOWN;
            mf_exec_timer_ms = 0;
            break;
        }

        /* ---------- 拾取 R2KFS ---------- */
        case MF_ACT_PICK_R2: {
            if (arm_back_busy || arm_back_state != ARM_BACK_IDLE ||
                arm.mode != ARM_MODE_FRONT) {
                /* 后向服务占用中或手臂不在前向，等待 */
                return BT_RUNNING;
            }

            /* 计算 R2 应该站立的世界坐标 */
            float stand_x, stand_y;
            mf_pick_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            float pick_yaw = mf_dir_to_yaw(act.dir);

            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, stand_x, stand_y, pick_yaw, 0, 1);
            taskEXIT_CRITICAL();

            arm_back_mode = ARM_BACK_MODE_STORE;
            mf_exec_state = MF_EXEC_PICK_R2;
            mf_exec_timer_ms = 0;
            break;
        }

        /* ---------- 前向工作区移开/丢弃 R2KFS ---------- */
        case MF_ACT_MOVE_R2: {
            if (arm_back_busy || arm_back_state != ARM_BACK_IDLE ||
                arm.mode != ARM_MODE_FRONT) {
                return BT_RUNNING;
            }

            float stand_x, stand_y;
            mf_pick_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            float pick_yaw = mf_dir_to_yaw(act.dir);

            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, stand_x, stand_y, pick_yaw, 0, 1);
            taskEXIT_CRITICAL();

            mf_exec_state = MF_EXEC_DISCARD_R2;
            mf_exec_timer_ms = 0;
            arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;
            break;
        }

        /* ---------- 等待 R1 清除 R1KFS ---------- */
        case MF_ACT_WAIT_R1: {
            /*
                * 🟡【需实测/需通信】等�?R1 机器人取�?R1KFS
                * 你需要做的事�?
                *   1. 通过通信模块（如裁判系统/自定义无线）接收 R1 的完成信�?
                *   2. 收到信号后调�?MFPlanner_NotifyR1KfsRemoved 更新地图
                *   3. 如果超时，可以强行通过（假�?R1 已完成但通信丢失�?
                *
                * 当前实现：轮�?cell 状态（由外�?NotifyR1KfsRemoved 更新�?
                * 调试阶段：超时后自动跳过
                *
                * 下层 task 的标志位/动作�?
                *   - 需要一个全局标志位，�?r1_clear_signal
                *   - 由通信 task 在收�?R1 信号后置�?
                *   - 本节点检查该标志�?
                */
            mf_exec_state = MF_EXEC_WAIT_R1;
            mf_exec_timer_ms = 0;
            break;
        }

        /* ---------- 扫描 ---------- */
        case MF_ACT_SCAN: {
            float scan_x, scan_y, scan_yaw;
            mf_scan_stand_position(&act, &scan_x, &scan_y, &scan_yaw);

            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, scan_x, scan_y, scan_yaw, 0, 1);
            clamp.lift_flag = 1;  // 扫描时将摄像机升起，避免干扰
            taskEXIT_CRITICAL();

            mf_exec_state = MF_EXEC_SCAN;
            mf_exec_timer_ms = 0;
            break;
        }

        default:
            return BT_FAILURE;
        }
        return BT_RUNNING;

    /* ============================================================ */
    /* SCAN 状态：等待底盘到扫描站位后触发对应扫描/规划              */
    /* ============================================================ */
    case MF_EXEC_SCAN:
    {
        float scan_x, scan_y, scan_yaw;
        mf_scan_stand_position(&act, &scan_x, &scan_y, &scan_yaw);

        if (chassis_if_arrived(scan_x, scan_y, scan_yaw)) {
            if (act.height_pick_diff == MF_SCAN_STAGE_ROW1) {
                // MFPlanner_ScanRows(&mf_planner_ctx, 1, 1);
                MFPlanner_ScanRow_1(&mf_planner_ctx);
                if (MF_PushAction(&mf_planner_ctx, MF_ACT_SCAN, MF_DIR_NORTH, 0, 1)) {
                    mf_planner_ctx.actions[mf_planner_ctx.action_count - 1].height_pick_diff = MF_SCAN_STAGE_ROW2;
                } else {
                    mf_exec_state = MF_EXEC_FAILED;
                    return BT_RUNNING;
                }
            } else if (act.height_pick_diff == MF_SCAN_STAGE_ROW2) {
                // MFPlanner_ScanRows(&mf_planner_ctx, 2, 2);
                MFPlanner_ScanRow_2(&mf_planner_ctx);
                if (!MFPlanner_BuildPhase1(&mf_planner_ctx, 0, 0, &mf_picked_phase1)) {
                    mf_exec_state = MF_EXEC_FAILED;
                    return BT_RUNNING;
                }
            } else if (act.height_pick_diff == MF_SCAN_STAGE_ROW34) {
                uint8_t r2_need_phase2 = 0;

                // MFPlanner_ScanRows(&mf_planner_ctx, 3, 4);
                MFPlanner_ScanRow_34(&mf_planner_ctx);
                if (mf_r2_kfs_all_need > mf_picked_phase1) {
                    r2_need_phase2 = mf_r2_kfs_all_need - mf_picked_phase1;
                }
                if (!MFPlanner_BuildPhase2(&mf_planner_ctx, r2_need_phase2)) {
                    mf_exec_state = MF_EXEC_FAILED;
                    return BT_RUNNING;
                }
                debug_if_reach_MF_phase2_scan = 1;
            } else {
                mf_exec_state = MF_EXEC_FAILED;
                return BT_RUNNING;
            }

            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
            mf_exec_timer_ms = 0;
        } else if (++mf_exec_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) {
            mf_exec_state = MF_EXEC_FAILED;
        }
        return BT_RUNNING;
    }

    /* ============================================================ */
    /* MOVE_FLAT 状态：等待底盘平移到位                              */
    /* ============================================================ */
    case MF_EXEC_MOVE_FLAT:
    {
        /*
            * 🟡【需实测】底盘到达判断条�?
            * 使用 chassis_if_arrived �?chassis_route_traj �?is_finished 标志
            * 需要与你的底盘路径执行逻辑（entry_chassis）对�?
            */

        float tx, ty;
        mf_cell_to_world(act.row, act.col, &tx, &ty, mf_planner_ctx.map_color);

        if(chassis_if_arrived(tx, ty, Radar_Pos.reliable_yaw))  //底盘到位即可，不要求底盘强制面朝位移方向
        {
            /* 平移到位，推进到下一个动�?*/
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
        }
        else if (++mf_exec_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS) 
        {
            mf_exec_state = MF_EXEC_FAILED;
        }
        return BT_RUNNING;
    }
    /* ============================================================ */
    /* STEP_UP 状态：等待上台�?FSM 完成                              */
    /* ============================================================ */
    case MF_EXEC_STEP_UP:
    {
        /*
        * �?10ms 推进一次台�?FSM�?
        *   BT_RUNNING  �?台阶动作仍在进行�?
        *   BT_SUCCESS  �?上台阶完成，推进到下一�?MF 动作
        *   BT_FAILURE  �?超时或异常，进入 MF 失败处理
        *
        * 当前扩展：FSM 进入 STEP_UP_DRIVE_TO_REAR_TOUCH（前柱已完全收回）时�?
        * 向底盘四个全向轮发送台阶目标格世界坐标，辅助轮+全向轮共同前进，
        * 同时抑制后微动触发台阶时的反弹趋势�?
        */

        static uint8_t step_up_fsm_started = 0;    /* FSM 是否已启�?*/
        static uint8_t step_up_chassis_sent = 0;  /* 防止重复发送路径点 */

        MF_Action act;
        MFPlanner_PeekAction(&mf_planner_ctx, &act);

        // float cur_cell_x, cur_cell_y;
        // mf_cell_to_world(act.row, act.col, &cur_cell_x, &cur_cell_y, mf_planner_ctx.map_color);
        float step_yaw = mf_dir_to_yaw(act.dir);

            /* ---- 阶段 1：等待底盘移�?转向到位（面朝台阶方向） ---- */
        if (!step_up_fsm_started)
        {
            float stand_x, stand_y;
            mf_step_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            /* 目标朝向 = 拾取方向对应�?yaw */
            float step_yaw;
            step_yaw = mf_dir_to_yaw(act.dir);

            if (chassis_if_arrived(stand_x, stand_y, step_yaw))
            {
                /* 底盘已到位，启动上台�?FSM */
                ChassisStepType step_type = mf_step_type_from_action(&act);
                if (step_type == STEP_NONE) {
                    mf_exec_state = MF_EXEC_FAILED;
                    return BT_RUNNING;
                }
                chassis_step_fsm_start(&g_step_fsm, step_type);
                step_up_fsm_started = 1;
                mf_exec_timer_ms = 0;
            }
            else if (++mf_exec_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                mf_exec_state = MF_EXEC_FAILED;
            }
            return BT_RUNNING;
        }

        BT_Status step_status = chassis_step_fsm_tick(&g_step_fsm);

        /* 刚启动抬升时就发下个台阶的中心点 */
        if (!step_up_chassis_sent &&
            g_step_fsm.state == STEP_UP_WAIT_DEPLOY_DONE)
        {
            /* 计算台阶目标格（当前格沿 act.dir 方向偏移一格）的世界坐�?*/
            int8_t target_row = (int8_t)act.row;
            int8_t target_col = (int8_t)act.col;

            float step_tx, step_ty;
            mf_cell_to_world((uint8_t)target_row, (uint8_t)target_col,
                             &step_tx, &step_ty, mf_planner_ctx.map_color);
            float step_yaw = mf_dir_to_yaw(act.dir);

            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, step_tx, step_ty, step_yaw, 0, 1);
            step_up_chassis_sent = 1;
            taskEXIT_CRITICAL();

        }

        if (step_status == BT_SUCCESS) {
            step_up_chassis_sent = 0;
            step_up_fsm_started = 0;
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
        } else if (step_status == BT_FAILURE) {
            step_up_chassis_sent = 0;
            step_up_fsm_started = 0;
            mf_exec_state = MF_EXEC_FAILED;
        }
        return BT_RUNNING;
    }

    /* ============================================================ */
    /* STEP_DOWN 状态：等待下台�?FSM 完成                            */
    /* ============================================================ */
    case MF_EXEC_STEP_DOWN:
    {
        static uint8_t step_down_fsm_started = 0;  /* FSM 是否已启�?*/
        static uint8_t step_down_chassis_sent = 0;  /* 防止重复发送路径点 */

        MF_Action act;
        MFPlanner_PeekAction(&mf_planner_ctx, &act);

        // float cur_cell_x, cur_cell_y;
        // mf_cell_to_world(act.row, act.col, &cur_cell_x, &cur_cell_y, mf_planner_ctx.map_color);
        float step_yaw = mf_dir_to_yaw(act.dir);

        /* ---- 阶段 1：等待底盘转向到位（面朝台阶方向、到达当前格中心�?---- */
        if (!step_down_fsm_started)
        {
            float stand_x, stand_y;
            mf_step_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
            /* 目标朝向 = 拾取方向对应�?yaw */
            float step_yaw;
            step_yaw = mf_dir_to_yaw(act.dir);

            if (chassis_if_arrived(stand_x, stand_y, step_yaw))
            {
                /* 底盘已到位，启动下台�?FSM */
                ChassisStepType step_type = mf_step_type_from_action(&act);
                if (step_type == STEP_NONE) {
                    mf_exec_state = MF_EXEC_FAILED;
                    return BT_RUNNING;
                }
                chassis_step_fsm_start(&g_step_fsm, step_type);
                step_down_fsm_started = 1;
                mf_exec_timer_ms = 0;
            }
            else if (++mf_exec_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                mf_exec_state = MF_EXEC_FAILED;
            }
            return BT_RUNNING;
        }

        BT_Status step_status = chassis_step_fsm_tick(&g_step_fsm);

        /* 刚启动下台阶时就发下个台阶的中心点，并使底盘禁用 */
        if (!step_down_chassis_sent &&
            g_step_fsm.state == STEP_DOWN_DRIVE_FRONT_LEAVE)
        {
            /* 计算台阶目标格（当前格沿 act.dir 方向偏移一格）的世界坐�?*/
            int8_t target_row = (int8_t)act.row;
            int8_t target_col = (int8_t)act.col;

            float step_tx, step_ty;
            mf_cell_to_world((uint8_t)target_row, (uint8_t)target_col,
                             &step_tx, &step_ty, mf_planner_ctx.map_color);
            float step_yaw = mf_dir_to_yaw(act.dir);

            taskENTER_CRITICAL();
            chassis_r2.chassis_en_flag = 0;  // 禁用底盘
            chassis_route_point_add(&chassis_route_traj, step_tx, step_ty, step_yaw, 0, 1);
            step_down_chassis_sent = 1;
            taskEXIT_CRITICAL();

        }


        if (step_status == BT_SUCCESS) {
            step_down_chassis_sent = 0;
            step_down_fsm_started = 0;
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
        } else if (step_status == BT_FAILURE) {
            step_down_chassis_sent = 0;
            step_down_fsm_started = 0;
            mf_exec_state = MF_EXEC_FAILED;
        }
        return BT_RUNNING;
    }

    /* ============================================================ */
    /* PICK_R2 状态：等待机械臂拾取完成（统一存储/丢弃，仅背向工作区发点不同） */
    /* ============================================================ */
    case MF_EXEC_PICK_R2:
    {
        /* 计算 R2 应该站立的世界坐标 */
        float stand_x, stand_y;
        mf_pick_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
        float pick_yaw = mf_dir_to_yaw(act.dir);

        /* 底盘到位，并且之前没有发过点，再发出机械臂的抓取动作 */
        if (chassis_if_arrived(stand_x, stand_y, pick_yaw) && arm_if_send_point == 0 && arm.mode == ARM_MODE_FRONT)
        {
            arm_if_send_point = 1;
            arm_send_pick_trajectory(act.height_pick_diff);
        }

        if (!arm_back_busy && !arm_back_request && !arm_back_payload_done && !arm_back_failed)
        {
            if (arm_if_arrived_xy(PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y))
            {
                if (mf_picked_kfs_count == 0)
                {
                    arm_back_busy = 1;
                    /* 第一次 PICK：存到车身后向存储位 */
                    arm_back_mode = ARM_BACK_MODE_STORE;
                    arm_back_slot = mf_bb.kfs_stored;
                    arm_back_request = 1;
                    mf_exec_timer_ms = 0;
                }
                else
                {
                    /* 第二次 PICK：保持吸附，带着 KFS 进入 CZ */
                    arm_if_send_point = 0;
                    mf_picked_kfs_count++;
                    taskENTER_CRITICAL();
                    arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, ARM_PUMP_SUCK, 0.0f);
                    taskEXIT_CRITICAL();
                    MFPlanner_AdvanceAction(&mf_planner_ctx);
                    mf_exec_state = MF_EXEC_IDLE;
                    mf_exec_timer_ms = 0;
                }
            }
            else if (++mf_exec_timer_ms * 10 >= MF_PICK_TIMEOUT_MS)
            {
                mf_exec_state = MF_EXEC_FAILED;
            }
        }


        return BT_RUNNING;
    }

    /* ============================================================ */
    /* DISCARD_R2 state: front-workspace pick, release, and rest     */
    /* ============================================================ */
    case MF_EXEC_DISCARD_R2:
    {
        float stand_x, stand_y;
        mf_pick_stand_position(act.row, act.col, act.dir, &stand_x, &stand_y);
        float pick_yaw = mf_dir_to_yaw(act.dir);

        if (chassis_if_arrived(stand_x, stand_y, pick_yaw) &&
            arm_front_discard_state == ARM_FRONT_DISCARD_IDLE &&
            arm.mode == ARM_MODE_FRONT) {
            arm_front_discard_state = ARM_FRONT_DISCARD_PICK_SENT;
            arm_send_pick_trajectory(act.height_pick_diff);
        }

        if (arm_front_discard_state == ARM_FRONT_DISCARD_PICK_SENT &&
            arm_if_arrived_xy(PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y)) {
            arm_front_discard_state = ARM_FRONT_DISCARD_RELEASE_SENT;
            mf_exec_timer_ms = 0;
            arm_send_front_discard_trajectory();
        } else if (arm_front_discard_state == ARM_FRONT_DISCARD_RELEASE_SENT &&
                   arm_if_arrived_xy(PICK_REST_X, PICK_REST_Y)) {
            arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
            mf_exec_timer_ms = 0;
        } else if (++mf_exec_timer_ms * 10 >= MF_FRONT_DISCARD_TIMEOUT_MS) {
            arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;
            mf_exec_state = MF_EXEC_FAILED;
        }

        return BT_RUNNING;
    }
    /* ============================================================ */
    /* WAIT_R1 状态：等待 R1 清除 R1KFS                             */
    /* ============================================================ */
    case MF_EXEC_WAIT_R1:
        /*
            * 🟡【需实测/需通信】R1 完成信号判断
            * 方案1（当前）：检�?cell 是否已被 NotifyR1KfsRemoved 置为 EMPTY
            * 方案2（推荐）：检查通信模块�?r1_clear_signal 标志�?
            *
            * 下层 task 的标志位/动作�?
            *   - 🟡【需新增】一个全局变量：uint8_t r1_clear_signal = 0;
            *   - 由通信 task（如裁判系统回调）在收到 R1 清除信号后置 1
            *   - 此处检查该标志�?
            */

        // if (mf_planner_ctx.cell[act.row][act.col] == MF_CELL_EMPTY) 
        
        /* 调试阶段假设R1KFS早已被取�?*/
        if(1)
        {
            /* R1 已清除该�?*/
            MFPlanner_NotifyR1KfsRemoved(&mf_planner_ctx, act.row, act.col);
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
        } 
        else if (++mf_exec_timer_ms * 10 >= MF_WAIT_R1_TIMEOUT_MS) 
        {
            /* 超时：强行通过（假设通信丢失�?R1 已完成） */
            MFPlanner_NotifyR1KfsRemoved(&mf_planner_ctx, act.row, act.col);
            MFPlanner_AdvanceAction(&mf_planner_ctx);
            mf_exec_state = MF_EXEC_IDLE;
        }
        return BT_RUNNING;

    /* ============================================================ */
    /* 失败处理                                                      */
    /* ============================================================ */
    case MF_EXEC_FAILED:
    default:
        return BT_FAILURE;
    }
}

/**
 * @brief [动作节点] 阶段三：MF 任务收尾清理
 */
static BT_Status Act_MF_Finish_Cleanup(void)
{
    MC_bb.MF_MISSION_Req = 0;

    mf_exec_state = MF_EXEC_IDLE;
    mf_exec_timer_ms = 0;
    mf_task_request_flag = 0;
    mf_phase_flag = 0;
    mf_picked_phase1 = 0;
    mf_picked_kfs_count = 0;

    arm_back_request = 0;
    arm_back_busy = 0;
    arm_back_payload_done = 0;
    arm_back_failed = 0;
    arm_back_slot = 0;
    arm_back_state = ARM_BACK_IDLE;
    arm_back_timer_ms = 0;
    arm_back_mode = ARM_BACK_MODE_STORE;
    arm_if_send_point = 0;
    arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;

    // mf_bb.kfs_stored = 0;

    if (mf_finish_start_cz) {
        CZ_bb.req_start = 1;  // MF 完成后触发 CZ 对抗区任务
    }
    mf_r2_kfs_all_need = R2_KFS_ALL_NEED;
    mf_finish_start_cz = 1;

    return BT_SUCCESS;
}

/* ================= 4. 顶层行为�?================= */

/** MF 行为树的 Memory Sequence 索引 */
static uint8_t seq_mem_idx_mf = 0;

/**
 * @brief MF 任务行为�?Root 根节�?
 * @note 流程�?
 *   1. 等待 MF 任务请求
 *   2. 初始化 + 预输入全局规划（一次性）
 *   3. 逐个执行动作列表（内�?FSM，多 Tick 返回 RUNNING�?
 *   4. 清理
 */
static BT_Status BT_MF_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_MF_Request,       /* 1. �?MF 任务请求�?*/
        Act_MF_Init_And_Plan,      /* 2. 初始化 + 预输入全局规划 */
        Act_MF_Execute_Action,     /* 3. 执行动作列表（内�?FSM�?*/
        Act_MF_Finish_Cleanup      /* 4. 清理 */
    };

    return BT_Sequence_Memory(seq,
                              (uint8_t)(sizeof(seq) / sizeof(seq[0])),
                              &seq_mem_idx_mf);
}

/* ================= 5. 外部接口 ================= */

/**
 * @brief 发起 MF 任务请求（由按键/遥控/通信等触发）
 */
static void BT_MF_Start_Task_Config(uint8_t r2_kfs_all_need, uint8_t finish_start_cz)
{
    mf_task_request_flag = 1;
    mf_phase_flag = 0;
    mf_exec_state = MF_EXEC_IDLE;
    mf_picked_kfs_count = 0;
    mf_picked_phase1 = 0;
    mf_r2_kfs_all_need = r2_kfs_all_need;
    mf_finish_start_cz = finish_start_cz;
    arm_if_send_point = 0;
    arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;
}

void BT_MF_Start_Task(void)
{
    BT_MF_Start_Task_Config(R2_KFS_ALL_NEED, 1);
}

/**
 * @brief 初始�?重置 MF 任务状态（紧急停止或上电时调用）
 */
void BT_MF_Init_Task(void)
{
    mf_task_request_flag = 0;
    mf_phase_flag = 0;
    mf_exec_state = MF_EXEC_IDLE;
    mf_exec_timer_ms = 0;
    mf_picked_phase1 = 0;
    mf_picked_kfs_count = 0;
    mf_r2_kfs_all_need = R2_KFS_ALL_NEED;
    mf_finish_start_cz = 1;

    arm_back_request = 0;
    arm_back_busy = 0;
    arm_back_payload_done = 0;
    arm_back_failed = 0;
    arm_back_slot = 0;
    arm_back_state = ARM_BACK_IDLE;
    arm_back_timer_ms = 0;
    arm_back_mode = ARM_BACK_MODE_STORE;
    arm_if_send_point = 0;
    arm_front_discard_state = ARM_FRONT_DISCARD_IDLE;

    mf_bb.kfs_stored = 0;
    seq_mem_idx_mf = 0;
}

/**
 * @brief MF 行为�?Tick 入口（在 cmd_task �?for(;;) �?1ms 周期调用�?
 * @note 内部降频�?10ms
 */
// void BT_Customized_Tick1ms_MF(void)
// {
//     static uint8_t div10 = 0;
//     div10++;
//     if (div10 < 10) return;
//     div10 = 0;
//     Arm_Store_Service_Tick();
//     BT_Status status = BT_MF_Task_Tree_Root();
//     /* 异常自愈：BT 树崩溃时自动清理 */
//     if (status == BT_FAILURE) {
//         Act_MF_Finish_Cleanup();
//         seq_mem_idx_mf = 0;
//     }
// }







/* ================================================ CZ 对抗区行为树 ================================================ */
/* ================================================ CZ 对抗区行为树 ================================================ */
/* ================================================ CZ 对抗区行为树 ================================================ */

CZ_Blackboard CZ_bb;

/**
 * @brief [条件节点] 检查是否有对抗区任务请�?
 */
static BT_Status Cond_Has_CZ_Request(void)
{
    return CZ_bb.req_start ? BT_SUCCESS : BT_FAILURE;
}

/**
 * @brief [动作节点] 对抗区任务执�?
 * @note 流程：坡道起�?�?上坡（坡道终点）�?原地慢转90° �?中央扫描位置 �?摄像头扫�?
 *       �?九宫格列 �?抬机械臂 �?等待机械臂到�?�?底盘前进送块 �?关气�?�?等待 �?策略分支
 */
static BT_Status Act_CZ_Mission(void)
{
    switch (CZ_bb.cz_mission_state) {

        case CZ_IDLE:
        {
            CZ_bb.cz_mission_state = CZ_NAV_TO_SLOPE_START;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_NAV_TO_SLOPE_START:
        {
            /* 平地参数导航到坡道起�?+ 坡道终点（预入队，靠 look-ahead 衔接�?*/
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SLOPE_START_X, R_CHASSIS_SLOPE_START_Y, R_CHASSIS_SLOPE_START_W, 0, 0);
            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SLOPE_END_X, R_CHASSIS_SLOPE_END_Y, R_CHASSIS_SLOPE_END_W, 0, 0);
            taskEXIT_CRITICAL();

            CZ_bb.cz_mission_state = CZ_NAV_TO_SLOPE_END;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_NAV_TO_SLOPE_END:
        {
            /* 接近坡道起点时启用上坡参数，只对坡道段生�?*/
            if (fabsf(Radar_Pos.reliable_x - R_CHASSIS_SLOPE_START_X) < 250.0f &&
                fabsf(Radar_Pos.reliable_y - R_CHASSIS_SLOPE_START_Y) < 250.0f)
            {
                chassis_route_set_slope_mode(1);
            }

            /* 等待到达坡道终点 */
            if (chassis_if_arrived(R_CHASSIS_SLOPE_END_X, R_CHASSIS_SLOPE_END_Y, R_CHASSIS_SLOPE_END_W))
            {
                chassis_route_set_slope_mode(0);  /* 上坡完成，恢复平地参�?*/

                /* 原地慢速旋�?0° */
                chassis_route_set_rotate_slow(1);
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SLOPE_END_X, R_CHASSIS_SLOPE_END_Y, 90.0f, 0, 1);
                taskEXIT_CRITICAL();

                CZ_bb.cz_mission_state = CZ_ROTATE_90;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_ROTATE_90:
        {
            /* 等待原地旋转90°完成，旋转速度较慢 */
            if (chassis_if_arrived(R_CHASSIS_SLOPE_END_X, R_CHASSIS_SLOPE_END_Y, 90.0f))
            {
                chassis_route_set_rotate_slow(0);  /* 旋转完成，恢复正常角速度 */

                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, R_CHASSIS_CZ_CENTER_X, R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W, 0, 1);
                taskEXIT_CRITICAL();

                CZ_bb.cz_mission_state = CZ_NAV_TO_SCAN;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                chassis_route_set_rotate_slow(0);  /* 超时也要恢复 */
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_NAV_TO_SCAN:
        {
            /* 等待到达中央扫描位置 */
            if (chassis_if_arrived(R_CHASSIS_CZ_CENTER_X, R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W))
            {
                CZ_bb.cz_mission_state = CZ_CAMERA_SCAN;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_CAMERA_SCAN:
        {
            /* 等待底盘到达中央扫描位置并稳�?*/
            if (chassis_if_arrived(R_CHASSIS_CZ_CENTER_X, R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W))
            {
				clamp.lift_flag = 1;
                CZ_bb.pos_stable_times++;
                if (CZ_bb.pos_stable_times > CZ_POS_STABLE_CONDITION)
                {
                    chassis_set_lock(1);  /* 扫描时锁止底盘，防止移动 */
                    CZ_bb.camera_scan_done = 1;//是否需要发送指令给摄像头还未确�?

                    /* 根据摄像头扫描结果决定下一�?
                       KFS_PUT: CZ_put_pos 一定是 M1/M2/M3
                       WAIT:    CZ_put_pos 一定是 Invalid(9)
                       RESCAN:  CZ_put_pos 一定是 Invalid(9) */
                    //    taskENTER_CRITICAL();
                    switch (CZ_action)
                    {
                    case KFS_PUT:
                        /* 先只下发底盘路径去九宫格，机械臂等底盘到位后再抬 */
                    taskENTER_CRITICAL();
                        if (CZ_put_pos == M1)
                            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_3_X, R_CHASSIS_SUDUKU_3_Y, R_CHASSIS_SUDUKU_3_W, 0, 1);
                        else if (CZ_put_pos == M2)
                            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_2_X, R_CHASSIS_SUDUKU_2_Y, R_CHASSIS_SUDUKU_2_W, 0, 1);
                        else /* M3 */
                            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_SUDUKU_1_X, R_CHASSIS_SUDUKU_1_Y, R_CHASSIS_SUDUKU_1_W, 0, 1);
                     taskEXIT_CRITICAL();
                       
                        if (CZ_put_pos == M1)
                        { CZ_SUDUKU_X = R_CHASSIS_SUDUKU_3_X; CZ_SUDUKU_Y = R_CHASSIS_SUDUKU_3_Y; CZ_SUDUKU_W = R_CHASSIS_SUDUKU_3_W; }
                        else if (CZ_put_pos == M2)
                        { CZ_SUDUKU_X = R_CHASSIS_SUDUKU_2_X; CZ_SUDUKU_Y = R_CHASSIS_SUDUKU_2_Y; CZ_SUDUKU_W = R_CHASSIS_SUDUKU_2_W; }
                        else /* M3 */
                        { CZ_SUDUKU_X = R_CHASSIS_SUDUKU_1_X; CZ_SUDUKU_Y = R_CHASSIS_SUDUKU_1_Y; CZ_SUDUKU_W = R_CHASSIS_SUDUKU_1_W; }

                        CZ_bb.cz_mission_state = CZ_NAV_TO_SUDUKU;
                        CZ_bb.cz_timer_ms = 0;
                        CZ_bb.pos_stable_times = 0;
                        chassis_set_lock(0);  /* 扫描完成，解锁底盘 */
                        return BT_RUNNING;

                    case WAIT:
                        /* 等待+9：保持不动，直接完成 */
                        CZ_bb.cz_mission_state = CZ_DONE;
                        CZ_bb.cz_timer_ms = 0;
                        CZ_bb.pos_stable_times = 0;
                        chassis_set_lock(0);  /* 扫描完成，解锁底盘 */
                        return BT_RUNNING;

                    case RESCAN:
                        /* 重新扫描+9：微调位置后重新扫描 */
                        chassis_route_point_add(&chassis_route_traj, R_CHASSIS_CZ_CENTER_X , R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W, 0, 0);
                        CZ_bb.cz_mission_state = CZ_CAMERA_SCAN;
                        CZ_bb.cz_timer_ms = 0;
                        CZ_bb.pos_stable_times = 0;
                        chassis_set_lock(0);  /* 扫描完成，解锁底盘 */
                        return BT_RUNNING;

                    default:
                        CZ_bb.cz_mission_state = CZ_DONE;
                        CZ_bb.cz_timer_ms = 0;
                        CZ_bb.pos_stable_times = 0;
                        chassis_set_lock(0);  /* 扫描完成，解锁底盘 */
                        return BT_RUNNING;
                    }
                }
            }
            else
            {
                CZ_bb.pos_stable_times = 0;  /* 还没到位，重置稳定计�?*/
                if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
                    return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_NAV_TO_SUDUKU:
        {
            /* 等待底盘到达目标九宫格列（只检查底盘，不检查机械臂�?*/
            if (chassis_if_arrived(CZ_SUDUKU_X, CZ_SUDUKU_Y, CZ_SUDUKU_W))
            {
                CZ_bb.cz_mission_state = CZ_ARM_RAISE;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS * 2)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_ARM_RAISE:
        {
            /* 底盘已到九宫格，下发机械臂抬升到放置位置 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            // arm.effector_pitch_target = CZ_ARM_PLACE_PITCH * 3.14f / 180.0f;
            taskEXIT_CRITICAL();
            
            CZ_bb.cz_mission_state = CZ_ARM_RAISE_WAIT;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_ARM_RAISE_WAIT:
        {
            /* 等待机械臂到达放置位�?*/
            if (arm_if_arrived_xy(CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y))
            {
                /* 机械臂到位后，底盘往前送块一小段 */
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, CZ_SUDUKU_X, CZ_SUDUKU_Y + 450.0f, CZ_SUDUKU_W, 0, 0);
                taskEXIT_CRITICAL();

                CZ_bb.cz_mission_state = CZ_FORWARD_PUSH;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_FORWARD_PUSH:
        {
            /* 等待底盘前进送块到位 */
            if (chassis_if_arrived(CZ_SUDUKU_X, CZ_SUDUKU_Y + 450.0f, CZ_SUDUKU_W))
            {
                CZ_bb.cz_mission_state = CZ_ARM_PLACE;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_ARM_PLACE:
        {
            /* 底盘到位，关气泵释放�?*/
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y, 0, ARM_PUMP_RELEASE, CZ_ARM_PLACE_PITCH/180.0f*PI);
            taskEXIT_CRITICAL();

            CZ_bb.cz_mission_state = CZ_PUMP_WAIT;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_PUMP_WAIT:
        {
            /* 关气泵后等待一段时间再让底盘离开 */
            if (++CZ_bb.cz_timer_ms * 10 >= CZ_PUMP_WAIT_MS)
            {
                CZ_bb.scan_count++;
                /* 07.01.1013 R2已改为存一拿一，因此共2个块 */
                if (CZ_bb.scan_count >= 1)
                {
                    taskENTER_CRITICAL();
                    /* 单次放置完成，将机械臂过渡到背向 */
                    arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_FRONT_TO_BACK);
                    /* 返回扫描点进行第二次扫描 */
                    chassis_route_point_add(&chassis_route_traj, R_CHASSIS_CZ_CENTER_X, R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W, 0, 0);
                    taskEXIT_CRITICAL();
                    CZ_bb.cz_mission_state = CZ_PICK_STORE_F2B;
                }
                CZ_bb.cz_timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case CZ_RETURN_TO_SCAN:
        {
            /* 返回中央扫描位置 */
            if (chassis_if_arrived(R_CHASSIS_CZ_CENTER_X, R_CHASSIS_CZ_CENTER_Y, R_CHASSIS_CZ_CENTER_W))
            {
                /* 到扫描点后先取存块（仅机械臂），再做第二次扫描 */
                 taskENTER_CRITICAL();
                    arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, 1, 0.0f);
                    taskEXIT_CRITICAL();
                CZ_bb.cz_mission_state = CZ_PICK_STORE_AT_SCAN;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

       
        case CZ_PICK_STORE_AT_SCAN:
        {
            /*
             * 🟡 到扫描点后取存块 �?仅机械臂动作，底盘不动（暂空，立即完成）
             * TODO: 填写机械臂取存块的轨迹点
             */
            CZ_bb.cz_mission_state = CZ_CAMERA_SCAN;
            CZ_bb.cz_timer_ms = 0;
            CZ_bb.pos_stable_times = 0;
            return BT_RUNNING;
        }

        case CZ_PICK_STORE_F2B:
        {
            if(arm.mode == ARM_MODE_BACK)
            {
                taskENTER_CRITICAL();
                arm_queue_add_point(&arm_traj, STORE_PRE_X, STORE_PRE_Y, 0, ARM_PUMP_SUCK, 270.0f/180.0f*PI);
                arm_queue_add_point(&arm_traj, STORE_X+0.065f, STORE_Y-0.08f, STORE_TIME, ARM_PUMP_SUCK, 270.0f/180.0f*PI);
                taskEXIT_CRITICAL();
                CZ_bb.cz_mission_state = CZ_PICK_STORE_PICK;
                CZ_bb.cz_timer_ms = 0;
                return BT_RUNNING;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }

            return BT_RUNNING;
        }

        case CZ_PICK_STORE_PICK:
        {
            if(arm_if_arrived_xy(STORE_X+0.065f, STORE_Y-0.03f))
            {
                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);  // 关闭存块气泵
                taskENTER_CRITICAL();
                arm_mode_switch_to_transition(&arm, &arm_traj, &arm_trans_traj, ARM_TRANS_BACK_TO_FRONT);
                taskEXIT_CRITICAL();
                CZ_bb.cz_mission_state = CZ_PICK_STORE_B2F;
                CZ_bb.cz_timer_ms = 0;
                return BT_RUNNING;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }

            return BT_RUNNING;
        }

        case CZ_PICK_STORE_B2F:
        {
            if(arm.mode == ARM_MODE_FRONT)
            {
                taskENTER_CRITICAL();
                arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, ARM_PUMP_SUCK, CZ_ARM_PLACE_PITCH/180.0f*PI);
                taskEXIT_CRITICAL();
                CZ_bb.cz_mission_state = CZ_DONE;
                CZ_bb.cz_timer_ms = 0;
                CZ_bb.pos_stable_times = 0;
                return BT_RUNNING;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }

            return BT_RUNNING;
        }

        case CZ_DONE:
        {
            /* 两轮放置完成，前往 R1 平台等待�?*/
            clamp.lift_flag = 0;
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, R_CHASSIS_R1_WAIT_X, R_CHASSIS_R1_WAIT_Y, R_CHASSIS_R1_WAIT_W, 0, 1);
            taskEXIT_CRITICAL();

            CZ_bb.cz_mission_state = CZ_NAV_TO_R1_WAIT;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }




        /* ==================== R1 平台阶段 ==================== */

        case CZ_NAV_TO_R1_WAIT:
        {
            /* 等待到达 R1 平台等待�?*/
            if (chassis_if_arrived(R_CHASSIS_R1_WAIT_X, R_CHASSIS_R1_WAIT_Y, R_CHASSIS_R1_WAIT_W))
            {
                CZ_bb.cz_mission_state = CZ_WAIT_R1_PLATFORM;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_WAIT_R1_PLATFORM:
        {
            /* 等待 R1 发出"可以上平�?指令，下发台阶接近点（台阶攀登前的位置） */
            if (CZ_bb.r1_platform_cmd)
            {
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, R_CHASSIS_R1_STEP_APPROACH_X, R_CHASSIS_R1_STEP_APPROACH_Y, R_CHASSIS_R1_STEP_APPROACH_W, 0, 0);
                taskEXIT_CRITICAL();

                CZ_bb.cz_mission_state = CZ_NAV_TO_R1_PLATFORM;
                CZ_bb.cz_timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case CZ_NAV_TO_R1_PLATFORM:
        {
            /*
             * 台阶跨越 FSM：底盘先到台阶接近点 �?面朝台阶方向 �?启动上 400mm 台阶 FSM
             * 🟡【需实测】台阶方向应�?R_CHASSIS_R1_STEP_APPROACH_W 对齐
             */
            static uint8_t step_fsm_started = 0;
            static uint8_t platform_route_sent = 0;

            if (!step_fsm_started)
            {
                /* 阶段1：等待底盘到达台阶接近点并面朝台阶方�?*/
                if (chassis_if_arrived(R_CHASSIS_R1_STEP_APPROACH_X,
                    R_CHASSIS_R1_STEP_APPROACH_Y, R_CHASSIS_R1_STEP_APPROACH_W))
                {
                    chassis_step_fsm_start(&g_step_fsm, STEP_UP_400);
                    step_fsm_started = 1;
                    CZ_bb.cz_timer_ms = 0;
                }
                else if (++CZ_bb.cz_timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS * 2)
                {
                    return BT_FAILURE;
                }
                return BT_RUNNING;
            }

            /* 阶段2：每 Tick 推进台阶 FSM */
            BT_Status step_status = chassis_step_fsm_tick(&g_step_fsm);

            /* 刚启动抬升时关闭底盘使能 */
            if (!platform_route_sent &&
                g_step_fsm.state == STEP_UP_WAIT_DEPLOY_DONE)
            {
                taskENTER_CRITICAL();
                // chassis_route_point_add(&chassis_route_traj, R_CHASSIS_R1_PLATFORM_X, R_CHASSIS_R1_PLATFORM_Y, R_CHASSIS_R1_PLATFORM_W, 0, 0);
                chassis_r2.chassis_en_flag = 0; // 关闭底盘使能
                platform_route_sent = 1;
                taskEXIT_CRITICAL();
            }

            if (step_status == BT_SUCCESS)
            {
                /* 上台阶完成，不需要恢复底盘使能，锁止底盘 */
                taskENTER_CRITICAL();
                chassis_set_lock(1);
                taskEXIT_CRITICAL();

                step_fsm_started = 0;
                platform_route_sent = 0;
                CZ_bb.cz_mission_state = CZ_ON_R1_PLATFORM;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (step_status == BT_FAILURE)
            {
                step_fsm_started = 0;
                platform_route_sent = 0;
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_ON_R1_PLATFORM:
        {
            /*
             * �?R1 平台上等�?R1 发出"去第三层放块"指令�?
             * 底盘保持关闭（chassis_en_flag=0），只动机械臂�?
             */
            if (CZ_bb.r1_place_cmd)
            {
                /* 直接抬机械臂到第三层，底盘不�?*/
                CZ_bb.cz_mission_state = CZ_ARM_RAISE_L3;
                CZ_bb.cz_timer_ms = 0;
            }
            return BT_RUNNING;
        }

        /* ========== 九宫格第三层放块（仅机械臂，底盘始终关闭�?========== */

        case CZ_ARM_RAISE_L3:
        {
            /* 机械臂抬�?�?前送到第三层放置位置（两步�?*/
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_X, CZ_ARM_PLACE_L3_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            // arm.effector_pitch_target = CZ_ARM_PLACE_L3_PITCH * 3.14f / 180.0f;
            taskEXIT_CRITICAL();

            CZ_bb.cz_mission_state = CZ_ARM_RAISE_WAIT_L3;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_ARM_RAISE_WAIT_L3:
        {
            /* 等待机械臂到达前送终�?*/
            if (arm_if_arrived_xy(CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y))
            {
                CZ_bb.cz_mission_state = CZ_ARM_PLACE_L3;
                CZ_bb.cz_timer_ms = 0;
            }
            else if (++CZ_bb.cz_timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case CZ_ARM_PLACE_L3:
        {
            /* 前送到位，关气泵释放块 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y, 0, ARM_PUMP_RELEASE, 0.0f);
            taskEXIT_CRITICAL();

            CZ_bb.cz_mission_state = CZ_PUMP_WAIT_L3;
            CZ_bb.cz_timer_ms = 0;
            return BT_RUNNING;
        }

        case CZ_PUMP_WAIT_L3:
        {
            /* 关气泵后等待一段时�?*/
            if (++CZ_bb.cz_timer_ms * 10 >= CZ_PUMP_WAIT_MS)
            {
                CZ_bb.cz_mission_state = CZ_FINAL_DONE;
                CZ_bb.cz_timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case CZ_FINAL_DONE:
        {
            return BT_SUCCESS;
        }

        default:
        {
            return BT_FAILURE;
        }
    }
}

/**
 * @brief [动作节点] 对抗区任务收尾清�?
 */
static BT_Status Act_CZ_FinishCleanup(void)
{
    CZ_bb.req_start = 0;
    CZ_bb.cz_mission_state = CZ_IDLE;
    CZ_bb.cz_timer_ms = 0;
    CZ_bb.camera_scan_done = 0;
    CZ_bb.scan_count = 0;
    CZ_bb.pos_stable_times = 0;
    CZ_bb.r1_platform_cmd = 0;
    CZ_bb.r1_place_cmd = 0;
    chassis_route_set_rotate_slow(0);  /* 确保恢复正常旋转速度 */
    return BT_SUCCESS;
}

/**
 * @brief 对抗区行为树 Root 根节�?
 */
static BT_Status BT_CZ_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_CZ_Request,     /* 1. 有对抗区任务请求�?*/
        Act_CZ_Mission,           /* 2. 执行对抗区任�?*/
        Act_CZ_FinishCleanup      /* 3. 清理 */
    };

    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &CZ_bb.seq_mem_idx);
}

/* =========== 外部接口 =========== */

void BT_CZ_Start_Task(void)
{
    CZ_bb.req_start = 1;
    CZ_bb.cz_mission_state = CZ_IDLE;
    CZ_bb.cz_timer_ms = 0;
    CZ_bb.camera_scan_done = 0;
    CZ_bb.scan_count = 0;
    CZ_bb.pos_stable_times = 0;
    CZ_bb.r1_platform_cmd = 0;
    CZ_bb.r1_place_cmd = 0;
}

void BT_CZ_Init_Task(void)
{
    CZ_bb.req_start = 0;
    CZ_bb.cz_mission_state = CZ_IDLE;
    CZ_bb.seq_mem_idx = 0;
    CZ_bb.cz_timer_ms = 0;
    CZ_bb.camera_scan_done = 0;
    CZ_bb.scan_count = 0;
    CZ_bb.pos_stable_times = 0;
    CZ_bb.r1_platform_cmd = 0;
    CZ_bb.r1_place_cmd = 0;
}

void BT_Customized_Tick1ms_CZ(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return;
    div10 = 0;

    BT_Status status = BT_CZ_Task_Tree_Root();

    if (status == BT_FAILURE) {
        Act_CZ_FinishCleanup();
        CZ_bb.seq_mem_idx = 0;
    }
}







/* ================================================ MASTER 行为�?================================================ */
/* ================================================ MASTER 行为�?================================================ */
/* ================================================ MASTER 行为�?================================================ */



/* ================================== MASTER 状态区 ================================== */

uint8_t master_task_request_flag = 0;  /* MASTER 总任务请求标�?*/
uint8_t master_seq_mem_idx = 0;        /* MASTER Memory Sequence 索引 */

/* 次任�?Wrapper 状�?*/
typedef enum {
    SUBTASK_IDLE = 0,
    SUBTASK_RUNNING,
    SUBTASK_DONE,
    SUBTASK_FAILED
} Subtask_State;
Subtask_State mc_subtask_state = SUBTASK_IDLE;      /* MC 任务 wrapper 状态 */
Subtask_State mf_subtask_state = SUBTASK_IDLE;      /* MF 任务 wrapper 状态 */
Subtask_State cz_subtask_state = SUBTASK_IDLE;   /* ARENA 任务 wrapper 状态 */


/* ================================== MASTER 条件节点 ================================= */

/**
 * @brief [条件节点] 检查是否有 MASTER 总任务请�?
 */
static BT_Status Cond_Has_MASTER_Request(void)
{
    
    return master_task_request_flag ? BT_SUCCESS : BT_FAILURE;
}


/* ================================== MASTER 动作节点 ================================= */


/* ====================== 动作（子任务）节�?====================== */

/**
 * @brief [动作节点] MC(武馆)任务 Wrapper
 * @note 内部调用已有�?BT_Combined_Task_Tree_Root
 * @note 自动触发 MC 任务，等待其完成后返�?SUCCESS
 */
static BT_Status Act_MC_Task_Wrapper(void)
{
    switch (mc_subtask_state) {

    case SUBTASK_IDLE: {
        /* 触发 MC 任务 */
        BT_Start_R2_MC_Mission();
        mc_subtask_state = SUBTASK_RUNNING;
        return BT_RUNNING;
    }
    case SUBTASK_RUNNING: {
        /* 运行 MC 行为�?*/
        BT_Status mc_status = BT_R2_MC_Tree_Root();
        if (mc_status == BT_SUCCESS) {
            mc_subtask_state = SUBTASK_DONE;
            return BT_SUCCESS;
        } else if (mc_status == BT_FAILURE) {
            mc_subtask_state = SUBTASK_FAILED;
            return BT_FAILURE;
        }
        return BT_RUNNING;
    }
    case SUBTASK_DONE:{
        return BT_SUCCESS;
    }
    case SUBTASK_FAILED:{
        return BT_FAILURE;
    }
    default:{
        return BT_FAILURE;
    }

    }
}

/**
 * @brief [动作节点] MF(梅林)任务 Wrapper
 * @note 内部调用已有�?BT_MF_Task_Tree_Root
 * @note 自动触发 MF 任务，等待其完成后返�?SUCCESS
 */
/** @note 【重要】Arm_Back_Service_Tick() 必须在 BT_MF_Task_Tree_Root() 之前调用，
 *  保证 arm_back_payload_done 在同一 tick 内被 MF_EXEC_PICK_R2 消费。 */
static BT_Status Act_MF_Task_Wrapper(void)
{
    switch (mf_subtask_state) {

    case SUBTASK_IDLE:{
        /* 触发 MF 任务 */
        BT_MF_Start_Task();
        mf_subtask_state = SUBTASK_RUNNING;
        return BT_RUNNING;
    }
    case SUBTASK_RUNNING: {
        Arm_Back_Service_Tick();

        /* 运行 MF 行为树 */
        BT_Status mf_status = BT_MF_Task_Tree_Root();
        if (mf_status == BT_SUCCESS) {
            // if(arm_store_busy || arm_store_request || !arm_store_payload_done) {
            //     /* 机械臂存储服务仍在忙，等待其完成 */
            //     return BT_RUNNING;
            // }
            mf_subtask_state = SUBTASK_DONE;
            return BT_SUCCESS;
        } else if (mf_status == BT_FAILURE) {
            mf_subtask_state = SUBTASK_FAILED;
            return BT_FAILURE;
        }
        return BT_RUNNING;
    }
    case SUBTASK_DONE:{
        return BT_SUCCESS;
    }
    case SUBTASK_FAILED:{
        return BT_FAILURE;
    }
    default:{
        return BT_FAILURE;
    }

    }
}


/**
 * @brief [动作节点] ARENA(对抗�?任务 Wrapper
 * @note 内部调用 BT_CZ_Task_Tree_Root
 */
static BT_Status Act_CZ_Task_Wrapper(void)
{
    switch (cz_subtask_state) {

    case SUBTASK_IDLE: {
        /* 触发 CZ 任务 */
        BT_CZ_Start_Task();
        cz_subtask_state = SUBTASK_RUNNING;
        return BT_RUNNING;
    }
    case SUBTASK_RUNNING: {
        /* 运行 CZ 行为树 */
        BT_Status cz_status = BT_CZ_Task_Tree_Root();
        if (cz_status == BT_SUCCESS) {
            cz_subtask_state = SUBTASK_DONE;
            return BT_SUCCESS;
        } else if (cz_status == BT_FAILURE) {
            cz_subtask_state = SUBTASK_FAILED;
            return BT_FAILURE;
        }
        return BT_RUNNING;
    }
    case SUBTASK_DONE:{
        return BT_SUCCESS;
    }
    case SUBTASK_FAILED:{
        return BT_FAILURE;
    }
    default:{
        return BT_FAILURE;
    }

    }
}


/* ====================== 清理节点 ====================== */

/**
 * @brief [动作节点] MASTER 任务收尾清理
 */
static BT_Status Act_MASTER_Finish_Cleanup(void)
{
    master_task_request_flag = 0;

    mc_subtask_state = SUBTASK_IDLE;
    mf_subtask_state = SUBTASK_IDLE;
    cz_subtask_state = SUBTASK_IDLE;

    /* 清理各子任务状态，防止残留 */
    BT_Init_R2_MC_Mission();
    BT_MF_Init_Task();
    // BT_ARENA_Init_Task(); 此处如果�?ARENA 任务，也需要调用其初始化函数，�?BT_ARENA_Init_Task();

    return BT_SUCCESS;
}


/* ====================== MASTER 顶层 ====================== */

/**
 * @brief MASTER 总任务行为树 Root 根节�?
 * @note 按顺序执行：
 *   1. 等待总请�?
 *   2. MC(武馆)任务
 *   3. MF(梅林)任务
 *   4. CZ(对抗区)任务
 *   5. 清理
 */
static BT_Status BT_MASTER_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_MASTER_Request,       /* 1. 有总任务请求？ */
        // Act_MC_Task_Wrapper,           /* 2. MC(武馆) */
        Cond_Has_MF_Request,           /* 3. 有 MF 任务请求 */
        Act_MF_Task_Wrapper,           /* 4. MF(梅林) */
        // Cond_Has_CZ_Request,           /* 5. 有 CZ 任务请求 */
        // Act_CZ_Task_Wrapper,           /* 6. CZ(对抗区) */
        Act_MASTER_Finish_Cleanup      /* 7. 清理 */
    };

    return BT_Sequence_Memory(seq,
                              (uint8_t)(sizeof(seq) / sizeof(seq[0])),
                              &master_seq_mem_idx);
}


/* ====================== MASTER 外部接口 ====================== */

/**
 * @brief 发起 MASTER 总任务请�?
 */
void BT_MASTER_Start_Task(void)
{
    master_task_request_flag = 1;
    mc_subtask_state = SUBTASK_IDLE;
    mf_subtask_state = SUBTASK_IDLE;
    cz_subtask_state = SUBTASK_IDLE;
}

/**
 * @brief 初始�?重置 MASTER 任务状�?
 */
void BT_MASTER_Init_Task(void)
{
    master_task_request_flag = 0;
    master_seq_mem_idx = 0;
    mc_subtask_state = SUBTASK_IDLE;
    mf_subtask_state = SUBTASK_IDLE;
    cz_subtask_state = SUBTASK_IDLE;
}

/**
 * @brief MASTER 行为�?Tick 入口
 * @note 内部降频�?10ms
 */
void BT_MASTER_Tick(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return;
    div10 = 0;

    BT_Status status = BT_MASTER_Task_Tree_Root();

    if (status == BT_FAILURE) {
        Act_MASTER_Finish_Cleanup();
        master_seq_mem_idx = 0;
    }
}


/* ================================================ PS 预选赛行为�?================================================ */
/* ================================================ PS 预选赛行为�?================================================ */
/* ================================================ PS 预选赛行为�?================================================ */

PS_MC_Blackboard PS_bb;

static BT_Status Cond_Has_PS_Request(void)
{
    return PS_bb.req_start ? BT_SUCCESS : BT_FAILURE;
}

static BT_Status Act_PS_MC_Mission(void)
{
    /* 矛架位置表：每个位置独立 X/Y/W，修改此处即可适配红/蓝区 */
    static const struct { float x, y, w; } ps_rack[3] = {
        {PS_RACK_1_X, PS_RACK_1_Y, PS_RACK_1_W},
        {PS_RACK_2_X, PS_RACK_2_Y, PS_RACK_2_W},
        {PS_RACK_3_X, PS_RACK_3_Y, PS_RACK_3_W},
    };

    switch (PS_bb.state) {

        case PS_MC_IDLE:
        {
            /* 启动入口已写入 round_count：A=0，B=1，C=2。 */
            PS_bb.state = PS_bb.direct_mf ? PS_MC_ENTER_MF : PS_MC_CLAMP_INIT;
            PS_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case PS_MC_CLAMP_INIT:
        {
            clamp.lift_flag = 1;
            clamp.rotate_flag = 1;
            PS_bb.state = PS_MC_SELF_ROTATE;
            PS_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case PS_MC_SELF_ROTATE:
        {
            /* 原地自转：先转到目标 yaw，再平移去矛架 */
            uint8_t r = PS_bb.round_count;
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, Radar_Pos.reliable_x, Radar_Pos.reliable_y, ps_rack[r].w, 2000, 0);
            taskEXIT_CRITICAL();
            PS_bb.state = PS_MC_NAV_TO_RACK;
            PS_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case PS_MC_NAV_TO_RACK:
        {
            uint8_t r = PS_bb.round_count;
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, ps_rack[r].x, ps_rack[r].y, ps_rack[r].w, 0, 0);
            taskEXIT_CRITICAL();
            PS_bb.state = PS_MC_RACK_PICK;
            PS_bb.timer_ms = 0;
            PS_bb.pos_stable_times = 0;
            PS_bb.pick_success = 0;
            PS_bb.ext_timer_ms = 0;
            PS_bb.ext_jam_retries = 0;
            return BT_RUNNING;
        }

        case PS_MC_RACK_PICK:
        {
            uint8_t r = PS_bb.round_count;
            if (chassis_if_arrived(ps_rack[r].x, ps_rack[r].y, ps_rack[r].w))
            {
                PS_bb.pos_stable_times++;
                if (PS_bb.pos_stable_times > B_RACK_STABLE_TIMES_CONDITION)
                {
                    
                    clamp.retractable_flag = 1;
                    PS_bb.last_ext_angle = m3508_fdcan2[1].Angle_Sum; /* 记录伸出起始角度 */
                    {
                        if (abs_int32(m3508_fdcan2[1].Angle_Sum - CLAMP_EXTEND_MAX_POSITION) < 4000)
                        {
                            /* 伸到位 → 抓取 */
                            PS_bb.ext_jam_retries = 0;
                            clamp.catch_flag = 1;
                            osDelay(500);
                            if (PS_bb.pick_success == 1)
                            {
                                clamp.rotate_flag = 0;
                                PS_bb.state = PS_MC_NAV_TO_DOCK;
                                PS_bb.timer_ms = 0;
                                PS_bb.pos_stable_times = 0;
                                return BT_RUNNING; /* 防止穿透到堵转检测 */
                            }
                            else
                            {
                                clamp.catch_flag = 0;
                                clamp.retractable_flag = 0;
                                osDelay(300);
                                PS_bb.pos_stable_times = 0;
                            }
                        }
                    }
                }
            }
            if (clamp.retractable_flag == 1)
            {
                /*
                 * 夹爪伸出中，若电机Angle_Sum连续不变（堵转/撞歪），
                 * 计时超时后缩回重试，超过最大重试次数则放弃本次抓取。
                 * 若电机恢复转动则清零计时。
                 */
                int32_t cur_angle = m3508_fdcan2[1].Angle_Sum;
                if (abs_int32(cur_angle - PS_bb.last_ext_angle) < 3000)
                {
                    /* 本Tick电机基本没动，累计堵转计时 */
                    if (++PS_bb.ext_timer_ms * 10 >= B_EXT_TIMEOUT_MS)
                    {
                         clamp.catch_flag = 1;
                            osDelay(400);
                            if (PS_bb.pick_success == 1)
                            {
                               clamp.rotate_flag = 0;
                                PS_bb.state = PS_MC_NAV_TO_DOCK;
                                PS_bb.timer_ms = 0;
                                PS_bb.pos_stable_times = 0;
                                return BT_RUNNING; /* 防止穿透到堵转检测 */
                            }
                            else
                            {
                                clamp.catch_flag = 0;
                            }
                        PS_bb.ext_timer_ms = 0;
                        if (PS_bb.ext_jam_retries >= B_EXT_MAX_RETRIES)
                        {
                            /* 已重试过最大次数，放弃本次抓取，重新从底盘稳定开始等待 */
                            PS_bb.ext_jam_retries = 0;
                            PS_bb.ext_timer_ms = 0;
                            PS_bb.pos_stable_times = 0;
                            clamp.catch_flag = 0;
                            clamp.retractable_flag = 0;
                            osDelay(300);
                        }
                        else
                        {
                            PS_bb.ext_jam_retries++;       /* 记录本次重试 */
                            clamp.retractable_flag = 0;    /* 缩回重试 */
                            PS_bb.ext_timer_ms = 0;
                            PS_bb.pos_stable_times = 0;    /* 重试时重新等待稳定 */
                        }
                    }
                }
                else
                {
                    /* 电机仍在转动，重置堵转计时 */
                    PS_bb.ext_timer_ms = 0;
                }
                PS_bb.last_ext_angle = cur_angle; /* 始终更新，比较相邻Tick差值 */
            }
           
            return BT_RUNNING;
        }

        case PS_MC_NAV_TO_DOCK:
        {
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, PS_DOCK_X, PS_DOCK_Y, PS_DOCK_W, 0, 0);
            taskEXIT_CRITICAL();
           
            PS_bb.state = PS_MC_DOCK_WAIT;
            PS_bb.timer_ms = 0;
            PS_bb.dock_success = 0;
            return BT_RUNNING;
        }

        case PS_MC_DOCK_WAIT:
        {
            if (chassis_if_arrived(PS_DOCK_X, PS_DOCK_Y, PS_DOCK_W))
            {  
							PS_bb.pos_stable_times++;
                if (PS_bb.pos_stable_times > B_RACK_STABLE_TIMES_CONDITION)
                { 
							  clamp.lift_flag = 0;
                chassis_set_lock(1);
								}
            }
            if (PS_bb.dock_success == 1)
            {
                clamp.catch_flag = 0;
                osDelay(600);
                clamp.rotate_flag = 1;
                clamp.retractable_flag = 0;
                osDelay(1000);
                chassis_set_lock(0);
                PS_bb.state = PS_MC_ROUND_CHECK;
                PS_bb.dock_success = 0;
                PS_bb.timer_ms = 0;
            }
            else if (++PS_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS * 3)
                return BT_FAILURE;
            return BT_RUNNING;
        }

        case PS_MC_ROUND_CHECK:
        {
            PS_bb.round_count++;
            PS_bb.state = (PS_bb.round_count >= 3) ? PS_MC_NAV_TO_START : PS_MC_CLAMP_INIT;
            PS_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case PS_MC_NAV_TO_START:
        {
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj,
                                    B_CHASSIS_HOME_MC_X,
                                    B_CHASSIS_HOME_MC_Y,
                                    B_CHASSIS_HOME_MC_W,
                                    0,
                                    0);
            taskEXIT_CRITICAL();

            PS_bb.state = PS_MC_WAIT_START;
            PS_bb.timer_ms = 0;
            PS_bb.pos_stable_times = 0;
            PS_bb.mf_req = 0;
            return BT_RUNNING;
        }

        case PS_MC_WAIT_START:
        {
            if (chassis_if_arrived(B_CHASSIS_HOME_MC_X,
                                   B_CHASSIS_HOME_MC_Y,
                                   B_CHASSIS_HOME_MC_W))
            {
                PS_bb.pos_stable_times++;
                if (PS_bb.pos_stable_times > B_RACK_STABLE_TIMES_CONDITION)
                {
                    chassis_set_lock(1);
                    PS_bb.state = PS_MC_WAIT_MF_CMD;
                    PS_bb.timer_ms = 0;
                    PS_bb.mf_req = 0;
                }
            }
            else
            {
                PS_bb.pos_stable_times = 0;
            }

            if (++PS_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
                return BT_FAILURE;
            return BT_RUNNING;
        }

        case PS_MC_WAIT_MF_CMD:
        {
            if (PS_bb.mf_req == 0)
                return BT_RUNNING;

            PS_bb.mf_req = 0;
            chassis_set_lock(0);
            PS_bb.state = PS_MC_ENTER_MF;
            return BT_RUNNING;
        }

        case PS_MC_ENTER_MF:
        {
            PS_bb.state = PS_MC_IDLE;
            return BT_SUCCESS;
        }

        default:
            return BT_FAILURE;
    }
}

/**
 * @brief [动作节点] PS(预选赛)MF区任务，复用主赛流程
 * @note 与主赛 Act_MF_Task_Wrapper 流程一致，只替换 KFS 数量和收尾行为。
 */
static BT_Status Act_PS_MF_Mission(void)
{
    switch (mf_subtask_state) {

    case SUBTASK_IDLE:{
        /* 触发 PS MF 任务：预选赛只取 2 个 R2KFS，完成后停在 MF 结束区 */
        BT_MF_Start_Task_Config(PS_R2_KFS_ALL_NEED, 0);
        mf_subtask_state = SUBTASK_RUNNING;
        return BT_RUNNING;
    }
    case SUBTASK_RUNNING: {
        Arm_Back_Service_Tick();

        /* 运行 MF 行为树 */
        BT_Status mf_status = BT_MF_Task_Tree_Root();
        if (mf_status == BT_SUCCESS) {
            mf_subtask_state = SUBTASK_DONE;
            return BT_SUCCESS;
        } else if (mf_status == BT_FAILURE) {
            mf_subtask_state = SUBTASK_FAILED;
            return BT_FAILURE;
        }
        return BT_RUNNING;
    }
    case SUBTASK_DONE:{
        return BT_SUCCESS;
    }
    case SUBTASK_FAILED:{
        return BT_FAILURE;
    }
    default:{
        return BT_FAILURE;
    }

    }
}


static BT_Status Act_PS_FinishCleanup(void)
{
    /* ！！！ 待测试 ！！！ */
    chassis_set_lock(1);    /* 底盘完成后锁止在原地 */

    PS_bb.req_start = 0;
    PS_bb.state = PS_MC_IDLE;
    PS_bb.timer_ms = 0;
    PS_bb.pos_stable_times = 0;
    PS_bb.pick_success = 0;
    PS_bb.dock_success = 0;
    PS_bb.mf_req = 0;
    PS_bb.round_count = 0;
    PS_bb.ext_jam_retries = 0;
    PS_bb.ext_timer_ms = 0;
    PS_bb.last_ext_angle = 0;
    PS_bb.direct_mf = 0;
    mf_subtask_state = SUBTASK_IDLE;
    BT_MF_Init_Task();
    // chassis_set_lock(0);
    return BT_SUCCESS;
}

static BT_Status BT_PS_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_PS_Request,
        Act_PS_MC_Mission,
        Act_PS_MF_Mission,
        Act_PS_FinishCleanup
    };
    return BT_Sequence_Memory(seq, (uint8_t)(sizeof(seq) / sizeof(seq[0])), &PS_bb.seq_mem_idx);
}

void BT_PS_Start_Task(void)
{
    BT_PS_Start_Task_FromRack(0);
}

void BT_PS_Start_Task_FromRack(uint8_t rack_index)
{
    chassis_r2.chassis_lock_flag = 0;
    chassis_r2.chassis_en_flag = 1;

    PS_bb.req_start = 1;
    PS_bb.state = PS_MC_IDLE;
    PS_bb.timer_ms = 0;
    PS_bb.pos_stable_times = 0;
    PS_bb.pick_success = 0;
    PS_bb.dock_success = 0;
    PS_bb.mf_req = 0;
    PS_bb.round_count = (rack_index <= 2U) ? rack_index : 0U;
    PS_bb.ext_jam_retries = 0;
    PS_bb.ext_timer_ms = 0;
    PS_bb.last_ext_angle = 0;
    PS_bb.direct_mf = 0;
    mf_subtask_state = SUBTASK_IDLE;
}

void BT_PS_Start_MF_Direct(void)
{
    BT_PS_Start_Task_FromRack(0);
    /* 让抓矛动作节点直接完成，行为树下一节点即为 PS MF 任务。 */
    PS_bb.direct_mf = 1;
}

void BT_PS_Init_Task(void)
{
    PS_bb.req_start = 0;
    PS_bb.state = PS_MC_IDLE;
    PS_bb.seq_mem_idx = 0;
    PS_bb.timer_ms = 0;
    PS_bb.pos_stable_times = 0;
    PS_bb.pick_success = 0;
    PS_bb.dock_success = 0;
    PS_bb.mf_req = 0;
    PS_bb.round_count = 0;
    PS_bb.ext_jam_retries = 0;
    PS_bb.ext_timer_ms = 0;
    PS_bb.last_ext_angle = 0;
    PS_bb.direct_mf = 0;
    mf_subtask_state = SUBTASK_IDLE;
    BT_MF_Init_Task();
}

void BT_PS_Request_MF_Start(void)
{


    if (PS_bb.req_start && PS_bb.state == PS_MC_WAIT_MF_CMD)
        PS_bb.mf_req = 1;
}

void BT_PS_Tick(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return;
    div10 = 0;
    BT_Status status = BT_PS_Task_Tree_Root();
    if (status == BT_FAILURE) {
        Act_PS_FinishCleanup();
        PS_bb.seq_mem_idx = 0;
    }
}






/* ================================================ JGCB 预选赛行为树 ================================================ */
/* ================================================ JGCB 预选赛行为树 ================================================ */
/* ================================================ JGCB 预选赛行为树 ================================================ */






JGCB_Blackboard JGCB_bb;

/* 判断 R1 红外给出的九宫格列号是否合法。 */
static uint8_t JGCB_IsValidColumn(uint8_t col)
{
    return (col >= 1 && col <= 3);
}

/* 将 1/2/3 列转换为实际底盘放块点。 */
static void JGCB_SetSudokuTarget(uint8_t col)
{
    if (col == 1) {
        CZ_SUDUKU_X = B_CHASSIS_SUDUKU_1_X;
        CZ_SUDUKU_Y = B_CHASSIS_SUDUKU_1_Y;
        CZ_SUDUKU_W = B_CHASSIS_SUDUKU_1_W;
    } else if (col == 2) {
        CZ_SUDUKU_X = B_CHASSIS_SUDUKU_2_X;
        CZ_SUDUKU_Y = B_CHASSIS_SUDUKU_2_Y;
        CZ_SUDUKU_W = B_CHASSIS_SUDUKU_2_W;
    } else {
        CZ_SUDUKU_X = B_CHASSIS_SUDUKU_3_X;
        CZ_SUDUKU_Y = B_CHASSIS_SUDUKU_3_Y;
        CZ_SUDUKU_W = B_CHASSIS_SUDUKU_3_W;
    }
}

/* 读取并清除一次 R1 红外列命令。 */
static uint8_t JGCB_ConsumePlaceColumn(void)
{
    uint8_t col = JGCB_bb.ir_column_cmd;

    if (JGCB_IsValidColumn(col)) {
        JGCB_bb.ir_column_cmd = 0;
        JGCB_bb.active_col = col;
        return 1;
    }

    return 0;
}

/* 下发前往“等待放块点”的底盘路径。 */
static void JGCB_RouteToWaitPoint(uint8_t clear_queue)
{
    taskENTER_CRITICAL();
    chassis_route_point_add(&chassis_route_traj,
                            B_CHASSIS_JGCB_WAIT_X,
                            B_CHASSIS_JGCB_WAIT_Y,
                            B_CHASSIS_JGCB_WAIT_W,
                            0,
                            clear_queue);
    taskEXIT_CRITICAL();
}

/* 下发前往指定九宫格列前方的底盘路径。 */
static void JGCB_RouteToSudoku(uint8_t col)
{
    JGCB_SetSudokuTarget(col);

    taskENTER_CRITICAL();
    chassis_route_point_add(&chassis_route_traj, CZ_SUDUKU_X, CZ_SUDUKU_Y, CZ_SUDUKU_W, 0, 1);
    taskEXIT_CRITICAL();
}

/* 机械臂到位后，底盘前移把 KFS 送入格内。 */
static void JGCB_RouteSudokuForward(void)
{
    taskENTER_CRITICAL();
    chassis_route_point_add(&chassis_route_traj, CZ_SUDUKU_X, CZ_SUDUKU_Y - 450.0f, CZ_SUDUKU_W, 0, 0);
    taskEXIT_CRITICAL();
}

/* 放块后先退回列前方，再决定取下一块或结束。 */
static void JGCB_RouteSudokuBack(void)
{
    taskENTER_CRITICAL();
    chassis_route_point_add(&chassis_route_traj, CZ_SUDUKU_X, CZ_SUDUKU_Y, CZ_SUDUKU_W, 0, 1);
    taskEXIT_CRITICAL();
}

/* 场边 KFS 中心点：1 号最近，2/3 号依次远离九宫格。 */
static float JGCB_KfsCenterX(uint8_t kfs_index)
{
    (void)kfs_index;
    return B_CHASSIS_JGCB_KFS_CENTER_X;
}

static float JGCB_KfsCenterY(uint8_t kfs_index)
{
    if (kfs_index == 2) {
        return B_CHASSIS_JGCB_KFS_2_CENTER_Y;
    } else if (kfs_index == 3) {
        return B_CHASSIS_JGCB_KFS_3_CENTER_Y;
    }
    return B_CHASSIS_JGCB_KFS_1_CENTER_Y;
}

/* R2 取块站位：X 比 KFS 中心小 900mm，Y 与 KFS 中心一致。 */
static float JGCB_KfsPickX(uint8_t kfs_index)
{
    return JGCB_KfsCenterX(kfs_index) - B_CHASSIS_JGCB_KFS_PICK_OFFSET_X;
}

/**
 * @brief R2 取块站位：Y 比 KFS 中心小 110mm，保证机械臂伸出后能抓到 KFS。
 * @note 
*/
static float JGCB_KfsPickY(uint8_t kfs_index)
{
    return (JGCB_KfsCenterY(kfs_index));
}

/* 下发前往场边指定 KFS 的取块点。 */
static void JGCB_RouteToKfsPick(uint8_t kfs_index)
{
    taskENTER_CRITICAL();
    chassis_route_point_add(&chassis_route_traj,
                            JGCB_KfsPickX(kfs_index),
                            JGCB_KfsPickY(kfs_index),
                            B_CHASSIS_JGCB_KFS_PICK_W,
                            0,
                            1);
    taskEXIT_CRITICAL();
}

/* 判断是否已到达场边指定 KFS 的取块点。 */
static uint8_t JGCB_AtKfsPick(uint8_t kfs_index)
{
    return chassis_if_arrived(JGCB_KfsPickX(kfs_index),
                              JGCB_KfsPickY(kfs_index),
                              B_CHASSIS_JGCB_KFS_PICK_W);
}

static BT_Status Cond_Has_JGCB_Request(void)
{
    return JGCB_bb.req_start ? BT_SUCCESS : BT_FAILURE;
}

/**
 * @brief [动作节点] JGCB(九宫藏宝)单项赛 R2 任务。
 * @note 只保留 R1 红外控制；红外模块写 JGCB_bb 内的命令变量即可。
 */
static BT_Status Act_JGCB_Mission(void)
{
    switch (JGCB_bb.state) {

        case JGCB_IDLE:
        {
            /* 预装 KFS 已由机械臂持有；R2 启动后先等待，再上坡前往 CZ 等待点。 */
            JGCB_bb.state = JGCB_START_DELAY;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_START_DELAY:
        {
            /* 给 R1 启动和场上初始动作留出稳定时间。 */
            if (++JGCB_bb.timer_ms * 10U >= JGCB_START_DELAY_MS)
            {
                JGCB_bb.state = JGCB_NAV_TO_SLOPE_START;
                JGCB_bb.timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case JGCB_NAV_TO_SLOPE_START:
        {
            /* 从启动区进入坡道，预先把坡底和坡顶点入队。 */
            taskENTER_CRITICAL();
            chassis_route_point_add(&chassis_route_traj, B_CHASSIS_SLOPE_START_X, B_CHASSIS_SLOPE_START_Y, B_CHASSIS_SLOPE_START_W, 0, 1);
            chassis_route_point_add(&chassis_route_traj, B_CHASSIS_SLOPE_END_X, B_CHASSIS_SLOPE_END_Y, B_CHASSIS_SLOPE_END_W, 0, 0);
            taskEXIT_CRITICAL();

            JGCB_bb.state = JGCB_NAV_TO_SLOPE_END;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_NAV_TO_SLOPE_END:
        {
            /* 接近坡道起点后启用上坡参数，到坡顶后恢复平地参数。 */
            if (fabsf(Radar_Pos.reliable_x - B_CHASSIS_SLOPE_START_X) < 250.0f &&
                fabsf(Radar_Pos.reliable_y - B_CHASSIS_SLOPE_START_Y) < 250.0f)
            {
                chassis_route_set_slope_mode(1);
            }

            if (chassis_if_arrived(B_CHASSIS_SLOPE_END_X, B_CHASSIS_SLOPE_END_Y, B_CHASSIS_SLOPE_END_W))
            {
                chassis_route_set_slope_mode(0);
                chassis_route_set_rotate_slow(1);

                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, B_CHASSIS_SLOPE_END_X, B_CHASSIS_SLOPE_END_Y, -90.0f, 0, 1);
                taskEXIT_CRITICAL();

                JGCB_bb.state = JGCB_ROTATE_90;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                chassis_route_set_slope_mode(0);
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_ROTATE_90:
        {
            /* 坡顶原地转向，随后带着预装 KFS 前往 CZ 等待点。 */
            if (chassis_if_arrived(B_CHASSIS_SLOPE_END_X, B_CHASSIS_SLOPE_END_Y, -90.0f))
            {
                chassis_route_set_rotate_slow(0);

                JGCB_RouteToWaitPoint(1);

                JGCB_bb.state = JGCB_NAV_TO_WAIT_POINT;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                chassis_route_set_rotate_slow(0);
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_NAV_TO_WAIT_POINT:
        {
            /* 等待到达 R1 红外指令的等待点。 */
            if (chassis_if_arrived(B_CHASSIS_JGCB_WAIT_X, B_CHASSIS_JGCB_WAIT_Y, B_CHASSIS_JGCB_WAIT_W))
            {
                JGCB_bb.state = JGCB_WAIT_PLACE_CMD;
                JGCB_bb.timer_ms = 0;
                JGCB_bb.pos_stable_times = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_WAIT_PLACE_CMD:
        {
            /* 等待 R1 指令：列号=去对应列，上 R1=进入上平台分支。 */
            if (!chassis_if_arrived(B_CHASSIS_JGCB_WAIT_X, B_CHASSIS_JGCB_WAIT_Y, B_CHASSIS_JGCB_WAIT_W)) {
                JGCB_RouteToWaitPoint(1);
                JGCB_bb.state = JGCB_NAV_TO_WAIT_POINT;
                JGCB_bb.timer_ms = 0;
                return BT_RUNNING;
            }

            if (JGCB_bb.r1_platform_cmd && JGCB_bb.held_kfs) {
                JGCB_bb.r1_platform_cmd = 0;
                JGCB_bb.state = JGCB_ARM_RAISE_FOR_R1;
                JGCB_bb.timer_ms = 0;
                return BT_RUNNING;
            }

            if (!JGCB_bb.held_kfs) {
                /* 手上无 KFS 时，按 2 号、3 号顺序去场边取块。 */
                if (JGCB_bb.next_kfs_index <= 3) {
                    JGCB_RouteToKfsPick(JGCB_bb.next_kfs_index);
                    JGCB_bb.state = JGCB_NAV_TO_KFS;
                } else {
                    JGCB_bb.state = JGCB_DONE;
                }
                JGCB_bb.timer_ms = 0;
                return BT_RUNNING;
            }

            if (JGCB_ConsumePlaceColumn()) {
                JGCB_RouteToSudoku(JGCB_bb.active_col);
                JGCB_bb.arm_raise_sent = 0;
                JGCB_bb.state = JGCB_NAV_TO_SUDUKU;
                JGCB_bb.timer_ms = 0;
                JGCB_bb.pos_stable_times = 0;
                return BT_RUNNING;
            }

            return BT_RUNNING;
        }

        case JGCB_NAV_TO_SUDUKU:
        {
            /* 前往目标列时提前抬臂，底盘行进和机械臂动作并行节约时间。 */
            if (!JGCB_bb.arm_raise_sent && arm.mode == ARM_MODE_FRONT)
            {
                taskENTER_CRITICAL();
                arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
                taskEXIT_CRITICAL();
                JGCB_bb.arm_raise_sent = 1;
            }

            if (chassis_if_arrived(CZ_SUDUKU_X, CZ_SUDUKU_Y, CZ_SUDUKU_W))
            {
                JGCB_bb.state = JGCB_bb.arm_raise_sent ? JGCB_ARM_RAISE_WAIT : JGCB_ARM_RAISE;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS * 2)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_ARM_RAISE:
        {
            /* 兜底：如果行进中未能提前抬臂，到位后再补发抬臂轨迹。 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            taskEXIT_CRITICAL();

            JGCB_bb.arm_raise_sent = 1;
            JGCB_bb.state = JGCB_ARM_RAISE_WAIT;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_ARM_RAISE_WAIT:
        {
            /* 机械臂到位后，等待 R1 再次发送同一列命令。 */
            if (arm_if_arrived_xy(CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y))
            {
                JGCB_bb.state = JGCB_WAIT_PUSH_CMD;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_WAIT_PUSH_CMD:
        {
            /*
             * 在列前再次收到同列信号才前移放块；
             * 若收到另一列信号，则取消当前列选择并改去新列前方准备。
             */
            if (JGCB_bb.ir_column_cmd == JGCB_bb.active_col) {
                JGCB_bb.ir_column_cmd = 0;
                JGCB_RouteSudokuForward();
                JGCB_bb.state = JGCB_FORWARD_PUSH;
                JGCB_bb.timer_ms = 0;
            }
            else if (JGCB_IsValidColumn(JGCB_bb.ir_column_cmd))
            {
                JGCB_bb.active_col = JGCB_bb.ir_column_cmd;
                JGCB_bb.ir_column_cmd = 0;
                JGCB_RouteToSudoku(JGCB_bb.active_col);
                JGCB_bb.state = JGCB_NAV_TO_SUDUKU;
                JGCB_bb.timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case JGCB_FORWARD_PUSH:
        {
            /* 等底盘前移到位后再关泵释放。 */
            if (chassis_if_arrived(CZ_SUDUKU_X, CZ_SUDUKU_Y - 450.0f, CZ_SUDUKU_W))
            {
                JGCB_bb.state = JGCB_ARM_PLACE;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_ARM_PLACE:
        {
            /* 关气泵释放 KFS。 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_1_X, CZ_ARM_PLACE_1_Y, 0, ARM_PUMP_RELEASE, CZ_ARM_PLACE_PITCH/180.0f*PI);
            taskEXIT_CRITICAL();

            JGCB_bb.state = JGCB_PUMP_WAIT;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_PUMP_WAIT:
        {
            /* 释放后短暂停留，再退回列前方。 */
            if (++JGCB_bb.timer_ms * 10 >= CZ_PUMP_WAIT_MS)
            {
                JGCB_bb.placed_count++;
                JGCB_bb.held_kfs = 0;
                JGCB_RouteSudokuBack();
                JGCB_bb.state = JGCB_BACK_FROM_SUDUKU;
                JGCB_bb.timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case JGCB_BACK_FROM_SUDUKU:
        {
            /* 退回后若还有 2/3 号 KFS 未取，则去场边取下一块。 */
            if (chassis_if_arrived(CZ_SUDUKU_X, CZ_SUDUKU_Y, CZ_SUDUKU_W))
            {
                if (JGCB_bb.next_kfs_index <= 3) {
                    JGCB_RouteToKfsPick(JGCB_bb.next_kfs_index);
                    JGCB_bb.state = JGCB_NAV_TO_KFS;
                } else {
                    JGCB_bb.state = JGCB_DONE;
                }
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_NAV_TO_KFS:
        {
            /* 到达场边 KFS 取块点。 */
            if (JGCB_AtKfsPick(JGCB_bb.next_kfs_index))
            {
                JGCB_bb.state = JGCB_PICK_KFS;
                JGCB_bb.timer_ms = 0;
                JGCB_bb.pick_sent = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_PICK_KFS:
        {
            /* 下发前向取块轨迹，只发一次。 */
            if (!JGCB_bb.pick_sent && arm.mode == ARM_MODE_FRONT)
            {
                JGCB_bb.pick_sent = 1;
                arm_send_pick_trajectory(0);
                JGCB_bb.state = JGCB_PICK_KFS_WAIT;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= MF_PICK_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_PICK_KFS_WAIT:
        {
            /* 取块完成后保持吸附，机械臂回前向待机点。 */
            if (arm_if_arrived_xy(PICK_PRE_TRANS_X, PICK_PRE_TRANS_Y))
            {
                JGCB_bb.held_kfs = JGCB_bb.next_kfs_index;
                JGCB_bb.next_kfs_index++;
                JGCB_bb.pick_sent = 0;

                taskENTER_CRITICAL();
                arm_queue_add_point(&arm_traj, PICK_REST_X, PICK_REST_Y, 0, ARM_PUMP_SUCK, 0.0f);
                taskEXIT_CRITICAL();

                JGCB_RouteToWaitPoint(1);
                JGCB_bb.state = JGCB_RETURN_WAIT_POINT;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= MF_PICK_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_RETURN_WAIT_POINT:
        {
            /* 带着新 KFS 回等待点，等待下一次 R1 红外列命令。 */
            if (chassis_if_arrived(B_CHASSIS_JGCB_WAIT_X, B_CHASSIS_JGCB_WAIT_Y, B_CHASSIS_JGCB_WAIT_W))
            {
                JGCB_bb.state = JGCB_WAIT_PLACE_CMD;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_ARM_RAISE_FOR_R1:
        {
            /* 收到上 R1 命令后，先把 KFS 抬高到平台放块姿态。 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_X, CZ_ARM_PLACE_L3_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y, 0, ARM_PUMP_KEEP, CZ_ARM_PLACE_PITCH/180.0f*PI);
            taskEXIT_CRITICAL();

            JGCB_bb.state = JGCB_ARM_RAISE_FOR_R1_WAIT;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_ARM_RAISE_FOR_R1_WAIT:
        {
            /* 机械臂到位后，前往 R1 平台接近点。 */
            if (arm_if_arrived_xy(CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y))
            {
                taskENTER_CRITICAL();
                chassis_route_point_add(&chassis_route_traj, B_CHASSIS_R1_STEP_APPROACH_X, B_CHASSIS_R1_STEP_APPROACH_Y, B_CHASSIS_R1_STEP_APPROACH_W, 0, 1);
                taskEXIT_CRITICAL();

                JGCB_bb.step_fsm_started = 0;
                JGCB_bb.platform_route_sent = 0;
                JGCB_bb.state = JGCB_NAV_TO_R1_PLATFORM;
                JGCB_bb.timer_ms = 0;
            }
            else if (++JGCB_bb.timer_ms * 10 >= CZ_ARM_PLACE_TIMEOUT_MS)
            {
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_NAV_TO_R1_PLATFORM:
        {
            /* 使用已有 400mm 上台阶 FSM 登上 R1。 */
            if (!JGCB_bb.step_fsm_started)
            {
                if (chassis_if_arrived(B_CHASSIS_R1_STEP_APPROACH_X,
                                       B_CHASSIS_R1_STEP_APPROACH_Y,
                                       B_CHASSIS_R1_STEP_APPROACH_W))
                {
                    chassis_step_fsm_start(&g_step_fsm, STEP_UP_400);
                    JGCB_bb.step_fsm_started = 1;
                    JGCB_bb.timer_ms = 0;
                }
                else if (++JGCB_bb.timer_ms * 10 >= CHASSIS_ROUTE_TIMEOUT_MS * 2)
                {
                    return BT_FAILURE;
                }
                return BT_RUNNING;
            }

            BT_Status step_status = chassis_step_fsm_tick(&g_step_fsm);

            if (!JGCB_bb.platform_route_sent &&
                g_step_fsm.state == STEP_UP_WAIT_DEPLOY_DONE)
            {
                taskENTER_CRITICAL();
                chassis_r2.chassis_en_flag = 0;
                JGCB_bb.platform_route_sent = 1;
                taskEXIT_CRITICAL();
            }

            if (step_status == BT_SUCCESS)
            {
                taskENTER_CRITICAL();
                chassis_set_lock(1);
                taskEXIT_CRITICAL();

                JGCB_bb.step_fsm_started = 0;
                JGCB_bb.platform_route_sent = 0;
                JGCB_bb.state = JGCB_ON_R1_WAIT_PLACE_CMD;
                JGCB_bb.timer_ms = 0;
            }
            else if (step_status == BT_FAILURE)
            {
                JGCB_bb.step_fsm_started = 0;
                JGCB_bb.platform_route_sent = 0;
                return BT_FAILURE;
            }
            return BT_RUNNING;
        }

        case JGCB_ON_R1_WAIT_PLACE_CMD:
        {
            /* 上 R1 后底盘不再自主移动，只等 R1 放块命令。 */
            if (JGCB_bb.r1_place_cmd && JGCB_bb.held_kfs)
            {
                JGCB_bb.r1_place_cmd = 0;
                JGCB_bb.state = JGCB_ON_R1_PLACE;
                JGCB_bb.timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case JGCB_ON_R1_PLACE:
        {
            /* 在 R1 上收到 3 号红外后，仅关泵释放，保持当前末端姿态不变。 */
            taskENTER_CRITICAL();
            arm_queue_add_point(&arm_traj, CZ_ARM_PLACE_L3_FWD_X, CZ_ARM_PLACE_L3_FWD_Y, 0,
                                ARM_PUMP_RELEASE, arm.effector_pitch_target);
            taskEXIT_CRITICAL();

            JGCB_bb.state = JGCB_ON_R1_PUMP_WAIT;
            JGCB_bb.timer_ms = 0;
            return BT_RUNNING;
        }

        case JGCB_ON_R1_PUMP_WAIT:
        {
            if (++JGCB_bb.timer_ms * 10 >= CZ_PUMP_WAIT_MS)
            {
                JGCB_bb.held_kfs = 0;
                JGCB_bb.placed_count++;
                JGCB_bb.state = JGCB_DONE;
                JGCB_bb.timer_ms = 0;
            }
            return BT_RUNNING;
        }

        case JGCB_DONE:
        {
            return BT_SUCCESS;
        }

        default:
        {
            return BT_FAILURE;
        }
    }
}

static BT_Status Act_JGCB_FinishCleanup(void)
{
    JGCB_bb.req_start = 0;
    JGCB_bb.state = JGCB_IDLE;
    JGCB_bb.timer_ms = 0;
    JGCB_bb.pos_stable_times = 0;
    JGCB_bb.placed_count = 0;
    JGCB_bb.held_kfs = 0;
    JGCB_bb.next_kfs_index = 2;
    JGCB_bb.active_col = 0;
    JGCB_bb.arm_raise_sent = 0;
    JGCB_bb.pick_sent = 0;
    JGCB_bb.step_fsm_started = 0;
    JGCB_bb.platform_route_sent = 0;
    JGCB_bb.ir_column_cmd = 0;
    JGCB_bb.r1_platform_cmd = 0;
    JGCB_bb.r1_place_cmd = 0;
    clamp.lift_flag = 0;
    chassis_route_set_slope_mode(0);
    chassis_route_set_rotate_slow(0);
    chassis_set_lock(1);
    return BT_SUCCESS;
}

static BT_Status BT_JGCB_Task_Tree_Root(void)
{
    static const BT_NodeFunc seq[] = {
        Cond_Has_JGCB_Request,
        Act_JGCB_Mission,
        Act_JGCB_FinishCleanup
    };

    return BT_Sequence_Memory(seq,
                              (uint8_t)(sizeof(seq) / sizeof(seq[0])),
                              &JGCB_bb.seq_mem_idx);
}

static void BT_JGCB_Start_Task_Config(uint8_t held_kfs,
                                      uint8_t next_kfs_index,
                                      uint8_t from_retry)
{
    chassis_r2.chassis_lock_flag = 0;
    chassis_r2.chassis_en_flag = 1;

    JGCB_bb.req_start = 1;
    JGCB_bb.seq_mem_idx = 0;
    JGCB_bb.timer_ms = 0;
    JGCB_bb.pos_stable_times = 0;
    JGCB_bb.placed_count = 0;
    JGCB_bb.active_col = 0;
    JGCB_bb.arm_raise_sent = 0;
    JGCB_bb.pick_sent = 0;
    JGCB_bb.step_fsm_started = 0;
    JGCB_bb.platform_route_sent = 0;
    JGCB_bb.ir_column_cmd = 0;
    JGCB_bb.r1_platform_cmd = 0;
    JGCB_bb.r1_place_cmd = 0;

    JGCB_bb.held_kfs = held_kfs ? 1U : 0U;
    JGCB_bb.next_kfs_index = (next_kfs_index == 3U) ? 3U : 2U;

    if (from_retry)
    {
        if (JGCB_bb.held_kfs)
        {
            /* B：车在 CZ 重试区且手上有块，直接回等待点。 */
            JGCB_RouteToWaitPoint(1);
            JGCB_bb.state = JGCB_NAV_TO_WAIT_POINT;
        }
        else
        {
            /* C/D：车在 CZ 重试区且手空，直接去取指定地上 KFS。 */
            JGCB_RouteToKfsPick(JGCB_bb.next_kfs_index);
            JGCB_bb.state = JGCB_NAV_TO_KFS;
        }
    }
    else
    {
        /* A：预装 KFS 已在机械臂上；地上 1 号 KFS 始终留给 R1。 */
        JGCB_bb.state = JGCB_IDLE;
    }
}

void BT_JGCB_Start_Task(void)
{
    BT_JGCB_Start_Task_Config(1, 2, 0);
}

void BT_JGCB_Start_Task_FromRetry(uint8_t held_kfs, uint8_t next_kfs_index)
{
    BT_JGCB_Start_Task_Config(held_kfs, next_kfs_index, 1);
}

void BT_JGCB_Init_Task(void)
{
    JGCB_bb.req_start = 0;
    JGCB_bb.seq_mem_idx = 0;
    JGCB_bb.state = JGCB_IDLE;
    JGCB_bb.timer_ms = 0;
    JGCB_bb.pos_stable_times = 0;
    JGCB_bb.placed_count = 0;
    JGCB_bb.held_kfs = 0;
    JGCB_bb.next_kfs_index = 2;
    JGCB_bb.active_col = 0;
    JGCB_bb.arm_raise_sent = 0;
    JGCB_bb.pick_sent = 0;
    JGCB_bb.step_fsm_started = 0;
    JGCB_bb.platform_route_sent = 0;
    JGCB_bb.ir_column_cmd = 0;
    JGCB_bb.r1_platform_cmd = 0;
    JGCB_bb.r1_place_cmd = 0;
}

void BT_JGCB_Tick(void)
{
    static uint8_t div10 = 0;
    div10++;
    if (div10 < 10) return;
    div10 = 0;

    BT_Status status = BT_JGCB_Task_Tree_Root();
    if (status == BT_FAILURE) {
        Act_JGCB_FinishCleanup();
        JGCB_bb.seq_mem_idx = 0;
    }
}








