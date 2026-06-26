#include "lift.h"
#include "own_lib.h"   /* abs_float, arm_cos_f32 等工具函数 */

/* ================================================================
 * 全局实例
 * ================================================================ */
ChassisStepFSM g_step_fsm;

/* 由 lift FSM 设置、freertos entry_chassis 消费的全局控制变量 */
LiftCtrlMode g_lift_ctrl_mode   = LIFT_CTRL_NONE;
float        g_lift_front_torque = 0.0f;
float        g_lift_rear_torque  = 0.0f;
float        g_lift_wheel_target = 0.0f;
float        g_lift_wheel_kp     = 0.0f;
float        g_lift_wheel_kd     = 0.0f;

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/**
 * @brief 根据台阶类型获取目标行程（取绝对值，正值）
 * @param type 台阶类型
 * @return 目标行程 (m)
 */
static float step_get_target_stroke(ChassisStepType type)
{
    switch (type) {
        case STEP_UP_200:
        case STEP_DOWN_200:
            return STEP_STROKE_200;
        case STEP_UP_400:
        case STEP_DOWN_400:
            return STEP_STROKE_400;
        default:
            return STEP_STROKE_200;  /* 默认 200mm */
    }
}

/**
 * @brief 获取台阶动作对应的辅助轮目标速度
 * @param type 台阶类型
 * @return 目标速度 (rpm)，下台阶比上台阶慢
 */
static float step_get_wheel_speed(ChassisStepType type)
{
    if (type == STEP_DOWN_200 || type == STEP_DOWN_400) {
        return STEP_DOWN_WHEEL_SPEED;
    }
    return STEP_UP_WHEEL_SPEED;
}

/**
 * @brief 获取台阶动作对应的辅助轮速度环 Kp
 */
static float step_get_wheel_kp(ChassisStepType type)
{
    if (type == STEP_DOWN_200 || type == STEP_DOWN_400) {
        return STEP_DOWN_WHEEL_KP;
    }
    return STEP_UP_WHEEL_KP;
}

/**
 * @brief 获取台阶动作对应的辅助轮速度环 Kd
 */
static float step_get_wheel_kd(ChassisStepType type)
{
    if (type == STEP_DOWN_200 || type == STEP_DOWN_400) {
        return STEP_DOWN_WHEEL_KD;
    }
    return STEP_UP_WHEEL_KD;
}

/**
 * @brief 判断前升降是否到达目标高度
 * @param target 目标高度 (m)，正值向上、负值向下
 * @return 1 = 已到达, 0 = 未到达
 * @note 阈值使用 chassis.h 中定义的 CHASSIS_LIFT_ARRIVAL_THRESHOLD
 */
static uint8_t lift_front_arrived(float target)
{
    return (abs_float(chassis_r2.lift_front_current - target) < CHASSIS_LIFT_ARRIVAL_THRESHOLD);
}

/**
 * @brief 判断后升降是否到达目标高度
 * @param target 目标高度 (m)
 * @return 1 = 已到达, 0 = 未到达
 */
static uint8_t lift_back_arrived(float target)
{
    return (abs_float(chassis_r2.lift_back_current - target) < CHASSIS_LIFT_ARRIVAL_THRESHOLD);
}

/**
 * @brief 判断前后升降两个轴是否都到达目标高度
 * @param target 目标高度 (m)
 * @return 1 = 两轴均已到达, 0 = 未全部到达
 */
static uint8_t lift_both_arrived(float target)
{
    return lift_front_arrived(target) && lift_back_arrived(target);
}

/**
 * @brief 停止辅助轮 — 清零全局目标变量
 */
static void step_wheel_stop(void)
{
    g_lift_wheel_target = 0.0f;
    g_lift_wheel_kp     = 3.0f;
    g_lift_wheel_kd     = 0.0f;
}

/**
 * @brief 驱动辅助轮 — 设置全局目标变量
 * @param speed  目标速度 (rpm)
 * @param kp     速度环 Kp
 * @param kd     速度环 Kd
 * @note freertos 的 entry_chassis 每 1ms 读取这些变量并调用 chassis_lift_wheel_excute
 */
static void step_wheel_drive(float speed, float kp, float kd)
{
    g_lift_wheel_target = speed;
    g_lift_wheel_kp     = kp;
    g_lift_wheel_kd     = kd;
}

/**
 * @brief 设置控制模式为位控（前后柱均位控）
 * @note freertos 消费 g_lift_ctrl_mode，执行 chassis_lift_excute
 */
static void step_set_position_mode(void)
{
    g_lift_ctrl_mode = LIFT_CTRL_POSITION;
}

/**
 * @brief 设置控制模式为力控：前后柱同时力控下压
 * @param front_torque 前柱扭矩 (Nm)
 * @param rear_torque  后柱扭矩 (Nm)
 */
static void step_set_force_both_mode(float front_torque, float rear_torque)
{
    g_lift_ctrl_mode    = LIFT_CTRL_FORCE_FRONT_REAR;
    g_lift_front_torque = front_torque;
    g_lift_rear_torque  = rear_torque;
}

/**
 * @brief 设置控制模式为混合：仅后柱力控，前柱位控
 * @param rear_torque 后柱扭矩 (Nm)
 */
static void step_set_force_rear_only_mode(float rear_torque)
{
    g_lift_ctrl_mode   = LIFT_CTRL_FORCE_REAR_ONLY;
    g_lift_rear_torque = rear_torque;
}

/* ================================================================
 * 公共 API 实现
 * ================================================================ */

/**
 * @brief 初始化台阶 FSM 到空闲状态
 */
void chassis_step_fsm_init(ChassisStepFSM *fsm)
{
    fsm->step_type   = STEP_NONE;
    fsm->state       = STEP_FSM_IDLE;
    fsm->timer_ms    = 0;
    fsm->target_stroke = STEP_STROKE_200;
    fsm->front_leave_snapshot = 0.0f;
    fsm->rear_leave_snapshot  = 0.0f;
}

/**
 * @brief 启动一次台阶跨越动作
 * @note 调用此函数后，FSM 进入首状态，等待 Tick 推进
 */
void chassis_step_fsm_start(ChassisStepFSM *fsm, ChassisStepType type)
{
    fsm->step_type     = type;
    fsm->target_stroke = step_get_target_stroke(type);
    fsm->timer_ms      = 0;
    fsm->front_leave_snapshot = 0.0f;
    fsm->rear_leave_snapshot  = 0.0f;

    /* 根据台阶类型进入对应的初始状态 */
    if (type == STEP_UP_200 || type == STEP_UP_400) {
        fsm->state = STEP_UP_DEPLOY;
    } else if (type == STEP_DOWN_200 || type == STEP_DOWN_400) {
        fsm->state = STEP_DOWN_FORCE_DEPLOY;
    } else {
        fsm->state = STEP_FSM_IDLE;  /* 无效类型，保持空闲 */
    }
}

/**
 * @brief 紧急停止台阶动作并收回所有支撑柱
 */
void chassis_step_fsm_emergency_stop(ChassisStepFSM *fsm)
{
    /* 立即停止辅助轮 */
    step_wheel_stop();

    /* 恢复为正常位控模式，下发收回指令 */
    g_lift_ctrl_mode = LIFT_CTRL_NONE;
    chassis_lift_queue_add_point(&chassis_lift_traj, LIFT_RETRACT_HEIGHT, LIFT_RETRACT_HEIGHT, 0);

    /* 重置状态机 */
    fsm->state       = STEP_FSM_IDLE;
    fsm->step_type   = STEP_NONE;
    fsm->timer_ms    = 0;
}

/* ================================================================
 * 台阶 FSM 主 Tick（核心状态机）
 * ================================================================ */


/**
 * @brief 台阶 FSM 主 Tick 函数
 * @note 由行为树每 10ms 调用一次；
 *       内部 timer_ms 不需要自行累加（由 Tick 调用方在 10ms 粒度下管理），
 *       但这里我们保留自增逻辑以兼容直接调用的场景。
 *
 * 状态机流程概述：
 *
 * ┌── 上台阶 ──────────────────────────────────────────────────┐
 * │                                                             │
 * │  DEPLOY → WAIT_DONE → DRIVE_FRONT → 前开关触发?             │
 * │     ↓                                      ↓(是)            │
 * │  RETRACT_FRONT → WAIT → DRIVE_REAR → 后开关触发?            │
 * │                                        ↓(是)                │
 * │  RETRACT_REAR → WAIT → FORWARD(500ms) → COMPLETE           │
 * │                                                             │
 * └─────────────────────────────────────────────────────────────┘
 *
 * ┌── 下台阶 ──────────────────────────────────────────────────┐
 * │                                                             │
 * │  FORCE_DOWN → DRIVE_FRONT → 前柱悬空?(位置突降)              │
 * │                               ↓(是)                         │
 * │  POS_FRONT → WAIT → DRIVE_REAR → 后柱悬空?                  │
 * │                                    ↓(是)                    │
 * │  POS_REAR → WAIT → CLEARANCE(800ms) → RETRACT_ALL          │
 * │                                          → WAIT → COMPLETE │
 * │                                                             │
 * └─────────────────────────────────────────────────────────────┘
 */
BT_Status chassis_step_fsm_tick(ChassisStepFSM *fsm)
{
    fsm->timer_ms++;

    float wheel_speed = step_get_wheel_speed(fsm->step_type);
    float wheel_kp    = step_get_wheel_kp(fsm->step_type);
    float wheel_kd    = step_get_wheel_kd(fsm->step_type);
    float neg_stroke  = -fsm->target_stroke;

    switch (fsm->state) {

    /* ================================================================ */
    /*                    空闲状态                                      */
    /* ================================================================ */
    case STEP_FSM_IDLE:
        fsm->timer_ms = 0;
        return BT_SUCCESS;

    /* ================================================================ */
    /*                    上台阶流程                                    */
    /* ================================================================ */

    /* ---------- 第 1 步：四根支撑柱向下伸出至目标行程 ---------- */
    case STEP_UP_DEPLOY:
        /*
         * 下发轨迹目标：前后柱同时向下伸出 neg_stroke。
         * freertos 的 entry_chassis 循环消费队列 + 自动位控执行。
         */
        step_set_position_mode();
        chassis_lift_queue_add_point(&chassis_lift_traj, neg_stroke, neg_stroke, 0);

        fsm->state = STEP_UP_WAIT_DEPLOY_DONE;
        fsm->timer_ms = 0;
        return BT_RUNNING;

    /* ---------- 等待支撑柱到位 ---------- */
    case STEP_UP_WAIT_DEPLOY_DONE:
        if (lift_both_arrived(neg_stroke)) {
            /* 前后柱均已到位，开始驱动辅助轮前进 */
            step_wheel_drive(wheel_speed, wheel_kp, wheel_kd);
            fsm->state = STEP_UP_DRIVE_TO_FRONT_TOUCH;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 第 2 步：驱动前进，等待前微动开关碰到台阶 ---------- */
    case STEP_UP_DRIVE_TO_FRONT_TOUCH:
        /*
         * 辅助轮已在上一状态启动（持续驱动中）。
         * 当前微动开关碰到台阶壁时，说明底盘前部已到台阶边缘。
         */
        if (STEP_SWITCH_FRONT()) {
            step_wheel_stop();
            /* 收回前支撑柱，后柱保持伸出 */
            chassis_lift_queue_add_point(&chassis_lift_traj, LIFT_RETRACT_HEIGHT, neg_stroke, 0);
            fsm->state = STEP_UP_WAIT_FRONT_RETRACT;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_SUBSTATE_TIMEOUT_MS) {
            step_wheel_stop();
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 等待前柱收回 ---------- */
    case STEP_UP_WAIT_FRONT_RETRACT:
        if (lift_front_arrived(LIFT_RETRACT_HEIGHT)) {
            /* 前柱已收回，继续前进等待后开关触发 */
            step_wheel_drive(wheel_speed, wheel_kp, wheel_kd);
            fsm->state = STEP_UP_DRIVE_TO_REAR_TOUCH;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 第 3 步：继续前进，等待后微动开关触发 ---------- */
    case STEP_UP_DRIVE_TO_REAR_TOUCH:
        /*
         * 前柱已收回、底盘前部搭在台阶上，继续前进直到后开关触发。
         */
        if (STEP_SWITCH_REAR()) {
            step_wheel_stop();
            /* 收回后支撑柱（全部收回） */
            chassis_lift_queue_add_point(&chassis_lift_traj, LIFT_RETRACT_HEIGHT, LIFT_RETRACT_HEIGHT, 0);
            fsm->state = STEP_UP_WAIT_REAR_RETRACT;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_SUBSTATE_TIMEOUT_MS) {
            step_wheel_stop();
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 等待后柱收回 ---------- */
    case STEP_UP_WAIT_REAR_RETRACT:
        if (lift_both_arrived(LIFT_RETRACT_HEIGHT)) {
            /* 全部收回，上台阶完成（安全前移暂弃用） */
            fsm->state = STEP_UP_CHASSIS_FORWARD;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 安全前移（当前跳过） ---------- */
    case STEP_UP_CHASSIS_FORWARD:
        /* 暂时弃用安全前移，直接完成 */
        fsm->state = STEP_UP_COMPLETE;
        return BT_RUNNING;

    /* ---------- 上台阶完成 ---------- */
    case STEP_UP_COMPLETE:
        fsm->timer_ms = 0;
        return BT_SUCCESS;

    /* ================================================================ */
    /*                    下台阶流程                                    */
    /* ================================================================ */

    /* ---------- 第 1 步：力控下压 ---------- */
    case STEP_DOWN_FORCE_DEPLOY:
        /*
         * 前后支撑柱转为力控，持续向下压住台阶面。
         * 设置全局控制模式 = 力控前后，freertos 消费后调用 m8010_torque_control。
         */
        step_set_force_both_mode(-STEP_DOWN_FORCE_TORQUE, STEP_DOWN_FORCE_TORQUE);

        /* 记录初始位置快照，用于后续检测是否离开台阶面 */
        fsm->front_leave_snapshot = chassis_r2.lift_front_current;
        fsm->rear_leave_snapshot  = chassis_r2.lift_back_current;

        fsm->state = STEP_DOWN_DRIVE_FRONT_LEAVE;
        fsm->timer_ms = 0;
        return BT_RUNNING;

    /* ---------- 第 2 步：缓慢前移，检测前柱离开台阶 ---------- */
    case STEP_DOWN_DRIVE_FRONT_LEAVE:
        /*
         * 辅助轮缓慢前移，同时监测前柱位置。
         * 当前柱脱离台阶面悬空时，力控持续下压导致位置突降。
         */
        step_wheel_drive(wheel_speed, wheel_kp, wheel_kd);

        if ((chassis_r2.lift_front_current - fsm->front_leave_snapshot)
            < -STEP_DOWN_LEAVE_THRESHOLD) {
            /* 前柱已悬空 → 停车，转混合模式（前柱位控，后柱力控） */
            step_wheel_stop();
            step_set_force_rear_only_mode(-STEP_DOWN_FORCE_TORQUE);
            /* 设置前柱位控目标 = 向下伸出至目标行程 */
            chassis_lift_queue_add_point(&chassis_lift_traj, neg_stroke,
                                         chassis_r2.lift_back_current, 0);
            fsm->state = STEP_DOWN_WAIT_FRONT_POS;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_SUBSTATE_TIMEOUT_MS) {
            step_wheel_stop();
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 等待前柱位控到位 ---------- */
    case STEP_DOWN_WAIT_FRONT_POS:
        /*
         * 前柱正在位控向下伸出，后柱保持力控压住台阶面。
         * 控制模式 = LIFT_CTRL_FORCE_REAR_ONLY（上一状态已设置）。
         */
        if (lift_front_arrived(neg_stroke)) {
            /* 前柱已到位撑住，更新后柱快照，恢复前进 */
            fsm->rear_leave_snapshot = chassis_r2.lift_back_current;
            step_wheel_drive(wheel_speed, wheel_kp, wheel_kd);
            fsm->state = STEP_DOWN_DRIVE_REAR_LEAVE;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 第 3 步：继续前移，检测后柱离开台阶 ---------- */
    case STEP_DOWN_DRIVE_REAR_LEAVE:
        /*
         * 前柱已撑住，底盘前部悬在台阶下方。
         * 继续缓慢前进，直到后柱也脱离台阶面悬空。
         */
        if ((chassis_r2.lift_back_current - fsm->rear_leave_snapshot)
            < -STEP_DOWN_LEAVE_THRESHOLD) {
            /* 后柱已悬空 → 停车，切回正常位控模式 */
            step_wheel_stop();
            step_set_position_mode();
            /* 设置后柱位控目标 = 向下伸出至目标行程 */
            chassis_lift_queue_add_point(&chassis_lift_traj, neg_stroke, neg_stroke, 0);
            fsm->state = STEP_DOWN_WAIT_REAR_POS;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_SUBSTATE_TIMEOUT_MS) {
            step_wheel_stop();
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 等待后柱位控到位 ---------- */
    case STEP_DOWN_WAIT_REAR_POS:
        /*
         * 四根支撑柱全部处于位控模式，向下伸出至目标行程。
         */
        if (lift_both_arrived(neg_stroke)) {
            /* 四柱全部到位撑稳，开始安全前移（远离台阶边缘） */
            step_wheel_drive(wheel_speed, wheel_kp, wheel_kd);
            fsm->state = STEP_DOWN_CLEARANCE_FORWARD;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 第 4 步：向前走一段安全距离，远离台阶边缘 ---------- */
    case STEP_DOWN_CLEARANCE_FORWARD:
        /*
         * 四柱撑稳后向前走一小段，确保底盘离开原台阶正下方，
         * 避免收回支撑柱时底盘与台阶底部碰撞。
         */
        if (fsm->timer_ms * 10 >= STEP_DOWN_CLEARANCE_SAFE_MS) {
            step_wheel_stop();
            /* 安全距离已走完，全部收回到正常位置 */
            chassis_lift_queue_add_point(&chassis_lift_traj, 0.0f, 0.0f, 0);
            fsm->state = STEP_DOWN_WAIT_RETRACT_ALL;
            fsm->timer_ms = 0;
        }
        return BT_RUNNING;

    /* ---------- 等待全部收回 ---------- */
    case STEP_DOWN_WAIT_RETRACT_ALL:
        if (lift_both_arrived(0.0f)) {
            fsm->state = STEP_DOWN_COMPLETE;
            fsm->timer_ms = 0;
        } else if (fsm->timer_ms * 10 >= STEP_LIFT_TIMEOUT_MS) {
            fsm->state = STEP_FSM_FAILED;
        }
        return BT_RUNNING;

    /* ---------- 下台阶完成 ---------- */
    case STEP_DOWN_COMPLETE:
        fsm->timer_ms = 0;
        return BT_SUCCESS;

    /* ================================================================ */
    /*                    异常处理                                      */
    /* ================================================================ */
    case STEP_FSM_FAILED:
        /*
         * 超时或异常 → 紧急恢复：
         *   - 停止辅助轮
         *   - 恢复为正常位控模式
         *   - 下发全部收回指令
         */
        step_wheel_stop();
        g_lift_ctrl_mode = LIFT_CTRL_NONE;
        chassis_lift_queue_add_point(&chassis_lift_traj, 0.0f, 0.0f, 0);

        fsm->step_type = STEP_NONE;
        fsm->state     = STEP_FSM_IDLE;
        fsm->timer_ms  = 0;
        return BT_FAILURE;

    default:
        fsm->state = STEP_FSM_FAILED;
        fsm->timer_ms = 0;
        return BT_FAILURE;
    }
}
