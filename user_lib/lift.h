#ifndef __LIFT_H
#define __LIFT_H

#include "main.h"
#include "BT_core.h"
#include "chassis.h"
#include "go_m8010.h"
#include "m3508.h"

/* ================================================================
 * 台阶跨越（Chassis Step）模块
 * ------------------------------------------------
 * 该模块封装了机器人跨越 200mm / 400mm 台阶的完整时序逻辑，
 * 包括上台阶（STEP_UP）和下台阶（STEP_DOWN）两种模式。
 *
 * 硬件映射：
 *   前升降关节电机 = m8010_bus_3.motors[0]（带前两根柱状支撑）
 *   后升降关节电机 = m8010_bus_3.motors[1]（带后两根柱状支撑）
 *   后辅助轮(左)   = m3508_fdcan1[1]
 *   后辅助轮(右)   = m3508_fdcan1[2]
 *   前微动开关(x2) = 由 STEP_SWITCH_FRONT 宏读取（需按实际 GPIO 定义）
 *   后微动开关(x2) = 由 STEP_SWITCH_REAR  宏读取（需按实际 GPIO 定义）
 *
 * 与 MF 行为树的关系：
 *   本模块对外暴露 BT_Status 接口，由 Act_MF_Execute_Action()
 *   中的 MF_ACT_STEP_UP / MF_ACT_STEP_DOWN 分支消费。
 * ================================================================ */

/* ================================================================
 * 1. 台阶类型枚举
 * ================================================================ */

/** 台阶动作类型，200/400 共享同一状态机，通过 target_stroke 区分行程 */
typedef enum {
    STEP_NONE = 0,
    STEP_UP_200,      /**< 上 200mm 台阶 */
    STEP_UP_400,      /**< 上 400mm 台阶（预留） */
    STEP_DOWN_200,    /**< 下 200mm 台阶 */
    STEP_DOWN_400     /**< 下 400mm 台阶（预留） */
} ChassisStepType;

/* ================================================================
 * 2. 台阶 FSM 内部状态枚举
 * ================================================================ */

/**
 * @brief 台阶跨越状态机内部状态
 * @note 状态名中的 UP/DOWN 与台阶类型对应；
 *       FRONT/REAR 分别指前支撑柱（面向台阶一侧）和后支撑柱。
 */
typedef enum {
    STEP_FSM_IDLE = 0,           /**< 空闲，无台阶动作进行中 */

    /* ---------- 上台阶流程（8 步） ---------- */
    STEP_UP_DEPLOY,              /**< [上] 四根支撑柱向下伸出至目标行程 */
    STEP_UP_WAIT_DEPLOY_DONE,    /**< [上] 等待支撑柱到位 */
    STEP_UP_DRIVE_TO_FRONT_TOUCH,/**< [上] 后轮驱动前进，等待前微动开关碰到台阶触发 */
    STEP_UP_RETRACT_FRONT,       /**< [上] 前支撑柱收回（抬起到正常高度） */
    STEP_UP_WAIT_FRONT_RETRACT,  /**< [上] 等待前柱收回到位 */
    STEP_UP_DRIVE_TO_REAR_TOUCH, /**< [上] 继续前进，等待后微动开关碰到台阶触发 */
    STEP_UP_RETRACT_REAR,        /**< [上] 后支撑柱收回 */
    STEP_UP_WAIT_REAR_RETRACT,   /**< [上] 等待后柱收回到位 */
    STEP_UP_CHASSIS_FORWARD,     /**< [上] 底盘再向前行进一小段，确保完全在台阶上 */
    STEP_UP_COMPLETE,            /**< [上] 上台阶流程完成 */

    /* ---------- 下台阶流程（10 步） ---------- */
    STEP_DOWN_FORCE_DEPLOY,      /**< [下] 前后支撑柱转为力控，持续向下压住台阶面 */
    STEP_DOWN_DRIVE_FRONT_LEAVE, /**< [下] 缓慢前移，监测前支撑柱是否脱离台阶面悬空 */
    STEP_DOWN_POSITION_FRONT,    /**< [下] 前柱转位控 → 向下伸出至目标行程，撑住 */
    STEP_DOWN_WAIT_FRONT_POS,    /**< [下] 等待前柱位控到位 */
    STEP_DOWN_DRIVE_REAR_LEAVE,  /**< [下] 继续前移，监测后支撑柱是否脱离台阶面悬空 */
    STEP_DOWN_POSITION_REAR,     /**< [下] 后柱转位控 → 向下伸出至目标行程，撑住 */
    STEP_DOWN_WAIT_REAR_POS,     /**< [下] 等待后柱位控到位 */
    STEP_DOWN_CLEARANCE_FORWARD, /**< [下] 四柱撑稳后，再向前走一段，远离台阶边缘 */
    STEP_DOWN_RETRACT_ALL,       /**< [下] 所有支撑柱收回到正常状态 */
    STEP_DOWN_WAIT_RETRACT_ALL,  /**< [下] 等待全部收回 */
    STEP_DOWN_COMPLETE,          /**< [下] 下台阶流程完成 */

    /* ---------- 异常 ---------- */
    STEP_FSM_FAILED              /**< 超时或异常，进入紧急恢复 */
} ChassisStepFSMState;

/* ================================================================
 * 3. 台阶 FSM 上下文（黑板）
 * ================================================================ */

/**
 * @brief 台阶跨越状态机上下文
 * @note 全局单例 g_step_fsm，由行为树节点每 Tick 推进
 */
typedef struct {
    ChassisStepType      step_type;         /**< 当前台阶动作类型 */
    ChassisStepFSMState  state;             /**< 当前状态机状态 */
    uint32_t             timer_ms;          /**< 状态内计时器（10ms 粒度，由调用方累加） */
    float                target_stroke;     /**< 目标行程 (m)，200mm→0.200, 400mm→0.400 */

    /* ---------- 下台阶专用快照 ---------- */
    float                front_leave_snapshot; /**< 前柱离开台阶面瞬间的高度快照 (m) */
    float                rear_leave_snapshot;  /**< 后柱离开台阶面瞬间的高度快照 (m) */
} ChassisStepFSM;

/* ================================================================
 * 3.5 抬升控制模式枚举（由 lift FSM 设置，freertos 消费）
 * ================================================================ */

/**
 * @brief 抬升控制模式
 * @note lift FSM 通过设置全局变量 g_lift_ctrl_mode 来切换控制模式，
 *       freertos 的 entry_chassis 1ms 循环根据此枚举决定调用
 *       m8010_position_control 还是 m8010_torque_control。
 */
typedef enum {
    LIFT_CTRL_NONE = 0,       /**< 无台阶动作，使用正常轨迹队列系统 */
    LIFT_CTRL_POSITION,       /**< 位控模式：m8010_position_control */
    LIFT_CTRL_FORCE_FRONT_REAR,/**< 力控模式：前后柱都用力控压下 */
    LIFT_CTRL_FORCE_REAR_ONLY  /**< 力控模式：仅后柱力控，前柱位控 */
} LiftCtrlMode;

/* 由 lift FSM 设置、freertos 消费的全局控制变量 */
extern LiftCtrlMode g_lift_ctrl_mode;       /**< 当前控制模式 */
extern float       g_lift_front_torque;     /**< 前柱力控目标扭矩 (Nm) */
extern float       g_lift_rear_torque;      /**< 后柱力控目标扭矩 (Nm) */
extern float       g_lift_wheel_target;     /**< 辅助轮目标速度 (rpm) */
extern float       g_lift_wheel_kp;         /**< 辅助轮速度环 Kp */
extern float       g_lift_wheel_kd;         /**< 辅助轮速度环 Kd */

/* ================================================================
 * 4. 可配置参数宏（可按需微调）
 * ================================================================ */

/* ---------- 台阶行程参数 ---------- */

/** 200mm 台阶的实际支撑柱伸出行程 (m)，负号表示向下 */
#define STEP_STROKE_200         0.200f
/** 400mm 台阶的实际支撑柱伸出行程 (m)，负号表示向下 */
#define STEP_STROKE_400         0.400f
/* 平时的支撑收回高度 */
#define LIFT_RETRACT_HEIGHT     0.010f

/* ---------- 上台阶参数 ---------- */

/** 上台阶时辅助轮驱动速度（3508转子单位）*/
#define STEP_UP_WHEEL_SPEED     3000.0f
/** 上台阶辅助轮速度环 Kp */
#define STEP_UP_WHEEL_KP        4.0f
/** 上台阶辅助轮速度环 Kd */
#define STEP_UP_WHEEL_KD        0.5f

/** 上台阶支撑柱位置环 Kp */
#define STEP_UP_BRACE_KP        1.8f
/** 上台阶支撑柱位置环 Kd */
#define STEP_UP_BRACE_KD        0.10f
/* ---------- 下台阶参数 ---------- */

/** 下台阶时辅助轮驱动速度（3508转子单位），比上台阶更慢以保证稳定 */
#define STEP_DOWN_WHEEL_SPEED   2000.0f
/** 下台阶辅助轮速度环 Kp */
#define STEP_DOWN_WHEEL_KP      3.0f
/** 下台阶辅助轮速度环 Kd */
#define STEP_DOWN_WHEEL_KD      0.3f
/** 下台阶力控模式下向下的持续扭矩 (Nm)，使支撑柱始终压住台阶面 */
#define STEP_DOWN_FORCE_TORQUE  0.1f
/** 判定支撑柱"已脱离台阶面悬空"的位置跳变阈值 (m) */
#define STEP_DOWN_LEAVE_THRESHOLD 0.02f

/* ---------- 安全距离 ---------- */

/** 上台阶完成后额外前进的安全时间 (ms)，确保底盘完全在台阶上 */
#define STEP_UP_FORWARD_SAFE_MS      250
/** 下台阶四柱撑稳后额外前进的安全时间 (ms)，远离台阶边缘 */
#define STEP_DOWN_CLEARANCE_SAFE_MS  400

/* ---------- 超时参数 ---------- */

/** ！！！！！！（待确定，此处为暂时值）上台阶总超时 (ms) */
#define STEP_UP_TIMEOUT_MS           60000
/** ！！！！！！（待确定，此处为暂时值）下台阶总超时 (ms) */
#define STEP_DOWN_TIMEOUT_MS         60000
/** ！！！！！！（待确定，此处为暂时值）单个子阶段超时 (ms)，如等待升降到位、等待微动开关触发 */
#define STEP_SUBSTATE_TIMEOUT_MS     60000
/** ！！！！！！（待确定，此处为暂时值）等待升降到位的超时 (ms) */
#define STEP_LIFT_TIMEOUT_MS         60000

/* ---------- 微动开关 GPIO 读取宏 ---------- */
/**
 * 🟡【需根据实际硬件接线修改】
 * 微动开关安装在支撑柱朝前的面上，碰到台阶壁时触发。
 * 假设：前微动开关 = PD1，后微动开关 = PD2，均为低电平有效（常开，碰到台阶闭合→GND）。
 *
 * 使用示例：
 *   PD1 接前左/前右微动开关（并联，任一触发即有效）
 *   PD2 接后左/后右微动开关（并联，任一触发即有效）
 */
#define STEP_SWITCH_FRONT()     (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET)
#define STEP_SWITCH_REAR()      (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == GPIO_PIN_RESET)

/* ================================================================
 * 5. API 接口声明
 * ================================================================ */

void chassis_step_fsm_init(ChassisStepFSM *fsm);

void chassis_step_fsm_start(ChassisStepFSM *fsm, ChassisStepType type);

BT_Status chassis_step_fsm_tick(ChassisStepFSM *fsm);

static inline uint8_t chassis_step_fsm_is_idle(const ChassisStepFSM *fsm)
{
    return (fsm->state == STEP_FSM_IDLE ||
            fsm->state == STEP_UP_COMPLETE ||
            fsm->state == STEP_DOWN_COMPLETE);
}

void chassis_step_fsm_emergency_stop(ChassisStepFSM *fsm);


/* ================================================================
 * 6. 全局实例声明
 * ================================================================ */

/** 全局台阶 FSM 单例，供 MF 行为树节点消费 */
extern ChassisStepFSM g_step_fsm;

#endif // __LIFT_H
