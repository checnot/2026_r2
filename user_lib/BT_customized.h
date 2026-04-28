#ifndef __BT_CUSTOMIZED_H
#define __BT_CUSTOMIZED_H

#include "main.h"
#include "BT_core.h"
//================== 客制化头文件 ==================
#include "own_lib.h"
#include "robotic_arm.h"
#include "ht10a.h"
#include "FreeRTOS.h"     // 用于 taskENTER_CRITICAL / taskEXIT_CRITICAL
#include "task.h"
//================== 客制化头文件 ==================

/* ------------ 预设可配置参数（按需微调） ------------ */
#define ARM_PICK_APPROACH_X_1   0.30f   /* 预抓取上方点1 X */
#define ARM_PICK_APPROACH_Y_1   0.45f   /* 预抓取上方点1 Y */
#define ARM_PICK_APPROACH_X_2   0.80f   /* 预抓取上方点2 X */
#define ARM_PICK_APPROACH_Y_2   0.45f   /* 预抓取上方点2 Y */
#define ARM_PICK_TARGET_X     0.80f   /* 真正抓取点 X */
#define ARM_PICK_TARGET_Y     0.36f   /* 真正抓取点 Y */
#define ARM_PICK_TARGET_TIME  (uint16_t)1250 /* 抓取点停留时间 (ms) */
#define ARM_SAFE_LIFT_X       0.80f   /* 抬升到安全高度的 X */
#define ARM_SAFE_LIFT_Y       0.40f   /* 抬升到安全高度的 Y */
#define ARM_SAFE_LIFT_TIME    (uint16_t)200 /* 抬升到安全高度过后停留时间 (ms) */
#define ARM_STORE_X           0.40f   /* 存储点 X */
#define ARM_STORE_Y           0.40f   /* 存储点 Y */

#define ARM_HOME_X            0.15f   /* 机械臂待机位置 X */
#define ARM_HOME_Y            0.15f   /* 机械臂待机位置 Y */

#define ARM_SUCK_ON           1       /* 吸泵吸取动作标志 */
#define ARM_SUCK_OFF          0       /* 吸泵停止动作标志 */

#define ARRIVE_TOL        0.015f  /* 到位容差 (m) */
#define PUMP_WAIT_MS      180     /* 吸泵等待时间 (ms) */
#define ACTION_TIMEOUT_MS 4500    /* 单步动作超时 (ms) */
/* ------------ 预设可配置参数（按需微调） ------------ */


/* ------------ 黑板 (行为树状态共享区) ------------ */
static volatile struct {
    uint8_t req_pick;     /* 发起一次自动取放请求 (1 = 有请求) */
    uint8_t retry_count;  /* 重试计数（示例） */
} bb;
/* ------------ 黑板 (行为树状态共享区) ------------ */

/* 保存 Sequence 的记忆索引（Memory Sequence 使用） */
static uint8_t seq_mem_idx = 0;


/* ------------ 下层 FSM 类型与状态 --------------- */
/* 抓取动作的内部状态机 */
typedef enum {
    PICK_IDLE,
    PICK_APPROACH_1,
    PICK_APPROACH_2,
    PICK_LOWER,
    PICK_LIFT,
    PICK_STORE,
    PICK_COMPLETE,
    PICK_FAILED
} ArmPickState;
/* ------------ 下层 FSM 类型与状态 --------------- */


#endif // __BT_CUSTOMIZED_
