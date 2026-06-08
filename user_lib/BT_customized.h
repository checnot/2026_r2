#ifndef __BT_CUSTOMIZED_H
#define __BT_CUSTOMIZED_H

#include "main.h"
#include "BT_core.h"
//================== 客制化头文件 ==================
#include "own_lib.h"
#include "robotic_arm.h"
#include "ht10a.h"
#include "FreeRTOS.h" // 用于 taskENTER_CRITICAL / taskEXIT_CRITICAL
#include "task.h"
#include "chassis.h"
#include "clamp.h"
//================== 客制化头文件 ==================

/* ------------ 机械臂预设可配置参数（按需微调） ------------ */

#define ARM_PICK_P200_APPROACH_X_1 0.30f         /* 预抓取上方点1 X */
#define ARM_PICK_P200_APPROACH_Y_1 0.45f         /* 预抓取上方点1 Y */
#define ARM_PICK_P200_APPROACH_X_2 0.80f         /* 预抓取上方点2 X */
#define ARM_PICK_P200_APPROACH_Y_2 0.45f         /* 预抓取上方点2 Y */
#define ARM_PICK_P200_TARGET_X 0.80f             /* 真正抓取点 X */
#define ARM_PICK_P200_TARGET_Y 0.35f             /* 真正抓取点 Y */
#define ARM_PICK_P200_TARGET_TIME (uint16_t)1250 /* 抓取点停留时间 (ms) */
#define ARM_SAFE_P200_LIFT_X 0.80f               /* 抬升到安全高度的 X */
#define ARM_SAFE_P200_LIFT_Y 0.40f               /* 抬升到安全高度的 Y */
#define ARM_SAFE_P200_LIFT_TIME (uint16_t)500    /* 抬升到安全高度过后停留时间 (ms) */

#define ARM_STORE_X 0.40f /* 存储点 X */
#define ARM_STORE_Y 0.45f /* 存储点 Y */

#define ARM_HOME_X 0.15f /* 机械臂待机位置 X */
#define ARM_HOME_Y 0.15f /* 机械臂待机位置 Y */

#define ARM_SUCK_ON 1  /* 吸泵吸取动作标志 */
#define ARM_SUCK_OFF 0 /* 吸泵停止动作标志 */

#define ARRIVE_TOL 0.03f       /* 到位容差 (m) */
#define ACTION_TIMEOUT_MS 5000 /* 单步动作超时 (ms) */
/* ------------ 机械臂预设可配置参数（按需微调） ------------ */

/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */

#define CHASSIS_HOME_MC_X 400.0f  /* 武馆区域 底盘 HOME 位置 X (mm) */
#define CHASSIS_HOME_MC_Y 1400.0f /* 武馆区域 底盘 HOME 位置 Y (mm) */
#define CHASSIS_HOME_MC_W -180.0f    /* 武馆区域 底盘 HOME 位置 W (deg) */

// 矛头位置：最靠近坐标系原点的第一个矛头为1，依次递增
#define CLAMP_TO_CHASSIS_CENTER_DIST 139.0f /* 夹爪中心到底盘中心的X方向距离 (mm) */
#define CHASSIS_RACK_1_X 590.0f  /* 武馆区域 第一个矛头位置 X (mm) */
#define CHASSIS_RACK_2_X 790.0f  /* 武馆区域 第二个矛头位置 X (mm) */
#define CHASSIS_RACK_3_X 990.0f  /* 武馆区域 第三个矛头位置 X (mm) */
#define CHASSIS_RACK_4_X 1190.0f /* 武馆区域 第四个矛头位置 X (mm) */
#define CHASSIS_RACK_5_X 1390.0f /* 武馆区域 第五个矛头位置 X (mm) */
#define CHASSIS_RACK_6_X 1590.0f /* 武馆区域 第六个矛头位置 X (mm) */
#define CHASSIS_RACK_Y 560.0f    /* 武馆区域 取矛头位置 Y (mm) */
#define CHASSIS_RACK_W -180.0f      /* 武馆区域 取矛头位置 W (deg) */
#define RACK_STABLE_TIMES_CONDITION 200 /*取矛头时到达目标位置次数，保证稳定抓取*/
// 与R1的对接位置（暂定该位置，后续可重设）
#define CHASSIS_DOCKING_X 1200.0f /* 武馆区域 对接位置 X (mm) */
#define CHASSIS_DOCKING_Y 1400.0f /* 武馆区域 对接位置 Y (mm) */
#define CHASSIS_DOCKING_W 0.0f  /* 武馆区域 对接位置 W (deg) */
// 使用相机扫描梅林位置（暂定该位置，后续可重设）
#define CHASSIS_SCANNING_X 2000.0f /* 武馆区域 扫描位置 X (mm) */
#define CHASSIS_SCANNING_Y 3000.0f /* 武馆区域 扫描位置 Y (mm) */
#define CHASSIS_SCANNING_W 90.0f   /* 武馆区域 扫描位置 W (deg) */
// 梅林入口区域地面的三个预设位置，供底盘在扫描后前往，等待机械臂拾取
#define CHASSIS_ENTRY_1_X 2600.0f /* 武馆到梅林区域 入口位置1 X (mm) */
#define CHASSIS_ENTRY_1_Y 4200.0f /* 武馆到梅林区域 入口位置1 Y (mm) */
#define CHASSIS_ENTRY_1_W 0.0f    /* 武馆到梅林区域 入口位置1 W (deg) */
#define CHASSIS_ENTRY_2_X 2600.0f /* 武馆到梅林区域 入口位置2 X (mm) */
#define CHASSIS_ENTRY_2_Y 3000.0f /* 武馆到梅林区域 入口位置2 Y (mm) */
#define CHASSIS_ENTRY_2_W 0.0f    /* 武馆到梅林区域 入口位置2 W (deg) */
#define CHASSIS_ENTRY_3_X 2600.0f /* 武馆到梅林区域 入口位置3 X (mm) */
#define CHASSIS_ENTRY_3_Y 1800.0f /* 武馆到梅林区域 入口位置3 Y (mm) */
#define CHASSIS_ENTRY_3_W 0.0f    /* 武馆到梅林区域 入口位置3 W (deg) */

/**
 * @brief 综合任务中专属的底盘子状态机。
 * @note 为什么不复用原来的 CHASSIS_ROUTE_IDLE 那些？
 *       因为在这个综合任务中，底盘被切分成了两段不连续的动作：
 *       第一段：走到拾取区 (包括点1和拾取点2)；
 *       第二段：抓完之后走到待机区。
 *       所以我们需要一个专为该综合任务服务的子状态机。
 */
typedef enum
{
    CHASSIS_IDLE = 0,     /* 状态空闲，准备下发前往拾取区的第一个点 */
    CHASSIS_TO_RACK,      /* 正在前往拾取准备点 */
    CHASSIS_TO_DOCKING,   /* 正在前往真正的拾取点 */
    CHASSIS_TO_SCANNING,  /* 前往扫描位置，准备扫描 */
    CHASSIS_TO_PICK,      /* 前往拾取点 */
    CHASSIS_TO_PICK_DONE, /* 到达拾取点，底盘第一段任务完成 */

    CHASSIS_TO_STANDBY, // 第二段任务开始：正在前往待机点
    CHASSIS_DONE        // 所有底盘动作完成
} CombinedChassisState;

/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */

#define CHASSIS_ROUTE_TIMEOUT_MS 30000 /* 底盘路径超时 (ms) */

/* ------------ 下层 FSM 类型与状态 --------------- */
/* 抓取动作的内部状态机 */
typedef enum
{
    PICK_IDLE,
    PICK_APPROACH_1,
    PICK_APPROACH_2,
    PICK_LOWER,
    PICK_LIFT,
    PICK_STORE,
    PICK_COMPLETE,
    PICK_FAILED
} ArmPickState;

/*武馆任务状态机*/
typedef enum
{   
    MC_IDLE,
    NIVEGATE_TO_RACK,// 从起点导航到第3个矛架位置
    RACK_PICK,       // 在第3个矛架位置执行拾取动作
    RACK_FAILED_1,   // 第3个矛架位置拾取失败的第一次重试   
    RACK_FAILED_2,   // 第3个矛架位置拾取失败的第二次重试   
    RACK_FAILED_3,   // 第3个矛架位置拾取失败的第三次重试   
    RACK_FAILED_4,   // 第3个矛架位置拾取失败的第四次重试   
    RACK_FAILED_5,   // 第3个矛架位置拾取失败的第五次重试   
    DOCKING,
    WAIT_DOCKING_COMPLETE, // 等待对接完成
    WAIT_MF_INSTRUCTION,   // 等待进入梅林指令
} R2_MC_MISSION_STATE;    

typedef struct
{
    uint8_t req_pick;    /* 是否请求执行拾取动作的标志位 */
    uint8_t retry_count; /* 已经执行的重试次数 */
    uint8_t seq_mem_idx; 
    uint8_t seq_mem_idx_combined; 
    uint8_t R2_MC_timer_ms ; //以 BT tick (10ms为周期) 计数，或在 Tick1ms 中累加 
    uint8_t RACK_POS_STABLE_TIMES; // 矛架位置稳定的计数器，达到一定次数后认为矛架位置稳定，准备执行夹取
    uint8_t RACK_PICK_SUCCESS ; //是否成功检夹取矛头的标志，由微动开关触发
    uint8_t DOCKING_SUCCESS ; //是否成功完成对接的标志，由R1发送指令控制 
    uint8_t MF_ENTRY_INSTRUCTION  ; //是否收到R1发送的进入MF指令的标志，由R1发送指令控制，收到后进入MF任务流程 
    uint8_t MF_MISSION_Req ; //MF任务执行标志，收到进入MF指令后置1
    R2_MC_MISSION_STATE mc_mission_state; /* 武馆任务状态机当前状态 */
}MC_Blackboard; 

/* ------------ 下层 FSM 类型与状态 --------------- */

void BT_Customized_Init_arm_test(void);
void BT_RequestSimplePick(void);
void BT_Customized_Tick1ms_arm_test(void);

void BT_Start_R2_MC_Mission(void);
void BT_Init_R2_MC_Mission(void);   
void BT_Customized_Tick1ms_R2_MC(void);

void BT_Init_Combined_Task(void);
void BT_Customized_Tick1ms_Combined(void);
#endif // __BT_CUSTOMIZED_
