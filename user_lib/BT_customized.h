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
#include "chassis.h"
//================== 客制化头文件 ==================


/* ------------ 机械臂预设可配置参数（按需微调） ------------ */

#define ARM_PICK_P200_APPROACH_X_1   0.30f   /* 预抓取上方点1 X */
#define ARM_PICK_P200_APPROACH_Y_1   0.45f   /* 预抓取上方点1 Y */
#define ARM_PICK_P200_APPROACH_X_2   0.80f   /* 预抓取上方点2 X */
#define ARM_PICK_P200_APPROACH_Y_2   0.45f   /* 预抓取上方点2 Y */
#define ARM_PICK_P200_TARGET_X     0.80f   /* 真正抓取点 X */
#define ARM_PICK_P200_TARGET_Y     0.35f   /* 真正抓取点 Y */
#define ARM_PICK_P200_TARGET_TIME  (uint16_t)1250 /* 抓取点停留时间 (ms) */
#define ARM_SAFE_P200_LIFT_X       0.80f   /* 抬升到安全高度的 X */
#define ARM_SAFE_P200_LIFT_Y       0.40f   /* 抬升到安全高度的 Y */
#define ARM_SAFE_P200_LIFT_TIME    (uint16_t)500 /* 抬升到安全高度过后停留时间 (ms) */

#define ARM_STORE_X           0.40f   /* 存储点 X */
#define ARM_STORE_Y           0.45f   /* 存储点 Y */

#define ARM_HOME_X            0.15f   /* 机械臂待机位置 X */
#define ARM_HOME_Y            0.15f   /* 机械臂待机位置 Y */

#define ARM_SUCK_ON           1       /* 吸泵吸取动作标志 */
#define ARM_SUCK_OFF          0       /* 吸泵停止动作标志 */

#define ARRIVE_TOL        0.03f  /* 到位容差 (m) */
#define ACTION_TIMEOUT_MS 5000    /* 单步动作超时 (ms) */
/* ------------ 机械臂预设可配置参数（按需微调） ------------ */


/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */


#define CHASSIS_HOME_MC_X 400.0f      /* 武馆区域 底盘 HOME 位置 X (mm) */
#define CHASSIS_HOME_MC_Y 1400.0f     /* 武馆区域 底盘 HOME 位置 Y (mm) */
#define CHASSIS_HOME_MC_W 0.0f        /* 武馆区域 底盘 HOME 位置 W (deg) */

// 矛头位置：最靠近坐标系原点的第一个矛头为1，依次递增
#define CHASSIS_RACK_6_X 1450.0f      /* 武馆区域 第六个矛头位置 X (mm) */
#define CHASSIS_RACK_6_Y 600.0f      /* 武馆区域 第六个矛头位置 Y (mm) */
#define CHASSIS_RACK_6_W 0.0f        /* 武馆区域 第六个矛头位置 W (deg) */
// 与R1的对接位置（暂定该位置，后续可重设）
#define CHASSIS_DOCKING_X 1200.0f      /* 武馆区域 对接位置 X (mm) */
#define CHASSIS_DOCKING_Y 1400.0f      /* 武馆区域 对接位置 Y (mm) */
#define CHASSIS_DOCKING_W 179.0f      /* 武馆区域 对接位置 W (deg) */
// 使用相机扫描梅林位置（暂定该位置，后续可重设）
#define CHASSIS_SCANNING_X 2000.0f      /* 武馆区域 扫描位置 X (mm) */
#define CHASSIS_SCANNING_Y 3000.0f      /* 武馆区域 扫描位置 Y (mm) */
#define CHASSIS_SCANNING_W 90.0f      /* 武馆区域 扫描位置 W (deg) */
// 梅林入口区域地面的三个预设位置，供底盘在扫描后前往，等待机械臂拾取
#define CHASSIS_ENTRY_1_X 2600.0f      /* 武馆到梅林区域 入口位置1 X (mm) */        
#define CHASSIS_ENTRY_1_Y 4200.0f     /* 武馆到梅林区域 入口位置1 Y (mm) */
#define CHASSIS_ENTRY_1_W 0.0f      /* 武馆到梅林区域 入口位置1 W (deg) */
#define CHASSIS_ENTRY_2_X 2600.0f      /* 武馆到梅林区域 入口位置2 X (mm) */
#define CHASSIS_ENTRY_2_Y 3000.0f     /* 武馆到梅林区域 入口位置2 Y (mm) */
#define CHASSIS_ENTRY_2_W 0.0f      /* 武馆到梅林区域 入口位置2 W (deg) */
#define CHASSIS_ENTRY_3_X 2600.0f      /* 武馆到梅林区域 入口位置3 X (mm) */
#define CHASSIS_ENTRY_3_Y 1800.0f     /* 武馆到梅林区域 入口位置3 Y (mm) */
#define CHASSIS_ENTRY_3_W 0.0f      /* 武馆到梅林区域 入口位置3 W (deg) */


/**
 * @brief 综合任务中专属的底盘子状态机。
 * @note 为什么不复用原来的 CHASSIS_ROUTE_IDLE 那些？
 *       因为在这个综合任务中，底盘被切分成了两段不连续的动作：
 *       第一段：走到拾取区 (包括点1和拾取点2)；
 *       第二段：抓完之后走到待机区。
 *       所以我们需要一个专为该综合任务服务的子状态机。
 */
typedef enum {
    CHASSIS_IDLE = 0,      /* 状态空闲，准备下发前往拾取区的第一个点 */
    CHASSIS_TO_RACK,     /* 正在前往拾取准备点 */
    CHASSIS_TO_DOCKING,     /* 正在前往真正的拾取点 */
    CHASSIS_TO_SCANNING,     /* 前往扫描位置，准备扫描 */
    CHASSIS_TO_PICK,        /* 前往拾取点 */
    CHASSIS_TO_PICK_DONE,  /* 到达拾取点，底盘第一段任务完成 */
    
    CHASSIS_TO_STANDBY,    // 第二段任务开始：正在前往待机点
    CHASSIS_DONE           // 所有底盘动作完成
} CombinedChassisState;


/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */


#define CHASSIS_ROUTE_TIMEOUT_MS 10000 /* 底盘路径超时 (ms) */





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

typedef enum{
    CHASSIS_ROUTE_IDLE,
    CHASSIS_ROUTE_1,
    CHASSIS_ROUTE_2,
    CHASSIS_ROUTE_3,
    CHASSIS_FAILED

}ChassisRouteState;
/* ------------ 下层 FSM 类型与状态 --------------- */


void BT_Customized_Init_arm_test(void);
void BT_RequestSimplePick(void);
void BT_Customized_Tick1ms_arm_test(void);

void BT_Customized_Tick1ms_chassis_test(void);


void BT_Start_Combined_Task(void);
void BT_Init_Combined_Task(void);
void BT_Customized_Tick1ms_Combined(void);
#endif // __BT_CUSTOMIZED_
