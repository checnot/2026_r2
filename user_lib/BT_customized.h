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
#include "MF_planner.h"
#include "lift.h"
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

// #define ARM_SUCK_ON 1  /* 吸泵吸取动作标志 */
// #define ARM_SUCK_OFF 0 /* 吸泵停止动作标志 */

#define ARRIVE_TOL 0.03f       /* 到位容差 (m) */
#define ACTION_TIMEOUT_MS 5000 /* 单步动作超时 (ms) */
/* ------------ 机械臂预设可配置参数（按需微调） ------------ */

/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */
/************************************************************************************
 * ***
 * ***蓝区Y为正，红区Y为负，以各自底角为原点
*************************************************************************************/
/***************************************
 * 通用参数（红蓝区共用数值）
**************************************** */
#define R_CLAMP_TO_CHASSIS_CENTER_DIST 132.0f /* 夹爪中心到底盘中心的X方向距离 (mm) */
#define B_CLAMP_TO_CHASSIS_CENTER_DIST 132.0f /* 夹爪中心到底盘中心的X方向距离 (mm) */
#define R_RACK_STABLE_TIMES_CONDITION 80      /* 取矛头时到达目标位置次数，保证稳定抓取 */
#define B_RACK_STABLE_TIMES_CONDITION 80      /* 取矛头时到达目标位置次数，保证稳定抓取 */

/***************************************
 * 🔴 红区路径点 (Y为负)
**************************************** */
#define R_CHASSIS_HOME_MC_X 400.0f            /* 红区 底盘 HOME 位置 X (mm) */
#define R_CHASSIS_HOME_MC_Y -1400.0f          /* 红区 底盘 HOME 位置 Y (mm) */
#define R_CHASSIS_HOME_MC_W 0.0f           /* 红区 底盘 HOME 位置 W (deg) */

// 矛头位置：最靠近坐标系原点的第一个矛头为1，依次递增
#define R_CHASSIS_RACK_1_X 425.0f             /* 红区 第一个矛头位置 X (mm) */
#define R_CHASSIS_RACK_2_X 650.0f - R_CLAMP_TO_CHASSIS_CENTER_DIST  /* 红区 第二个矛头位置 X (mm) */
#define R_CHASSIS_RACK_3_X 850.0f - R_CLAMP_TO_CHASSIS_CENTER_DIST  /* 红区 第三个矛头位置 X (mm) */
#define R_CHASSIS_RACK_4_X 1050.0f - R_CLAMP_TO_CHASSIS_CENTER_DIST /* 红区 第四个矛头位置 X (mm) */
#define R_CHASSIS_RACK_5_X 1250.0f - R_CLAMP_TO_CHASSIS_CENTER_DIST /* 红区 第五个矛头位置 X (mm) */
#define R_CHASSIS_RACK_6_X 1450.0f - R_CLAMP_TO_CHASSIS_CENTER_DIST /* 红区 第六个矛头位置 X (mm) */
#define R_CHASSIS_RACK_Y -590.0f              /* 红区 取矛头位置 Y (mm) */
#define R_CHASSIS_RACK_Y_1 -605.0f            /* 红区 取第一个矛头位置 Y (mm) */
#define R_CHASSIS_RACK_W_1 10.0f              /* 红区 取第一个矛头位置 W (deg) */
#define R_CHASSIS_RACK_W 0.0f                 /* 红区 取矛头位置 W (deg) */

// 与R1的对接位置（暂定该位置，后续可重设）
#define R_CHASSIS_DOCKING_X 1200.0f           /* 红区 对接位置 X (mm) */
#define R_CHASSIS_DOCKING_Y -1400.0f          /* 红区 对接位置 Y (mm) */
#define R_CHASSIS_DOCKING_W -180.0f           /* 红区 对接位置 W (deg) */

// 使用相机扫描梅林位置（暂定该位置，后续可重设）
#define R_CHASSIS_SCANNING_X 1600.0f          /* 红区 扫描位置 X (mm) */
#define R_CHASSIS_SCANNING_Y -3000.0f         /* 红区 扫描位置 Y (mm) */
#define R_CHASSIS_SCANNING_W 0.0f             /* 红区 扫描位置 W (deg) */

// 梅林入口区域地面的三个预设位置，供底盘在扫描后前往，等待机械臂拾取
#define R_CHASSIS_ENTRY_1_X 2600.0f           /* 红区 入口位置1 X (mm) */
#define R_CHASSIS_ENTRY_1_Y -4200.0f          /* 红区 入口位置1 Y (mm) */
#define R_CHASSIS_ENTRY_1_W 0.0f              /* 红区 入口位置1 W (deg) */
#define R_CHASSIS_ENTRY_2_X 2600.0f           /* 红区 入口位置2 X (mm) */
#define R_CHASSIS_ENTRY_2_Y -3000.0f          /* 红区 入口位置2 Y (mm) */
#define R_CHASSIS_ENTRY_2_W 0.0f              /* 红区 入口位置2 W (deg) */
#define R_CHASSIS_ENTRY_3_X 2600.0f           /* 红区 入口位置3 X (mm) */
#define R_CHASSIS_ENTRY_3_Y -1800.0f          /* 红区 入口位置3 Y (mm) */
#define R_CHASSIS_ENTRY_3_W 0.0f              /* 红区 入口位置3 W (deg) */

// 出梅林
#define R_CHASSIS_EXIT_1_X 8650.0f            /* 红区 出口位置1 X (mm) */
#define R_CHASSIS_EXIT_1_Y -1800.0f           /* 红区 出口位置1 Y (mm) */
#define R_CHASSIS_EXIT_1_W 0.0f               /* 红区 出口位置1 W (deg) */
#define R_CHASSIS_EXIT_2_X 8650.0f            /* 红区 出口位置2 X (mm) */
#define R_CHASSIS_EXIT_2_Y -4200.0f           /* 红区 出口位置2 Y (mm) */
#define R_CHASSIS_EXIT_2_W 0.0f               /* 红区 出口位置2 W (deg) */

// 梅林进入对抗区
#define R_CHASSIS_SLOPE_START_X 8650.0f       /* 红区 坡道起点 X (mm) */
#define R_CHASSIS_SLOPE_START_Y -5250.0f      /* 红区 坡道起点 Y (mm) */
#define R_CHASSIS_SLOPE_START_W 0.0f          /* 红区 坡道起点 W (deg) */
#define R_CHASSIS_SLOPE_END_X 11300.0f        /* 红区 坡道终点 X (mm) */
#define R_CHASSIS_SLOPE_END_Y -5250.0f        /* 红区 坡道终点 Y (mm) */
#define R_CHASSIS_SLOPE_END_W 0.0f            /* 红区 坡道终点 W (deg) */

// 对抗区路径点位
#define R_CHASSIS_CZ_INIT_X 11500.0f          /* 红区 重试区 X (mm) */
#define R_CHASSIS_CZ_INIT_Y -5500.0f          /* 红区 重试区 Y (mm) */
#define R_CHASSIS_CZ_INIT_W 90.0f             /* 红区 重试区 W (deg) */
#define R_CHASSIS_CZ_CENTER_X 11030.0f        /* 红区 对抗区扫描 X (mm) */
#define R_CHASSIS_CZ_CENTER_Y -2320.0f        /* 红区 对抗区扫描 Y (mm) */
#define R_CHASSIS_CZ_CENTER_W 90.0f           /* 红区 对抗区扫描 W (deg) */

#define R_CHASSIS_SUDUKU_1_X 11290.0f         /* 红区 九宫格第一列 X (mm) */
#define R_CHASSIS_SUDUKU_1_Y -600.0f          /* 红区 九宫格第一列 Y (mm) */
#define R_CHASSIS_SUDUKU_1_W 90.0f            /* 红区 九宫格第一列 W (deg) */
#define R_CHASSIS_SUDUKU_2_X 10750.0f         /* 红区 九宫格第二列 X (mm) */
#define R_CHASSIS_SUDUKU_2_Y -600.0f          /* 红区 九宫格第二列 Y (mm) */
#define R_CHASSIS_SUDUKU_2_W 90.0f            /* 红区 九宫格第二列 W (deg) */
#define R_CHASSIS_SUDUKU_3_X 10210.0f         /* 红区 九宫格第三列 X (mm) */
#define R_CHASSIS_SUDUKU_3_Y -600.0f          /* 红区 九宫格第三列 Y (mm) */
#define R_CHASSIS_SUDUKU_3_W 90.0f            /* 红区 九宫格第三列 W (deg) */

/***************************************
 * 🔵 蓝区路径点 (Y为正)
**************************************** */
#define B_CHASSIS_HOME_MC_X 400.0f            /* 蓝区 底盘 HOME 位置 X (mm) */
#define B_CHASSIS_HOME_MC_Y 1400.0f           /* 蓝区 底盘 HOME 位置 Y (mm) */
#define B_CHASSIS_HOME_MC_W 0.0f           /* 蓝区 底盘 HOME 位置 W (deg) */

// 矛头位置
#define B_CHASSIS_RACK_1_X 450.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST  /* 蓝区 第一个矛头位置 X (mm) */
#define B_CHASSIS_RACK_2_X 650.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST  /* 蓝区 第二个矛头位置 X (mm) */
#define B_CHASSIS_RACK_3_X 850.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST  /* 蓝区 第三个矛头位置 X (mm) */
#define B_CHASSIS_RACK_4_X 1050.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST /* 蓝区 第四个矛头位置 X (mm) */
#define B_CHASSIS_RACK_5_X 1250.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST /* 蓝区 第五个矛头位置 X (mm) */
#define B_CHASSIS_RACK_6_X 1450.0f + B_CLAMP_TO_CHASSIS_CENTER_DIST /* 蓝区 第六个矛头位置 X (mm) */
#define B_CHASSIS_RACK_Y 607.0f               /* 蓝区 取矛头位置 Y (mm) */
#define B_CHASSIS_RACK_W -180.0f                 /* 蓝区 取矛头位置 W (deg) */

// 与R1的对接位置（暂定该位置，后续可重设）
#define B_CHASSIS_DOCKING_X 1200.0f           /* 蓝区 对接位置 X (mm) */
#define B_CHASSIS_DOCKING_Y 1400.0f           /* 蓝区 对接位置 Y (mm) */
#define B_CHASSIS_DOCKING_W 0.0f           /* 蓝区 对接位置 W (deg) */

// 使用相机扫描梅林位置（暂定该位置，后续可重设）
#define B_CHASSIS_SCANNING_X 1600.0f          /* 蓝区 扫描位置 X (mm) */
#define B_CHASSIS_SCANNING_Y 3000.0f          /* 蓝区 扫描位置 Y (mm) */
#define B_CHASSIS_SCANNING_W 0.0f             /* 蓝区 扫描位置 W (deg) */

// 梅林入口区域地面的三个预设位置
#define B_CHASSIS_ENTRY_1_X 2600.0f           /* 蓝区 入口位置1 X (mm) */
#define B_CHASSIS_ENTRY_1_Y 4200.0f           /* 蓝区 入口位置1 Y (mm) */
#define B_CHASSIS_ENTRY_1_W 0.0f              /* 蓝区 入口位置1 W (deg) */
#define B_CHASSIS_ENTRY_2_X 2600.0f           /* 蓝区 入口位置2 X (mm) */
#define B_CHASSIS_ENTRY_2_Y 3000.0f           /* 蓝区 入口位置2 Y (mm) */
#define B_CHASSIS_ENTRY_2_W 0.0f              /* 蓝区 入口位置2 W (deg) */
#define B_CHASSIS_ENTRY_3_X 2600.0f           /* 蓝区 入口位置3 X (mm) */
#define B_CHASSIS_ENTRY_3_Y 1800.0f           /* 蓝区 入口位置3 Y (mm) */
#define B_CHASSIS_ENTRY_3_W 0.0f              /* 蓝区 入口位置3 W (deg) */

// 出梅林
#define B_CHASSIS_EXIT_1_X 8650.0f            /* 蓝区 出口位置1 X (mm) */
#define B_CHASSIS_EXIT_1_Y 1800.0f            /* 蓝区 出口位置1 Y (mm) */
#define B_CHASSIS_EXIT_1_W 0.0f               /* 蓝区 出口位置1 W (deg) */
#define B_CHASSIS_EXIT_2_X 8650.0f            /* 蓝区 出口位置2 X (mm) */
#define B_CHASSIS_EXIT_2_Y 4200.0f            /* 蓝区 出口位置2 Y (mm) */
#define B_CHASSIS_EXIT_2_W 0.0f               /* 蓝区 出口位置2 W (deg) */

// 梅林进入对抗区
#define B_CHASSIS_SLOPE_START_X 8650.0f       /* 蓝区 坡道起点 X (mm) */
#define B_CHASSIS_SLOPE_START_Y 5500.0f       /* 蓝区 坡道起点 Y (mm) */
#define B_CHASSIS_SLOPE_START_W 0.0f          /* 蓝区 坡道起点 W (deg) */
#define B_CHASSIS_SLOPE_END_X 11500.0f        /* 蓝区 坡道终点 X (mm) */
#define B_CHASSIS_SLOPE_END_Y 5500.0f         /* 蓝区 坡道终点 Y (mm) */
#define B_CHASSIS_SLOPE_END_W 0.0f            /* 蓝区 坡道终点 W (deg) */

// 对抗区路径点位
#define B_CHASSIS_CZ_INIT_X 11500.0f          /* 蓝区 重试区 X (mm) */
#define B_CHASSIS_CZ_INIT_Y 5500.0f           /* 蓝区 重试区 Y (mm) */
#define B_CHASSIS_CZ_INIT_W -90.0f             /* 蓝区 重试区 W (deg) */#define B_CHASSIS_CZ_CENTER_X 10750.0f        /* 蓝区 对抗区中央 X (mm) */
#define B_CHASSIS_CZ_CENTER_Y 3000.0f         /* 蓝区 对抗区中央 Y (mm) */
#define B_CHASSIS_CZ_CENTER_W -90.0f          /* 蓝区 对抗区中央 W (deg) */
#define B_CHASSIS_SUDUKU_1_X 11290.0f         /* 蓝区 九宫格第一列 X (mm) */
#define B_CHASSIS_SUDUKU_1_Y 600.0f           /* 蓝区 九宫格第一列 Y (mm) */
#define B_CHASSIS_SUDUKU_1_W -90.0f            /* 蓝区 九宫格第一列 W (deg) */
#define B_CHASSIS_SUDUKU_2_X 10750.0f         /* 蓝区 九宫格第二列 X (mm) */
#define B_CHASSIS_SUDUKU_2_Y 600.0f           /* 蓝区 九宫格第二列 Y (mm) */
#define B_CHASSIS_SUDUKU_2_W -90.0f            /* 蓝区 九宫格第二列 W (deg) */
#define B_CHASSIS_SUDUKU_3_X 10210.0f         /* 蓝区 九宫格第三列 X (mm) */
#define B_CHASSIS_SUDUKU_3_Y 600.0f           /* 蓝区 九宫格第三列 Y (mm) */
#define B_CHASSIS_SUDUKU_3_W -90.0f            /* 蓝区 九宫格第三列 W (deg) */

/* ============================== 底盘路径预设可配置参数（按需微调） ============================== */






/* ================================================ MC 规划行为树接口 ================================================ */
/* ================================================ MC 规划行为树接口 ================================================ */
/* ================================================ MC 规划行为树接口 ================================================ */


#define CHASSIS_ROUTE_TIMEOUT_MS 30000 /*  底盘路径超时 (ms) */


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
    CLAMP_INIT,      //夹爪位置初始化
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
    CAMERA_SCAN,   //二区扫描
    MF_TASK_FLAG  //发送二区任务请求
} R2_MC_MISSION_STATE;    

typedef struct
{
    uint8_t req_pick;    /* 是否请求执行拾取动作的标志位 */
    uint8_t retry_count; /* 已经执行的重试次数 */
    uint8_t seq_mem_idx; 
    uint8_t seq_mem_idx_combined; 
    uint8_t R2_MC_timer_ms ; //以 BT tick (10ms为周期) 计数，或在 Tick1ms 中累加 
    uint16_t RACK_POS_STABLE_TIMES; // 矛架位置稳定的计数器，达到一定次数后认为矛架位置稳定，准备执行夹取
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
extern MC_Blackboard MC_bb;









/* ================================================ MF 规划行为树接口 ================================================ */
/* ================================================ MF 规划行为树接口 ================================================ */
/* ================================================ MF 规划行为树接口 ================================================ */

typedef struct
{
    uint8_t kfs_stored;     /* 存储KFS的数量 */
}MF_Blackboard;



/* ---------- MF 专用参数宏（需实测调整） ---------- */

#define R2_KFS_ALL_NEED 3 /* 需要拾取的 R2KFS 总数量（根据实际情况调整，并非所有的R2KFS） */

/* MF 区原点在世界坐标系中的位置 (mm) — row0,col0 的坐标 */
#define MF_ORIGIN_WORLD_X_BLUE  2600.0f   /* row0,col0 的中心在蓝地图世界坐标系中的x位置 */
#define MF_ORIGIN_WORLD_Y_BLUE  4200.0f   /* row0,col0 的中心在蓝地图世界坐标系中的y位置 */
#define MF_ORIGIN_WORLD_X_RED   2600.0f   /* row0,col0 的中心在红地图世界坐标系中的x位置 */
#define MF_ORIGIN_WORLD_Y_RED   (-1800.0f)   /* row0,col0 的中心在红地图世界坐标系中的y位置 */

/* 每个树林方块的边长 (mm) */
#define MF_CELL_SIZE_MM     1200.0f  /* 每个栅格的实际大小 */

/* 🟡【需实测】各方向对应的底盘朝向角 (度) */
#define MF_YAW_NORTH        0.0f     /* 北向的底盘 yaw 角 */
#define MF_YAW_EAST         -90.0f   /* 东向的底盘 yaw 角 */
#define MF_YAW_SOUTH        180.0f   /* 南向的底盘 yaw 角 */
#define MF_YAW_WEST         90.0f    /* 西向的底盘 yaw 角 */

/* 🟡【需实测】台阶升降的动作耗时 (ms) */
#define MF_STEP_TIMEOUT_MS  10000    /* 🟡 上/下台阶超时时间 */

/* 🟡【需实测】拾取动作的超时时间 (ms) */
#define MF_PICK_TIMEOUT_MS  5000     /* 🟡 拾取超时，需根据机械臂实际速度调整 */

/* 🟡【需实测】等待 R1 的超时时间 (ms) */
#define MF_WAIT_R1_TIMEOUT_MS 10000 /* 🟡 等待 R1 清理的超时，超时后强行跳过 */

/* 🟡【需实测】机械臂存储服务超时时间 */
#define ARM_STORE_TRANS_TIMEOUT_MS 5000 /* 工作区切换超时时间 */
#define ARM_STORE_PATH_TIMEOUT_MS 8000  /* 后向存储路径超时时间 */
#define ARM_STORE_REST_TIMEOUT_MS 5000  /* 返回前向待机点超时时间 */

/* ============================== 🟡（待定）机械臂预设拾取路径点 ============================== */

/**
 * @brief 机械臂存储服务状态
*/
typedef enum {
    ARM_STORE_IDLE = 0,

    ARM_STORE_REQ_F2B,
    ARM_STORE_WAIT_BACK,

    ARM_STORE_SEND_PATH,
    ARM_STORE_WAIT_DONE,

    ARM_STORE_REQ_B2F,
    ARM_STORE_WAIT_FRONT,

    ARM_STORE_SEND_REST,
    ARM_STORE_WAIT_REST,

    ARM_STORE_DONE,
    ARM_STORE_FAILED
} ArmStoreState;

extern ArmStoreState arm_store_state;
extern uint8_t arm_store_slot;

#define PICK_REST_X 0.35f             /* 机械臂待机位置 X */
#define PICK_REST_Y 0.35f             /* 机械臂待机位置 Y */

#define PICK_HANDS_UP_X 0.15f       /* 演示用抬手坐标 */
#define PICK_HANDS_UP_Y 0.9f       /* 演示用抬手坐标 */

#define PICK_N200_APR_X_1 0.568f          /* ！！！！待测试！！！！-200预抓取点1 X */
#define PICK_N200_APR_Y_1 0.065f          /* ！！！！待测试！！！！！-200预抓取点1 Y */
// #define PICK_N200_APR_X_2 0.5f          /* ！！！！待测试！！！！！-200预抓取点2 X */
// #define PICK_N200_APR_Y_2 0.5f          /* ！！！！待测试！！！！！-200预抓取点2 Y */
#define PICK_N200_TARGET_X 0.618f         /* ！！！！待测试！！！！！-200抓取点 X */
#define PICK_N200_TARGET_Y 0.065f         /* ！！！！待测试！！！！！-200抓取点 Y */

#define PICK_0_APR_X_1 0.568f             /* 0预抓取点1 X */
#define PICK_0_APR_Y_1 (-0.135f)          /* 0预抓取点1 Y */
// #define PICK_0_APR_X_2 0.5f             /* 0预抓取点2 X */
// #define PICK_0_APR_Y_2 0.5f             /* 0预抓取点2 Y */
#define PICK_0_TARGET_X 0.618f            /* 0抓取点 X */
#define PICK_0_TARGET_Y (-0.135f)            /* 0抓取点 Y */

#define PICK_P200_APR_X_1 0.568f     /* +200预抓取点1 X */
#define PICK_P200_APR_Y_1 0.065f     /* +200预抓取点1 Y */
// #define PICK_P200_APR_X_2 0.5f     /* +200预抓取点2 X */
// #define PICK_P200_APR_Y_2 0.5f     /* +200预抓取点2 Y */
#define PICK_P200_TARGET_X 0.618f         /* +200抓取点 X */
#define PICK_P200_TARGET_Y 0.065f         /* +200抓取点 Y */

#define PICK_P400_APR_X_1 0.568f     /* +400预抓取点1 X */
#define PICK_P400_APR_Y_1 0.265f     /* +400预抓取点1 Y */
// #define PICK_P400_APR_X_2 0.5f     /* +400预抓取点2 X */
// #define PICK_P400_APR_Y_2 0.5f     /* +400预抓取点2 Y */
#define PICK_P400_TARGET_X 0.618f         /* +400抓取点 X */
#define PICK_P400_TARGET_Y 0.265f         /* +400抓取点 Y */

#define PICK_PRE_TRANS_X 0.4f         /* 抓取完毕之后的过渡准备点 X */
#define PICK_PRE_TRANS_Y 0.6f         /* 抓取完毕之后的过渡准备点 Y */
#define PICK_TIME 1000       /* 抓取点停留时间 (ms) */

#define STORE_A_PRE_X -0.4f         /* 存储点a过渡点 X */
#define STORE_A_PRE_Y 0.6f         /* 存储点a过渡点 Y */
#define STORE_A_X -0.4f         /* 存储点a X */
#define STORE_A_Y 0.4f         /* 存储点a Y */

#define STORE_B_PRE_X -0.005f         /* 存储点b过渡点 X */
#define STORE_B_PRE_Y 0.45f         /* 存储点b过渡点 Y */
#define STORE_B_X -0.005f        /* 存储点b X */
#define STORE_B_Y 0.29f         /* 存储点b Y */

#define STORE_C_X -0.4f         /* ！！！ 待定 存储点c X */
#define STORE_C_Y 0.7f         /* ！！！ 待定存储点c Y */

#define STORE_PRE_TRANS_X -0.2f         /* 存储完毕之后的过渡准备点 X */
#define STORE_PRE_TRANS_Y 0.7f         /* 存储完毕之后的过渡准备点 Y */
#define STORE_TIME 1000       /* 存储点停留时间 (ms) */

/* ============================== 🟡（待定）机械臂预设拾取路径点 ============================== */




/* ---------- MF 执行状态机 ---------- */
typedef enum {
    MF_EXEC_IDLE = 0,
    MF_EXEC_MOVE_FLAT,
    MF_EXEC_STEP_UP,
    MF_EXEC_STEP_DOWN,
    MF_EXEC_PICK_R2,
    MF_EXEC_WAIT_R1,
    MF_EXEC_DONE,
    MF_EXEC_FAILED
} MF_ExecState;

/* ---------- MF 行为树接口 ---------- */
void BT_MF_Start_Task(void);
void BT_MF_Init_Task(void);
void BT_Customized_Tick1ms_MF(void);

/* ================================================ CZ 对抗区行为树接口 ================================================ */

#define CZ_POS_STABLE_CONDITION 50  /* 扫描位置稳定次数阈值，类似MC取矛头 */

/* 对抗区任务状态机 */
typedef enum
{
    CZ_IDLE = 0,
    CZ_NAV_TO_SLOPE_START,
    CZ_NAV_TO_SLOPE_END,
    CZ_NAV_TO_SCAN,
    CZ_CAMERA_SCAN,
    CZ_NAV_TO_SUDUKU,
    CZ_RETURN_TO_SCAN,
    CZ_DONE
} R2_CZ_MISSION_STATE;

/* 对抗区黑板 */
typedef struct
{
    uint8_t req_start;
    R2_CZ_MISSION_STATE cz_mission_state;
    uint8_t seq_mem_idx;
    uint32_t cz_timer_ms;
    uint8_t camera_scan_done;
    uint8_t scan_count;           /* 扫描次数: 0=第一次, 1=第二次 */
    uint16_t pos_stable_times;    /* 扫描位置稳定计数 */
} CZ_Blackboard;

extern CZ_Blackboard CZ_bb;
void BT_CZ_Start_Task(void);
void BT_CZ_Init_Task(void);
void BT_Customized_Tick1ms_CZ(void);









/* ================================================ MASTER 行为树 ================================================ */
/* ================================================ MASTER 行为树 ================================================ */
/* ================================================ MASTER 行为树 ================================================ */



void BT_MASTER_Start_Task(void);

void BT_MASTER_Init_Task(void);

void BT_MASTER_Tick(void);


#endif // __BT_CUSTOMIZED_
