#ifndef __MF_PLANNER_H
#define __MF_PLANNER_H

#include <stdint.h>
#include <stdbool.h>

/*
 * ================================================================
 * MF 区 R2 规划器接口说明
 * ------------------------------------------------
 * 1) MF 区抽象为 6x3 栅格，行从下到上 0~5，列从左到右 0~2。
 * 2) 仅在“走格子”层面做 A* 规划，不考虑具体动作代价。
 * 3) 规划输出为“动作列表”，包含平移、上/下台阶、取方块、等待。
 * 4) R2KFS 只能在相邻格面对拾取；R1KFS 可能需要等待清理；FAKEKFS 永不触碰。
 * 5) 生成动作列表后，执行端按顺序取动作执行即可，不要求重规划。
 * ================================================================
 */

/* 栅格尺寸定义：6x3，行从下到上，列从左到右 */
#define MF_ROWS 6
#define MF_COLS 3
#define MF_MAX_NODES (MF_ROWS * MF_COLS) /* A* 节点总数 */
/* 动作列表最大长度，可按需求调整(越大越占 RAM) */
#define MF_MAX_ACTIONS 64

typedef enum {
	MF_CELL_EMPTY = 0,
	MF_CELL_R1KFS,   /* R1KFS：可等待清除，不可登上 */
	MF_CELL_R2KFS,   /* R2KFS：不可登上，可相邻拾取 */
	MF_CELL_FAKEKFS  /* FAKEKFS：绝对禁止接触与登上 */
} MF_CellType;

typedef enum {
	MF_DIR_NORTH = 0,
	MF_DIR_EAST,  /* +列方向 */
	MF_DIR_SOUTH, /* -行方向 */
	MF_DIR_WEST   /* -列方向 */
} MF_Dir;

typedef enum {
	MF_ACT_MOVE_FLAT = 0, /* 平移(同高) */
	MF_ACT_STEP_UP,       /* 上台阶(+200) */
	MF_ACT_STEP_DOWN,     /* 下台阶(-200) */
	MF_ACT_PICK_R2,       /* 面对相邻 R2KFS 拾取 */
	MF_ACT_WAIT_R1        /* 等待 R1 清除 R1KFS */
} MF_ActionType;

typedef struct {
	MF_ActionType type;  /* 动作类别 */
	MF_Dir dir;          /* 动作方向(移动/取方块/等待均可用) */
	uint8_t row;         /* 目标格行(取方块/等待时使用) */
	uint8_t col;         /* 目标格列(取方块/等待时使用) */
	int16_t height_pick_diff; /* 取方块时的高度差(相对于当前格) */
} MF_Action;

typedef struct {
	uint16_t height[MF_ROWS][MF_COLS]; /* 高度矩阵(mm) */
	MF_CellType cell[MF_ROWS][MF_COLS]; /* KFS/空格状态 */
	MF_Action actions[MF_MAX_ACTIONS]; /* 规划输出动作列表 */
	uint16_t action_count;             /* 动作总数 */
	uint16_t action_index;             /* 当前执行到的动作索引 */
} MF_Planner;


/* 新增：把 AStarNode 暴露到头文件以让 AStarState 使用 */
/* A* 节点数据结构 */
typedef struct {
    int16_t g;              /* 起点到当前的代价 */
    int16_t f;              /* f = g + h */
    int8_t parent;          /* 回溯父节点索引 */
    uint8_t in_open;    /* 是否在开放集 */
    uint8_t in_closed;  /* 是否在关闭集 */
} AStarNode;


extern MF_Planner mf_planner_ctx;




void MFPlanner_Init(MF_Planner* ctx);

void MFPlanner_LoadFixedDemoMap(MF_Planner* ctx);

bool MFPlanner_BuildFullPlan(MF_Planner* ctx,
							 uint8_t start_row,
							 uint8_t start_col,
							 uint8_t r2kfs_need);

bool MFPlanner_PeekAction(const MF_Planner* ctx, MF_Action* out_action);

bool MFPlanner_AdvanceAction(MF_Planner* ctx);

void MFPlanner_NotifyR1KfsRemoved(MF_Planner* ctx, uint8_t row, uint8_t col);

const char* MFPlanner_ActionTypeName(MF_ActionType type);

const char* MFPlanner_DirName(MF_Dir dir);

uint16_t MFPlanner_DumpPlan(const MF_Planner* ctx, char* out_buf, uint16_t buf_len);

#endif // __MF_PLANNER_H
