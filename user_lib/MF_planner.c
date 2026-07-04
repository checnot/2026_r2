#include "MF_planner.h"
#include <string.h>
#include <stdio.h>

/*
 * ================================================================
 * R2 在 MF 区的“决策-路径规划-动作编排”一体化示例实现
 * ------------------------------------------------
 * 核心思想：
 * 1) MF 区被抽象为 6x3 栅格(行从下到上 0~5，列从左到右 0~2)。
 * 2) 每个格子带高度(0/200/400/600mm)，移动约束为相邻格且高度差为 0 或 +/-200。
 * 3) A* 在“走格子”层面算代价：每步代价=1，若目标格为 R1KFS 则额外加 MF_WAIT_R1_COST，
 *    使规划自动倾向于穿越次数更少的路径，仅在无法避开时才接受穿越。
 * 4) 规划结果不是路径点列表，而是包含“上台阶/下台阶/平移/取方块/等待”的动作序列。
 * 5) R2KFS 视为“轻度障碍”：不能登上，但可到相邻格面对并拾取，拾取后变通路。
 * 6) R1KFS 恒定可穿越，但穿越代价高：能避则避，若绕路代价更高或完全阻断，
 *    则接受穿过，自动插入“等待R1取走”动作。
 * 7) FAKEKFS 永久不可接触与不可登上。
 * 8) 规划一次生成全程动作列表；执行阶段若遇等待，R1移除后继续，不重新规划。
 *
 * 注意：
 * - 这里的地图与 KFS 分布为“固定演示矩阵”，用于替代一次视觉识别的结果。
 * - 具体与真实地图融合时，可把 MFPlanner_LoadFixedDemoMap 替换为视觉模块输入。
 * ================================================================
 */

/* A* 节点总数 = 网格总格子数 */
#define MF_MAX_NODES (MF_ROWS * MF_COLS)

/* 方向增量：北1、东1、南-1、西-1 */
static const int8_t k_dir_dr[4] = { 1, 0, -1, 0 };
static const int8_t k_dir_dc[4] = { 0, 1, 0, -1 };

/* 穿越一次 R1KFS 相对于普通一步的额外代价：总代价 = 步数 + MF_WAIT_R1_COST * 穿越次数 */
#define MF_WAIT_R1_COST 20

/* 移开一次 R2KFS 相对于普通一步的额外代价：总代价 = 1 + MF_MOVE_R2_COST */
#define MF_MOVE_R2_COST 5

MF_Planner mf_planner_ctx;

/**
 * @brief 判断坐标是否在 MF 区边界内
 * @param r 行(允许为负以便边界判断)
 * @param c 列(允许为负以便边界判断)
 * @return true 表示在边界内
 * @note 用 int8_t 是为了和方向增量的加减直接兼容
 */
static bool MF_InBounds(int8_t r, int8_t c)
{
    /* 行列同时满足边界才可用 */
    return (r >= 0 && r < MF_ROWS && c >= 0 && c < MF_COLS);
}

/**
 * @brief (r,c) 转一维索引，便于 A* 数组访问
 * @param r 行
 * @param c 列
 * @return 线性索引
 * @note 线性展开顺序：row-major
 */
static uint8_t MF_ToIndex(uint8_t r, uint8_t c)
{
    /* row-major 线性化：index = r*COLS + c */
    return (uint8_t)(r * MF_COLS + c);
}

/**
 * @brief 一维索引转 (r,c)
 * @param idx 线性索引
 * @param r 输出行指针
 * @param c 输出列指针
 */
static void MF_FromIndex(uint8_t idx, uint8_t* r, uint8_t* c)
{
    /* 逆向线性化得到行列 */
    *r = (uint8_t)(idx / MF_COLS);
    *c = (uint8_t)(idx % MF_COLS);
}

/**
 * @brief 曼哈顿距离(4邻域)作为 A* 启发式
 * @param r1 起点行
 * @param c1 起点列
 * @param r2 目标行
 * @param c2 目标列
 * @return 曼哈顿距离
 * @note 仅估算横纵格子数，不含高度差
 */
static int16_t MF_Manhattan(uint8_t r1, uint8_t c1, uint8_t r2, uint8_t c2)
{
    /* 计算行差 */
    int16_t dr = (int16_t)r1 - (int16_t)r2;
    /* 计算列差 */
    int16_t dc = (int16_t)c1 - (int16_t)c2;
    /* 取绝对值 */
    if (dr < 0) dr = -dr;
    if (dc < 0) dc = -dc;
    /* 曼哈顿距离 = |dr| + |dc| */
    return (int16_t)(dr + dc);
}

/**
 * @brief 判断格子是否可“登上”
 * @param ctx 规划器上下文
 * @param r 目标行
 * @param c 目标列
 * @param allow_r2kfs_cross 是否允许把 R2KFS 当作可移开的“轻度障碍”穿越；
 *        false 表示 R2KFS 视为不可登上的硬障碍(用于禁止最后一个 R2KFS 拾取完成后
 *        再规划 MOVE_R2 动作的场景，例如走向出口的路径搜索)
 * @return true 表示不可登上
 * @note R1KFS 恒定可“穿越”，但穿越代价高，由 MF_AStar 体现；FAKEKFS 永远不可登上
 * @note R2KFS 仅当 allow_r2kfs_cross 为 true 时才可“穿越”(移开)，
 *       若规划选择直接穿越则由 MF_AddPathActions 插入“移开”动作
 */
static bool MF_IsBlockedForMove(const MF_Planner* ctx, uint8_t r, uint8_t c, bool allow_r2kfs_cross)
{
    /* 读取目标格子类型 */
    MF_CellType t = ctx->cell[r][c];
    if (t == MF_CELL_EMPTY) {
        /* 空格可直接登上 */
        return false;
    }
    if (t == MF_CELL_UNKNOWN) {
        /* 未知格：尚未扫描，视为障碍，禁止登上 */
        return true;
    }
    if (t == MF_CELL_R1KFS) {
        /* R1KFS 恒定可穿越，穿越代价由 MF_AStar 中的 MF_WAIT_R1_COST 体现 */
        return false;
    }
    if (t == MF_CELL_R2KFS) {
        /* R2KFS 是否可“移开后穿越”取决于调用方是否仍允许 MOVE_R2 动作 */
        return !allow_r2kfs_cross;
    }
    /* FAKEKFS 永远不可登上 */
    return true;
}

/**
 * @brief 高度差约束：只允许 0 / +200 / -200
 * @param ctx 规划器上下文
 * @param r1 起点行
 * @param c1 起点列
 * @param r2 目标行
 * @param c2 目标列
 * @return true 表示允许通行
 * @note 上台阶必须面对台阶，下台阶必须背对台阶，该约束在执行层处理
 */
static bool MF_IsStepAllowed(const MF_Planner* ctx,
                             uint8_t r1, uint8_t c1,
                             uint8_t r2, uint8_t c2)
{
    /* 读取起点与终点高度 */
    int16_t h1 = (int16_t)ctx->height[r1][c1];
    int16_t h2 = (int16_t)ctx->height[r2][c2];
    /* 高度差 = 目标 - 当前 */
    int16_t diff = h2 - h1;
    /* 只允许 0 或 +/-200 的高差 */
    return (diff == 0 || diff == 200 || diff == -200);
}


/*============================================ ASTAR部分 ============================================*/

/**
 * @brief MF区域 A* 节点数组，大小为最大格子数
*/
AStarNode ASTAR_nodes[MF_MAX_NODES];

/**
 * @brief A* 路径规划(4邻域)
 * @param ctx 规划器上下文
 * @param sr 起点行
 * @param sc 起点列
 * @param gr 目标行
 * @param gc 目标列
 * @param allow_r2kfs_cross 是否允许把 R2KFS 当作可穿越(移开)的轻度障碍；
 *        false 表示 R2KFS 视为硬障碍，规划出的路径不会再产生 MOVE_R2 动作
 * @param out_path 输出路径(索引数组)
 * @param out_len 输出路径长度
 * @param out_cost 输出路径总代价(可为 NULL)，用于候选路径之间的比较
 * @return true 表示找到路径
 * @note 每步代价=1，若目标格为 R1KFS 则额外加 MF_WAIT_R1_COST；返回索引路径(包含起点与终点)
 * @note R1KFS 恒定可穿越，穿越次数越多、绕路带来的代价差越大，越容易被 A* 放弃
 */
static bool MF_AStar(const MF_Planner* ctx,
                     uint8_t sr, uint8_t sc,
                     uint8_t gr, uint8_t gc,
                     bool allow_r2kfs_cross,
                     uint8_t* out_path,
                     uint8_t* out_len,
                     int16_t* out_cost)
{
    for (uint8_t i = 0; i < MF_MAX_NODES; i++) {
        /* 初始化代价为极大值，表示尚未访问 */
        ASTAR_nodes[i].g = 0x3FFF;
        ASTAR_nodes[i].f = 0x3FFF;
        /* 父节点未知 */
        ASTAR_nodes[i].parent = -1;
        /* 集合标记清零 */
        ASTAR_nodes[i].in_open = 0;
        ASTAR_nodes[i].in_closed = 0;
    }

    /* 起点/终点转为索引 */
    uint8_t start_index = MF_ToIndex(sr, sc);
    uint8_t goal_index = MF_ToIndex(gr, gc);
    /* 起点 g=0，f=h */
    ASTAR_nodes[start_index].g = 0;
    ASTAR_nodes[start_index].f = MF_Manhattan(sr, sc, gr, gc);
    /* 起点加入开放集 */
    ASTAR_nodes[start_index].in_open = 1;

    while (1) {
        /* 在开放集中选择 f 最小的节点 */
        int16_t best_f = 0x3FFF;
        int16_t best_g = 0x3FFF;
        int8_t current = -1;
        for (uint8_t i = 0; i < MF_MAX_NODES; i++) {
            if (!ASTAR_nodes[i].in_open) {
                continue;
            }
            if (ASTAR_nodes[i].f < best_f || (ASTAR_nodes[i].f == best_f && ASTAR_nodes[i].g < best_g)) {
                best_f = ASTAR_nodes[i].f;
                best_g = ASTAR_nodes[i].g;
                current = (int8_t)i;
            }
        }

        if (current < 0) {
            /* 开放集为空 -> 无路可走 */
            return false;
        }

    // 找到终点，开始回溯路径
        if ((uint8_t)current == goal_index) {
            uint8_t path[MF_MAX_NODES];
            uint8_t count = 0;
            int8_t idx = current;
            while (idx >= 0 && count < MF_MAX_NODES) {
                /* 反向记录节点索引 */
                path[count++] = (uint8_t)idx;
                idx = ASTAR_nodes[idx].parent;
            }
            if (count == 0) {
                return false;
            }
            /* 反转为起点->终点 */
            for (uint8_t i = 0; i < count; i++) {
                out_path[i] = path[count - 1 - i];
            }
            *out_len = count;
            if (out_cost != NULL) {
                *out_cost = ASTAR_nodes[goal_index].g;
            }
            return true;
        }

        /* 当前节点加入关闭集 */
        ASTAR_nodes[current].in_open = 0;
        ASTAR_nodes[current].in_closed = 1;

        /* 取出当前节点的行列坐标 */
        uint8_t cr, cc;
        MF_FromIndex((uint8_t)current, &cr, &cc);
        for (uint8_t d = 0; d < 4; d++) {
            /* 4 邻域扩展 */
            int8_t nr = (int8_t)cr + k_dir_dr[d];
            int8_t nc = (int8_t)cc + k_dir_dc[d];
            if (!MF_InBounds(nr, nc)) {
                continue;
            }
            /* 强转为无符号便于索引 */
            uint8_t ur = (uint8_t)nr;
            uint8_t uc = (uint8_t)nc;
            if (MF_IsBlockedForMove(ctx, ur, uc, allow_r2kfs_cross)) {
                continue;
            }
            if (! MF_IsStepAllowed(ctx, cr, cc, ur, uc)) {
                continue;
            }
            /* 目标格索引 */
            uint8_t nidx = MF_ToIndex(ur, uc);
            if (ASTAR_nodes[nidx].in_closed) {
                /* 已在关闭集则跳过 */
                continue;
            }
            /* 新路径代价 = 当前代价 + 单步代价(R1KFS 额外加 MF_WAIT_R1_COST，R2KFS 额外加 MF_MOVE_R2_COST) */
            int16_t step_cost = 1;
            if (ctx->cell[ur][uc] == MF_CELL_R1KFS) {
                step_cost = (int16_t)(1 + MF_WAIT_R1_COST);
            } else if (ctx->cell[ur][uc] == MF_CELL_R2KFS) {
                step_cost = (int16_t)(1 + MF_MOVE_R2_COST);
            }
            int16_t tentative_g = (int16_t)(ASTAR_nodes[current].g + step_cost);
            if (!ASTAR_nodes[nidx].in_open || tentative_g < ASTAR_nodes[nidx].g) {
                /* 记录更优父节点与代价 */
                ASTAR_nodes[nidx].parent = current;
                ASTAR_nodes[nidx].g = tentative_g;
                ASTAR_nodes[nidx].f = (int16_t)(tentative_g + MF_Manhattan(ur, uc, gr, gc));
                ASTAR_nodes[nidx].in_open = 1;
            }
        }
    }
}

/*============================================ ASTAR部分 ============================================*/


/**
 * @brief 向动作列表尾部追加一个动作
 * @param ctx 规划器上下文
 * @param type 动作类型
 * @param dir 动作方向
 * @param row 目标行(取方块/等待用)
 * @param col 目标列(取方块/等待用)
 * @return true 表示追加成功
 * @note 超出容量直接返回 false
 */
bool MF_PushAction(MF_Planner* ctx, MF_ActionType type, MF_Dir dir, uint8_t row, uint8_t col)
{
    if (ctx->action_count >= MF_MAX_ACTIONS) {
        /* 动作列表溢出 */
        return false;
    }
    /* 写入动作结构体字段 */
    ctx->actions[ctx->action_count].type = type;
    ctx->actions[ctx->action_count].dir = dir;
    ctx->actions[ctx->action_count].row = row;
    ctx->actions[ctx->action_count].col = col;
    ctx->actions[ctx->action_count].height_pick_diff = 0;
    /* 递增动作数量 */
    ctx->action_count++;
    return true;
}

/**
 * @brief (dr,dc) 转方向枚举
 * @param dr 行方向增量(-1/0/1)
 * @param dc 列方向增量(-1/0/1)
 * @return 对应方向枚举
 * @note dr/dc 必须是单位步，否则结果不确定
 */
static MF_Dir MF_DirFromDelta(int8_t dr, int8_t dc)
{
    if (dr == 1 && dc == 0) return MF_DIR_NORTH;
    if (dr == 0 && dc == 1) return MF_DIR_EAST;
    if (dr == -1 && dc == 0) return MF_DIR_SOUTH;
    return MF_DIR_WEST;
}

/**
 * @brief 将路径点转为动作序列(平移/上台阶/下台阶/等待)
 * @param ctx 规划器上下文
 * @param path 路径索引数组
 * @param path_len 路径长度
 * @return true 表示转换成功
 * @note 该函数不生成“取方块”动作，只处理移动/等待
 */
static bool MF_AddPathActions(MF_Planner* ctx,
                              const uint8_t* path,
                              uint8_t path_len)
{
    for (uint8_t i = 1; i < path_len; i++) {
        /* 取相邻两个路径点 */
        uint8_t r1, c1, r2, c2;
        MF_FromIndex(path[i - 1], &r1, &c1);
        MF_FromIndex(path[i], &r2, &c2);

        /* 计算相对方向 */
        int8_t dr = (int8_t)r2 - (int8_t)r1;
        int8_t dc = (int8_t)c2 - (int8_t)c1;
        MF_Dir dir = MF_DirFromDelta(dr, dc);

        /* 遇到 R1KFS：先等 R1 清理，再进入该格 */
        if (ctx->cell[r2][c2] == MF_CELL_R1KFS) {
            if (!MF_PushAction(ctx, MF_ACT_WAIT_R1, dir, r2, c2)) {
                return false;
            }
            /* 规划阶段假定 R1 会清除该 KFS，后续路径可直接通过 */
            ctx->cell[r2][c2] = MF_CELL_EMPTY;
        }

        /* 遇到 R2KFS：路径选择直接穿越而非绕路/拾取，先移开(丢弃)再进入该格 */
        if (ctx->cell[r2][c2] == MF_CELL_R2KFS) {
            if (!MF_PushAction(ctx, MF_ACT_MOVE_R2, dir, r2, c2)) {
                return false;
            }
            ctx->actions[ctx->action_count - 1].height_pick_diff =
                (int16_t)ctx->height[r2][c2] - (int16_t)ctx->height[r1][c1];
            /* 移开后该格变通路 */
            ctx->cell[r2][c2] = MF_CELL_EMPTY;
        }

        /* 高度差决定平移/上/下 */
        int16_t h1 = (int16_t)ctx->height[r1][c1];
        int16_t h2 = (int16_t)ctx->height[r2][c2];
        int16_t diff = h2 - h1;
        if (diff == 0) {
            if (!MF_PushAction(ctx, MF_ACT_MOVE_FLAT, dir, r2, c2)) {
                return false;
            }
        } else if (diff == 200) {
            /* 上台阶：控制层应确保“面对台阶方向上” */
            if (!MF_PushAction(ctx, MF_ACT_STEP_UP, dir, r2, c2)) {
                return false;
            }
        } else if (diff == -200) {
            /* 下台阶：控制层应确保“背对台阶方向下” */
            if (!MF_PushAction(ctx, MF_ACT_STEP_DOWN, dir, r2, c2)) {
                return false;
            }
        } else {
            /* 超过 200 高差，不可通行 */
            return false;
        }
    }
    return true;
}

/**
 * @brief 计算到“某个目标格相邻格”的最短路径
 * @param ctx 规划器上下文
 * @param sr 起点行
 * @param sc 起点列
 * @param target_r 目标格行
 * @param target_c 目标格列
 * @param allow_r2kfs_cross 是否允许把沿途 R2KFS 当作可穿越(移开)的轻度障碍
 * @param out_path 输出路径索引数组
 * @param out_len 输出路径长度
 * @param out_adj_r 输出相邻格行
 * @param out_adj_c 输出相邻格列
 * @param out_cost 输出路径总代价(可为 NULL)
 * @return true 表示找到路径
 * @note R2 必须相邻面对才能取 R2KFS，因此这里找目标格的最佳相邻格
 * @note 返回的路径终点就是“可站立的相邻格”；候选相邻格按总代价(而非步数)比较
 */
static bool MF_FindPathToAdjacent(const MF_Planner* ctx,
                                  uint8_t sr, uint8_t sc,
                                  uint8_t target_r, uint8_t target_c,
                                  bool allow_r2kfs_cross,
                                  uint8_t* out_path,
                                  uint8_t* out_len,
                                  uint8_t* out_adj_r,
                                  uint8_t* out_adj_c,
                                  int16_t* out_cost)
{
    bool found = false;
    uint8_t best_path[MF_MAX_NODES];
    uint8_t best_len = 0;
    uint8_t best_r = 0;
    uint8_t best_c = 0;
    int16_t best_cost = 0x3FFF;

    for (uint8_t d = 0; d < 4; d++) {
        /* 枚举目标格四个相邻格 */
        int8_t ar = (int8_t)target_r + k_dir_dr[d];
        int8_t ac = (int8_t)target_c + k_dir_dc[d];
        if (!MF_InBounds(ar, ac)) {
            continue;
        }
        uint8_t ur = (uint8_t)ar;
        uint8_t uc = (uint8_t)ac;
        if (MF_IsBlockedForMove(ctx, ur, uc, allow_r2kfs_cross)) {
            continue;
        }
        /* 对每个可站立相邻格做 A* */
        uint8_t path[MF_MAX_NODES];
        uint8_t len = 0;
        int16_t cost = 0;
        if (MF_AStar(ctx, sr, sc, ur, uc, allow_r2kfs_cross, path, &len, &cost)) {
            if (!found || cost < best_cost) {
                /* 记录代价更低的路径 */
                found = true;
                best_len = len;
                best_cost = cost;
                memcpy(best_path, path, len);
                best_r = ur;
                best_c = uc;
            }
        }
    }

    if (!found) {
        /* 没有任何相邻格可达 */
        return false;
    }
    /* 输出最优路径与相邻格坐标 */
    memcpy(out_path, best_path, best_len);
    *out_len = best_len;
    *out_adj_r = best_r;
    *out_adj_c = best_c;
    if (out_cost != NULL) {
        *out_cost = best_cost;
    }
    return true;
}

/**
 * @brief Apply the same cell side effects that MF_AddPathActions will produce.
 * @note Used only for planning lookahead: WAIT_R1 and MOVE_R2 both make the
 *       traversed cell become EMPTY for subsequent path searches.
 */
static void MF_ApplyPathCellEffects(MF_Planner* ctx,
                                    const uint8_t* path,
                                    uint8_t path_len)
{
    for (uint8_t i = 1; i < path_len; i++) {
        uint8_t r, c;
        MF_FromIndex(path[i], &r, &c);
        if (ctx->cell[r][c] == MF_CELL_R1KFS ||
            ctx->cell[r][c] == MF_CELL_R2KFS) {
            ctx->cell[r][c] = MF_CELL_EMPTY;
        }
    }
}

/**
 * @brief Find the cheapest legal exit path cost after all required R2KFS are done.
 * @note Exit phase keeps the existing rule: remaining R2KFS are hard obstacles.
 */
static bool MF_FindBestExitCost(const MF_Planner* ctx,
                                uint8_t cur_r,
                                uint8_t cur_c,
                                int16_t* out_cost)
{
    bool found = false;
    int16_t best_cost = 0x3FFF;
    uint8_t path[MF_MAX_NODES];
    uint8_t len = 0;
    int16_t cost = 0;

    if (MF_AStar(ctx, cur_r, cur_c, 5, 0, false, path, &len, &cost)) {
        found = true;
        best_cost = cost;
    }
    if (MF_AStar(ctx, cur_r, cur_c, 5, 2, false, path, &len, &cost)) {
        if (!found || cost < best_cost) {
            found = true;
            best_cost = cost;
        }
    }

    if (!found) {
        return false;
    }
    if (out_cost != NULL) {
        *out_cost = best_cost;
    }
    return true;
}

/**
 * @brief Look ahead over the remaining R2KFS picks and final exit.
 * @note This does not change any movement rule. It only changes candidate
 *       scoring from greedy one-step cost to remaining mission cost.
 */
static bool MF_EvaluateRemainingPlanCost(const MF_Planner* ctx,
                                         uint8_t cur_r,
                                         uint8_t cur_c,
                                         uint8_t remaining_picks,
                                         int16_t* out_cost)
{
    if (remaining_picks == 0) {
        return MF_FindBestExitCost(ctx, cur_r, cur_c, out_cost);
    }

    bool found_any = false;
    int16_t best_total_cost = 0x3FFF;

    for (uint8_t r = 0; r < MF_ROWS; r++) {
        for (uint8_t c = 0; c < MF_COLS; c++) {
            if (ctx->cell[r][c] != MF_CELL_R2KFS) {
                continue;
            }

            uint8_t path[MF_MAX_NODES];
            uint8_t len = 0;
            uint8_t adj_r = 0;
            uint8_t adj_c = 0;
            int16_t path_cost = 0;

            if (!MF_FindPathToAdjacent(ctx, cur_r, cur_c, r, c,
                                       true, path, &len, &adj_r, &adj_c, &path_cost)) {
                continue;
            }

            MF_Planner next_ctx = *ctx;
            MF_ApplyPathCellEffects(&next_ctx, path, len);
            next_ctx.cell[r][c] = MF_CELL_EMPTY;

            int16_t remain_cost = 0;
            if (!MF_EvaluateRemainingPlanCost(&next_ctx, adj_r, adj_c,
                                              (uint8_t)(remaining_picks - 1), &remain_cost)) {
                continue;
            }

            int16_t total_cost = (int16_t)(path_cost + remain_cost);
            if (!found_any || total_cost < best_total_cost) {
                found_any = true;
                best_total_cost = total_cost;
            }
        }
    }

    if (!found_any) {
        return false;
    }
    if (out_cost != NULL) {
        *out_cost = best_total_cost;
    }
    return true;
}

/**
 * @brief 初始化规划器上下文
 * @param ctx 规划器上下文
 * @note 等价于清零，动作列表与地图都会被重置
 */
void MFPlanner_Init(MF_Planner* ctx, MF_MapColor color)
{
    /* 清空规划器上下文 */
    memset(ctx, 0, sizeof(*ctx));
    /* 设置地图颜色 */
    ctx->map_color = color;
}

/**
 * @brief 载入不同颜色的固定地图与高度
 * @param ctx 规划器上下文
 * @note 行号定义：row0 是入口区地面，row5 是出口区地面
 */
void MFPlanner_LoadFixedDemoMap(MF_Planner* ctx)
{
    /* 高度矩阵：row0 为入口区地面，row5 为出口区地面 */
    uint16_t fixed_height_blue[MF_ROWS][MF_COLS] = {
        { 0,   0,   0   }, /* row0: 入口区地面 */
        { 400, 200, 400 }, /* row1: 最下方绿色方块 */
        { 200, 400, 600 }, /* row2 */
        { 400, 600, 400 }, /* row3 */
        { 200, 400, 200 }, /* row4: 最上方绿色方块 */
        { 0,   0,   0   }  /* row5: 出口区地面 */
    };
    uint16_t fixed_height_red[MF_ROWS][MF_COLS] = {
        { 0,   0,   0   }, /* row0: 入口区地面 */
        { 400, 200, 400 }, /* row1: 最下方红色方块 */
        { 600, 400, 200 }, /* row2 */
        { 400, 600, 400 }, /* row3 */
        { 200, 400, 200 }, /* row4: 最上方红色方块 */
        { 0,   0,   0   }  /* row5: 出口区地面 */
    };

    // /* KFS 分布：固定演示矩阵 (可替换为视觉识别输入) */
    // MF_CellType demo_cell1[MF_ROWS][MF_COLS] = {
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
    //     { MF_CELL_R1KFS,  MF_CELL_EMPTY,  MF_CELL_R2KFS },
    //     { MF_CELL_R2KFS,  MF_CELL_EMPTY,  MF_CELL_R1KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_FAKEKFS, MF_CELL_R2KFS },
    //     { MF_CELL_R1KFS,  MF_CELL_R2KFS,  MF_CELL_EMPTY },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    // };

    // /* 另一个 预设KFS 分布：用于测试 */
    // MF_CellType demo_cell2[MF_ROWS][MF_COLS] = {
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_R2KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_R2KFS,  MF_CELL_R1KFS },
    //     { MF_CELL_R1KFS,  MF_CELL_FAKEKFS, MF_CELL_R1KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_R2KFS,  MF_CELL_R2KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    // };

    // /* 另一个 预设KFS 分布：用于测试 */
    // MF_CellType demo_cell3[MF_ROWS][MF_COLS] = {
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_R2KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_R1KFS },
    //     { MF_CELL_R1KFS,  MF_CELL_FAKEKFS, MF_CELL_R1KFS },
    //     { MF_CELL_R2KFS,  MF_CELL_R2KFS,  MF_CELL_R2KFS },
    //     { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    // };

    /* 固定分布地图，树林位置全部设为unknown */
    MF_CellType fixed_cell[MF_ROWS][MF_COLS] = {
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
        { MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN },
        { MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN },
        { MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN },
        { MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN,  MF_CELL_UNKNOWN },
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    };

    /* 拷贝到上下文中 */
    if(ctx->map_color == BLUE_MAP) {
        memcpy(ctx->height, fixed_height_blue, sizeof(fixed_height_blue));
    } else if(ctx->map_color == RED_MAP) {
        memcpy(ctx->height, fixed_height_red, sizeof(fixed_height_red));
    }
    memcpy(ctx->cell, fixed_cell, sizeof(fixed_cell));

    /* 将 row1~row4 标记为 UNKNOWN(模拟相机尚未扫描) */
    /* row0 和 row5 是地面，始终已知为 EMPTY */
    for (uint8_t r = 1; r <= 4; r++) {
        for (uint8_t c = 0; c < MF_COLS; c++) {
            ctx->cell[r][c] = MF_CELL_UNKNOWN;
        }
    }

}

// /**
//  * @brief 生成完整动作列表(一次性规划)
//  * @param ctx 规划器上下文
//  * @param start_row R2 初始行(入口区地面通常为 row0)
//  * @param start_col R2 初始列(0~2)
//  * @param r2kfs_need 需要拾取的 R2KFS 数量
//  * @return true 表示规划成功
//  * @note 决策核心
//  * @note 规划顺序为：依次选择一个最近的 R2KFS -> 走到相邻格 -> 面对拾取
//  * @note 若 R1KFS 阻断路径，则允许穿过并插入等待动作
//  */
// bool MFPlanner_BuildFullPlan(MF_Planner* ctx,
//                              uint8_t start_row,
//                              uint8_t start_col,
//                              uint8_t r2kfs_need)
// {

//     /* 重置动作列表与执行指针 */
//     ctx->action_count = 0;
//     ctx->action_index = 0;
//     /* 当前搜索起点(随拾取推进) */
//     uint8_t cur_r = start_row;
//     uint8_t cur_c = start_col;
//     for (uint8_t picked = 0; picked < r2kfs_need; picked++) {
//         /* 每轮选取一个总代价最低的 R2KFS(R1KFS 恒定可穿越，但穿越代价高) */
//         bool found_any = false;
//         /* 拾取阶段沿途仍允许穿越其他 R2KFS(产生 MOVE_R2)，只有最终出口路径禁止 */
//         bool allow_r2kfs_cross = true;
//         uint8_t best_target_r = 0;
//         uint8_t best_target_c = 0;
//         uint8_t best_path[MF_MAX_NODES];
//         uint8_t best_len = 0;
//         uint8_t best_adj_r = 0;
//         uint8_t best_adj_c = 0;
//         int16_t best_cost = 0x3FFF;
//         for (uint8_t r = 0; r < MF_ROWS; r++) {
//             for (uint8_t c = 0; c < MF_COLS; c++) {
//                 if (ctx->cell[r][c] != MF_CELL_R2KFS) {
//                     /* 不是 R2KFS 直接跳过 */
//                     continue;
//                 }
//                 /* 尝试到达该 R2KFS 的相邻格 */
//                 uint8_t path[MF_MAX_NODES];
//                 uint8_t len = 0;
//                 uint8_t adj_r = 0;
//                 uint8_t adj_c = 0;
//                 int16_t cost = 0;
//                 if (MF_FindPathToAdjacent(ctx, cur_r, cur_c, r, c,
//                                           allow_r2kfs_cross, path, &len, &adj_r, &adj_c, &cost)) {
//                     /* ============================================================ */
//                     /* 【修改3】检查拿完此 R2KFS 后能否到达出口                     */
//                     /* - 最后一步(picked == r2kfs_need-1)：只清除当前候选，       */
//                     /*   其余未拿 R2KFS 保留为真实障碍，避免选了到不了出口的 KFS   */
//                     /* - 非最后步：乐观清除所有 R2KFS（后续还会再选可打通）        */
//                     /* - 出口可达性按“最后拾取完成后禁止 MOVE_R2”的真实规则检查， */
//                     /*   即此处禁止穿越剩余 R2KFS，与后续真正规划出口路径的约束一致 */
//                     /* ============================================================ */
//                     {
//                         MF_CellType saved_cell[MF_ROWS][MF_COLS];
//                         uint8_t rr, cc;
//                         bool can_reach_exit;
//                         bool is_last_pick = (picked == r2kfs_need - 1);
//                         /* 最后一次拾取完成后不再允许 MOVE_R2，出口可达性检查需与之一致 */
//                         bool allow_r2kfs_cross_for_check = !is_last_pick;
//                         uint8_t ep[MF_MAX_NODES];
//                         uint8_t elen;
//                         int16_t ecost;
//                         memcpy(saved_cell, ctx->cell, sizeof(ctx->cell));
//                         if (is_last_pick) {
//                             /* 最后一步：只清除当前候选这一个 R2KFS */
//                             ctx->cell[r][c] = MF_CELL_EMPTY;
//                         } else {
//                             /* 非最后步：乐观清除所有 R2KFS（后续迭代还会再选） */
//                             for (rr = 0; rr < MF_ROWS; rr++) {
//                                 for (cc = 0; cc < MF_COLS; cc++) {
//                                     if (ctx->cell[rr][cc] == MF_CELL_R2KFS) {
//                                         ctx->cell[rr][cc] = MF_CELL_EMPTY;
//                                     }
//                                 }
//                             }
//                         }
//                         can_reach_exit = false;
//                         if (MF_AStar(ctx, adj_r, adj_c, 5, 0, allow_r2kfs_cross_for_check, ep, &elen, &ecost)) {
//                             can_reach_exit = true;
//                         }
//                         if (!can_reach_exit && MF_AStar(ctx, adj_r, adj_c, 5, 2, allow_r2kfs_cross_for_check, ep, &elen, &ecost)) {
//                             can_reach_exit = true;
//                         }
//                         memcpy(ctx->cell, saved_cell, sizeof(ctx->cell));
//                         if (!can_reach_exit) {
//                             continue; /* 该候选拿完后无法到出口，跳过 */
//                         }
//                     }
//                     if (!found_any || cost < best_cost) {
//                         /* 记录总代价更低的候选(步数 + R1KFS 穿越代价) */
//                         found_any = true;
//                         best_len = len;
//                         best_cost = cost;
//                         memcpy(best_path, path, len);
//                         best_target_r = r;
//                         best_target_c = c;
//                         best_adj_r = adj_r;
//                         best_adj_c = adj_c;
//                     }
//                 }
//             }
//         }
//         if (!found_any) {
//             /* 没有任何 R2KFS 可达，规划失败 */
//             return false;
//         }
//         /* 将路径翻译为动作 */
//         if (!MF_AddPathActions(ctx, best_path, best_len)) {
//             return false;
//         }
//         /* 在相邻格面对 R2KFS 取走 */
//         int8_t dr = (int8_t)best_target_r - (int8_t)best_adj_r;
//         int8_t dc = (int8_t)best_target_c - (int8_t)best_adj_c;
//         /* 面向目标格的方向即为拾取方向 */
//         MF_Dir pick_dir = MF_DirFromDelta(dr, dc);
//         /* 计算拾取动作的相对高度 */
//         int16_t height_diff = (int16_t)ctx->height[best_target_r][best_target_c]
//                                     - (int16_t)ctx->height[best_adj_r][best_adj_c];
//         /* 向数组中填入动作 */
//         if (!MF_PushAction(ctx, MF_ACT_PICK_R2, pick_dir, best_target_r, best_target_c)) {
//             return false;
//         }
//         /* 回填高度差到刚 push 的动作中 */
//         ctx->actions[ctx->action_count - 1].height_pick_diff = height_diff;
//         /* 取走后变为通路 */
//         ctx->cell[best_target_r][best_target_c] = MF_CELL_EMPTY;
//         /* 更新当前位置为相邻格 */
//         cur_r = best_adj_r;
//         cur_c = best_adj_c;
//     }
//     /* ================================================================ */
//     /* 所有 R2KFS 拾取完毕后，规划从当前位置走到出口区地面(row5)的路径 */
//     /* 出口候选列：col0 和 col2，选择总路程更短的那一个                */
//     /* 此时已完成全部所需 R2KFS 拾取，不允许再规划 MOVE_R2 动作，       */
//     /* 剩余 R2KFS 一律视为硬障碍，只能绕路或等待 R1KFS 清理             */
//     /* ================================================================ */
//     bool allow_r2kfs_cross_for_exit = false;
//     /* 两个出口候选：row5,col0 和 row5,col2 */
//     uint8_t exit_candidates_row[2] = { 5, 5 };    /* 两个候选出口行 */
//     uint8_t exit_candidates_col[2] = { 0, 2 };    /* 两个候选出口列 */
//     bool found_exit = false;                       /* 是否找到可达出口 */
//     uint8_t best_exit_path[MF_MAX_NODES];          /* 最优出口路径 */
//     uint8_t best_exit_len = 0;                     /* 最优出口路径长度 */
//     int16_t best_exit_cost = 0x3FFF;               /* 最优出口路径总代价 */
//     for (uint8_t e = 0; e < 2; e++) {
//         /* 依次对每个候选出口跑 A* */
//         uint8_t exit_r = exit_candidates_row[e];
//         uint8_t exit_c = exit_candidates_col[e];
//         /* 若当前位置已在候选出口上，直接选中(路程为0) */
//         if (cur_r == exit_r && cur_c == exit_c) {
//             found_exit = true;
//             best_exit_len = 0;
//             break; /* 已到出口，无需再找 */
//         }
//         uint8_t exit_path[MF_MAX_NODES];
//         uint8_t exit_len = 0;
//         int16_t exit_cost = 0;
//         /* A* 搜索从当前位置到该候选出口的路径(禁止穿越 R2KFS) */
//         if (MF_AStar(ctx, cur_r, cur_c, exit_r, exit_c,
//                         allow_r2kfs_cross_for_exit, exit_path, &exit_len, &exit_cost)) {
//             /* 比较总代价，保留更低的 */
//             if (!found_exit || exit_cost < best_exit_cost) {
//                 found_exit = true;
//                 best_exit_len = exit_len;
//                 best_exit_cost = exit_cost;
//                 memcpy(best_exit_path, exit_path, exit_len);
//             }
//         }
//     }
//     if (found_exit && best_exit_len > 0) {
//         /* 将最优出口路径翻译为移动动作序列 */
//         if (!MF_AddPathActions(ctx, best_exit_path, best_exit_len)) {
//             return false;
//         }
//     }
//     /* 【修改4】找不到任何出口路径时返回失败，不再静默吞掉 */
//     if (!found_exit) {
//         return false;
//     }
//     return true;
// }

/* ========================================== 多步/动态规划 ========================================== */

/**
 * @brief 对指定行范围进行扫描，将 UNKNOWN 格更新为真实地图数据
 * @param ctx        规划器上下文
 * @param start_row  扫描起始行(含)
 * @param end_row    扫描结束行(含)
 * @note 真实地图数据已在 LoadFixedDemoMap 中拷贝到 ctx->cell，
 *       这里"扫描"就是把指定行的 UNKNOWN 恢复为真实值
 * @note 实际比赛中应替换为从视觉模块获取的实时数据
 * @warning 实际比赛中应替换为从视觉模块获取的实时数据
 * @warning 实际比赛中应替换为从视觉模块获取的实时数据
 * @warning 实际比赛中应替换为从视觉模块获取的实时数据
 */
void MFPlanner_ScanRows(MF_Planner* ctx, uint8_t start_row, uint8_t end_row)
{
    /* 真实 KFS 分布(与 LoadFixedDemoMap 中的 demo_cellx 一致) */
    static const MF_CellType real_cell_debug_blue[MF_ROWS][MF_COLS] = {
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_R1KFS,  MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_EMPTY,  MF_CELL_R1KFS },
        { MF_CELL_EMPTY,  MF_CELL_FAKEKFS, MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_R1KFS,  MF_CELL_R2KFS },
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    };
    static const MF_CellType real_cell_debug_red[MF_ROWS][MF_COLS] = {
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_R1KFS,  MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_EMPTY,  MF_CELL_R1KFS },
        { MF_CELL_EMPTY,  MF_CELL_FAKEKFS, MF_CELL_EMPTY },
        { MF_CELL_R2KFS,  MF_CELL_R1KFS,  MF_CELL_R2KFS },
        { MF_CELL_EMPTY,  MF_CELL_EMPTY,  MF_CELL_EMPTY }
    };


    /* 遍历指定行范围 */
    for (uint8_t r = start_row; r <= end_row && r < MF_ROWS; r++) {
        for (uint8_t c = 0; c < MF_COLS; c++) {
            /* 仅更新仍为 UNKNOWN 的格子(保留已揭示/已修改的状态) */
            if (ctx->cell[r][c] == MF_CELL_UNKNOWN) {
                switch (ctx->map_color) {
                    case BLUE_MAP:
                        ctx->cell[r][c] = real_cell_debug_blue[r][c];
                        break;
                    case RED_MAP:
                        ctx->cell[r][c] = real_cell_debug_red[r][c];
                        break;
                }
            }
        }
    }
}

/**
 * @brief 分阶段规划：阶段1 先确保 R2 能登上 row1，拾取至少一个 row1 上的 R2KFS
 * @note 该函数内部包含了 1 个 MF_PushAction 用于添加扫描动作
 * @note 该函数内部同时有 2 个 MFPlanner_ScanRows 用于模拟从视觉模块获取数据并更新地图状态
 * @note 实际使用时需要替换为真正的视觉扫描，并添加对应逻辑
*/
bool MFPlanner_BuildPhase1(MF_Planner* ctx,
                           uint8_t start_row,
                           uint8_t start_col,
                           uint8_t* out_r2kfs_picked)
{
    /* 初始化输出 */
    if (out_r2kfs_picked != NULL) {
        *out_r2kfs_picked = 0;
    }

    // /* 重置动作列表与执行指针 */
    // ctx->action_count = 0;
    // ctx->action_index = 0;

    /* 阶段1不允许穿越 R2KFS(保守策略，路径规划应避开而非移开) */
    bool allow_r2kfs_cross = false;

    /* 追踪当前位置：随拾取/移动更新 */
    uint8_t cur_r = start_row;
    uint8_t cur_c = start_col;

    /* ============================================================ */
    /* 规则约束：若 (1,1) 上有 R2KFS 挡住登顶点，                  */
    /* R2 必须先站在 (0,1) 拾取该 R2KFS 才能登上 (1,1)             */
    /* ============================================================ */

    /* 在 row0,col1 面朝北扫描前三行(row1/row2/row3) */
    // if (MF_PushAction(ctx, MF_ACT_SCAN, MF_DIR_NORTH, 0, 1)) 
    // {
        /* === 一次扫描：在 row0,col1 面朝北，揭示 row1~row2 === */
        // MFPlanner_ScanRows(&mf_planner_ctx, 1, 2);
        /* 此时：row1/row2变为真实 KFS 分布，row3/row4 仍为 UNKNOWN */
    // }

    /* 此处应修改为实际的相机扫描 */
    MFPlanner_ScanRows(ctx, 1, 2);

    /* 【规则】只要 row1(col0/col1/col2 任意一列)上存在 R2KFS，
     * 阶段1就必须先拾取其中一个才能进入阶段2；若 (1,1) 本身是
     * R2KFS 则优先拾取它(直接挡住登顶点)，否则在 col0/col2 的
     * R2KFS 中选可达且代价最低的一个拾取，其余留给阶段2处理 */
    bool row1_has_r2kfs = false;
    uint8_t kfs_c = 0;

    if (ctx->cell[1][1] == MF_CELL_R2KFS) {
        row1_has_r2kfs = true;
        kfs_c = 1;
    } else {
        int16_t best_cost = 0x3FFF;
        for (uint8_t c = 0; c < MF_COLS; c++) {
            if (ctx->cell[1][c] != MF_CELL_R2KFS) {
                continue;
            }
            uint8_t cand_path[MF_MAX_NODES];
            uint8_t cand_len = 0;
            int16_t cand_cost = 0;
            if (!MF_AStar(ctx, cur_r, cur_c, 0, c,
                          allow_r2kfs_cross, cand_path, &cand_len, &cand_cost)) {
                continue;
            }
            if (!row1_has_r2kfs || cand_cost < best_cost) {
                row1_has_r2kfs = true;
                best_cost = cand_cost;
                kfs_c = c;
            }
        }
    }

    if (row1_has_r2kfs) {
        /* ======================================================== */
        /* row1,col(kfs_c) 上有 R2KFS：走到其正下方 row0 格，面北拾取 */
        /* 【规则】R2 必须站在 row0 拾取该 R2KFS，                   */
        /*        若 kfs_c==1 则拾取后 (1,1) 变通路，R2 才能 step_up */
        /* ======================================================== */
        uint8_t kfs_r = 1;
        uint8_t path[MF_MAX_NODES];
        uint8_t path_len = 0;
        uint8_t adj_r = 0, adj_c = 0;

        /* 固定相邻格为 row1 KFS 正下方的 row0 格（面北拾取） */
        adj_r = 0;
        adj_c = kfs_c;

        /* 校验 row0 相邻格是否在边界内且可站立 */
        if (!MF_InBounds((int8_t)adj_r, (int8_t)adj_c)) {
            return false;
        }
        if (MF_IsBlockedForMove(ctx, adj_r, adj_c, allow_r2kfs_cross)) {
            return false;
        }

        /* 规划从当前位置到 row0 相邻格的路径（纯 row0 平移，无高度差） */
        if (!MF_AStar(ctx, cur_r, cur_c, adj_r, adj_c,
                      allow_r2kfs_cross, path, &path_len, NULL)) {
            return false;
        }
        if (!MF_AddPathActions(ctx, path, path_len)) {
            return false;
        }

        /* 站在 row0 相邻格上面北拾取 row1 的 R2KFS */
        /* 方向固定为 NORTH（row0 面朝 row1） */
        MF_Dir pick_dir = MF_DIR_NORTH;
        int16_t height_diff = (int16_t)ctx->height[kfs_r][kfs_c]
                              - (int16_t)ctx->height[adj_r][adj_c];

        if (!MF_PushAction(ctx, MF_ACT_PICK_R2, pick_dir, kfs_r, kfs_c)) {
            return false;
        }
        *out_r2kfs_picked += 1; /* 统计已拾取的 R2KFS 数量 */
        ctx->actions[ctx->action_count - 1].height_pick_diff = height_diff;

        /* 取走后变通路 */
        ctx->cell[kfs_r][kfs_c] = MF_CELL_EMPTY;

        /* 更新当前位置为相邻格(拾取后 R2 仍站在这里) */
        cur_r = adj_r;
        cur_c = adj_c;
    }

    /* ============================================================ */
    /* 现在 row1 上已无障碍(R2KFS 已拾取或本来就没有)             */
    /* 从当前位置走到 (1,1) 进行第二次扫描                        */
    /* ============================================================ */
    {
        uint8_t target_r = 1;
        uint8_t target_c = 1;
        if (!(cur_r == target_r && cur_c == target_c)) {
            uint8_t path[MF_MAX_NODES];
            uint8_t path_len = 0;
            if (!MF_AStar(ctx, cur_r, cur_c, target_r, target_c,
                          allow_r2kfs_cross, path, &path_len, NULL)) {
                return false;
            }
            if (!MF_AddPathActions(ctx, path, path_len)) {
                return false;
            }
        }
    }

    /* 在 row1,col1 面朝北扫描后二行(row3/row4) */
    if (MF_PushAction(ctx, MF_ACT_SCAN, MF_DIR_NORTH, 1, 1)) 
    {
        /* === 第二次扫描(在 SCAN 动作执行后触发)：揭示 row3~row4 === */
        MFPlanner_ScanRows(ctx, 3, 4);
        /* 此时：row3/row4 全部已知(如已揭示则保持不变) */
    }

    return true;
}

/**
 * @brief 分阶段规划：阶段2 在 row1,col1 的基础上继续拾取剩余 R2KFS 并走到出口
 * @note 该函数假设阶段1已完成，当前位于 row1,col1，且至少一个 row1 上的 R2KFS 已被拾取
 * @note 该函数内部不包含扫描动作，因为阶段1结束时已添加扫描动作
*/
bool MFPlanner_BuildPhase2(MF_Planner* ctx, uint8_t r2kfs_need)
{
    /* 阶段2从 row1,col1 开始(阶段1结束时的位置) */
    uint8_t cur_r = 1;
    uint8_t cur_c = 1;

    /* ============================================================ */
    /* 以下逻辑与原 MFPlanner_BuildFullPlan 完全相同               */
    /* 唯一的区别：不重置 action_count(追加到阶段1的动作之后)     */
    /* ============================================================ */

    for (uint8_t picked = 0; picked < r2kfs_need; picked++) {
        /* 每轮选取“当前路径 + 后续剩余拾取 + 出口”的总代价最低 R2KFS */
        bool found_any = false;
        /* 拾取阶段沿途仍允许穿越其他 R2KFS(产生 MOVE_R2)，只有最终出口路径禁止 */
        bool allow_r2kfs_cross = true;
        uint8_t best_target_r = 0;
        uint8_t best_target_c = 0;
        uint8_t best_path[MF_MAX_NODES];
        uint8_t best_len = 0;
        uint8_t best_adj_r = 0;
        uint8_t best_adj_c = 0;
        int16_t best_cost = 0x3FFF;
        int16_t best_plan_cost = 0x3FFF;

        for (uint8_t r = 0; r < MF_ROWS; r++) {
            for (uint8_t c = 0; c < MF_COLS; c++) {
                if (ctx->cell[r][c] != MF_CELL_R2KFS) {
                    continue;
                }
                uint8_t path[MF_MAX_NODES];
                uint8_t len = 0;
                uint8_t adj_r = 0;
                uint8_t adj_c = 0;
                int16_t cost = 0;
                if (MF_FindPathToAdjacent(ctx, cur_r, cur_c, r, c,
                                          allow_r2kfs_cross, path, &len, &adj_r, &adj_c, &cost)) {
                    MF_Planner lookahead_ctx = *ctx;
                    MF_ApplyPathCellEffects(&lookahead_ctx, path, len);
                    lookahead_ctx.cell[r][c] = MF_CELL_EMPTY;

                    int16_t remaining_cost = 0;
                    if (!MF_EvaluateRemainingPlanCost(&lookahead_ctx,
                                                      adj_r,
                                                      adj_c,
                                                      (uint8_t)(r2kfs_need - picked - 1),
                                                      &remaining_cost)) {
                        continue;
                    }

                    int16_t plan_cost = (int16_t)(cost + remaining_cost);
                    if (!found_any ||
                        plan_cost < best_plan_cost ||
                        (plan_cost == best_plan_cost && cost < best_cost)) {
                        found_any = true;
                        best_len = len;
                        best_cost = cost;
                        best_plan_cost = plan_cost;
                        memcpy(best_path, path, len);
                        best_target_r = r;
                        best_target_c = c;
                        best_adj_r = adj_r;
                        best_adj_c = adj_c;
                    }
                }
            }
        }

        if (!found_any) {
            return false;
        }

        /* 将路径翻译为动作 */
        if (!MF_AddPathActions(ctx, best_path, best_len)) {
            return false;
        }

        /* 在相邻格面对 R2KFS 取走 */
        int8_t dr = (int8_t)best_target_r - (int8_t)best_adj_r;
        int8_t dc = (int8_t)best_target_c - (int8_t)best_adj_c;
        MF_Dir pick_dir = MF_DirFromDelta(dr, dc);

        /* 计算拾取动作的相对高度 */
        int16_t height_diff = (int16_t)ctx->height[best_target_r][best_target_c]
                              - (int16_t)ctx->height[best_adj_r][best_adj_c];

        if (!MF_PushAction(ctx, MF_ACT_PICK_R2, pick_dir, best_target_r, best_target_c)) {
            return false;
        }
        ctx->actions[ctx->action_count - 1].height_pick_diff = height_diff;

        /* 取走后变为通路 */
        ctx->cell[best_target_r][best_target_c] = MF_CELL_EMPTY;

        /* 更新当前位置 */
        cur_r = best_adj_r;
        cur_c = best_adj_c;
    }

    /* ============================================================ */
    /* 出口候选：row5,col0 和 row5,col2，选路程更短的            */
    /* 此时已完成全部所需 R2KFS 拾取，不允许再规划 MOVE_R2 动作， */
    /* 剩余 R2KFS 一律视为硬障碍                                  */
    /* ============================================================ */
    {
        bool allow_r2kfs_cross_for_exit = false;
        uint8_t exit_candidates_row[2] = { 5, 5 };
        uint8_t exit_candidates_col[2] = { 0, 2 };

        bool found_exit = false;
        uint8_t best_exit_path[MF_MAX_NODES];
        uint8_t best_exit_len = 0;
        int16_t best_exit_cost = 0x3FFF;

        for (uint8_t e = 0; e < 2; e++) {
            uint8_t exit_r = exit_candidates_row[e];
            uint8_t exit_c = exit_candidates_col[e];

            if (cur_r == exit_r && cur_c == exit_c) {
                found_exit = true;
                best_exit_len = 0;
                break;
            }

            uint8_t exit_path[MF_MAX_NODES];
            uint8_t exit_len = 0;
            int16_t exit_cost = 0;

            if (MF_AStar(ctx, cur_r, cur_c, exit_r, exit_c,
                         allow_r2kfs_cross_for_exit, exit_path, &exit_len, &exit_cost)) {
                if (!found_exit || exit_cost < best_exit_cost) {
                    found_exit = true;
                    best_exit_len = exit_len;
                    best_exit_cost = exit_cost;
                    memcpy(best_exit_path, exit_path, exit_len);
                }
            }
        }

        if (found_exit && best_exit_len > 0) {
            if (!MF_AddPathActions(ctx, best_exit_path, best_exit_len)) {
                return false;
            }
        }
        /* 【修改2】找不到任何出口路径时返回失败，不再静默吞掉 */
        if (!found_exit) {
            return false;
        }
    }

    return true;
}

/* ========================================== 多步/动态规划 ========================================== */


/* ================================================== 实现部分 ================================================== */
// 暂无
/* ================================================== 实现部分 ================================================== */



/* ================================================== 应用层调用部分 ================================================== */

/**
 * @brief 读取当前动作(不推进)
 * @param ctx 规划器上下文
 * @param out_action 输出动作结构体
 * @return true 表示存在动作
 * @note 通常在行为树 Tick 中读取，用于驱动底盘与夹取动作
 */
bool MFPlanner_PeekAction(const MF_Planner* ctx, MF_Action* out_action)
{
    if (ctx->action_index >= ctx->action_count) {
        /* 已无可执行动作 */
        return false;
    }
    if (out_action != NULL) {
        /* 输出当前动作 */
        *out_action = ctx->actions[ctx->action_index];
    }
    return true;
}

/**
 * @brief 推进动作指针
 * @param ctx 规划器上下文
 * @return true 表示推进成功
 * @note 只有当执行层确认动作完成后才调用
 */
bool MFPlanner_AdvanceAction(MF_Planner* ctx)
{
    if (ctx->action_index >= ctx->action_count) {
        /* 已到末尾不能再推进 */
        return false;
    }
    /* 指向下一条动作 */
    ctx->action_index++;
    return true;
}

/**
 * @brief R1 清除 R1KFS 后更新地图
 * @param ctx 规划器上下文
 * @param row 目标行
 * @param col 目标列
 * @note 若提前清除，可避免等待动作阻塞
 */
void MFPlanner_NotifyR1KfsRemoved(MF_Planner* ctx, uint8_t row, uint8_t col)
{
    if (row >= MF_ROWS || col >= MF_COLS) {
        /* 越界保护 */
        return;
    }
    if (ctx->cell[row][col] == MF_CELL_R1KFS) {
        /* 确认可清除后置为空格 */
        ctx->cell[row][col] = MF_CELL_EMPTY;
    }
}

/* ================================================== 应用层调用部分 ================================================== */


/* ================================================== 调试输出日志部分 ================================================== */
// 暂无
/* ================================================== 调试输出日志部分 ================================================== */
