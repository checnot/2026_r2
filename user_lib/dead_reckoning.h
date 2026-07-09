#ifndef DEAD_RECKONING_H
#define DEAD_RECKONING_H

#include "main.h"
#include "chassis.h"
#include "m3508.h"

/* ================= 全向轮底盘运动学参数 ================= */
#define DR_WHEEL_RADIUS         0.0678f   /* 轮子半径 (m)，🟡需实测 */
#define DR_DISTANCE_TO_CENTER   0.25266f /* 轮子离底盘中心距离 (m)，复用 chassis.h */
#define DR_ENCODER_RESOLUTION   8192u    /* M3508 编码器分辨率 (counts/rev) */
#define DR_REDUCTION_RATIO      3591/187      /* 减速比 19:1 */

/* 每个编码器计数对应的轮子位移 (m/count) */
#define DR_COUNT_TO_METER   ((2.0f * 3.1415926f * DR_WHEEL_RADIUS) \
                            / (float)(DR_ENCODER_RESOLUTION * DR_REDUCTION_RATIO))

/* 45° 全向轮逆运动学系数：轮轴驱动方向投影含 1/√2，逆解需乘回 √2 */
#define DR_SQRT2_OVER_4     0.35355339f   /* √2 / 4 */

/* ================= 航位推算实例 ================= */
typedef struct {
    float x;               /* 世界坐标系 X (m) */
    float y;               /* 世界坐标系 Y (m) */
    float yaw_deg;         /* 偏航角 (度)，取自陀螺仪 hi14 */

    int32_t prev_enc[4];   /* 上一Tick四轮编码器累计值 */
    uint8_t initialized;   /* 首次初始化完成标志 */
} DeadReckoning;

extern DeadReckoning dr;

/* ================= 外部接口 ================= */
void DR_Init(void);
void DR_Update(void);   /* 每Tick调用一次（建议 1ms 或 10ms 周期） */

/* ================= DR 替代雷达定位（MC 区无雷达覆盖） ================= */
extern uint8_t g_use_dr_pos;   /* 1=将 DR 位置灌入 Radar_Pos，供路径系统使用 */

/* 将 dr.x/y/yaw 同步到 Radar_Pos（在路径调度前调用） */
static inline void DR_SyncToRadar(void)
{
    Radar_Pos.reliable_x   = dr.x;
    Radar_Pos.reliable_y   = dr.y;
    Radar_Pos.reliable_yaw = dr.yaw_deg;
}

#endif /* DEAD_RECKONING_H */
