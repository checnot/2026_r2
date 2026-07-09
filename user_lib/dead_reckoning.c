#include "dead_reckoning.h"
#include "radar.h"       /* Radar_Pos.hi14_yaw (陀螺仪偏航角) */
#include <math.h>        /* cosf, sinf */

DeadReckoning dr;

/* DR 替代雷达定位开关：MC 区设 1，离开 MC 区设 0 */
uint8_t g_use_dr_pos = 0;

/* 调试变量 */
float dr_dbg_dL[4];
float dr_dbg_dx;
float dr_dbg_dy;

/**
 * @brief 航位推算初始化
 * @note 首次调用 DR_Update 前调用，置零位姿和编码器参考
 */
void DR_Init(void)
{
    dr.x = 0.0f;
    dr.y = 0.0f;
    dr.yaw_deg = 0.0f;
    dr.initialized = 0;
}

/**
 * @brief 航位推算更新（每 Tick 调用）
 * @note 用底盘四轮编码器增量反解机器人位移，再用陀螺仪偏航角旋转到世界系
 * @note 陀螺仪角度来源：Radar_Pos.hi14_yaw（度），由 hipnuc 惯导提供
 *
 * ==================== 45° 全向轮 X 布局运动学推导 ====================
 *
 * 轮布局（俯视，X前 Y左）—— 相对两轮平行，轴线交叉成 X 形：
 *   fdcan3 索引:  [0]=左前  [1]=右前  [2]=右后  [3]=左后
 *
 *           [0]左前              [1]右前
 *               ╲    wheel axis    ╱
 *           (轴 ∥ 右后)  ╲  ╱  (轴 ∥ 左后)
 *                        ╲╱
 *            ────────────╱╲────────────→ X
 *                       ╱  ╲
 *           (轴 ∥ 右前) ╱    ╲ (轴 ∥ 左前)
 *                     ╱        ╲
 *           [3]左后              [2]右后
 *
 *   [0]∥[2] 共轴（左前∥右后）  [1]∥[3] 共轴（右前∥左后）
 *
 * 每个全向轮：辊子 ⟂ 轮轴，轮轴与 X/Y 成 45°。
 * 驱动方向 di = ∥ 轮轴方向（辊子不滑移的分量）：
 *   d0 = (+1,-1)/√2    d1 = (-1,-1)/√2
 *   d2 = (-1,+1)/√2    d3 = (+1,+1)/√2
 *
 * 轮子转 dL[i]（切向位移 = 编码器换算的轮面位移），
 * 全部转为驱动力（辊子 ⊥ 轮轴，无衰减）；dL[i] 等于
 * 机器人位移在驱动方向上的投影：
 *
 *   dL0 = (dx-dy)/√2    dL1 = (-dx-dy)/√2
 *   dL2 = (-dx+dy)/√2    dL3 = (dx+dy)/√2
 *
 * 逆解（伪逆，最小二乘）：
 *   dx = √2/4 · (dL0 - dL1 - dL2 + dL3)
 *   dy = √2/4 · (-dL0 - dL1 + dL2 + dL3)
 *
 * 注意：驱动方向含 1/√2（投影），导致逆解乘 √2（不可抵消）。
 */
void DR_Update(void)
{
    /* ---- 1. 读取四轮编码器累计值 ---- */
    int32_t enc[4];
    enc[0] = m3508_fdcan3[0].Angle_Sum;
    enc[1] = m3508_fdcan3[1].Angle_Sum;
    enc[2] = m3508_fdcan3[2].Angle_Sum;
    enc[3] = m3508_fdcan3[3].Angle_Sum;

    /* ---- 2. 首次调用：记录初始编码器值，不清零坐标 ---- */
    if (!dr.initialized) {
        dr.prev_enc[0] = enc[0];
        dr.prev_enc[1] = enc[1];
        dr.prev_enc[2] = enc[2];
        dr.prev_enc[3] = enc[3];
        dr.yaw_deg =  hipnuc_raw.hi91.yaw;
        dr.initialized = 1;
        return;
    }

    /* ---- 3. 编码器增量 → 轮子切向位移增量 (m) ---- */
    float dL[4];
    for (int i = 0; i < 4; i++) {
        int32_t delta = enc[i] - dr.prev_enc[i];
        dr.prev_enc[i] = enc[i];
        dL[i] = (float)delta * DR_COUNT_TO_METER;
    }

    /* ---- 4. 45° 全向轮逆运动学：轮切向位移 → 机器人系位移增量 (m) ---- */
    float dx_robot = (dL[0] - dL[1] - dL[2] + dL[3]) * DR_SQRT2_OVER_4;
    float dy_robot = (-dL[0] - dL[1] + dL[2] + dL[3]) * DR_SQRT2_OVER_4;

    /* 调试：记录四轮位移和机器人系增量 (mm/tick) */
    dr_dbg_dL[0] = dL[0] * 1000.0f;
    dr_dbg_dL[1] = dL[1] * 1000.0f;
    dr_dbg_dL[2] = dL[2] * 1000.0f;
    dr_dbg_dL[3] = dL[3] * 1000.0f;
    dr_dbg_dx = dx_robot * 1000.0f;
    dr_dbg_dy = dy_robot * 1000.0f;

    /* ---- 5. 用陀螺仪偏航角旋转到世界坐标系 ---- */
    float yaw_rad = Radar_Pos.hi14_yaw * 3.1415926f / 180.0f;
    float cos_yaw = cosf(yaw_rad);
    float sin_yaw = sinf(yaw_rad);

    dr.x += (dx_robot * cos_yaw - dy_robot * sin_yaw)*1000.0f;
    dr.y += (dx_robot * sin_yaw + dy_robot * cos_yaw)*1000.0f;
    dr.yaw_deg =  hipnuc_raw.hi91.yaw;
}
