#ifndef __TRAJECTORY_H
#define __TRAJECTORY_H

#include "main.h"
#include <stdint.h>
#include <math.h>

/**
 * @brief 旋转关节 自适应负载 单段轨迹生成器结构体；
 *         类似引擎作用
*/
typedef struct {
    // 基础参数
    float start_pos;     // 初始位置
    float target_pos;    // 目标位置
    float v_max;         // 最大允许速度
    float a_max;         // 最大允许加速度
    
    // 运行状态
    float out_pos;       // 输出给PID的期望位置
    float out_vel;       // 当前期望速度
    
    // 误差跟踪与补救参数
    float max_tracking_error;  // 容忍的最大跟随误差
    uint8_t is_remedy_mode;    // 是否处于补救模式 (1: 是)
    uint8_t is_finished;       // 是否真正到达物理目标 (1: 完成)
} Traj_Single_struct;


/* ===================== 新增：多段路径规划 (队列与融合) ===================== */
#define TRAJ_MAX_WAYPOINTS 32  // 支持连拼的路径点缓存数量
// 路径缓存队列管理器
typedef struct {
    float points[TRAJ_MAX_WAYPOINTS];  // 目标位置数组
    uint16_t head;                     // 写入头下标
    uint16_t tail;                     // 读取尾下标（类似表示到哪个路径的flag）
    uint16_t count;                    // 尚未走完的路径点总数
    
    float blend_radius;                // 连续过弯/途经点的融合半径(设为 0 即精停)
} Traj_Queue_struct;



extern int32_t debug_count;

extern Traj_Single_struct debug_traj_a1;
extern Traj_Queue_struct debug_traj_queue_a1;
extern Traj_Single_struct debug_traj_m3508;
extern Traj_Queue_struct debug_traj_queue_m3508;

void Traj_Adaptive_Init(Traj_Single_struct *traj, float start_pos, float target_pos, float v_max, float a_max, float max_error);

void Traj_Adaptive_Update(Traj_Single_struct *traj, float actual_pos, float dt);


// ===================== 新增：多段路径规划 (队列与融合) ===================== */

void Traj_Path_Queue_Init(Traj_Queue_struct *queue, float blend_radius);

uint8_t Traj_Path_Queue_AddPoint(Traj_Queue_struct *queue, float target_pos);

void Traj_Path_Queue_Update(Traj_Queue_struct *queue, Traj_Single_struct *traj, float actual_pos);


#endif /* __TRAJECTORY_H */
