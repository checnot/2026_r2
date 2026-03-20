#include "filter.h"

/**
 * @brief  初始化一维卡尔曼滤波器
 * @param  kf:        滤波器结构体指针
 * @param  Q:         过程噪声协方差（系统内部状态发生改变时的方差，影响收敛速度与平滑度）
 * @param  R:         测量噪声协方差（传感器本身的物理噪声特征，通常由器件手册或实测决定）
 * @param  initial_P: 初始估计误差协方差（系统初始时的不确定度）
 * @param  initial_x: 初始状态估计值（可以是传感器的第一次读数或者固定初值）
 * @retval 无
 */
void Kalman_Init(KalmanFilter_stu *kf, float Q, float R, float initial_P, float initial_x)
{
    // ================= 初始化阶段 (Initialization) =================
    // 1. 设置过程噪声协方差：假设系统自身状态会有多大波动
    kf->Q = Q;
    // 2. 设置测量噪声协方差：假设传感器测量的数据有多大抖动
    kf->R = R;
    // 3. 设置初始误差协方差：开始时的不确定度
    kf->P = initial_P;
    // 4. 设置初始状态估计值：当前默认的起始值
    kf->x = initial_x;
}

/**
 * @brief  卡尔曼滤波器逐步更新函数
 * @param  kf:          滤波器结构体指针
 * @param  measurement: 当前传感器的实际测量新数据 (如 AD 读数、角度等)
 * @retval 经过卡尔曼滤波后输出的最优状态估计值
 */
float Kalman_Update(KalmanFilter_stu *kf, float measurement)
{
    /* ================= 第一步：预测阶段 (Predict) ================= */
    
    // 1. 预测当前状态
    // 在最简单的一维模型中，我们假设状态没有控制输入且自身平稳，所以上一时刻的状态就是当前的最优先验预测
    // 先验估计：x_predict = x_prev    (代码中无直接体现，因为 kf->x 就是保留的 x_prev)
    
    // 2. 预测误差协方差
    // 由于时间推移和模型环境影响，系统预测的不确定度会变大，所以要加上过程噪声协方差 Q
    kf->P = kf->P + kf->Q;

    /* ================= 第二步：更新阶段 (Update) ================= */
    
    // 3. 计算卡尔曼增益 (Kalman Gain)
    // 公式：K = P_predict / (P_predict + R)
    // 意义：计算“我们有多信任测量值”。如果测量噪声 R 很大，K 趋近 0，说明不信传感器；如果 R 很小，K 接近 1，说明非常信任传感器
    kf->K = kf->P / (kf->P + kf->R);

    // 4. 更新最优状态估计值 (计算后验估计)
    // 公式：x_new = x_predict + K * (measurement - x_predict)
    // 意义：根据实测值修正之前的先验预测值。用残差(measurement - kf->x)向真实测量值方向迈进了一步，迈出的比例由卡尔曼增益 K 决定
    kf->x = kf->x + kf->K * (measurement - kf->x);

    // 5. 更新误差协方差
    // 公式：P_new = (1 - K) * P_predict
    // 意义：当我们融合了新的传感器数据算出最优估计后，当前系统状态整体的不确定性应当随之减小
    kf->P = (1.0f - kf->K) * kf->P;

    // 返回当前计算出的最优估计值
    return kf->x;
}
