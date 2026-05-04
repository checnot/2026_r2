#ifndef FILTER_H
#define FILTER_H

#include "main.h"

/**
 * @brief 一维卡尔曼滤波器结构体
 * @warning 该结构体适用于一维状态估计问题，如单变量传感器数据的滤波。对于多维系统，需要扩展为矩阵形式的卡尔曼滤波器。
 */
typedef struct {
    float Q; /* 过程噪声协方差 (Process Noise Covariance)：反映模型自身的不确定性，值越大表明系统本身变化越快 */
    float R; /* 测量噪声协方差 (Measurement Noise Covariance)：反映传感器的噪声，值更大表明传感器越不可靠 */
    float P; /* 估计误差协方差 (Estimation Error Covariance)：当前估计值的误差不确定度 */
    float K; /* 卡尔曼增益 (Kalman Gain)：决定信任测量值还是预测值的权重比例 */
    float x; /* 状态估计值 (Estimated Value)：当前的最优估计结果 */
} KalmanFilter_stu;

void Kalman_Init(KalmanFilter_stu *kf, float Q, float R, float initial_P, float initial_x);
float Kalman_Update(KalmanFilter_stu *kf, float measurement);
#endif /* FILTER_H */
