// #include "xxx.h"
#include <math.h>
/*
wei ni ju zhen
$$A = \begin{bmatrix}
-\cos\theta - \sin\theta & -\sin\theta + \cos\theta & L \\
\cos\theta - \sin\theta & \sin\theta + \cos\theta & L \\
\cos\theta + \sin\theta & \sin\theta - \cos\theta & L \\
-\cos\theta + \sin\theta & -\sin\theta - \cos\theta & L
\end{bmatrix}$$

$$A^+ = \frac{1}{4} \begin{bmatrix}
-\cos\theta - \sin\theta & \cos\theta - \sin\theta & \cos\theta + \sin\theta & -\cos\theta + \sin\theta \\
-\sin\theta + \cos\theta & \sin\theta + \cos\theta & \sin\theta - \cos\theta & -\sin\theta - \cos\theta \\
\frac{1}{L} & \frac{1}{L} & \frac{1}{L} & \frac{1}{L}
\end{bmatrix}$$

*/

typedef struct {
    float L;    //底盘半对角线长度
    float R;    //轮子的半径
    float theta;    //当前底盘相对世界坐标系的夹角
    float Init_Sum_Angle[4];    //起始时候的电机累加角度
    float Now_Stage_Sum_Angle[4];// 当前电机的累加角度
    float Last_Stage_Sum_Angle[4];  //上一时刻电机的累加角度
    float x_move;   //x方向位移
    float y_move;   //y方向位移
    float theta_move;   //底盘转动角度
}Angle_Sum_Matrix;

Angle_Sum_Matrix Chassis_Angle_Matrix;

void Init_Angle_Sum_Matrix(M3508_Motor_Struct m3508)
{
    Chassis_Angle_Matrix.L = 1.0f;
    Chassis_Angle_Matrix.R = 0.05f;

    Chassis_Angle_Matrix.Init_Sum_Angle[0] = (m3508 + 0)->Angle_Sum;
    Chassis_Angle_Matrix.Init_Sum_Angle[1] = (m3508 + 1)->Angle_Sum;
    Chassis_Angle_Matrix.Init_Sum_Angle[2] = (m3508 + 2)->Angle_Sum;
    Chassis_Angle_Matrix.Init_Sum_Angle[3] = (m3508 + 3)->Angle_Sum;
}

void Update_Angle_Sum_Matrix(M3508_Motor_Struct m3508, Angle_Sum_Matrix *p) {
    //从陀螺仪更新底盘跟世界坐标系夹角theta
    // p->theta = ...;
    Chassis_Angle_Matrix.Now_Stage_Sum_Angle[0] = (m3508 + 0)->Angle_Sum -  Chassis_Angle_Matrix.Init_Sum_Angle[0];
    Chassis_Angle_Matrix.Now_Stage_Sum_Angle[1] = (m3508 + 1)->Angle_Sum -  Chassis_Angle_Matrix.Init_Sum_Angle[1];
    Chassis_Angle_Matrix.Now_Stage_Sum_Angle[2] = (m3508 + 2)->Angle_Sum -  Chassis_Angle_Matrix.Init_Sum_Angle[2];
    Chassis_Angle_Matrix.Now_Stage_Sum_Angle[3] = (m3508 + 3)->Angle_Sum -  Chassis_Angle_Matrix.Init_Sum_Angle[3];

    // 1. 预计算三角函数（使用当前底盘角度）
    float s = sinf(p->theta);
    float c = cosf(p->theta);
    
    // 2. 计算本周期的电机角度增量 delta_phi
    float d_phi[4];
    for (int i = 0; i < 4; i++) {
        d_phi[i] = p->Now_Stage_Sum_Angle[i] - p->Last_Stage_Sum_Angle[i];
    }

    // 3. 预计算常数系数 (根号2 * 1/4 * R)
    float k = sqrtf(2.0f) * 0.25f * p->R;
    float k_rot = k / p->L;

    float c_plus_s  = c + s;
    float c_minus_s = c - s;

    // 4. 正运动学解算：映射到世界坐标系位移增量
    // X 方向增量 (对应伪逆矩阵第一行)
    float dx = k * ((-c_plus_s) * d_phi[0] + 
                    ( c_minus_s) * d_phi[1] + 
                    ( c_plus_s)  * d_phi[2] + 
                    (-c_minus_s) * d_phi[3]);

    // Y 方向增量 (对应伪逆矩阵第二行)
    float dy = k * (( c_minus_s) * d_phi[0] + 
                    ( c_plus_s)  * d_phi[1] + 
                    (-c_minus_s) * d_phi[2] + 
                    (-c_plus_s)  * d_phi[3]);

    // 角度增量 (对应伪逆矩阵第三行)
    float d_theta = k_rot * (d_phi[0] + d_phi[1] + d_phi[2] + d_phi[3]);

    // 5. 更新结构体中的位移累加量和当前角度
    p->x_move += dx;
    p->y_move += dy;
    p->theta_move += d_theta; // 相对初始位置转过的总角度
    p->theta += d_theta;      // 更新当前朝向角，用于下一周期计算

    // 6. 角度范围归一化 [-PI, PI] (可选，建议保留)
    if (p->theta >  3.1415926f) p->theta -= 6.2831853f;
    if (p->theta < -3.1415926f) p->theta += 6.2831853f;

    // 7. 状态更新：将当前值存入上一时刻变量，准备下一周期
    for (int i = 0; i < 4; i++) {
        p->Last_Stage_Sum_Angle[i] = p->Now_Stage_Sum_Angle[i];
    }
}