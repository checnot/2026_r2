#include "m3508.h"

M3508_Motor_Struct m3508_fdcan1[8];
M3508_Motor_Struct m3508_fdcan2[8];
M3508_Motor_Struct m3508_fdcan3[8];


/**
 * @brief m3508电机速度控制函数
 * @param motor 电机结构体指针
 * @param target_speed 目标速度
 * @param max_output 最大电流输出
 */
void m3508_control_speed(M3508_Motor_Struct *motor, int16_t target_speed, int16_t max_output)
{
    PID_Calculate(&(motor->M3508_Speed_PID), (float)motor->Now_Speed, (float)target_speed, (float)max_output);
    motor->Current_Output = (int16_t)motor->M3508_Speed_PID.Output;
}

/**
 * @brief m3508电机角度速度双环控制函数
 * @param motor 电机结构体指针
 * @param target_anglesum 目标角度和
 * @param max_output 最大电流输出
 * @warning 在3.23更新中变为上电时刻角度和为0，使用时需注意
 */
void m3508_control_anglesum(M3508_Motor_Struct *motor, int32_t target_anglesum, int16_t max_output)
{
    PID_Calculate(&(motor->M3508_Angle_PID), (float)motor->Angle_Sum, (float)target_anglesum, (float)max_output);
    PID_Calculate(&(motor->M3508_Speed_PID), (float)motor->Now_Speed, (float)motor->M3508_Angle_PID.Output, (float)max_output);
    motor->Current_Output = (int16_t)motor->M3508_Speed_PID.Output;
}

/**
 * @brief m3508电机FDCAN发送函数
 * @param hfdcan FDCAN句柄
 * @param Motor 电机结构体指针
 * @warning 每次仅可以发送四个电机的数据
 */
void FDCAN_Send_M3508(FDCAN_HandleTypeDef *hfdcan, M3508_Motor_Struct *Motor)
{
    uint8_t FDCAN_Tx_Buffer[8]={0};
    memset(FDCAN_Tx_Buffer, 0, 8);

    FDCAN_Tx_Buffer[0] = (Motor[0].Current_Output>>8) & 0xff;
    FDCAN_Tx_Buffer[1] = ((Motor+4)->Current_Output) & 0xff;
    FDCAN_Tx_Buffer[2] = (Motor[1].Current_Output>>8) & 0xff;
    FDCAN_Tx_Buffer[3] = ((Motor+4)->Current_Output) & 0xff;
    FDCAN_Tx_Buffer[4] = (Motor[2].Current_Output>>8) & 0xff;
    FDCAN_Tx_Buffer[5] = ((Motor+4)->Current_Output) & 0xff;
    FDCAN_Tx_Buffer[6] = (Motor[3].Current_Output>>8) & 0xff;
    FDCAN_Tx_Buffer[7] = ((Motor+4)->Current_Output) & 0xff;
    
    if(hfdcan->Instance == FDCAN1)
    {HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN1_TxFrame_m3508.TxHeader, FDCAN_Tx_Buffer);}
    else if(hfdcan->Instance == FDCAN2)
    {HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN2_TxFrame_m3508.TxHeader, FDCAN_Tx_Buffer);}
    else if(hfdcan->Instance == FDCAN3)
    {HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN3_TxFrame_m3508.TxHeader, FDCAN_Tx_Buffer);}

}

/**
 * @brief m3508电机FDCAN接收处理函数，在中断回调中调用
 * @param RxHeader FDCAN接收报文头指针
 * @param Data 接收数据指针
 * @param Motor 电机结构体指针
 */
void M3508_Return_Process(FDCAN_RxHeaderTypeDef* RxHeader, uint8_t Data[], M3508_Motor_Struct *Motor)
{
		uint8_t num = RxHeader->Identifier & 0x00F;
		Motor[num-1].Now_Angle = Data[0]<<8 | Data[1];
		Motor[num-1].Now_Speed = Data[2]<<8 | Data[3];
	  Motor[num-1].Now_Current = Data[4]<<8 | Data[5];
	
    int16_t angle_add_1,angle_add_2,angle_add_fin=0;

    if(Motor[num-1].Now_Angle > Motor[num-1].Angle_Last)
    {
        angle_add_1 = Motor[num-1].Now_Angle - Motor[num-1].Angle_Last;
        angle_add_2 = -8191 + Motor[num-1].Now_Angle - Motor[num-1].Angle_Last;
    }
    else
    {
        angle_add_1 = Motor[num-1].Now_Angle - Motor[num-1].Angle_Last;
        angle_add_2 = 8191 - Motor[num-1].Angle_Last + Motor[num-1].Now_Angle;
    }
    
    if(abs_int16(angle_add_1) > abs_int16(angle_add_2))
    {angle_add_fin=angle_add_2;}
    else
    {angle_add_fin=angle_add_1;}

    Motor[num-1].Angle_Sum += angle_add_fin;
    Motor[num-1].Angle_Last = Motor[num-1].Now_Angle;
    
    if(Motor[num-1].Initial_flag < 200)
    {
        Motor[num-1].Angle_Sum = 0;
        Motor[num-1].Initial_flag++;
    }
}

/**
 * @brief m3508电机参数初始化函数，可以初始化电机的速度环，位置环的kp,kd参数
*/
void M3508_Parameter_Init(M3508_Motor_Struct *Motor, float K_p_speed, float K_d_speed, float K_p_angle, float K_d_angle)
{
    Motor->M3508_Speed_PID.Kp = K_p_speed;
    Motor->M3508_Speed_PID.Kd = K_d_speed;
    Motor->M3508_Angle_PID.Kp = K_p_angle;
    Motor->M3508_Angle_PID.Kd = K_d_angle;
}
