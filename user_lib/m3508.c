#include "m3508.h"

M3508_Motor_Struct m3508_fdcan1[8];
M3508_Motor_Struct m3508_fdcan2[8];
M3508_Motor_Struct m3508_fdcan3[8];

extern FDCAN_TxFrame_TypeDef FDCAN1_TxFrame_m3508;

/**
 * @brief m3508电机速度控制函数
 * @param motor 电机结构体指针
 * @param target_speed 目标速度
 * @param max_output 最大电流输出
 */
void m3508_control_speed(M3508_Motor_Struct *motor, int32_t target_speed, int32_t max_output)
{
    PID_Calculate(&(motor->M3508_Speed_PID), motor->Now_Speed, target_speed, max_output);
    motor->Current_Output = (int16_t)motor->M3508_Speed_PID.Output;
}

/**
 * @brief m3508电机角度速度双环控制函数
 * @param motor 电机结构体指针
 * @param target_angle 目标角度
 * @param max_output 最大电流输出
 */
void m3508_control_angle(M3508_Motor_Struct *motor, int32_t target_angle, int32_t max_output)
{
    PID_Calculate(&(motor->M3508_Angle_PID), motor->Now_Angle, target_angle, max_output);
    PID_Calculate(&(motor->M3508_Speed_PID), motor->Now_Speed, (int16_t)motor->M3508_Angle_PID.Output, max_output);
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
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN1_TxFrame_m3508.TxHeader, FDCAN_Tx_Buffer);
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
    
    if(Motor->Initial_flag < 200)
    {
        Motor->Angle_Sum = 0;
        Motor->Initial_flag++;
    }
}

