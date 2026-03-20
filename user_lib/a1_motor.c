#include "a1_motor.h"

#define reduction_ratio 9.0f // A1减速比


A1_Motor_Struct A1_debug[4];//调试用A1电机数组
MOTOR_send a1_motor_send;   //电机发送公共缓冲区
MOTOR_recv a1_motor_recv_1[3]; //属于串口1的a1电机接收缓冲区
MOTOR_recv a1_motor_recv_9[3]; //属于串口9的a1电机接收缓冲区

A1_USART_RECEIVETYPE a1_usart1_raw_recv;
A1_USART_RECEIVETYPE a1_usart9_raw_recv;



/**
 * @brief 用于A1电机的CRC32校验计算函数
*/
uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
	uint32_t xbit = 0;
	uint32_t data = 0;
	uint32_t bits = 0;
	uint32_t i = 0;
	uint32_t CRC32 = 0xFFFFFFFF;
	const uint32_t dwPolynomial = 0x04c11db7;
	
	for (i = 0; i < len; i++)
	{
			xbit = (uint32_t)1 << 31;
			data = ptr[i];
		for (bits = 0; bits < 32; bits++)
		{
			if (CRC32 & 0x80000000)
			{
				CRC32 <<= 1;
				CRC32 ^= dwPolynomial;
			}
			else
				CRC32 <<= 1;
			if (data & xbit)
				CRC32 ^= dwPolynomial;
			xbit >>= 1;
		}
	}
	return CRC32;
}

/**
 * @brief 用于将目标数据发送至A1的数据处理函数，将电机控制数据处理为stm32需求的格式
*/
int modify_data(MOTOR_send* motor)
{
	motor->motor_send_data.head.start[0] = 0xFE;
	motor->motor_send_data.head.start[1] = 0xEE;
	motor->motor_send_data.head.motorID = motor->id;
	
	motor->motor_send_data.Mdata.mode = motor ->mode;
	
    motor->motor_send_data.Mdata.ModifyBit = 0xff;
   // motor->motor_send_data.Mdata.ReadBit = 0x01;
    
	motor->motor_send_data.Mdata.T = (int16_t)((motor ->T)*256);
	motor->motor_send_data.Mdata.W = (int16_t)((motor ->W)*128);
	motor->motor_send_data.Mdata.Pos = (motor ->Pos)*16384/6.2832f;
	motor->motor_send_data.Mdata.K_P = (int16_t)((motor ->K_P)*2048);
	motor->motor_send_data.Mdata.K_W = (int16_t)((motor ->K_W)*1024);
	
	motor->motor_send_data.CRCdata.u32 = crc32_core((uint32_t*)motor,7);
	
	motor ->hex_len = 34;

	return 0;
}

/**
 * @brief 使用时调用该函数发送A1电机控制数据，一般在定时器里调用
 * @warning 注意只能使用阻塞式发送
*/
void A1_Motor_Send_Data(A1_Motor_Struct *p_send_data, UART_HandleTypeDef *huart)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护

    a1_motor_send.id = p_send_data->id;     // motor ID
    a1_motor_send.mode = p_send_data->mode; // switch to servo mode
    a1_motor_send.T = p_send_data->target.T;       // Nm, T<255.9
    a1_motor_send.W = p_send_data->target.W * reduction_ratio;       // rad/s, W<511.9
    a1_motor_send.Pos = p_send_data->target.Pos * reduction_ratio;   // rad, Pos<131071.9
    a1_motor_send.K_P = p_send_data->K_p;   // K_P<31.9     value should around 0.1
    a1_motor_send.K_W = p_send_data->K_w;   // K_W<63.9     value should around 3

    modify_data(&a1_motor_send);
    
    GPIO_TypeDef *gpio = GPIOH;
    uint16_t gpio_pin = GPIO_PIN_3;
    
    if(huart == &huart1)
    {
        gpio = GPIOD;
        gpio_pin = GPIO_PIN_8;
    }
    if(huart == &huart9)
    {
        gpio = GPIOC;
        gpio_pin = GPIO_PIN_8;
    }
    
    HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_SET);
	HAL_UART_Transmit(huart, (uint8_t *)&a1_motor_send, 34, 0xFFF);
//		HAL_UART_Transmit_DMA(huart, (uint8_t *)&a1_motor_send, 34);
    HAL_GPIO_WritePin(gpio, gpio_pin, GPIO_PIN_RESET);

	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

/**
 * @brief 用于A1电机回传数据处理，从4个uint8_t数据中还原float数据
*/
float get_float_from_4u8(unsigned char *p)
{
	float a;
	unsigned char *r;
	r=(unsigned char*)&a;
	*r=p[0];
	r++;
	*r=p[1];
	r++;
	*r=p[2];
	r++;
	*r=p[3];
	return(a);
}

/**
 * @brief A1电机回传数据处理函数，在串口中断中调用
*/
void A1_Motor_DataProcess(uint8_t* pData, MOTOR_recv* recv)//78字节
{
	//motor_recv = (MOTOR_recv)Usart1Type1.RX_pData;
	//memcpy(&motor_recv, Usart1Type1.RX_pData, 78);
	if(pData[0] == 0xFE && pData[1] == 0xEE)
	{
		int id = pData[2];
//		if((id != 0) && (id != 1) && (id != 2))
//			id = 2;
//        int id = 0;
		recv[id].motor_recv_data.head.start[0] = pData[0];
		recv[id].motor_recv_data.head.start[1] = pData[1];
		recv[id].motor_recv_data.head.motorID = pData[2];
		recv[id].motor_recv_data.head.reserved = pData[3];
		recv[id].motor_recv_data.Mdata.mode = pData[4];
		recv[id].motor_recv_data.Mdata.ReadBit = pData[5];
		recv[id].motor_recv_data.Mdata.Temp = pData[6];
		recv[id].motor_recv_data.Mdata.MError = pData[7];
		recv[id].motor_recv_data.Mdata.Read.u8[0] = pData[8];
		recv[id].motor_recv_data.Mdata.Read.u8[1] = pData[9];
		recv[id].motor_recv_data.Mdata.Read.u8[2] = pData[10];
		recv[id].motor_recv_data.Mdata.Read.u8[3] = pData[11];
		recv[id].motor_recv_data.Mdata.T = (pData[12] | pData[13] << 8);
		recv[id].motor_recv_data.Mdata.W = (pData[14] | pData[15] << 8);
		recv[id].motor_recv_data.Mdata.LW = get_float_from_4u8(&pData[16]);
		recv[id].motor_recv_data.Mdata.W2 = (pData[20] | pData[21] << 8);
		recv[id].motor_recv_data.Mdata.LW2 = get_float_from_4u8(&pData[22]);
		recv[id].motor_recv_data.Mdata.Acc = (pData[26] | pData[27] << 8);
		recv[id].motor_recv_data.Mdata.OutAcc = (pData[28] | pData[29] << 8);
		recv[id].motor_recv_data.Mdata.Pos = (pData[30] | pData[31] << 8 | pData[32] << 16 | pData[33] << 24);
		recv[id].motor_recv_data.Mdata.Pos2 = (pData[34] | pData[35] << 8 | pData[36] << 16 | pData[37] << 24);
		recv[id].motor_recv_data.Mdata.gyro[0] = (pData[38] | pData[39] << 8);
		recv[id].motor_recv_data.Mdata.gyro[1] = (pData[40] | pData[41] << 8);
		recv[id].motor_recv_data.Mdata.gyro[2] = (pData[42] | pData[43] << 8);
		recv[id].motor_recv_data.Mdata.acc[0] = (pData[44] | pData[45] << 8);
		recv[id].motor_recv_data.Mdata.acc[1] = (pData[46] | pData[47] << 8);
		recv[id].motor_recv_data.Mdata.acc[2] = (pData[48] | pData[49] << 8);
		recv[id].motor_recv_data.Mdata.Fgyro[0] = (pData[50] | pData[51] << 8);
		recv[id].motor_recv_data.Mdata.Fgyro[1] = (pData[52] | pData[53] << 8);
		recv[id].motor_recv_data.Mdata.Fgyro[2] = (pData[54] | pData[55] << 8);
		recv[id].motor_recv_data.Mdata.Facc[0] = (pData[56] | pData[57] << 8);
		recv[id].motor_recv_data.Mdata.Facc[1] = (pData[58] | pData[59] << 8);
		recv[id].motor_recv_data.Mdata.Facc[2] = (pData[60] | pData[61] << 8);
		recv[id].motor_recv_data.Mdata.Fmag[0] = (pData[62] | pData[63] << 8);
		recv[id].motor_recv_data.Mdata.Fmag[1] = (pData[64] | pData[65] << 8);
		recv[id].motor_recv_data.Mdata.Fmag[2] = (pData[66] | pData[67] << 8);
		recv[id].motor_recv_data.Mdata.Ftemp = pData[68];
		recv[id].motor_recv_data.Mdata.Force16 = (pData[69] | pData[70] << 8);
		recv[id].motor_recv_data.Mdata.Force8 = pData[71];
		recv[id].motor_recv_data.Mdata.FError = pData[72];
		recv[id].motor_recv_data.Mdata.Res[0] = pData[73];
		recv[id].motor_recv_data.CRCdata.u8[0] = pData[74];
		recv[id].motor_recv_data.CRCdata.u8[1] = pData[75];
		recv[id].motor_recv_data.CRCdata.u8[2] = pData[76];
		recv[id].motor_recv_data.CRCdata.u8[3] = pData[77];
        
	}
}

/**
 * @brief 对A1电机进行单力矩控制
 * @param target_torque 电机目标力矩，单位Nm
*/
void A1_Motor_Torque_Control(A1_Motor_Struct *A1_motor, float target_torque)
{
	A1_motor->mode = 10; //程控模式
	A1_motor->target.T = target_torque;

	A1_motor->target.W = 0.0f;
	A1_motor->K_w = 0.0f;
	A1_motor->target.Pos = 0.0f;
	A1_motor->K_p = 0.0f;
}

/**
 * @brief 对A1电机进行单速度控制
 * @param target_velocity 电机目标转速，单位rad/s
 * @param target_kw 电机速度环比例系数
*/
void A1_Motor_Velocity_Control(A1_Motor_Struct *A1_motor, float target_velocity, float target_kw)
{
	A1_motor->mode = 10; //程控模式
	A1_motor->target.W = target_velocity;
	A1_motor->K_w = target_kw;

	A1_motor->target.T = 0.0f;
	A1_motor->target.Pos = 0.0f;
	A1_motor->K_p = 0.0f;
}

/**
 * @brief 对A1电机进行单位置控制
 * @param target_position 电机目标位置，单位rad，以启动时位置为零点
 * @param target_kp 等效于电机位置环比例系数
 * @param target_kw 等效于电机位置环微分系数
 * 
 * @warning target_position为：以机构上电时刻初始位置为零点的目标位置，非电机系下的目标位置，在函数内部加上了初始位置偏移以方便计算
*/
void A1_Motor_Position_Control(A1_Motor_Struct *A1_motor, float target_position, float target_kw, float target_kp)
{
	A1_motor->mode = 10; //程控模式
	A1_motor->target.Pos = target_position + A1_motor->init_Pos;
    A1_motor->K_w = target_kw;
    A1_motor->K_p = target_kp;

	A1_motor->target.W = 0.0f;
	A1_motor->target.T = 0.0f;
	
}
