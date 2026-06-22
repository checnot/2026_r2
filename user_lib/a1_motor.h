#ifndef A1_MOTOR_H
#define A1_MOTOR_H

#include "main.h" 
#include "usart.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "pid.h"

#include "own_lib.h"

#define RX_LEN1 78
#define reduction_ratio 9.0f // A1减速比



#pragma pack(1)


// 发送用单个数据数据结构
typedef union{
        int32_t           L;
        uint8_t       u8[4];
       uint16_t      u16[2];
       uint32_t         u32;
          float           F;
}COMData32;

typedef struct {
	// 定义 数据包头
  unsigned char  start[2];     // 包头
	unsigned char  motorID;      // 电机ID  0,1,2,3 ...   0xBB 表示向所有电机广播（此时无返回）
	unsigned char  reserved;
}COMHead;

#pragma pack()

#pragma pack(1)


typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
	// 定义 数据
    uint8_t  mode;        // 关节模式选择
    uint8_t  ModifyBit;   // 电机控制参数修改位
    uint8_t  ReadBit;     // 电机控制参数发送位
    uint8_t  reserved;

    COMData32  Modify;     // 电机参数修改 的数据 
    //实际给FOC的指令力矩为：
    //K_P*delta_Pos + K_W*delta_W + T
    q15_t     T;      // 期望关节的输出力矩（电机本身的力矩）x256, 7 + 8 描述
    
    //原浮点数*256（2的8次方）后取整，然后赋值给T//
    
    q15_t     W;      // 期望关节速度 （电机本身的速度） x128,       8 + 7描述	
    int32_t   Pos;      // 期望关节位置 x 16384/6.2832, 14位编码器（主控0点修正，电机关节还是以编码器0点为准）

    q15_t    K_P;      // 关节刚度系数 x2048  4+11 描述
    q15_t    K_W;      // 关节速度系数 x1024  5+10 描述

    uint8_t LowHzMotorCmdIndex;     // 电机低频率控制命令的索引, 0-7, 分别代表LowHzMotorCmd中的8个字节
    uint8_t LowHzMotorCmdByte;      // 电机低频率控制命令的字节
	
     COMData32  Res[1];    // 通讯 保留字节  用于实现别的一些通讯内容
	
}MasterComdV3;   // 加上数据包的包头 和CRC 34字节

typedef struct {
	// 定义 电机控制命令数据包	
    COMHead head;    
    MasterComdV3 Mdata;
    COMData32 CRCdata;
}MasterComdDataV3;//返回数据


#pragma pack()

#pragma pack(1)

typedef struct {  // 以 4个字节一组排列 ，不然编译器会凑整
    // 定义 数据
    uint8_t  mode;        // 当前关节模式
    uint8_t  ReadBit;     // 电机控制参数修改     是否成功位
    int8_t  Temp;        // 电机当前平均温度   
    uint8_t  MError;      // 电机错误 标识
 
    COMData32  Read;     // 读取的当前 电机 的控制数据 
    int16_t     T;      // 当前实际电机输出力矩       7 + 8 描述

    int16_t     W;      // 当前实际电机速度（高速）   8 + 7 描述
    float      LW;      // 当前实际电机速度（低速）   

    int16_t     W2;      // 当前实际关节速度（高速）   8 + 7 描述
    float      LW2;      // 当前实际关节速度（低速）   

    int16_t    Acc;           // 电机转子加速度       15+0 描述  惯量较小
    int16_t    OutAcc;        // 输出轴加速度         12+3 描述  惯量较大
		 
    int32_t   Pos;      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）
    int32_t   Pos2;     // 关节编码器位置(输出编码器)

    int16_t     gyro[3];  // 电机驱动板6轴传感器数据
    int16_t     acc[3];   

    // 力传感器的数据   
    int16_t     Fgyro[3];  //  
    int16_t     Facc[3];
    int16_t     Fmag[3];
    uint8_t     Ftemp;     // 8位表示的温度  7位（-28~100度）  1位0.5度分辨率
    
    int16_t     Force16;   // 力传感器高16位数据
    int8_t      Force8;    // 力传感器低8位数据
		
    uint8_t     FError;    //  足端传感器错误标识
		
    int8_t      Res[1];    // 通讯 保留字节
	
}ServoComdV3;  // 加上数据包的包头 和CRC 78字节（4+70+4）

typedef struct {
    // 定义 电机控制命令数据包	
    COMHead        head;
    ServoComdV3      Mdata;

    COMData32    CRCdata;

}ServoComdDataV3;	//发送数据


#pragma pack()

//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00 00 00 
//  00 00 00
// 数据包默认初始化 
// 主机发送的数据包 
/*
                 Tx485Data[_FR][i].head.start[0] = 0xFE ;     Tx485Data[_FR][i].head.start[1] = 0xEE; // 数据包头					 
				 Tx485Data[_FR][i].Mdata.ModifyBit = 0xFF;    Tx485Data[_FR][i].Mdata.mode = 0;   // 默认不修改数据 和 电机的默认工作模式				
				 Tx485Data[_FR][i].head.motorID = i;    0                                          // 目标电机标号
				 Tx485Data[_FR][i].Mdata.T = 0.0f;                           // 默认目标关节输出力矩                      motor1.Extra_Torque = motorRxData[1].Mdata.T*0.390625f;     // N.M  转化为 N.CM   IQ8描述
				 Tx485Data[_FR][i].Mdata.Pos = 0x7FE95C80;                   // 默认目标关节位置  不启用位置环          14位分辨率 
				 Tx485Data[_FR][i].Mdata.W = 16000.0f;                       // 默认目标关节速度  不启用速度环          1+8+7描述     motor1.Target_Speed =  motorRxData[1].Mdata.W*0.0078125f;   // 单位 rad/s	       IQ7描述
				 Tx485Data[_FR][i].Mdata.K_P = (q15_t)(0.6f*(1<<11));        // 默认关节刚度系数   4+11 描述                     motor1.K_Pos = ((float)motorRxData[1].Mdata.K_P)/(1<<11);      // 描述刚度的通讯数据格式  4+11
				 Tx485Data[_FR][i].Mdata.K_W = (q15_t)(1.0f*(1<<10));        // 默认关节速度系数   5+10 描述                    motor1.K_Speed = ((float)motorRxData[1].Mdata.K_W)/(1<<10);    // 描述阻尼的通讯数据格式  5+10
*/ 

typedef struct {
	// 定义 发送格式化数据
    MasterComdDataV3  motor_send_data;  //电机控制数据结构体，详见motor_msg.h
	int hex_len;                    //发送的16进制命令数组长度, 34
    long long send_time;            //The time that message was sent发送该命令的时间, 微秒(us)
    // 待发送的各项数据
    unsigned short id;              //电机ID，0代表全部电机
    unsigned short mode;            //0:空闲, 5:开环转动, 10:闭环FOC控制
    //实际给FOC的指令力矩为：
    //K_P*delta_Pos + K_W*delta_W + T
    float T;                        //期望关节的输出力矩（电机本身的力矩）（Nm）
    float W;                        //期望关节速度（电机本身的速度）(rad/s)
    float Pos;                      //期望关节位置（rad）
    float K_P;                      //关节刚度系数
    float K_W;                      //关节速度系数
}MOTOR_send;//A1电机发送数据结构体

typedef struct
{
    // 定义 接收数据
    ServoComdDataV3 motor_recv_data;     //电机接收数据结构体，详见motor_msg.h
    int hex_len;                    //接收的16进制命令数组长度, 78
    long long resv_time;            //接收该命令的时间, 微秒(us)
    int correct;                   //接收数据是否完整（1完整，0不完整）
    //解读得出的电机数据
    unsigned char motor_id;         //电机ID
    unsigned char mode;             //0:空闲, 5:开环转动, 10:闭环FOC控制
    int Temp;                       //温度
    unsigned char MError;           //错误码

    float T;                        // 当前实际电机输出力矩
    float W;                        // 当前实际电机速度（高速）
    float LW;                       // 当前实际电机速度（低速）
    int Acc;                      // 电机转子加速度
    float Pos;                      // 当前电机位置（主控0点修正，电机关节还是以编码器0点为准）

    float gyro[3];                  // 电机驱动板6轴传感器数据
    float acc[3];

}MOTOR_recv;//A1电机接收数据结构体

typedef struct
{
    float T;
    float W;
    float Pos;//注意：A1从上电限位处开始计数的输出轴位置(rad)
} A1_Motor_Recv_Struct;

typedef struct
{
    float T;
    float W;
    float Pos;//从上电时刻电机编码器随机位置开始计数的位置

    float Start_Position;//不参与发送过程，仅在位置控制时使用，记录初始位置
    uint8_t Change_Start_Position;//不参与发送过程，仅在位置控制时使用，标记是否需要更新初始位置
} A1_Motor_Send_Struct;

typedef struct
{
	uint8_t RX_flag:1;
	uint16_t RX_Size;
	uint8_t RX_pData[RX_LEN1];
} A1_USART_RECEIVETYPE;//用于A1电机的串口接收结构体，存储原始回传数据


typedef struct
{
    uint16_t id;
    uint16_t mode;
    float K_p;
    float K_w;
    A1_Motor_Send_Struct target;
    A1_Motor_Recv_Struct measure;
    int16_t Pos_init_flag;
    float init_Pos;
} A1_Motor_Struct;//A1电机总结构体


extern A1_Motor_Struct A1_debug_485_1[3];
extern A1_Motor_Struct A1_debug_485_2[3];
// extern MOTOR_send a1_motor_send;
extern MOTOR_recv a1_motor_recv_6[3];
extern MOTOR_recv a1_motor_recv_9[3];
extern A1_USART_RECEIVETYPE a1_usart6_raw_recv;
extern A1_USART_RECEIVETYPE a1_usart9_raw_recv;

int modify_data(MOTOR_send* motor);    //将数据处理为stm32需求的格式
uint32_t crc32_core(uint32_t*, uint32_t);    //待校验数组指针，数组长度（余数舍去）
void A1_Motor_Send_Data(A1_Motor_Struct *p_send_data, UART_HandleTypeDef *huart);
void A1_Motor_DataProcess(uint8_t* pData, MOTOR_recv* recv);

void A1_Motor_Torque_Control(A1_Motor_Struct *A1_motor, float target_torque);
void A1_Motor_Velocity_Control(A1_Motor_Struct *A1_motor, float target_velocity, float target_kw);
void A1_Motor_Position_Control(A1_Motor_Struct *A1_motor, float target_position, float target_kw, float target_kp);
//--------弃用
// void A1_Motor_Position_Advanced_Control(A1_Motor_Struct *A1_motor, float target_kw, float start_position, float target_position, float start_time, float ETA, float accel_ratio, float decel_ratio);

void A1_Motor_Position_Improved_Control(A1_Motor_Struct *A1_motor, float target_kw, float now_position, float start_position, float target_position, float ETA);
void A1_Motor_Parameter_Init(A1_Motor_Struct *A1_motor, uint16_t id, uint16_t mode, float K_p, float K_w);


#endif /* A1_MOTOR_H */
