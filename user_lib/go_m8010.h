#ifndef __GO_M8010_H
#define __GO_M8010_H

#include "main.h"
#include "usart.h"


/* === 用户可配置参数 ====================================================== */
#define GOM_DMA_RX_BUF_SIZE   64
#define GOM_RX_FRAME_SIZE     16   /* RIS_MotorData_t 字节数 */
#define REDUCTION_RATIO       6.33f  /* go-m8010电机减速比 */



#pragma pack(1)
/**
 * @brief 电机模式控制信息
 *
 */
typedef struct
{
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RIS_Mode_t;            // 控制模式 1Byte

/**
 * @brief 电机状态控制信息
 *
 */
typedef struct
{
    int16_t tor_des; // 期望转子扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望输出轴速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望输出轴位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)

} RIS_Comd_t; // 控制参数 12Byte

/**
 * @brief 电机状态反馈信息
 *
 */
typedef struct
{
    int16_t torque;      // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;       // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;         // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;         // 电机温度: -128~127°C
    uint8_t MError : 3;  // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12; // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;    // 保留位
} RIS_Fbk_t;             // 状态数据 11Byte

/**
 * @brief 控制数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Comd_t comd; // 电机期望数据 12Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_ControlData_t; // 主机控制命令     17Byte

/**
 * @brief 电机反馈数据包格式
 *
 */
typedef struct
{
    uint8_t head[2]; // 包头         2Byte
    RIS_Mode_t mode; // 电机控制模式  1Byte
    RIS_Fbk_t fbk;   // 电机反馈数据 11Byte
    uint16_t CRC16;  // CRC          2Byte

} RIS_MotorData_t; // 电机返回数据     16Byte

#pragma pack()

/**
 * @brief 电机指令结构体
 */
typedef struct
{
    unsigned short id;   // 电机ID，15代表广播数据包
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)

    RIS_ControlData_t motor_send_data;

} MotorCmd_t;

/**
 * @brief 电机反馈数据结构体
*/
typedef struct
{
    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 0:空闲 1:FOC控制 2:电机标定
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    int footForce;          // 足端力传感器原始数值

    int16_t Pos_init_flag;  // 位置初始化标志，0表示未初始化，一般用200
    float init_Pos;         // 电机上电时刻的位置，用于位置初始化

    uint16_t calc_crc;
    uint32_t timeout;       // 通讯超时 数量
    uint32_t bad_msg;       // CRC校验错误 数量
    RIS_MotorData_t motor_recv_data; // 电机接收数据结构体

} MotorData_t;

/* === 通信状态枚举 ======================================================== */
/**
 * @brief m8010通信状态枚举
*/
typedef enum {
    GOM_STATE_IDLE = 0,
    GOM_STATE_TX_BUSY,
    GOM_STATE_WAIT_RX,
    GOM_STATE_RX_READY,
    GOM_STATE_TIMEOUT,
    GOM_STATE_ERROR
} Gom485State_t;

#define GOM_MOTORS_PER_BUS  3   /* 每条 RS485 总线挂载的电机数，需根据实际改 */
#define GOM_MAX_BUSES       3   /* 最大 RS485 总线数，需根据实际改 */

/* === 单个电机结构体 ====================================================== */
typedef struct {
    MotorCmd_t  target;
    MotorData_t measure;
} GOM_Motor_t;

/* === 内部通讯变量 ====================================================== */
typedef struct {
    /* DMA 接收缓冲（同总线电机共享） */
    uint8_t  rx_dma_buf[GOM_DMA_RX_BUF_SIZE];
    volatile uint8_t rx_frame_ready;
    volatile uint8_t rx_data_len;
    volatile uint8_t tx_done_flag;
    volatile Gom485State_t g_state;
    uint32_t debug_cnt;
    uint32_t last_tick;
    uint32_t tx_start_tick;
    uint32_t rx_timeout_cnt;
    uint32_t rx_noise_cnt;
} GOM_Internal_t;


/* === RS485 总线实例（一个串口对应一个） ==================================== */
typedef struct {
    UART_HandleTypeDef *p_huart;
    TIM_HandleTypeDef  *p_htim;

    uint8_t             current_idx;   /* 当前通信的电机: 0,1,2 循环 */
    /* 电机指针数组（gom_bus_attach_motor 填充） */
    GOM_Motor_t motors[GOM_MOTORS_PER_BUS];
    
    GOM_Internal_t internal;    // 内部通讯变量
} Gom485Bus_t;



extern Gom485Bus_t m8010_bus_1;
extern Gom485Bus_t m8010_bus_2;
extern Gom485Bus_t m8010_bus_3;

/* ============================= 函数声明 ============================= */
/* ============================= 函数声明 ============================= */
/* ============================= 函数声明 ============================= */


void m8010_bus1_rx_mode(void);
void m8010_bus1_tx_mode(void);
void m8010_bus2_rx_mode(void);
void m8010_bus2_tx_mode(void);
void m8010_bus3_rx_mode(void);
void m8010_bus3_tx_mode(void);

Gom485Bus_t *m8010_find_by_huart(UART_HandleTypeDef *huart);
Gom485Bus_t *m8010_find_by_htim(TIM_HandleTypeDef *htim);



uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

void m8010_modify_data(MotorCmd_t *motor_s);
void m8010_extract_data(MotorData_t *motor_r);

void m8010_bus_init(Gom485Bus_t *bus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim);
Gom485State_t m8010_bus_poll(Gom485Bus_t *bus);

void m8010_485_timer_trigger(TIM_HandleTypeDef *htim);
void m8010_485_idle_callback(UART_HandleTypeDef *huart);

void m8010_parameter_init(MotorCmd_t *motor, uint16_t id, uint16_t mode, float K_P, float K_W);
void m8010_torque_control(GOM_Motor_t *motor, float T);
void m8010_velocity_control(GOM_Motor_t *motor, float W, float K_W);
void m8010_position_control(GOM_Motor_t *motor, float Pos, float K_P, float K_W);



#endif
