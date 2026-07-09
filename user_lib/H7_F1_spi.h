#ifndef __H7_F1_SPI_H
#define __H7_F1_SPI_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * H7 <-> F103C8T6 SPI 通信协议。
 *
 * 本文件用于 R2 的 H723 主控读取 F103C8T6 红外接收结果：
 * - H7 是 SPI Master，主动拉低 CS 并产生时钟；
 * - F103C8T6 是 SPI Slave，被 H7 轮询；
 * - 每次 SPI 固定交换 8 个字节。
 *
 * F1 发给 H7 的状态帧：
 *   [0] flags: bit7=在线/alive, bit0=有新的红外帧, bit1=曾发生覆盖
 *   [1] seq:   红外帧序号，每收到一帧加 1
 *   [2] ch:    红外接收通道，0=USART1, 1=USART2, 2=USART3
 *   [3] data:  应用数据字节，目前等于 frame[0]
 *   [4] frame0 红外模块输出的第 1 字节
 *   [5] frame1 红外模块输出的第 2 字节
 *   [6] frame2 红外模块输出的第 3 字节
 *   [7] xor:   [0..6] 异或校验
 *
 * H7 发给 F1 的请求帧：
 *   [0] command: GET_STATUS=0xB1, ACK_STATUS=0xB2
 *   [1] ack_seq: ACK 时填要确认的 seq，GET 时填 0
 *   [2..6] 保留，填 0
 *   [7] xor: [0..6] 异或校验
 */
#define H7_F1_SPI_FRAME_LEN          8U
#define H7_F1_SPI_IR_FRAME_LEN       3U

#define H7_F1_SPI_CMD_NOP            0x00U
#define H7_F1_SPI_CMD_GET_STATUS     0xB1U
#define H7_F1_SPI_CMD_ACK_STATUS     0xB2U

#define H7_F1_SPI_FLAG_ALIVE         0x80U
#define H7_F1_SPI_FLAG_NEW           0x01U
#define H7_F1_SPI_FLAG_OVERRUN       0x02U

typedef struct
{
    uint8_t flags;
    uint8_t seq;
    uint8_t ch;
    uint8_t data;
    uint8_t frame[H7_F1_SPI_IR_FRAME_LEN];
} H7_F1_SPI_StatusTypeDef;

/* 调试变量：可在 Keil Watch 里直接观察 */
extern volatile uint32_t h7_f1_spi_rx_ok_count;
extern volatile uint32_t h7_f1_spi_error_count;
extern volatile uint32_t h7_f1_spi_checksum_error_count;
extern volatile uint32_t h7_f1_spi_invalid_frame_count;
extern volatile uint8_t  h7_f1_spi_last_flags;
extern volatile uint8_t  h7_f1_spi_last_seq;
extern volatile uint8_t  h7_f1_spi_last_ch;
extern volatile uint8_t  h7_f1_spi_last_data;
extern volatile uint8_t  h7_f1_spi_last_frame[H7_F1_SPI_IR_FRAME_LEN];
extern volatile uint32_t h7_f1_spi_task_count;
extern volatile uint32_t h7_f1_spi_txrx_count;
extern volatile uint8_t  h7_f1_spi_debug_tx[H7_F1_SPI_FRAME_LEN];
extern volatile uint8_t  h7_f1_spi_debug_rx[H7_F1_SPI_FRAME_LEN];

/* 初始化：在 MX_SPI1_Init() 之后调用 H7_F1_SPI_Init(&hspi1) */
void H7_F1_SPI_Init(SPI_HandleTypeDef *hspi);

/* 周期任务：建议 5~20ms 调用一次；返回 1 表示本次读到了新的红外帧 */
uint8_t H7_F1_SPI_Task(void);

/* 底层接口：手动读状态 / 手动 ACK，一般只在调试时直接用 */
uint8_t H7_F1_SPI_ReadStatus(H7_F1_SPI_StatusTypeDef *status);
uint8_t H7_F1_SPI_AckStatus(uint8_t seq);

/* 上层读取接口 */
uint8_t H7_F1_SPI_IsLinkAlive(void);
uint8_t H7_F1_SPI_HasNewIr(void);
uint8_t H7_F1_SPI_GetIrChannel(void);
uint8_t H7_F1_SPI_GetIrData(void);
uint8_t H7_F1_SPI_GetIrSeq(void);
void    H7_F1_SPI_GetIrFrame(uint8_t frame[H7_F1_SPI_IR_FRAME_LEN]);
void    H7_F1_SPI_ClearNewIrFlag(void);

#ifdef __cplusplus
}
#endif

#endif /* __H7_F1_SPI_H */
