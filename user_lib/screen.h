#ifndef __SCREEN_H
#define __SCREEN_H

#include "main.h"
#include <stdint.h>

/* ===================== 方块状态定义 ===================== */

#define EMPTY_KFS  0U    // 空
#define R1_KFS     1U    // R1
#define R2_KFS     2U    // R2
#define FAKE_KFS   3U    // F

extern uint8_t merlin[12];

/* ===================== 串口屏接口 ===================== */

void Screen_InitDisplay(void);
void Screen_GotoPage0(void);

void Screen_UpdateCell(uint8_t index, uint8_t state);
void Screen_UpdateAllCells(void);
void Screen_ClearAllCells(void);

void Screen_UART7_StartReceiveToIdleDMA(void);

void Screen_TestDisplay(void);

/* ===================== 梅林区业务接口 ===================== */

void Merlin_SetBlock(uint8_t index, uint8_t state);

/* ===================== UART7 回调合并接口 =====================
 * 你工程中 HAL_UART_xxxCallback 已经在 myuart.c 里定义，
 * 所以 screen.c 不重复定义 HAL 全局回调，由 myuart.c 转调下面函数。
 */
void Screen_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void Screen_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void Screen_UART_ErrorCallback(UART_HandleTypeDef *huart);

#endif
