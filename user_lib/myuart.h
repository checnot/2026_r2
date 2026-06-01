#ifndef MYUART_H
#define MYUART_H

#include "main.h"
#include "usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "a1_motor.h"
#include "hi14.h"
#include "ht10a.h"
#include "radar.h"
#include "go_m8010.h"

// //引入uart1供 a1
// extern DMA_HandleTypeDef hdma_usart1_tx;
// extern DMA_HandleTypeDef hdma_usart1_rx;
// //引入uart9供 a1
// extern DMA_HandleTypeDef hdma_uart9_tx;
// extern DMA_HandleTypeDef hdma_uart9_rx;

// //--------注意：发现串口10存在接收问题，发送能够正常使用，暂时不清楚原因，先不使用uart10的接收功能，后续如果需要使用再进行调试--------
// //引入uart10供 vofa/anotc输出调试数据
// extern DMA_HandleTypeDef hdma_usart10_tx;
// extern DMA_HandleTypeDef hdma_usart10_rx;

// //引入uart8供 ht10a
// extern DMA_HandleTypeDef hdma_uart8_tx;
// extern DMA_HandleTypeDef hdma_uart8_rx;

// //引入uart6供 hi14
// extern DMA_HandleTypeDef hdma_usart6_tx;
// extern DMA_HandleTypeDef hdma_usart6_rx;

//引入uart5供 radar
extern DMA_HandleTypeDef hdma_uart5_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;

//引入uart3供 485bus1
extern DMA_HandleTypeDef hdma_uart3_tx;
extern DMA_HandleTypeDef hdma_uart3_rx;
//引入uart6供 485bus2
extern DMA_HandleTypeDef hdma_uart6_tx;
extern DMA_HandleTypeDef hdma_uart6_rx;
//引入uart4供 485bus3
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


#endif /* MYUART_H */
