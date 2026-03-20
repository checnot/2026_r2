#ifndef MYUART_H
#define MYUART_H

#include "main.h"
#include "usart.h"
#include "freeRTOS.h"
#include "task.h"
#include "a1_motor.h"


//引入uart1供a1
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart1_rx;
//引入uart9供a1
extern DMA_HandleTypeDef hdma_uart9_tx;
extern DMA_HandleTypeDef hdma_uart9_rx;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);


#endif /* MYUART_H */
