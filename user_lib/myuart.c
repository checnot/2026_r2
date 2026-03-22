#include "myuart.h"


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护
	
//usart1供a1电机回传数据
	if(huart->Instance == USART1) 
	{
		HAL_UART_DMAStop(huart);
		A1_Motor_DataProcess(a1_usart1_raw_recv.RX_pData, a1_motor_recv_1);
		if(a1_usart1_raw_recv.RX_pData[0] == 0xFE && a1_usart1_raw_recv.RX_pData[1] == 0xEE)
		{
            uint8_t id = a1_usart1_raw_recv.RX_pData[2];
            if(id == 0 || id == 1)
            {
                //等待一段时间使电机位置稳定，再进行位置初始化
                if(A1_debug[id].Pos_init_flag < 200)
                {
                    A1_debug[id].Pos_init_flag +=1 ;
                    A1_debug[id].init_Pos = a1_motor_recv_1[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f;
                }
                A1_debug[id].measure.Pos = a1_motor_recv_1[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f - A1_debug[id].init_Pos;
                A1_debug[id].measure.W = a1_motor_recv_1[id].motor_recv_data.Mdata.W / 128.0f / 9.0f;
                A1_debug[id].measure.T = a1_motor_recv_1[id].motor_recv_data.Mdata.T / 256.0f;
            }
		}

		memset(a1_usart1_raw_recv.RX_pData, 0, sizeof(a1_usart1_raw_recv.RX_pData));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a1_usart1_raw_recv.RX_pData, RX_LEN1);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
	}
//usart9供a1电机回传数据
    if(huart->Instance == UART9)
    {
        HAL_UART_DMAStop(huart);
        A1_Motor_DataProcess(a1_usart9_raw_recv.RX_pData, a1_motor_recv_9);
        if(a1_usart9_raw_recv.RX_pData[0] == 0xFE && a1_usart9_raw_recv.RX_pData[1] == 0xEE)
        {
            uint8_t id = a1_usart9_raw_recv.RX_pData[2];
            if(id == 0 || id == 1)
            {
                //等待一段时间使电机位置稳定，再进行位置初始化
                if(A1_debug[id+2].Pos_init_flag < 200)
                {
                    A1_debug[id+2].Pos_init_flag +=1 ;
                    A1_debug[id+2].init_Pos = a1_motor_recv_9[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f;
                }
                A1_debug[id+2].measure.Pos = a1_motor_recv_9[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f - A1_debug[id+2].init_Pos;
                A1_debug[id+2].measure.W = a1_motor_recv_9[id].motor_recv_data.Mdata.W / 128.0f / 9.0f;
                A1_debug[id+2].measure.T = a1_motor_recv_9[id].motor_recv_data.Mdata.T / 256.0f;
            }
        }

		memset(a1_usart9_raw_recv.RX_pData, 0, sizeof(a1_usart9_raw_recv.RX_pData));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart9, a1_usart9_raw_recv.RX_pData, RX_LEN1);
		__HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
    }
//usart10供ht10a回传数据
    if(huart->Instance == USART10)
    {
		HT10A_process(ht10a_buffer);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, ht10a_buffer, 30);
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx,DMA_IT_HT);
    }
//uart8供hi14回传数据
    if(huart->Instance == UART8)
    {
		memset(uart_rx_buf, 0, sizeof(uart_rx_buf));
		
		uart_rx_index = 0;
		for(uint16_t i = 0; i < Size; i++)
		{
			uart_rx_buf[uart_rx_index++] = dma_rx_buf[i];
	  	}
		process_data();
		uart_rx_index = 0;

		memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart8, dma_rx_buf, sizeof(dma_rx_buf));
		__HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    }

	taskEXIT_CRITICAL_FROM_ISR(status_value); // 退出临界段
}

/**
 * @brief UART错误回调函数，在串口发生错误时调用
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) 
    {
        memset(a1_usart1_raw_recv.RX_pData, 0, sizeof(a1_usart1_raw_recv.RX_pData));
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a1_usart1_raw_recv.RX_pData, RX_LEN1);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    if(huart->Instance == UART9)
    {
        memset(a1_usart9_raw_recv.RX_pData, 0, sizeof(a1_usart9_raw_recv.RX_pData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart9, a1_usart9_raw_recv.RX_pData, RX_LEN1);
        __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
    }
}

