#include "myuart.h"
#include "screen.h"






/**
 * ====================================================================================
 * 
 * @brief 每次修改串口相关时，此处应该记录修改之后的串口配置，以便后续维护
 * 
 * @note 目前涉及的串口有 3，6，4，5，8，7，10，2
 * 
 * @note 用于 RS485 通信：3-4851 6-4952 4-4853
 * @note 用于雷达回传：5
 * @note 用于矩阵键盘回传：8 2
 * @note 用于调试输出：7
 * @note 用于 hi14 回传：10
 * 
 * ====================================================================================
 * 
 */









void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    uint32_t status_value;
    status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护

    // usart1供 a1 电机回传数据
    // if (huart->Instance == USART1)
    // {
    //     HAL_UART_DMAStop(huart);
    //     A1_Motor_DataProcess(a1_usart1_raw_recv.RX_pData, a1_motor_recv_1);
    //     if (a1_usart1_raw_recv.RX_pData[0] == 0xFE && a1_usart1_raw_recv.RX_pData[1] == 0xEE)
    //     {
    //         uint8_t id = a1_usart1_raw_recv.RX_pData[2];
    //         if (id == 0 || id == 1)
    //         {
    //             // 等待一段时间使电机位置稳定，再进行位置初始化
    //             if (A1_debug_485_1[id].Pos_init_flag < 200)
    //             {
    //                 A1_debug_485_1[id].Pos_init_flag += 1;
    //                 A1_debug_485_1[id].init_Pos = a1_motor_recv_1[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / reduction_ratio;
    //             }
    //             A1_debug_485_1[id].measure.Pos = a1_motor_recv_1[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / reduction_ratio - A1_debug_485_1[id].init_Pos;
    //             A1_debug_485_1[id].measure.W = a1_motor_recv_1[id].motor_recv_data.Mdata.W / 128.0f / reduction_ratio;
    //             A1_debug_485_1[id].measure.T = a1_motor_recv_1[id].motor_recv_data.Mdata.T / 256.0f;
    //         }
    //     }
    //     memset(a1_usart1_raw_recv.RX_pData, 0, sizeof(a1_usart1_raw_recv.RX_pData));
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a1_usart1_raw_recv.RX_pData, RX_LEN1);
    //     __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    // }
    // // usart9供 a1 电机回传数据
    // if (huart->Instance == UART9)
    // {
    //     HAL_UART_DMAStop(huart);
    //     A1_Motor_DataProcess(a1_usart9_raw_recv.RX_pData, a1_motor_recv_9);
    //     if (a1_usart9_raw_recv.RX_pData[0] == 0xFE && a1_usart9_raw_recv.RX_pData[1] == 0xEE)
    //     {
    //         uint8_t id = a1_usart9_raw_recv.RX_pData[2];
    //         if (id == 0 || id == 1)
    //         {
    //             // 等待一段时间使电机位置稳定，再进行位置初始化
    //             if (A1_debug_485_2[id].Pos_init_flag < 200)
    //             {
    //                 A1_debug_485_2[id].Pos_init_flag += 1;
    //                 A1_debug_485_2[id].init_Pos = a1_motor_recv_9[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f;
    //             }
    //             A1_debug_485_2[id].measure.Pos = a1_motor_recv_9[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / 9.0f - A1_debug_485_2[id].init_Pos;
    //             A1_debug_485_2[id].measure.W = a1_motor_recv_9[id].motor_recv_data.Mdata.W / 128.0f / 9.0f;
    //             A1_debug_485_2[id].measure.T = a1_motor_recv_9[id].motor_recv_data.Mdata.T / 256.0f;
    //         }
    //     }
    //     memset(a1_usart9_raw_recv.RX_pData, 0, sizeof(a1_usart9_raw_recv.RX_pData));
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart9, a1_usart9_raw_recv.RX_pData, RX_LEN1);
    //     __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
    // }
    // usart8供 ht10a 回传数据
    // if (huart->Instance == UART8)
    // {
    //     HT10A_process(ht10a_buffer);
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart8, ht10a_buffer, sizeof(ht10a_buffer));
    //     __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    // }

    if (huart->Instance == USART6)
    {
        HAL_UART_DMAStop(huart);
        A1_Motor_DataProcess(a1_usart6_raw_recv.RX_pData, a1_motor_recv_6);
        if (a1_usart6_raw_recv.RX_pData[0] == 0xFE && a1_usart6_raw_recv.RX_pData[1] == 0xEE)
        {
            uint8_t id = a1_usart6_raw_recv.RX_pData[2];
            if (id == 0 || id == 1)
            {
                // 等待一段时间使电机位置稳定，再进行位置初始化
                if (A1_debug_485_2[id].Pos_init_flag < 200)
                {
                    A1_debug_485_2[id].Pos_init_flag += 1;
                    A1_debug_485_2[id].init_Pos = a1_motor_recv_6[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / reduction_ratio;
                }
                A1_debug_485_2[id].measure.Pos = a1_motor_recv_6[id].motor_recv_data.Mdata.Pos * 2 * PI / 16384.0f / reduction_ratio - A1_debug_485_2[id].init_Pos;
                A1_debug_485_2[id].measure.W = a1_motor_recv_6[id].motor_recv_data.Mdata.W / 128.0f / reduction_ratio;
                A1_debug_485_2[id].measure.T = a1_motor_recv_6[id].motor_recv_data.Mdata.T / 256.0f;
            }
        }
        memset(a1_usart6_raw_recv.RX_pData, 0, sizeof(a1_usart6_raw_recv.RX_pData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, a1_usart6_raw_recv.RX_pData, RX_LEN1);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }

    // uart5-供雷达(0x20)+摄像头网格(0x10)接收 (HEU 单串口多路复用)
    if (huart->Instance == UART5)
    {
         Data_process_String(( char *)Rec_Data);
        //Radar_ProcessBuffer(Rec_Data, Size);    /* 雷达遥测 src=0x20 */
        Camera_ProcessBuffer(Rec_Data, Size);   /* 摄像头网格 src=0x10 */
        memset(Rec_Data, 0, sizeof(Rec_Data));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rec_Data, sizeof(Rec_Data));
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    // usart10-供hi14回传
    if (huart->Instance == USART10)
    {
        memset(uart_rx_buf, 0, sizeof(uart_rx_buf));

        uart_rx_index = 0;
        for (uint16_t i = 0; i < Size; i++)
        {
            uart_rx_buf[uart_rx_index++] = dma_rx_buf[i];
        }
        process_data();
        uart_rx_index = 0;

        memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buf, sizeof(dma_rx_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
    }

    // uart8-供矩阵键盘回传
    if (huart->Instance == UART8)
    {
        keyboard_process();
        memset(keyboard_buffer, 0, sizeof(keyboard_buffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart8, keyboard_buffer, sizeof(keyboard_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    }

    // uart2-供第二矩阵键盘回传
    if (huart->Instance == USART2)
    {
        keyboard_2_process();
        memset(keyboard_2_buffer, 0, sizeof(keyboard_2_buffer));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart2, keyboard_2_buffer, sizeof(keyboard_2_buffer));
        __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    }

    // uart7 - serial screen
    if (huart->Instance == UART7)
    {
        Screen_UART_RxEventCallback(huart, Size);
    }
    taskEXIT_CRITICAL_FROM_ISR(status_value); // 退出临界段
}

/**
 * @brief UART错误回调函数，在串口发生错误时调用
 **/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint32_t status_value;
    status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护

    __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_FEF | UART_CLEAR_PEF);
    if (huart->Instance == USART6)
    {
        memset(a1_usart6_raw_recv.RX_pData, 0, sizeof(a1_usart6_raw_recv.RX_pData));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart6, a1_usart6_raw_recv.RX_pData, RX_LEN1);
        __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
    }
    // if (huart->Instance == UART9)
    // {
    //     memset(a1_usart9_raw_recv.RX_pData, 0, sizeof(a1_usart9_raw_recv.RX_pData));
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart9, a1_usart9_raw_recv.RX_pData, RX_LEN1);
    //     __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
    // }
    // if (huart->Instance == UART8)
    // {
    //     memset(ht10a_buffer, 0, sizeof(ht10a_buffer));
    //     HAL_UARTEx_ReceiveToIdle_DMA(&huart8, ht10a_buffer, sizeof(ht10a_buffer));
    //     __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    // }

    // uart5-雷达+摄像头 (HEU 单串口多路复用)
    if (huart->Instance == UART5)
    {
        memset(Rec_Data, 0, sizeof(Rec_Data));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rec_Data, sizeof(Rec_Data));
        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
    }
    // usart10-hi14
    if (huart->Instance == USART10)
    {
        memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
        HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buf, sizeof(dma_rx_buf));
        __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);
    }

    // uart7 - serial screen
    if (huart->Instance == UART7)
    {
        Screen_UART_ErrorCallback(huart);
    }
    taskEXIT_CRITICAL_FROM_ISR(status_value); // 退出临界段
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 只有 USART3、USART6 和 UART4 用于 m8010 电机通信，其他串口的发送完成回调不处理
    if (huart->Instance == USART3 || huart->Instance == UART4)
    {
        Gom485Bus_t *b = m8010_find_by_huart(huart);
        if (b == &m8010_bus_1)
        {
            m8010_bus1_rx_mode();
            b->internal.g_state = GOM_STATE_WAIT_RX;
        }
        // if (b == &m8010_bus_2)
        // {
        //     m8010_bus2_rx_mode();
        //     b->internal.g_state = GOM_STATE_WAIT_RX;
        // }
        if (b == &m8010_bus_3)
        {
            m8010_bus3_rx_mode();
            b->internal.g_state = GOM_STATE_WAIT_RX;
        }
    }

    // uart7 - serial screen
    if (huart->Instance == UART7)
    {
        Screen_UART_TxCpltCallback(huart);
    }

}



