#include "keyboard.h"

uint8_t keyboard_buffer[KEYBOARD_BUFFER_SIZE];  /* 矩阵键盘接受缓存区 */
uint8_t key_now;
uint8_t key_last;

/**
 * @brief 重启串口矩阵键盘接收
*/
void Keyboard_StartReceive(void)
{
    memset(keyboard_buffer, 0, sizeof(keyboard_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, keyboard_buffer, sizeof(keyboard_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

/**
 * @brief 在串口中断中处理键盘数据
*/
void keyboard_process(void)
{
    key_last = key_now;
    key_now = keyboard_buffer[0];
}

