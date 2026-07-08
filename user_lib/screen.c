#include "screen.h"
#include <stdio.h>
#include <string.h>

/* =========================================================
 * 串口屏使用 UART7
 * ========================================================= */

extern UART_HandleTypeDef huart7;

#define SCREEN_UART huart7

/* =========================================================
 * 梅林区数组
 *
 * 当前屏幕布局：
 *
 * 三区
 * m9    m10    m11
 * m6    m7     m8
 * m3    m4     m5
 * m0    m1     m2
 * 一区
 *
 * 数组对应关系：
 * merlin[0]  -> m0
 * merlin[1]  -> m1
 * ...
 * merlin[11] -> m11
 * ========================================================= */

uint8_t merlin[12] = {0U};

/* =========================================================
 * 串口屏 TX DMA 命令队列
 * ========================================================= */

#define SCREEN_QUEUE_SIZE   32U
#define SCREEN_CMD_MAX_LEN  64U

typedef struct
{
    uint8_t data[SCREEN_CMD_MAX_LEN];
    uint16_t len;
} Screen_Cmd_t;

static Screen_Cmd_t screen_queue[SCREEN_QUEUE_SIZE];

volatile uint8_t screen_q_head  = 0U;
volatile uint8_t screen_q_tail  = 0U;
volatile uint8_t screen_q_count = 0U;
volatile uint8_t screen_tx_busy = 0U;

/* =========================================================
 * UART7 IDLE DMA 接收缓冲
 * ========================================================= */

#define SCREEN_RX_BUF_SIZE  64U

static uint8_t screen_rx_buf[SCREEN_RX_BUF_SIZE];

/* =========================================================
 * 内部函数声明
 * ========================================================= */

static void Screen_TryStartTx(void);
static int Screen_SendCmdAsync(const char *cmd);
static const char *Merlin_StateToText(uint8_t state);
static void Screen_ProcessRxData(uint8_t *data, uint16_t len);

/* =========================================================
 * 启动 UART7 IDLE DMA 接收
 * ========================================================= */

void Screen_UART7_StartReceiveToIdleDMA(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&SCREEN_UART, screen_rx_buf, SCREEN_RX_BUF_SIZE);

    /*
     * 关闭 DMA 半传输中断。
     * 避免接收一半数据时也进入回调。
     */
    if (SCREEN_UART.hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(SCREEN_UART.hdmarx, DMA_IT_HT);
    }
}

/* =========================================================
 * 尝试启动下一条 TX DMA 发送
 * ========================================================= */

static void Screen_TryStartTx(void)
{
    uint8_t *p_data;
    uint16_t len;
    HAL_StatusTypeDef ret;

    __disable_irq();

    if ((screen_tx_busy != 0U) || (screen_q_count == 0U))
    {
        __enable_irq();
        return;
    }

    screen_tx_busy = 1U;
    p_data = screen_queue[screen_q_tail].data;
    len = screen_queue[screen_q_tail].len;

    __enable_irq();

    ret = HAL_UART_Transmit_DMA(&SCREEN_UART, p_data, len);

    if (ret != HAL_OK)
    {
        __disable_irq();
        screen_tx_busy = 0U;
        __enable_irq();
    }
}

/* =========================================================
 * 异步发送一条陶晶驰命令
 *
 * 输入示例：
 * m0.txt="R1"
 *
 * 实际发送：
 * m0.txt="R1" 0xFF 0xFF 0xFF
 * ========================================================= */

static int Screen_SendCmdAsync(const char *cmd)
{
    uint16_t cmd_len;
    uint8_t index;

    if (cmd == NULL)
    {
        return -1;
    }

    cmd_len = (uint16_t)strlen(cmd);

    if ((cmd_len + 3U) > SCREEN_CMD_MAX_LEN)
    {
        return -2;
    }

    __disable_irq();

    if (screen_q_count >= SCREEN_QUEUE_SIZE)
    {
        __enable_irq();
        return -3;
    }

    index = screen_q_head;

    memcpy(screen_queue[index].data, cmd, cmd_len);
    screen_queue[index].data[cmd_len + 0U] = 0xFFU;
    screen_queue[index].data[cmd_len + 1U] = 0xFFU;
    screen_queue[index].data[cmd_len + 2U] = 0xFFU;
    screen_queue[index].len = (uint16_t)(cmd_len + 3U);

    screen_q_head++;
    if (screen_q_head >= SCREEN_QUEUE_SIZE)
    {
        screen_q_head = 0U;
    }

    screen_q_count++;

    __enable_irq();

    Screen_TryStartTx();

    return 0;
}

/* =========================================================
 * UART7 TX DMA 发送完成处理
 *
 * 注意：HAL_UART_TxCpltCallback 在 myuart.c 中，
 * myuart.c 判断 UART7 后调用本函数。
 * ========================================================= */

void Screen_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        __disable_irq();

        if (screen_q_count > 0U)
        {
            screen_q_tail++;
            if (screen_q_tail >= SCREEN_QUEUE_SIZE)
            {
                screen_q_tail = 0U;
            }

            screen_q_count--;
        }

        screen_tx_busy = 0U;

        __enable_irq();

        Screen_TryStartTx();
    }
}

/* =========================================================
 * UART7 IDLE DMA 接收事件处理
 *
 * 注意：HAL_UARTEx_RxEventCallback 在 myuart.c 中，
 * myuart.c 判断 UART7 后调用本函数。
 * ========================================================= */

void Screen_UART_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == UART7)
    {
        if ((Size > 0U) && (Size <= SCREEN_RX_BUF_SIZE))
        {
            Screen_ProcessRxData(screen_rx_buf, Size);
        }

        Screen_UART7_StartReceiveToIdleDMA();
    }
}

/* =========================================================
 * UART7 错误处理
 *
 * 注意：HAL_UART_ErrorCallback 在 myuart.c 中，
 * myuart.c 判断 UART7 后调用本函数。
 * ========================================================= */

void Screen_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7)
    {
        __disable_irq();
        screen_tx_busy = 0U;
        __enable_irq();

        Screen_TryStartTx();

        Screen_UART7_StartReceiveToIdleDMA();
    }
}

/* =========================================================
 * 串口屏返回数据处理
 *
 * 当前无触摸屏业务，暂不处理返回数据。
 * 后续如需触摸事件，可在此处扩展。
 * ========================================================= */

static void Screen_ProcessRxData(uint8_t *data, uint16_t len)
{
    (void)data;
    (void)len;
}

/* =========================================================
 * 状态转文本
 * ========================================================= */

static const char *Merlin_StateToText(uint8_t state)
{
    switch (state)
    {
        case EMPTY_KFS:
            return "";

        case R1_KFS:
            return "R1";

        case R2_KFS:
            return "R2";

        case FAKE_KFS:
            return "F";

        default:
            return "";
    }
}

/* =========================================================
 * 进入 page0
 * ========================================================= */

void Screen_GotoPage0(void)
{
    (void)Screen_SendCmdAsync("page page0");
}

/* =========================================================
 * 更新单个格子
 *
 * index: 0 ~ 11
 * state: 0 空, 1 R1, 2 R2, 3 F
 * ========================================================= */

void Screen_UpdateCell(uint8_t index, uint8_t state)
{
    char cmd[40];
    const char *text;

    if (index >= 12U)
    {
        return;
    }

    text = Merlin_StateToText(state);

    (void)snprintf(cmd, sizeof(cmd), "m%u.txt=\"%s\"", (unsigned int)index, text);

    (void)Screen_SendCmdAsync(cmd);
}

/* =========================================================
 * 更新全部格子
 * ========================================================= */

void Screen_UpdateAllCells(void)
{
    uint8_t i;

    for (i = 0U; i < 12U; i++)
    {
        Screen_UpdateCell(i, merlin[i]);
    }
}

/* =========================================================
 * 清空全部格子
 * ========================================================= */

void Screen_ClearAllCells(void)
{
    uint8_t i;

    for (i = 0U; i < 12U; i++)
    {
        merlin[i] = EMPTY_KFS;
        Screen_UpdateCell(i, EMPTY_KFS);
    }
}

/* =========================================================
 * 矩阵键盘输入后调用
 *
 * index: 0 ~ 11
 * state: 0 空, 1 R1, 2 R2, 3 F
 * ========================================================= */

void Merlin_SetBlock(uint8_t index, uint8_t state)
{
    if (index >= 12U)
    {
        return;
    }

    if (state > 3U)
    {
        return;
    }

    merlin[index] = state;

    Screen_UpdateCell(index, state);
}


/* =========================================================
 * 串口屏初始化
 * ========================================================= */

void Screen_InitDisplay(void)
{
    Screen_UART7_StartReceiveToIdleDMA();

    Screen_GotoPage0();

    Screen_ClearAllCells();
}

/* =========================================================
 * 测试函数
 * ========================================================= */

void Screen_TestDisplay(void)
{
    Merlin_SetBlock(0U, R1_KFS);
    Merlin_SetBlock(1U, R2_KFS);
    Merlin_SetBlock(2U, FAKE_KFS);

    Merlin_SetBlock(3U, R1_KFS);
    Merlin_SetBlock(4U, R2_KFS);
    Merlin_SetBlock(5U, FAKE_KFS);

    Merlin_SetBlock(9U, R1_KFS);
    Merlin_SetBlock(10U, R2_KFS);
    Merlin_SetBlock(11U, FAKE_KFS);
}
