#include "H7_F1_spi.h"
#include "main.h"
#include <string.h>
#include <stddef.h>

/* SPI 阻塞传输超时时间。8 字节传输很短，10ms 足够用于调试。 */
#define H7_F1_SPI_TIMEOUT_MS         10U

/* 保存当前使用的 SPI 句柄，例如 &hspi1 */
static SPI_HandleTypeDef *s_hspi = NULL;

/* H7 侧缓存的最后一次有效状态 */
static H7_F1_SPI_StatusTypeDef s_last_status;

/* H7 侧“新红外帧”标志：H7_F1_SPI_Task() 收到新 seq 后置 1，应用处理完后清 0 */
static volatile uint8_t s_has_new_ir = 0;
static volatile uint8_t s_link_alive = 0;
static uint8_t s_last_reported_seq = 0;

/* 调试变量：可在 Keil Watch 里观察 */
volatile uint32_t h7_f1_spi_rx_ok_count = 0;
volatile uint32_t h7_f1_spi_error_count = 0;
volatile uint32_t h7_f1_spi_checksum_error_count = 0;
volatile uint32_t h7_f1_spi_invalid_frame_count = 0;
volatile uint8_t  h7_f1_spi_last_flags = 0;
volatile uint8_t  h7_f1_spi_last_seq = 0;
volatile uint8_t  h7_f1_spi_last_ch = 0;
volatile uint8_t  h7_f1_spi_last_data = 0;
volatile uint8_t  h7_f1_spi_last_frame[H7_F1_SPI_IR_FRAME_LEN] = {0};
volatile uint32_t h7_f1_spi_task_count = 0;
volatile uint32_t h7_f1_spi_txrx_count = 0;
volatile uint8_t  h7_f1_spi_debug_tx[H7_F1_SPI_FRAME_LEN] = {0};
volatile uint8_t  h7_f1_spi_debug_rx[H7_F1_SPI_FRAME_LEN] = {0};

/* 计算前 len 个字节的异或校验 */
static uint8_t H7_F1_SPI_XorChecksum(const uint8_t *buf, uint8_t len)
{
    uint8_t checksum = 0;
    uint8_t i;

    for (i = 0; i < len; i++)
    {
        checksum ^= buf[i];
    }

    return checksum;
}

static uint8_t H7_F1_SPI_IsChecksumOk(const uint8_t *buf)
{
    return (H7_F1_SPI_XorChecksum(buf, H7_F1_SPI_FRAME_LEN - 1U) ==
            buf[H7_F1_SPI_FRAME_LEN - 1U]);
}

/* 组 H7 发给 F1 的 8 字节请求帧 */
static void H7_F1_SPI_BuildRequest(uint8_t *tx_buf, uint8_t cmd, uint8_t seq)
{
    memset(tx_buf, 0, H7_F1_SPI_FRAME_LEN);
    tx_buf[0] = cmd;
    tx_buf[1] = seq;
    tx_buf[H7_F1_SPI_FRAME_LEN - 1U] =
        H7_F1_SPI_XorChecksum(tx_buf, H7_F1_SPI_FRAME_LEN - 1U);
}

/* 很短的片选建立/保持延时：拉低 CS 后稍等一下，再开始 SPI 时钟。 */
static void H7_F1_SPI_ShortDelay(void)
{
    volatile uint32_t i;
    for (i = 0; i < 80U; i++)
    {
        __NOP();
    }
}

/* 拉低/拉高 CS。H7 PA4 软件控制，连接到 F103 PA4 硬件 NSS。 */
static void H7_F1_SPI_CS_Low(void)
{
    HAL_GPIO_WritePin(C8_SPI_CS_GPIO_Port, C8_SPI_CS_Pin, GPIO_PIN_RESET);
    H7_F1_SPI_ShortDelay();
}

static void H7_F1_SPI_CS_High(void)
{
    H7_F1_SPI_ShortDelay();
    HAL_GPIO_WritePin(C8_SPI_CS_GPIO_Port, C8_SPI_CS_Pin, GPIO_PIN_SET);
}

/* 一次完整的 SPI 8 字节交换 */
static uint8_t H7_F1_SPI_TransmitReceive(const uint8_t *tx_buf, uint8_t *rx_buf)
{
    HAL_StatusTypeDef ret;
    uint8_t i;

    if (s_hspi == NULL)
    {
        h7_f1_spi_error_count++;
        return 0;
    }

    memset(rx_buf, 0, H7_F1_SPI_FRAME_LEN);

    H7_F1_SPI_CS_Low();
    ret = HAL_SPI_TransmitReceive(s_hspi,
                                  (uint8_t *)tx_buf,
                                  rx_buf,
                                  H7_F1_SPI_FRAME_LEN,
                                  H7_F1_SPI_TIMEOUT_MS);
    H7_F1_SPI_CS_High();

    for (i = 0; i < H7_F1_SPI_FRAME_LEN; i++)
    {
        h7_f1_spi_debug_tx[i] = tx_buf[i];
        h7_f1_spi_debug_rx[i] = rx_buf[i];
    }
    h7_f1_spi_txrx_count++;

    if (ret != HAL_OK)
    {
        h7_f1_spi_error_count++;
        HAL_SPI_Abort(s_hspi);
        return 0;
    }

    return 1;
}

static void H7_F1_SPI_SaveStatus(const H7_F1_SPI_StatusTypeDef *status)
{
    s_last_status = *status;

    h7_f1_spi_last_flags = status->flags;
    h7_f1_spi_last_seq = status->seq;
    h7_f1_spi_last_ch = status->ch;
    h7_f1_spi_last_data = status->data;
    h7_f1_spi_last_frame[0] = status->frame[0];
    h7_f1_spi_last_frame[1] = status->frame[1];
    h7_f1_spi_last_frame[2] = status->frame[2];

    s_link_alive = ((status->flags & H7_F1_SPI_FLAG_ALIVE) != 0U) ? 1U : 0U;
}

void H7_F1_SPI_Init(SPI_HandleTypeDef *hspi)
{
    s_hspi = hspi;
    memset(&s_last_status, 0, sizeof(s_last_status));
    s_has_new_ir = 0;
    s_link_alive = 0;
    s_last_reported_seq = 0;

    /* 空闲时 CS 必须为高；F1 硬件 NSS 看到低电平才会被选中。 */
    HAL_GPIO_WritePin(C8_SPI_CS_GPIO_Port, C8_SPI_CS_Pin, GPIO_PIN_SET);
}

uint8_t H7_F1_SPI_ReadStatus(H7_F1_SPI_StatusTypeDef *status)
{
    uint8_t tx_buf[H7_F1_SPI_FRAME_LEN];
    uint8_t rx_buf[H7_F1_SPI_FRAME_LEN];

    if (status == NULL)
    {
        h7_f1_spi_error_count++;
        return 0;
    }

    H7_F1_SPI_BuildRequest(tx_buf, H7_F1_SPI_CMD_GET_STATUS, 0U);

    if (!H7_F1_SPI_TransmitReceive(tx_buf, rx_buf))
    {
        return 0;
    }

    if (!H7_F1_SPI_IsChecksumOk(rx_buf))
    {
        h7_f1_spi_checksum_error_count++;
        return 0;
    }

    /*
     * F103 正常响应时 flags 一定带 ALIVE(0x80)。
     * 全 0 数据的 xor 校验也等于 0，不能当作有效帧。
     */
    if ((rx_buf[0] & H7_F1_SPI_FLAG_ALIVE) == 0U)
    {
        h7_f1_spi_invalid_frame_count++;
        return 0;
    }

    status->flags = rx_buf[0];
    status->seq = rx_buf[1];
    status->ch = rx_buf[2];
    status->data = rx_buf[3];
    status->frame[0] = rx_buf[4];
    status->frame[1] = rx_buf[5];
    status->frame[2] = rx_buf[6];

    H7_F1_SPI_SaveStatus(status);
    h7_f1_spi_rx_ok_count++;

    return 1;
}

uint8_t H7_F1_SPI_AckStatus(uint8_t seq)
{
    uint8_t tx_buf[H7_F1_SPI_FRAME_LEN];
    uint8_t rx_buf[H7_F1_SPI_FRAME_LEN];

    H7_F1_SPI_BuildRequest(tx_buf, H7_F1_SPI_CMD_ACK_STATUS, seq);
    return H7_F1_SPI_TransmitReceive(tx_buf, rx_buf);
}

uint8_t H7_F1_SPI_Task(void)
{
    H7_F1_SPI_StatusTypeDef status;
    uint8_t got_new = 0;

    h7_f1_spi_task_count++;

    if (!H7_F1_SPI_ReadStatus(&status))
    {
        s_link_alive = 0;
        return 0;
    }

    /* F1 告诉 H7：有新的红外帧 */
    if ((status.flags & H7_F1_SPI_FLAG_NEW) != 0U)
    {
        /* 同一个 seq 不重复上报，避免 ACK 未清掉前反复触发 */
        if (status.seq != s_last_reported_seq)
        {
            s_last_reported_seq = status.seq;
            s_has_new_ir = 1;
            got_new = 1;
        }

        /* H7 已经读到该帧，立即 ACK，让 F1 清除 NEW 标志 */
        (void)H7_F1_SPI_AckStatus(status.seq);
    }

    return got_new;
}

uint8_t H7_F1_SPI_IsLinkAlive(void)
{
    return s_link_alive;
}

uint8_t H7_F1_SPI_HasNewIr(void)
{
    return s_has_new_ir;
}

uint8_t H7_F1_SPI_GetIrChannel(void)
{
    return s_last_status.ch;
}

uint8_t H7_F1_SPI_GetIrData(void)
{
    return s_last_status.data;
}

uint8_t H7_F1_SPI_GetIrSeq(void)
{
    return s_last_status.seq;
}

void H7_F1_SPI_GetIrFrame(uint8_t frame[H7_F1_SPI_IR_FRAME_LEN])
{
    if (frame == NULL)
    {
        return;
    }

    frame[0] = s_last_status.frame[0];
    frame[1] = s_last_status.frame[1];
    frame[2] = s_last_status.frame[2];
}

void H7_F1_SPI_ClearNewIrFlag(void)
{
    s_has_new_ir = 0;
}
