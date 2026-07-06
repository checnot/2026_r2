#include "radar.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "chassis.h"

/** ================================================================
 *  雷达数据接收处理 (HEU 帧协议, src=0x20)
 *
 *  机器人端 (STM32) 通过 UART5 接收上位机发来的雷达定位数据.
 *  帧格式: HEU(3) + src(1) + seq(1) + len(2,LE) + payload(len)
 *  载荷: "x,y,z,yaw,vx,vy" (ASCII, 逗号分隔, 3位小数)
 *  例: "1.234,5.678,0.000,45.000,0.100,-0.200"
 *
 *  逐字节状态机解析 HEU 帧 → 提取 6 浮点数 → Radar_Pos → 解算底盘坐标
 * ================================================================ */

/* ========== HEU 帧接收状态机 ========== */
typedef enum {
    RADAR_SYNC1 = 0,   /* 等待 0x48 'H' */
    RADAR_SYNC2,       /* 等待 0x45 'E' */
    RADAR_SYNC3,       /* 等待 0x55 'U' */
    RADAR_SRC,         /* 读取 source_id */
    RADAR_SEQ,         /* 读取 seq */
    RADAR_LEN_LO,      /* 读取 len 低字节 */
    RADAR_LEN_HI,      /* 读取 len 高字节 */
    RADAR_PAYLOAD      /* 读取 payload */
} RadarRxState_t;

static RadarRxState_t radar_state = RADAR_SYNC1;
static uint8_t  radar_rx_src;
static uint8_t  radar_rx_seq;
static uint16_t radar_rx_len;
static uint16_t radar_rx_idx;
static uint8_t  radar_rx_buf[HEU_RADAR_PAYLOAD_MAX];

/* ========== 全局变量 ========== */
RADAR_POSITION    Radar_Pos;
CHASSIS_POSITION  Chassis_Pos;
KalmanFilter_stu  Radar_Kalman_X;
KalmanFilter_stu  Radar_Kalman_Y;
KalmanFilter_stu  Radar_Kalman_Z;
KalmanFilter_stu  Radar_Kalman_Yaw;
float x, y, z, yaw;
float temp_a, temp_b, temp_c, temp_d;
uint8_t Rec_Data[65];

static void Radar_ParseTelemetry(char *payload, uint16_t len);

/* ================================================================
 *  逐字节喂入 HEU 帧解析器
 * ================================================================ */
static void Radar_HEU_FeedByte(uint8_t byte)
{
    switch (radar_state) {
    case RADAR_SYNC1:
        if (byte == HEU_SYNC1) radar_state = RADAR_SYNC2;
        break;
    case RADAR_SYNC2:
        if (byte == HEU_SYNC2) radar_state = RADAR_SYNC3;
        else if (byte == HEU_SYNC1) radar_state = RADAR_SYNC2;
        else radar_state = RADAR_SYNC1;
        break;
    case RADAR_SYNC3:
        if (byte == HEU_SYNC3) radar_state = RADAR_SRC;
        else if (byte == HEU_SYNC1) radar_state = RADAR_SYNC2;
        else radar_state = RADAR_SYNC1;
        break;
    case RADAR_SRC:
        radar_rx_src = byte;
        radar_state = (radar_rx_src == HEU_SRC_RADAR) ? RADAR_SEQ : RADAR_SYNC1;
        break;
    case RADAR_SEQ:
        radar_rx_seq = byte;
        (void)radar_rx_seq;
        radar_state = RADAR_LEN_LO;
        break;
    case RADAR_LEN_LO:
        radar_rx_len = byte;
        radar_state = RADAR_LEN_HI;
        break;
    case RADAR_LEN_HI:
        radar_rx_len |= (uint16_t)byte << 8;
        if (radar_rx_len == 0) radar_state = RADAR_SYNC1;
        else if (radar_rx_len <= HEU_RADAR_PAYLOAD_MAX) {
            radar_rx_idx = 0;
            radar_state = RADAR_PAYLOAD;
        } else radar_state = RADAR_SYNC1;
        break;
    case RADAR_PAYLOAD:
        radar_rx_buf[radar_rx_idx++] = byte;
        if (radar_rx_idx >= radar_rx_len) {
            radar_rx_buf[radar_rx_len] = '\0';
            Radar_ParseTelemetry((char *)radar_rx_buf, radar_rx_len);
            radar_state = RADAR_SYNC1;
        }
        break;
    default:
        radar_state = RADAR_SYNC1;
        break;
    }
}

/* ================================================================
 *  批量处理 (供 UART 中断回调)
 * ================================================================ */
void Radar_ProcessBuffer(uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
        Radar_HEU_FeedByte(data[i]);
}

/* ================================================================
 *  解析 "x,y,z,yaw,vx,vy" → Radar_Pos (米→毫米)
 * ================================================================ */
static void Radar_ParseTelemetry(char *payload, uint16_t len)
{
    (void)len;
    float tx, ty, tz, tyaw, tvx, tvy;
    if (sscanf(payload, "%f,%f,%f,%f,%f,%f",
               &tx, &ty, &tz, &tyaw, &tvx, &tvy) >= 6)
    {
        Radar_Pos.radar_x   = 1000.0f * tx;
        Radar_Pos.radar_y   = 1000.0f * ty;
        Radar_Pos.radar_z   = 1000.0f * tz;
        Radar_Pos.radar_yaw = tyaw;
        Radar_Pos.radar_vx  = tvx;
        Radar_Pos.radar_vy  = tvy;
        Radar_Pos.hi14_yaw  = hipnuc_raw.hi91.yaw;
        Calculate_Chassis_Position();
    }
}

/* ================================================================
 *  底盘中心坐标解算 (mm), 镜像时改 Y
 * ================================================================ */
void Calculate_Chassis_Position(void)
{
    const float OFFSET_X = -290.15f;
    const float OFFSET_Y = -152.15f;
    float yaw_rad = Radar_Pos.hi14_yaw * 3.1415926f / 180.0f;
    float dx = OFFSET_X * cosf(yaw_rad) - OFFSET_Y * sinf(yaw_rad);
    float dy = OFFSET_X * sinf(yaw_rad) + OFFSET_Y * cosf(yaw_rad);
    Radar_Pos.radar_x = Radar_Pos.radar_x + dx;
    Radar_Pos.radar_y = Radar_Pos.radar_y + dy;
}


