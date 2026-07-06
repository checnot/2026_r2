
#include <camera.h>

/** ================================================================
 *  相机通信处理 (HEU 帧协议, 机器人端)
 *
 *  发送 (机器人→MCU):
 *    帧: HEU(3) + src=0x01 + seq(1) + len=1(2,LE) + cmd(1)
 *    命令码: 0x01=CREAT1, 0x02=CREAT2, 0x03=CREAT34, 0x06=QUIT, 0x07=CLEAR
 *
 *  接收 (MCU→机器人):
 *    帧: HEU(3) + src=0x10 + seq(1) + len=12(2,LE) + 12字符网格
 *    网格字符: R=红方, T=T系, F=F系, 0=无目标, ?=未确定
 *
 *  物理层: UART5 (huart5), TX/RX 双向
 * ================================================================ */

/* ========== HEU 接收状态机 (匹配 src=0x10 网格结果) ========== */
typedef enum {
    CAM_SYNC1 = 0,     /* 等待 0x48 'H' */
    CAM_SYNC2,         /* 等待 0x45 'E' */
    CAM_SYNC3,         /* 等待 0x55 'U' */
    CAM_SRC,           /* 读取 source_id */
    CAM_SEQ,           /* 读取 seq */
    CAM_LEN_LO,        /* 读取 len 低字节 */
    CAM_LEN_HI,        /* 读取 len 高字节 */
    CAM_PAYLOAD        /* 读取 payload */
} CamRxState_t;

/* 状态机上下文 (static) */
static CamRxState_t cam_state = CAM_SYNC1;
static uint8_t  cam_src;
static uint8_t  cam_seq;
static uint16_t cam_len;
static uint16_t cam_payload_idx;
static uint8_t  cam_payload_buf[HEU_CAM_PAYLOAD_MAX];

/* 机器人→MCU 命令 seq 滚动计数器 */
static uint8_t cam_tx_seq = 0;

/* ========== 全局变量 ========== */
uint8_t Camera_Rec_Data[128];
CZ_ACTION_TYPE CZ_action = ACTION_IDLE;
CZ_PUT_POS CZ_put_pos = POS_IDLE;
float CZ_SUDUKU_X = 0.0f;
float CZ_SUDUKU_Y = 0.0f;
float CZ_SUDUKU_W = 0.0f;
uint8_t grid[ROWS][COLS];
uint8_t MEILIN_STATE[4][3]; /* 四行三列 */

/* ========== 内部函数声明 ========== */
static void Camera_HEU_FeedByte(uint8_t byte);
static void Store_CZCmd(uint8_t action, uint8_t target);

/* ================================================================
 *  HEU 命令发送 (机器人 → MCU, src=0x01)
 *  帧: 48 45 55 | 01 | seq | 01 00 | cmd   (共 8 字节)
 * ================================================================ */
void Camera_SendCommand(uint8_t cmd)
{
    static uint8_t frame[8];    /* static: DMA 异步发送期间保持有效 */

    frame[0] = 0x48;            /* H */
    frame[1] = 0x45;            /* E */
    frame[2] = 0x55;            /* U */
    frame[3] = HEU_SRC_CMD;     /* src = 0x01 */
    frame[4] = cam_tx_seq++;    /* seq 滚动 */
    frame[5] = 0x01;               /* len 低 = 1 */
    frame[6] = 0x00;               /* len 高 = 0 */
    frame[7] = cmd;             /* 命令码 */

    /* 轮询发送: 8 字节 @921600 ≈ 87μs, 避免 DMA 冲突 */
    HAL_UART_Transmit(&huart5, frame, sizeof(frame), 10);
}

/* ================================================================
 *  MF_Scan_Command_Send (兼容旧接口)
 *  1→CREAT1, 2→CREAT2, 3/4→CREAT34
 * ================================================================ */
void MF_Scan_Command_Send(uint8_t Command_Number)
{
    switch (Command_Number)
    {
    case 1:
        Camera_SendCommand(HEU_CMD_CREAT1);
        break;
    case 2:
        Camera_SendCommand(HEU_CMD_CREAT2);
        break;
    case 3:
    case 4:
        Camera_SendCommand(HEU_CMD_CREAT34);
        break;
    default:
        break;
    }
}

/* ================================================================
 *  HEU 帧逐字节状态机 (接收 src=0x10 网格结果)
 * ================================================================ */
static void Camera_HEU_FeedByte(uint8_t byte)
{
    switch (cam_state) {

    case CAM_SYNC1:
        if (byte == 0x48) cam_state = CAM_SYNC2;
        break;

    case CAM_SYNC2:
        if (byte == 0x45) cam_state = CAM_SYNC3;
        else if (byte == 0x48) cam_state = CAM_SYNC2;
        else cam_state = CAM_SYNC1;
        break;

    case CAM_SYNC3:
        if (byte == 0x55) cam_state = CAM_SRC;
        else if (byte == 0x48) cam_state = CAM_SYNC2;
        else cam_state = CAM_SYNC1;
        break;

    case CAM_SRC:
        cam_src = byte;
        cam_state = (cam_src == HEU_SRC_GRID) ? CAM_SEQ : CAM_SYNC1;
        break;

    case CAM_SEQ:
        cam_seq = byte;
        (void)cam_seq;
        cam_state = CAM_LEN_LO;
        break;

    case CAM_LEN_LO:
        cam_len = byte;
        cam_state = CAM_LEN_HI;
        break;

    case CAM_LEN_HI:
        cam_len |= (uint16_t)byte << 8;
        if (cam_len == 0) cam_state = CAM_SYNC1;
        else if (cam_len <= HEU_CAM_PAYLOAD_MAX) {
            cam_payload_idx = 0;
            cam_state = CAM_PAYLOAD;
        } else cam_state = CAM_SYNC1;
        break;

    case CAM_PAYLOAD:
        cam_payload_buf[cam_payload_idx++] = byte;
        if (cam_payload_idx >= cam_len) {
            Camera_Scan_Data_Progress(cam_payload_buf);
            cam_state = CAM_SYNC1;
        }
        break;

    default:
        cam_state = CAM_SYNC1;
        break;
    }
}

/* ================================================================
 *  批量处理 (供 UART 中断回调)
 * ================================================================ */
void Camera_ProcessBuffer(uint8_t *data, uint16_t len)
{
    /* CZ 文本指令兼容 */
    Camera_CZ_Data_Progress((char *)data);

    /* HEU 帧逐字节解析 (匹配 src=0x10 网格) */
    for (uint16_t i = 0; i < len; i++)
        Camera_HEU_FeedByte(data[i]);
}

/* ================================================================
 *  12 字符网格 → MEILIN_STATE[4][3] (行优先)
 * ================================================================ */
void Camera_Scan_Data_Progress(uint8_t buf[])
{
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            MEILIN_STATE[r][c] = buf[r * COLS + c];
}

/* ================================================================
 *  CZ 指令解析 ("R2,action,target")
 * ================================================================ */
static void Store_CZCmd(uint8_t action, uint8_t target)
{
    CZ_action = (CZ_ACTION_TYPE)(action + 1);
    if (target != 9)
        CZ_put_pos = (CZ_PUT_POS)(target + 1);
    else
        CZ_put_pos = (CZ_PUT_POS)target;
}

void Camera_CZ_Data_Progress(char *str)
{
    char *header_ptr = strstr(str, "R2");
    if (header_ptr != NULL)
    {
        int action = 0, target = 0;
        if (sscanf(header_ptr + 2, ",%d,%d", &action, &target) == 2)
            Store_CZCmd((uint8_t)action, (uint8_t)target);
    }
}
