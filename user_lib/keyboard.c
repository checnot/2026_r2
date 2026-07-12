#include "keyboard.h"
#include "myuart.h"
#include <string.h>

uint8_t keyboard_buffer[KEYBOARD_BUFFER_SIZE];  /* UART8 矩阵键盘接收缓存区 */
uint8_t key_now;    /* 当前按下的键，范围为：0~9, A~D, *, # */
uint8_t key_last;   /* 上次按下的键，范围为：0~9, A~D, *, # */
MF_CellType keyboard_mf_cell[MF_ROWS][MF_COLS]; /* MF 规划器使用的二维地图，row1~row4 由键盘维护 */
uint8_t keyboard_kfs_screen_info[KEYBOARD_MF_KFS_COUNT]; /* 屏幕显示用的一维 KFS 状态，顺序与 12 个地图键一致 */

uint8_t keyboard_2_buffer[KEYBOARD_BUFFER_SIZE];  /* UART2 矩阵键盘接收缓存区 */
uint8_t key_2_now;    /* 第二键盘当前按下的键，范围为：1~16 */
uint8_t key_2_last;   /* 第二键盘上次按下的键，范围为：1~16 */

int16_t x_offset_by_key_times;  // 由第二键盘设置的x偏移次数
int16_t y_offset_by_key_times;  // 由第二键盘设置的x偏移次数


/**
 * @brief 将键盘模块回传的 1 字节原始码转换为可读按键字符。
 * @param raw UART8 收到的原始键值，范围通常为 0x01~0x10。
 * @param out_key 输出按键字符，可能为 0~9、A~D、*、#。
 * @return true 表示转换成功；false 表示未知码，调用方应忽略本次输入。
 */
static bool Keyboard_DecodeRawKey(uint8_t raw, uint8_t* out_key)
{
    if (out_key == NULL) {
        return false;
    }

    if (raw == 0x01)      *out_key = '1';
    else if (raw == 0x02) *out_key = '2';
    else if (raw == 0x03) *out_key = '3';
    else if (raw == 0x04) *out_key = 'A';
    else if (raw == 0x05) *out_key = '4';
    else if (raw == 0x06) *out_key = '5';
    else if (raw == 0x07) *out_key = '6';
    else if (raw == 0x08) *out_key = 'B';
    else if (raw == 0x09) *out_key = '7';
    else if (raw == 0x0A) *out_key = '8';
    else if (raw == 0x0B) *out_key = '9';
    else if (raw == 0x0C) *out_key = 'C';
    else if (raw == 0x0D) *out_key = '*';
    else if (raw == 0x0E) *out_key = '0';
    else if (raw == 0x0F) *out_key = '#';
    else if (raw == 0x10) *out_key = 'D';
    else return false;

    return true;
}

/**
 * @brief 将第二矩阵键盘回传的 1 字节原始码转换为可读按键字符。
 * @param raw UART2 收到的原始键值，范围通常为 0x01~0x10。
 * @param out_key 输出按键字符，范围为 1~16。
*/
static bool Keyboard_2_DecodeRawKey(uint8_t raw, uint8_t* out_key)
{
    if (out_key == NULL) {
        return false;
    }

    if (raw >= 1 && raw <= 16) {
        *out_key = raw; // 第二键盘直接使用原始码作为按键值
        return true;
    }
    return false;
}

/**
 * @brief 将 12 个地图按键映射到 MF 的 row/col 与屏幕数组下标。
 * @note 只处理 1~9、*、0、#；A/B/C/D 当前不参与 MF 地图输入。
 */
bool Keyboard_MF_KeyToCell(uint8_t key,
                           uint8_t* out_row,
                           uint8_t* out_col,
                           uint8_t* out_screen_index)
{
    uint8_t row = 0;
    uint8_t col = 0;

    /* 物理键盘最下面一行对应 MF row1，最上面一行对应 MF row4。 */
    switch (key) {
    case '*': row = 1; col = 0; break;
    case '0': row = 1; col = 1; break;
    case '#': row = 1; col = 2; break;
    case '7': row = 2; col = 0; break;
    case '8': row = 2; col = 1; break;
    case '9': row = 2; col = 2; break;
    case '4': row = 3; col = 0; break;
    case '5': row = 3; col = 1; break;
    case '6': row = 3; col = 2; break;
    case '1': row = 4; col = 0; break;
    case '2': row = 4; col = 1; break;
    case '3': row = 4; col = 2; break;
    default:
        return false;
    }

    if (out_row != NULL) {
        *out_row = row;
    }
    if (out_col != NULL) {
        *out_col = col;
    }
    if (out_screen_index != NULL) {
        /* 屏幕数组按 MF 行顺序展开：[0..11] = row1, row2, row3, row4。 */
        *out_screen_index = (uint8_t)((row - 1U) * MF_COLS + col);
    }
    return true;
}

/**
 * @brief 计算单个 MF 格子下一次按键后的 KFS 类型。
 * @note 循环顺序：EMPTY -> R1KFS -> R2KFS -> FAKEKFS -> EMPTY。
 */
static MF_CellType Keyboard_MF_NextCellType(MF_CellType current)
{
    switch (current) {
    case MF_CELL_EMPTY:   return MF_CELL_R1KFS;
    case MF_CELL_R1KFS:   return MF_CELL_R2KFS;
    case MF_CELL_R2KFS:   return MF_CELL_FAKEKFS;
    case MF_CELL_FAKEKFS: return MF_CELL_EMPTY;
    default:              return MF_CELL_EMPTY;
    }
}

/**
 * @brief 清空键盘维护的 MF 预输入地图和屏幕显示数组。
 * @note 全局变量上电默认为 0，本函数用于后续手动重置或绑定 A/B/C/D 功能。
 */
void Keyboard_MF_ResetMap(void)
{
    for (uint8_t r = 0; r < MF_ROWS; r++) {
        for (uint8_t c = 0; c < MF_COLS; c++) {
            keyboard_mf_cell[r][c] = MF_CELL_EMPTY;
        }
    }
    memset(keyboard_kfs_screen_info, (uint8_t)MF_CELL_EMPTY, sizeof(keyboard_kfs_screen_info));
}

/**
 * @brief 处理一次已经解析出的按键字符，并同步更新 MF 地图与屏幕数组。
 * @note 非地图键(A/B/C/D 等)不会改变 MF 地图，只保留在 key_now/key_last 中。
 */
static void Keyboard_MF_ProcessKey(uint8_t key)
{
    uint8_t row = 0;
    uint8_t col = 0;
    uint8_t screen_index = 0;

    if (!Keyboard_MF_KeyToCell(key, &row, &col, &screen_index)) {
        return;
    }

    /* 先更新规划器地图，再把同一状态镜像到屏幕用的一维数组。 */
    keyboard_mf_cell[row][col] = Keyboard_MF_NextCellType(keyboard_mf_cell[row][col]);
    keyboard_kfs_screen_info[screen_index] = (uint8_t)keyboard_mf_cell[row][col];
}

/**
 * @brief 重启串口矩阵键盘接收。
 * @note 键盘当前使用 UART8 + ReceiveToIdle DMA，每次接收 1 字节按键码。
*/
void Keyboard_StartReceive(void)
{
    memset(keyboard_buffer, 0, sizeof(keyboard_buffer));
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, keyboard_buffer, sizeof(keyboard_buffer));
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
}

/**
 * @brief 在 UART8 接收回调中处理键盘数据。
 * @note 流程：原始码 -> 按键字符 -> key_now/key_last -> MF 地图/屏幕数组。
*/
void keyboard_process(void)
{
    uint8_t decoded_key = 0;

    /* 未知按键码直接忽略，避免错误数据污染地图。 */
    if (!Keyboard_DecodeRawKey(keyboard_buffer[0], &decoded_key)) {
        return;
    }

    /* A/B/C/D 也会记录到 key_now，但不会进入 MF 地图。 */
    key_last = key_now;
    key_now = decoded_key;
    Keyboard_MF_ProcessKey(key_now);
}

/**
 * @brief 在 UART2 接收回调中处理第二矩阵键盘数据。
 * @note 流程：原始码 -> 按键字符 -> key_2_now/key_2_last。第二键盘不参与 MF 地图。
*/
void keyboard_2_process(void)
{
    uint8_t decoded_key = 0;

    /* 未知按键码直接忽略，避免错误数据污染地图。 */
    if (!Keyboard_2_DecodeRawKey(keyboard_2_buffer[0], &decoded_key)) {
        return;
    }

    key_2_last = key_2_now;
    key_2_now = decoded_key;
}

