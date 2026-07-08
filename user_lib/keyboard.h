#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include "main.h"
#include "MF_planner.h"

#define KEYBOARD_BUFFER_SIZE 1  /* 当前矩阵键盘每次回传 1 字节按键码 */
#define KEYBOARD_MF_KFS_COUNT 12 /* MF 树林区 4 行 x 3 列，共 12 个可输入 KFS 格 */


extern uint8_t keyboard_buffer[KEYBOARD_BUFFER_SIZE]; /* UART8 DMA 接收缓冲区 */
extern uint8_t key_now;                               /* 当前解析出的按键字符 */
extern uint8_t key_last;                              /* 上一次解析出的按键字符 */
extern MF_CellType keyboard_mf_cell[MF_ROWS][MF_COLS]; /* 供 MF 规划器读取的二维 KFS 地图 */
extern uint8_t keyboard_kfs_screen_info[KEYBOARD_MF_KFS_COUNT]; /* 供屏幕发送层读取的一维 KFS 状态表 */

void Keyboard_StartReceive(void);
void keyboard_process(void);
void Keyboard_MF_ResetMap(void);

bool Keyboard_MF_KeyToCell(uint8_t key,
                           uint8_t* out_row,
                           uint8_t* out_col,
                           uint8_t* out_screen_index);

#endif
