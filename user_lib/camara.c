
#include <camera.h>

char Camera_Rec_Data[128];

uint8_t grid[ROWS][COLS];
uint8_t MEILIN_STATE[4][3]; // 四行三列
uint8_t tx_buffer1 = 0x01;
uint8_t tx_buffer2 = 0x02;
uint8_t tx_buffer3 = 0x03;
uint8_t tx_buffer4 = 0x04;

/*
 * @brief MF区扫描发送指令
 * @param Command_Number 每个指令对应的序号
 */
void MF_Scan_Command_Send(uint8_t Command_Number)
{
    switch (Command_Number)
    {
    case 1:
        HAL_UART_Transmit_DMA(&huart8, &tx_buffer1, sizeof(tx_buffer1));
        break;
    case 2:
        HAL_UART_Transmit_DMA(&huart8, &tx_buffer2, sizeof(tx_buffer2));
        break;
    case 3:
        HAL_UART_Transmit_DMA(&huart8, &tx_buffer3, sizeof(tx_buffer3));
        break;
    case 4:
        HAL_UART_Transmit_DMA(&huart8, &tx_buffer4, sizeof(tx_buffer4));
        break;
    default:
        break;
    }
}

// void parse_full_table(const char *data) {
//     char buf[256];
//     strcpy(buf, data);   // 输入保证足够小且规范，直接复制

//     int row = 0;
//     char *line = strtok(buf, "\r\n");
//     while (line && row < ROWS) {
//         int col = 0;
//         char *token = strtok(line, " \t");
//         while (token && col < COLS) {
//             strcpy(grid[row][col], token);   // token长度最大2字符+'\0'，安全
//             col++;
//             token = strtok(NULL, " \t");
//         }
//         row++;
//         line = strtok(NULL, "\r\n");
//     }
// }

/*
 *@brief 相机扫描回传处理函数
 */
void Camera_Scan_Data_Progress(uint8_t buf[])
{
    for (int r = 0; r < ROWS; r++)
        for (int c = 0; c < COLS; c++)
            MEILIN_STATE[r][c] = buf[r * COLS + c];
}
