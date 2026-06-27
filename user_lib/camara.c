
#include <camera.h>

uint8_t Camera_Rec_Data[128];
CZ_ACTION_TYPE CZ_action = ACTION_IDLE;
CZ_PUT_POS CZ_put_pos = POS_IDLE;
float CZ_SUDUKU_X = 0.0f;
float CZ_SUDUKU_Y = 0.0f;
float CZ_SUDUKU_W = 0.0f;
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

static void Store_CZCmd(uint8_t action, uint8_t  target)
{
    CZ_action = (CZ_ACTION_TYPE)(action + 1);
    if (target != 9)
    {
        CZ_put_pos = (CZ_PUT_POS)(target + 1);
    }
    else CZ_put_pos = (CZ_PUT_POS)target ;

}

void Camera_CZ_Data_Progress(char * str)
{

    char *header_ptr = strstr(str, "R2");
    if (header_ptr != NULL)
    {
        int action = 0, target = 0;
        if (sscanf(header_ptr + 2, ",%d,%d", &action, &target) == 2)
        {
            Store_CZCmd((uint8_t)action, (uint8_t)target);
        }
    }
}

