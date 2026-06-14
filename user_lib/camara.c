
#include <camera.h>

char Camera_Rec_Data[128];

char grid[ROWS][COLS][4];  
uint8_t MEILIN_STATE[4][3]; //四行三列
uint8_t tx_buffer1= 0x01;
uint8_t tx_buffer2 =0x02;
uint8_t tx_buffer3[] = "creatright\r\n";
uint8_t tx_buffer4[] = "infer\r\n";

/*
* @brief MF区扫描发送指令
* @param Command_Number 每个指令对应的序号
*/
void MF_Scan_Command_Send(uint8_t Command_Number)
{
    switch (Command_Number)
    {
    case 1:HAL_UART_Transmit(&huart8, &tx_buffer1, sizeof(tx_buffer1),1000);
        break;
    case 2:HAL_UART_Transmit_DMA(&huart8, &tx_buffer2, sizeof(tx_buffer2));
        break;
    case 3:HAL_UART_Transmit_DMA(&huart8, tx_buffer3, sizeof(tx_buffer3) - 1);
        break;
    case 4:HAL_UART_Transmit_DMA(&huart8, tx_buffer4, sizeof(tx_buffer4) - 1);
        break;
    default:
        break;
    }

}
		
void parse_full_table(const char *data) {
    char buf[256];
    strcpy(buf, data);   // 输入保证足够小且规范，直接复制

    int row = 0;
    char *line = strtok(buf, "\r\n");
    while (line && row < ROWS) {
        int col = 0;
        char *token = strtok(line, " \t");
        while (token && col < COLS) {
            strcpy(grid[row][col], token);   // token长度最大2字符+'\0'，安全
            col++;
            token = strtok(NULL, " \t");
        }
        row++;
        line = strtok(NULL, "\r\n");
    }
}

void Data_process_Camera_String(char *str)
{
    // 1. 寻找帧头 "heu"
    char *header_ptr = strstr(str, "heu");
    
    if (header_ptr != NULL)
    {
//					float temp_a, temp_b, temp_c, temp_d;
//					
//					// 2. 尝试解析四个浮点数
//					if (sscanf(header_ptr + 3, "%f,%f,%f,%f", &temp_a, &temp_b, &temp_c, &temp_d) >= 4)
//					{

//					}
    }
}
