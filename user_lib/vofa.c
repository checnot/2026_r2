#include "vofa.h"

float data_send_vofa[8];//用于向vofa上位机发送数据，将需要查看的数据赋值入该数组中

#define CH_COUNT_MAX 8      //vofa最大通道数限制为8，防止卡死
#pragma pack(push, 1)   //确保结构体无填充字节，压栈
typedef struct {
    float fdata[CH_COUNT_MAX];
    uint8_t tail[4];    //固定帧尾
} JustFloat_Frame;

#pragma pack(pop)
JustFloat_Frame vofa_frame = {.tail = {0x00, 0x00, 0x80, 0x7f} };//用于将想传输的数据转换成justfloat格式的缓存区

/**
 * @brief 以justfloat格式将数据发送至vofa上位机，适用于需要查看变量的情况，目前最高支持八通道
*/
void vofa_send_to_show_justfloat(UART_HandleTypeDef *huart, float *data_send)
{
    vofa_frame.fdata[0] = data_send[0];
    vofa_frame.fdata[1] = data_send[1];
    vofa_frame.fdata[2] = data_send[2];
    vofa_frame.fdata[3] = data_send[3];
    vofa_frame.fdata[4] = data_send[4];
    vofa_frame.fdata[5] = data_send[5];
    vofa_frame.fdata[6] = data_send[6];
    vofa_frame.fdata[7] = data_send[7];

    HAL_UART_Transmit_DMA( huart , (uint8_t*)&vofa_frame, sizeof(JustFloat_Frame));
}
