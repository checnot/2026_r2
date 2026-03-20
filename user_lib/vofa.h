#ifndef VOFA_H
#define VOFA_H

#include "main.h"
#include "usart.h"
#include "stdio.h"
#include "string.h"

extern float data_send_vofa[8];

void vofa_send_to_show_justfloat(UART_HandleTypeDef *huart, float *data_send);



#endif /* VOFA_H */
