#ifndef REMOTE_H_
#define REMOTE_H_

#include "main.h"
#include "usart.h"
#include "myuart.h"

typedef struct
{
	int16_t left_x_last;
	int16_t left_y_last;
	int16_t right_x_last;
	int16_t right_y_last;
	int16_t switch_a_last;
	int16_t switch_b_last;
	int16_t switch_c_last;
	int16_t switch_d_last;
	int16_t knob_a_last;
	int16_t knob_b_last;
}LAST_REMOTE_STATU_STU;

/**
 * @brief ht10a遥控器状态结构体，包含了遥控器的所有输入状态以及上一次的输入状态，方便进行边缘判断
*/
typedef struct
{
	int16_t left_x;
	int16_t left_y;
	int16_t right_x;
	int16_t right_y;
	int16_t switch_a;
	int16_t switch_b;
	int16_t switch_c;
	int16_t switch_d;
	int16_t knob_a;
	int16_t knob_b;
	LAST_REMOTE_STATU_STU last_status;
}REMOTE_STU;



extern int16_t _channels[16];
extern uint8_t ht10a_buffer[30];				//遥控器原始数据
extern REMOTE_STU HT10A;
extern REMOTE_STU HT10A_last; //在freertos.c中定义的遥控器last状态

void HT10A_process(uint8_t buffer[30]);
void HT10A_task(void);

void customized_judgment(void);

#endif

