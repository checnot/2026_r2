#ifndef KEYBOARD_H_
#define KEYBOARD_H_

#include "main.h"
#include "myuart.h"

#define KEYBOARD_BUFFER_SIZE 1  /* 7.8目前使用的键盘为一字节回传 */


extern uint8_t keyboard_buffer[KEYBOARD_BUFFER_SIZE];
extern uint8_t key_now;
extern uint8_t key_last;

void Keyboard_StartReceive(void);
void keyboard_process(void);

#endif
