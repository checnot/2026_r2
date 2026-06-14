#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "myuart.h"
#define ROWS 4
#define COLS 3

extern char Camera_Rec_Data[128];
void MF_Scan_Command_Send(uint8_t Command_Number);
void parse_full_table(const char *data);
#endif
