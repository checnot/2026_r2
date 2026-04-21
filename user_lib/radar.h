#ifndef __RADAR_H
#define __RADAR_H

#include "main.h"

typedef struct {
		float radar_x;
		float radar_y;
		float radar_z;
		float radar_yaw;
}RADAR_POSITION;

extern RADAR_POSITION Radar_Pos;
extern uint8_t Rec_Data[65];

void Data_process_String(char *str);

#endif
