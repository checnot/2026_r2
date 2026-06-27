#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "myuart.h"
#define ROWS 4
#define COLS 3

 
typedef enum
{
  ACTION_IDLE,  //初始状态
  KFS_PUT,      //放置KFS
  WAIT,         //等待
  RESCAN,       //重新扫描
}CZ_ACTION_TYPE;

typedef enum
{
  POS_IDLE,  //初始状态
  M1,        //放置到中层第一列
  M2,        //放置到中层第二列
  M3,         //放置到中层第三列
  Invalid = 9 //无效，不进行放置动作
}CZ_PUT_POS;


extern uint8_t Camera_Rec_Data[128];
extern CZ_ACTION_TYPE CZ_action;
extern CZ_PUT_POS CZ_put_pos;
extern float CZ_SUDUKU_X ;
extern float CZ_SUDUKU_Y ;
extern float CZ_SUDUKU_W ;
void MF_Scan_Command_Send(uint8_t Command_Number);
void Camera_Scan_Data_Progress(uint8_t buf[]);
void Camera_CZ_Data_Progress(char * str);
#endif
