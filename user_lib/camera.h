#ifndef __CAMERA_H__
#define __CAMERA_H__

#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "myuart.h"

/* ========== 网格常量 ========== */
#define ROWS 4
#define COLS 3

/* ========== HEU 帧协议常量 ========== */
#define HEU_SRC_CMD         0x01  /* 命令 (机器人→MCU, 机器人发送) */
#define HEU_SRC_GRID        0x10  /* 网格结果 (MCU→机器人, 机器人接收) */
#define HEU_GRID_PAYLOAD_LEN 12   /* 网格载荷固定 12 字符 */
#define HEU_CAM_PAYLOAD_MAX 32    /* 相机接收缓冲最大值 */

/* ========== HEU 命令码 (机器人→MCU, src=0x01) ========== */
#define HEU_CMD_CREAT1  0x01  /* 刷新第1行网格 */
#define HEU_CMD_CREAT2  0x02  /* 刷新第2行网格 */
#define HEU_CMD_CREAT34 0x03  /* 刷新第3-4行网格 */
#define HEU_CMD_QUIT    0x06  /* 忽略 */
#define HEU_CMD_CLEAR   0x07  /* 清空网格缓存 */

/* ========== 网格字符含义 ==========
 * 'R' - 红色方块 (R1系)
 * 'T' - T系方块
 * 'F' - F系方块
 * '0' - 无目标
 * '?' - 未确定
 */

/* ========== 放置动作类型 ========== */
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
extern uint8_t MEILIN_STATE[4][3];

/* ========== 指令发送 (机器人→MCU, src=0x01) ========== */
void Camera_SendCommand(uint8_t cmd);           /* 发送单条 HEU 命令帧 */
void MF_Scan_Command_Send(uint8_t Command_Number); /* 兼容旧接口 */

/* ========== 网格接收 (MCU→机器人, src=0x10) ========== */
void Camera_ProcessBuffer(uint8_t *data, uint16_t len); /* HEU 帧批量喂入 */
void Camera_Scan_Data_Progress(uint8_t buf[]);          /* 12字符→MEILIN_STATE */

/* ========== CZ 指令解析 (兼容旧协议) ========== */
void Camera_CZ_Data_Progress(char * str);

#endif

