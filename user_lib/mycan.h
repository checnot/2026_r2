#ifndef MYCAN_H
#define MYCAN_H

#include "main.h"
#include "fdcan.h"
#include "freeRTOS.h"
#include "task.h"
#include "m3508.h"

typedef struct
{
	FDCAN_HandleTypeDef *hcan;
	FDCAN_TxHeaderTypeDef TxHeader;
}FDCAN_TxFrame_TypeDef;//自定义FDCAN发送相关结构体

typedef struct
{
	FDCAN_HandleTypeDef *hcan;
	FDCAN_RxHeaderTypeDef RxHeader;
}FDCAN_RxFrame_TypeDef;//自定义FDCAN接收相关结构体

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_1;
// extern FDCAN_HandleTypeDef hfdcan2;
// extern FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_2;
// extern FDCAN_HandleTypeDef hfdcan3;
// extern FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_3;

void FDCan_Start(FDCAN_FilterTypeDef *FDCAN_FilterConfig_1);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);


#endif /* MYCAN_H */
