#include "mycan.h"

//FDCAN1m3508发送相关结构体
FDCAN_TxFrame_TypeDef FDCAN1_TxFrame_m3508 = {
	.hcan = &hfdcan1,
	.TxHeader.Identifier = 0x0200,
	.TxHeader.IdType = FDCAN_STANDARD_ID,
	.TxHeader.TxFrameType = FDCAN_DATA_FRAME,
	.TxHeader.DataLength = 8,
	.TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	.TxHeader.BitRateSwitch = FDCAN_BRS_OFF,
	.TxHeader.FDFormat = FDCAN_CLASSIC_CAN,
	.TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	.TxHeader.MessageMarker = 0,
};
//FDCAN1m3508接收相关结构体
FDCAN_RxFrame_TypeDef FDCAN1_RxFrame_m3508;

//FDCAN1m3508发送相关结构体
FDCAN_TxFrame_TypeDef FDCAN2_TxFrame_m3508 = {
	.hcan = &hfdcan2,
	.TxHeader.Identifier = 0x0200,
	.TxHeader.IdType = FDCAN_STANDARD_ID,
	.TxHeader.TxFrameType = FDCAN_DATA_FRAME,
	.TxHeader.DataLength = 8,
	.TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	.TxHeader.BitRateSwitch = FDCAN_BRS_OFF,
	.TxHeader.FDFormat = FDCAN_CLASSIC_CAN,
	.TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	.TxHeader.MessageMarker = 0,
};
//FDCAN2m3508接收相关结构体
FDCAN_RxFrame_TypeDef FDCAN2_RxFrame_m3508;

//FDCAN3m3508发送相关结构体
FDCAN_TxFrame_TypeDef FDCAN3_TxFrame_m3508 = {
	.hcan = &hfdcan3,
	.TxHeader.Identifier = 0x0200,
	.TxHeader.IdType = FDCAN_STANDARD_ID,
	.TxHeader.TxFrameType = FDCAN_DATA_FRAME,
	.TxHeader.DataLength = 8,
	.TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE,
	.TxHeader.BitRateSwitch = FDCAN_BRS_OFF,
	.TxHeader.FDFormat = FDCAN_CLASSIC_CAN,
	.TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS,
	.TxHeader.MessageMarker = 0,
};
//FDCAN3m3508接收相关结构体
FDCAN_RxFrame_TypeDef FDCAN3_RxFrame_m3508;


//FDCAN默认接收结构体
FDCAN_RxFrame_TypeDef FDCAN_Default_RxFrame;
//FDCAN默认过滤器配置结构体
FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_1;
FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_2;
FDCAN_FilterTypeDef FDCAN_Default_FilterConfig_3;

/**
 * @brief 所有FDCAN的初始化，包括过滤器的配置以及开启所有的CAN
*/
void FDCan_Start(FDCAN_FilterTypeDef *FDCAN_FilterConfig_1)
{
//	FDCAN_FilterConfig->IdType = FDCAN_STANDARD_ID;
	FDCAN_FilterConfig_1->FilterIndex = 0;
	FDCAN_FilterConfig_1->FilterType = FDCAN_FILTER_MASK;
	FDCAN_FilterConfig_1->FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	FDCAN_FilterConfig_1->FilterID1 = 0x00000000;
	FDCAN_FilterConfig_1->FilterID2 = 0x00000000;
	
	HAL_FDCAN_ConfigFilter(&hfdcan1, FDCAN_FilterConfig_1);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan1);
	
	HAL_FDCAN_ConfigFilter(&hfdcan2, FDCAN_FilterConfig_1);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan2);

	HAL_FDCAN_ConfigFilter(&hfdcan3, FDCAN_FilterConfig_1);
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
	HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	HAL_FDCAN_Start(&hfdcan3);
}


/**
 * @brief FDCAN接收中断回调函数
*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR(); // 临界段代码保护

	uint8_t FDCAN1_Data_Rx[8];	
	uint8_t FDCAN2_Data_Rx[8];
	uint8_t FDCAN3_Data_Rx[8];

    if(hfdcan->Instance == FDCAN1)
    {
        HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&(FDCAN1_RxFrame_m3508.RxHeader), FDCAN1_Data_Rx);
        if((FDCAN1_RxFrame_m3508.RxHeader.Identifier & 0x0ff0) == 0x0200)
        {
			M3508_Return_Process(&(FDCAN1_RxFrame_m3508.RxHeader), FDCAN1_Data_Rx, m3508_fdcan1);
        }	
    }
    if(hfdcan->Instance == FDCAN2)
    {
        HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&(FDCAN2_RxFrame_m3508.RxHeader), FDCAN2_Data_Rx);
        if((FDCAN2_RxFrame_m3508.RxHeader.Identifier & 0x0ff0) == 0x0200)
        {
			M3508_Return_Process(&(FDCAN2_RxFrame_m3508.RxHeader), FDCAN2_Data_Rx, m3508_fdcan2);
        }	
    }
	if(hfdcan->Instance == FDCAN3)
	{
		HAL_FDCAN_GetRxMessage(hfdcan,FDCAN_RX_FIFO0,&(FDCAN3_RxFrame_m3508.RxHeader), FDCAN3_Data_Rx);
		if((FDCAN3_RxFrame_m3508.RxHeader.Identifier & 0x0ff0) == 0x0200)
		{
			M3508_Return_Process(&(FDCAN3_RxFrame_m3508.RxHeader), FDCAN3_Data_Rx, m3508_fdcan3);
		}
	}

	taskEXIT_CRITICAL_FROM_ISR(status_value);
}

