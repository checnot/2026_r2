#include "ht10a.h"

int16_t _channels[16];
uint8_t ht10a_buffer[30];				//遥控器原始数据

REMOTE_STU HT10A;

/**
 * 接受HT10A的数据并进行处理
 * @param buffer用于接收原始数据
 * @param _channels用于接收处理过后的数据
*/
void HT10A_process(uint8_t buffer[30])
{
	
  HT10A.last_status.knob_a_last = HT10A.knob_a;
  HT10A.last_status.knob_b_last = HT10A.knob_b;
  HT10A.last_status.left_x_last = HT10A.left_x;
  HT10A.last_status.left_y_last = HT10A.left_y;
  HT10A.last_status.right_x_last = HT10A.right_x;
  HT10A.last_status.right_y_last = HT10A.right_y;
  HT10A.last_status.switch_a_last = HT10A.switch_a;
  HT10A.last_status.switch_b_last = HT10A.switch_b;
  HT10A.last_status.switch_c_last = HT10A.switch_c;
  HT10A.last_status.switch_d_last = HT10A.switch_d;

  if (buffer[0] != 0x0F || buffer[24] != 0x00)
  return; // 数据包头尾正确则开始处理

  _channels[0] = (buffer[1] | ((buffer[2] & 0x07) << 8)) & 0x07FF;

  // 通道1: bits 11-21 (byte2低5位 + byte3高6位)
  _channels[1] = ((buffer[2] >> 3) | (buffer[3] << 5)) & 0x07FF;
  // 通道2: bits 22-32 (byte3低2位 + byte4全8位 + byte5高1位)
  _channels[2] = ((buffer[3] >> 6) | (buffer[4] << 2) | ((buffer[5] & 0x01) << 10)) & 0x07FF;
  // 通道3: bits 33-43 (byte5低7位 + byte6高4位)
  _channels[3] = ((buffer[5] >> 1) | (buffer[6] << 7)) & 0x07FF;
  // 通道4: bits 44-54 (byte6低4位 + byte7高7位)
  _channels[4] = ((buffer[6] >> 4) | (buffer[7] << 4)) & 0x07FF;
  // 通道5: bits 55-65 (byte7低1位 + byte8全8位 + byte9高2位)
  _channels[5] = ((buffer[7] >> 7) | (buffer[8] << 1) | ((buffer[9] & 0x03) << 9)) & 0x07FF;
  // 通道6: bits 66-76 (byte9低6位 + byte10高5位)
  _channels[6] = ((buffer[9] >> 2) | (buffer[10] << 6)) & 0x07FF;
  // 通道7: bits 77-87 (byte10低3位 + byte11全8位)
  _channels[7] = ((buffer[10] >> 5) | (buffer[11] << 3)) & 0x07FF;
  // 通道8: bits 88-98 (byte12低8位 + byte13高3位)
  _channels[8] = (buffer[12] | ((buffer[13] & 0x07) << 8)) & 0x07FF;
  // 通道9: bits 99-109 (byte13低5位 + byte14高6位)
  _channels[9] = ((buffer[13] >> 3) | (buffer[14] << 5)) & 0x07FF;
  // 通道10: bits 110-120 (byte14低2位 + byte15全8位 + byte16高1位)
  _channels[10] = ((buffer[14] >> 6) | (buffer[15] << 2) | ((buffer[16] & 0x01) << 10)) & 0x07FF;
  // 通道11: bits 121-131 (byte16低7位 + byte17高4位)
  _channels[11] = ((buffer[16] >> 1) | (buffer[17] << 7)) & 0x07FF;
  // 通道12: bits 132-142 (byte17低4位 + byte18高7位)
  _channels[12] = ((buffer[17] >> 4) | (buffer[18] << 4)) & 0x07FF;
  // 通道13: bits 143-153 (byte18低1位 + byte19全8位 + byte20高2位)
  _channels[13] = ((buffer[18] >> 7) | (buffer[19] << 1) | ((buffer[20] & 0x03) << 9)) & 0x07FF;
  // 通道14: bits 154-164 (byte20低6位 + byte21高5位)
  _channels[14] = ((buffer[20] >> 2) | (buffer[21] << 6)) & 0x07FF;
  // 通道15: bits 165-175 (byte21低3位 + byte22全8位)
  _channels[15] = ((buffer[21] >> 5) | (buffer[22] << 3)) & 0x07FF;

  //	摇杆[-4000,4000]
  HT10A.left_x = (_channels[3] - 992) * 5;
  HT10A.left_y = (_channels[2] - 992) * 5;
  HT10A.right_x = (_channels[0] - 992) * 5;		//9.18更新负号
  HT10A.right_y = (_channels[1] - 992) * 5;

  //	开关-1,0,1
  HT10A.switch_a = (_channels[4] - 992) / 800;
  HT10A.switch_b = (_channels[5] - 992) / 800;
  HT10A.switch_c = (_channels[6] - 992) / 800;
  HT10A.switch_d = (_channels[7] - 992) / 800;

  //	旋钮[-8000,8000]
  HT10A.knob_a = (_channels[8] - 992) * 10;
  HT10A.knob_b = (_channels[9] - 992) * 10;

/*=========================以下为特定判断逻辑，移植时可注释==================================*/

  // if(HT10A.switch_a == 1 && HT10A.last_status.switch_a_last == 0)
  // {
  //     jump_flag_debug = 1;
  // }
  // else
  // {
  //     jump_flag_debug = 0;
  // }

}

// /**
//  * @brief ht10a任务函数，在freertos中对应的task里调用
// */
// void HT10A_task(void)
// {
// //	memset(buffer, 0, sizeof(buffer));
//   HAL_UARTEx_ReceiveToIdle_DMA(&huart10, buffer, 30);
//   __HAL_DMA_DISABLE_IT(&hdma_uart7_rx,DMA_IT_HT);
// 	HT10A_process(buffer);
	
// }

