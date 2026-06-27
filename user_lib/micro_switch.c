#include "micro_switch.h"

uint8_t front = 0;
uint8_t back = 0;
uint8_t clamp_catch_flag = 0;
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t status_value;
  status_value = taskENTER_CRITICAL_FROM_ISR();

  //夹爪微动开关
  if (GPIO_Pin == clamp_weidong_Pin)
  {
    if (HAL_GPIO_ReadPin(clamp_weidong_GPIO_Port, clamp_weidong_Pin) == 0)
    {
     // osDelay(20);
//      if (HAL_GPIO_ReadPin(clamp_weidong_GPIO_Port, clamp_weidong_Pin) == 0)
//      {
         MC_bb.RACK_PICK_SUCCESS = 1;
			  clamp_catch_flag =1;
    //  }
    }
    else 
    {
        MC_bb.RACK_PICK_SUCCESS = 0;
        MC_bb.RACK_PICK_SUCCESS = 0;
			  clamp_catch_flag =0;
    }
    
  }
  // //前辅助轮微动开关
  //  if (GPIO_Pin == front_wheel_weidong_Pin)
  // {
  //   if (HAL_GPIO_ReadPin(front_wheel_weidong_GPIO_Port, front_wheel_weidong_Pin) == 0)
  //   {
  //     front = 1;
  //   }
  //   else
  //     front =0;
  // }
  // //后辅助轮微动开关
  // if (GPIO_Pin == back_wheel_weidong_Pin)
  // {
  //  if (HAL_GPIO_ReadPin(back_wheel_weidong_GPIO_Port, back_wheel_weidong_Pin) == 0)
  //   {
  //     back = 1;
  //   }
  //   else
  //     back = 0;
  // }
  taskEXIT_CRITICAL_FROM_ISR(status_value); // 退出临界段
}

