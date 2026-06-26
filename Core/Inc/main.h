/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint32_t timer_ms;//定时器计时变量，单位ms
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RS485_1_TX_Pin GPIO_PIN_8
#define RS485_1_TX_GPIO_Port GPIOD
#define RS485_1_RX_Pin GPIO_PIN_9
#define RS485_1_RX_GPIO_Port GPIOD
#define RS485_1_RE_Pin GPIO_PIN_10
#define RS485_1_RE_GPIO_Port GPIOD
#define clamp_catch_Pin GPIO_PIN_14
#define clamp_catch_GPIO_Port GPIOD
#define clamp_weidong_Pin GPIO_PIN_15
#define clamp_weidong_GPIO_Port GPIOD
#define clamp_weidong_EXTI_IRQn EXTI15_10_IRQn
#define RS485_2_TX_Pin GPIO_PIN_6
#define RS485_2_TX_GPIO_Port GPIOC
#define RS485_2_RX_Pin GPIO_PIN_7
#define RS485_2_RX_GPIO_Port GPIOC
#define RS485_2_RE_Pin GPIO_PIN_8
#define RS485_2_RE_GPIO_Port GPIOC
#define RS485_3_TX_Pin GPIO_PIN_10
#define RS485_3_TX_GPIO_Port GPIOC
#define RS485_3_RX_Pin GPIO_PIN_11
#define RS485_3_RX_GPIO_Port GPIOC
#define RS485_3_RE_Pin GPIO_PIN_0
#define RS485_3_RE_GPIO_Port GPIOD
#define front_wheel_weidong_Pin GPIO_PIN_1
#define front_wheel_weidong_GPIO_Port GPIOD
#define front_wheel_weidong_EXTI_IRQn EXTI1_IRQn
#define back_wheel_weidong_Pin GPIO_PIN_2
#define back_wheel_weidong_GPIO_Port GPIOD
#define back_wheel_weidong_EXTI_IRQn EXTI2_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
