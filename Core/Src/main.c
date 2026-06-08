/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "fdcan.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "own_lib.h"
#include "mycan.h"
#include "myuart.h"
#include "filter.h"
#include "observer.h"
#include "pid.h"
#include "m3508.h"
#include "a1_motor.h"
#include "ht10a.h"
#include "hi14.h"
#include "clamp.h"
#include "go_m8010.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern osSemaphoreId bisem_m8010Handle;   // ← 新增（v1 中类型名可能是 osSemaphoreId）


uint8_t timer_a1_flag = 0;//用于使定时器中对A1电机依次发送的标志位，0-3分别对应4个电机

uint32_t timer_ms = 0;//定时器计时变量，单位ms

// uint8_t m3508_send_flag = 0;//用于使定时器中对M3508电机依次发送的标志位
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_TIM2_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USART10_UART_Init();
  MX_UART7_Init();
  MX_UART7_Init();
  /* USER CODE BEGIN 2 */
  
//FDCAN初始化，配置过滤器并开启CAN
  FDCan_Start(&FDCAN_Default_FilterConfig_1);

//uart1-a1
  // memset(a1_usart1_raw_recv.RX_pData, 0, sizeof(a1_usart1_raw_recv.RX_pData));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, a1_usart1_raw_recv.RX_pData, RX_LEN1);
  // __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
//uart9-a1
  // memset(a1_usart9_raw_recv.RX_pData, 0, sizeof(a1_usart9_raw_recv.RX_pData));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart9, a1_usart9_raw_recv.RX_pData, RX_LEN1);
  // __HAL_DMA_DISABLE_IT(&hdma_uart9_rx, DMA_IT_HT);
//uart8-ht10a
  // memset(ht10a_buffer, 0, sizeof(ht10a_buffer));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart8, ht10a_buffer, sizeof(ht10a_buffer));
  // __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
//uart6-ht10a-仅调试用，实际不适用
  // memset(ht10a_buffer, 0, sizeof(ht10a_buffer));
  // HAL_UARTEx_ReceiveToIdle_DMA(&huart6, ht10a_buffer, sizeof(ht10a_buffer));
  // __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
   // uart10-hi14
      memset(dma_rx_buf, 0, sizeof(dma_rx_buf));
      HAL_UARTEx_ReceiveToIdle_DMA(&huart10, dma_rx_buf, sizeof(dma_rx_buf));
      __HAL_DMA_DISABLE_IT(&hdma_usart10_rx, DMA_IT_HT);

//uart5-radar
  memset(Rec_Data, 0, sizeof(Rec_Data));
  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, Rec_Data, sizeof(Rec_Data));
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);


// 雷达卡尔曼滤波器初始化
  Kalman_Init(&Radar_Kalman_X, 0.0001f, 100.0f, 0.0f, 400.0f);
  Kalman_Init(&Radar_Kalman_Y, 0.0001f, 100.0f, 0.0f, 1400.0f);
  Kalman_Init(&Radar_Kalman_Z, 0.0001f, 100.0f, 0.0f, 140.0f);
  Kalman_Init(&Radar_Kalman_Yaw, 0.0001f, 100.0f, 0.0f, 0.0f); 

// go_m8010电机总线初始化
  m8010_bus_init(&m8010_bus_1, &huart3, &htim6);
  m8010_bus_init(&m8010_bus_2, &huart6, &htim6);
  m8010_bus_init(&m8010_bus_3, &huart4, &htim6);

  // A1_Motor_Parameter_Init(&A1_debug_485_2[0], 0, 0, 0, 0);
  // A1_Motor_Parameter_Init(&A1_debug_485_2[1], 1, 0, 0, 0);



//clamp_position_init(&m3508_fdcan2[1],&m3508_fdcan2[3],&m3508_fdcan2[2]);
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	uint32_t status_value;
	status_value = taskENTER_CRITICAL_FROM_ISR();

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

//TIM2定时累加时间，每1ms累加一次，单位为ms
  if(htim->Instance == TIM2)
  {
    timer_ms++;
  }

//TIM6定时发送数据控制m8010电机，0.5ms会发送一次，每次发送一个电机的数据，依次循环4个电机
	if(htim->Instance == TIM6)
	{
		//  switch(timer_a1_flag)
		//  {
		//  	// case 0:{A1_Motor_Send_Data(&A1_debug_485_1[0], &huart1); break;}
		//  	// case 1:{A1_Motor_Send_Data(&A1_debug_485_1[1], &huart1); break;}
		//   // case 2:{A1_Motor_Send_Data(&A1_debug_485_2[0], &huart9); break;}
		//  	// case 3:{A1_Motor_Send_Data(&A1_debug_485_2[1], &huart9); break;}
		//  }
		//  timer_a1_flag++;
		//  if (timer_a1_flag > 3)	timer_a1_flag = 0;

    m8010_485_timer_trigger(htim);

    // 必须要等 RTOS 初始化完毕，信号量句柄不为空才能 Give 信号量
    if (bisem_m8010Handle != NULL)
    {
        // 去掉 static，并确保每次进中断时初始化为 pdFALSE
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(bisem_m8010Handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
	}
//TIM7定时发送数据控制m3508电机，1ms会发送一次
  if(htim->Instance == TIM7)
  {

      FDCAN_Send_M3508(&hfdcan1, m3508_fdcan1);
      FDCAN_Send_M3508(&hfdcan2, m3508_fdcan2);
      FDCAN_Send_M3508(&hfdcan3, m3508_fdcan3);

  }
	taskEXIT_CRITICAL_FROM_ISR(status_value);// 退出临界段

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
