/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  *
  * 本工程只保留两个中断:
  *   TIM2          —— HAL 的 1ms 时基(见 stm32f1xx_hal_timebase_tim.c)
  *   DMA1_Channel1 —— ADC 循环采样搬运(luby_adc.c 的双缓冲)
  * 电机 USART3 / 蓝牙 / zigbee / 树莓派的中断本工程都不使用。
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_adc1;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/

void NMI_Handler(void)        { while (1) { } }
void HardFault_Handler(void)  { while (1) { } }
void MemManage_Handler(void)  { while (1) { } }
void BusFault_Handler(void)   { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void DebugMon_Handler(void)   { }

/**
  * @brief  SysTick 中断。
  * @note   本工程用 TIM2 做 HAL 时基, SysTick 并未使能, 正常不会进入。
  *         保留 HAL_IncTick() 只是保险。
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief  DMA1 通道1 中断 — ADC 循环采样完成/半完成
  *         内部经 HAL_DMA_IRQHandler → callbacks.c 的 ADC 回调
  */
void DMA1_Channel1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

/**
  * @brief  TIM2 中断 — 1ms HAL 时基
  */
void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2);
}
