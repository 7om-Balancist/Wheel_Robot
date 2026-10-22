/**
 * @file    callbacks.c
 * @brief   本工程不需要任何 HAL 回调
 *
 * flash 测试是纯轮询的裸机程序: 不用 ADC 的 DMA 回调、不用任何 UART 接收
 * 中断、也不跑 FreeRTOS。这里保留一个空文件是为了让三个工程的目录结构一致。
 */
#include "stm32f1xx_hal.h"
