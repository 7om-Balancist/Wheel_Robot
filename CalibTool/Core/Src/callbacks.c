/**
 * @file    callbacks.c
 * @brief   HAL 回调分派 — 本工程只需要 ADC DMA 两个回调
 *
 * 标定工程要跑 LCD 引导的三步灰度采集, 所以需要 ADC 的 DMA 完成/半完成
 * 通知来切换双缓冲读取端(见 luby_adc.c)。
 * 蓝牙/zigbee/树莓派那些 UART 回调本工程一概不需要, 因此不在此分派。
 */
#include "stm32f1xx_hal.h"
#include "luby_adc.h"

/* ADC DMA 一整轮 16 通道扫描完成 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ADC_OnDmaComplete(hadc);
    }
}

/* ADC DMA 前半 8 通道完成(双缓冲切换) */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ADC_OnDmaHalfComplete(hadc);
    }
}
