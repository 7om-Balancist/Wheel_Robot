#ifndef __LUBY_ADC_H
#define __LUBY_ADC_H

#include "stm32f1xx_hal.h"

#define LUBY_ADC_CHANNEL_COUNT 16U

extern ADC_HandleTypeDef hadc1;

HAL_StatusTypeDef ADC_Init(void);
uint16_t ADC_GetValue(uint8_t port_num);
void ADC_GetAllValues(uint16_t out_array[LUBY_ADC_CHANNEL_COUNT]);
void ADC_OnDmaHalfComplete(ADC_HandleTypeDef *hadc);
void ADC_OnDmaComplete(ADC_HandleTypeDef *hadc);
uint32_t ADC_GetSequence(void);

#endif
