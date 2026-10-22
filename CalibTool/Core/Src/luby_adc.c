#include "luby_adc.h"
#include <string.h>

/* Each half of this circular DMA buffer is one complete A0..A15 scan. */
static uint16_t adc_dma_buffer[2][LUBY_ADC_CHANNEL_COUNT];
static volatile uint8_t adc_completed_bank;
static volatile uint32_t adc_sequence;

HAL_StatusTypeDef ADC_Init(void)
{
    HAL_StatusTypeDef status;

    memset(adc_dma_buffer, 0, sizeof(adc_dma_buffer));
    adc_completed_bank = 0U;
    adc_sequence = 0U;

    status = HAL_ADCEx_Calibration_Start(&hadc1);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_ADC_Start_DMA(&hadc1,
                             (uint32_t *)adc_dma_buffer,
                             2U * LUBY_ADC_CHANNEL_COUNT);
}

uint16_t ADC_GetValue(uint8_t port_num)
{
    uint8_t bank;

    if ((port_num >= LUBY_ADC_CHANNEL_COUNT) || (adc_sequence == 0U)) {
        return 0U;
    }

    bank = adc_completed_bank;
    return adc_dma_buffer[bank][port_num];
}

void ADC_GetAllValues(uint16_t out_array[LUBY_ADC_CHANNEL_COUNT])
{
    uint8_t bank;

    if (out_array == NULL) {
        return;
    }

    if (adc_sequence == 0U) {
        memset(out_array, 0, LUBY_ADC_CHANNEL_COUNT * sizeof(uint16_t));
        return;
    }

    bank = adc_completed_bank;
    memcpy(out_array, adc_dma_buffer[bank],
           LUBY_ADC_CHANNEL_COUNT * sizeof(uint16_t));
}

void ADC_OnDmaHalfComplete(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1)) {
        adc_completed_bank = 0U;
        adc_sequence++;
    }
}

void ADC_OnDmaComplete(ADC_HandleTypeDef *hadc)
{
    if ((hadc != NULL) && (hadc->Instance == ADC1)) {
        adc_completed_bank = 1U;
        adc_sequence++;
    }
}

uint32_t ADC_GetSequence(void)
{
    return adc_sequence;
}
