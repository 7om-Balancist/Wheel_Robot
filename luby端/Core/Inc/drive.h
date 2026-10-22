#ifndef __DRIVE_H
#define __DRIVE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

HAL_StatusTypeDef Drive_Init(void);
HAL_StatusTypeDef Drive_SetSpeed(int16_t left, int16_t right);
HAL_StatusTypeDef Drive_Stop(void);

#endif
