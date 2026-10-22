#ifndef __INPUT_H
#define __INPUT_H

#include "stm32f1xx_hal.h"

// 面板按键编号枚举
typedef enum {
    INPUT_KEY_UP    = 0,
    INPUT_KEY_DOWN  = 1,
    INPUT_KEY_LEFT  = 2,
    INPUT_KEY_RIGHT = 3
} Input_Key_TypeDef;

/**
 * @brief  获取指定按键的状态
 * @param  key: 要查询的按键
 * @retval 1-按下, 0-未按下
 */
uint8_t Input_Get(Input_Key_TypeDef key);

#endif
