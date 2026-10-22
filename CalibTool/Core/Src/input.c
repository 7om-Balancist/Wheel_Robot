#include "input.h"

/**
 * @brief  获取指定按键的状态
 */
uint8_t Input_Get(Input_Key_TypeDef key)
{
    GPIO_TypeDef* port;
    uint16_t pin;

    // 映射表：根据 LUBY 原理图定义的按键引脚
    switch(key) {
        case INPUT_KEY_DOWN:  port = GPIOD; pin = GPIO_PIN_13; break; // KEY1
        case INPUT_KEY_UP:    port = GPIOD; pin = GPIO_PIN_14; break; // KEY2
        case INPUT_KEY_RIGHT: port = GPIOD; pin = GPIO_PIN_12; break; // KEY3 / BACK
        case INPUT_KEY_LEFT:  port = GPIOD; pin = GPIO_PIN_15; break; // KEY4 / OK
        default: return 0;
    }

    // 读取 GPIO 电平 (假设按下为低电平 GPIO_PIN_RESET)
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
}
