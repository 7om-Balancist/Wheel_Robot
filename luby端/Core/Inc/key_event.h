#ifndef __KEY_EVENT_H
#define __KEY_EVENT_H

#include "input.h"
#include <stdint.h>

#define KEY_EVENT_COUNT 4U

typedef enum {
    KEY_EVENT_DOWN = 0,
    KEY_EVENT_UP,
    KEY_EVENT_RIGHT,
    KEY_EVENT_LEFT
} KeyEvent_KeyTypeDef;

void KeyEvent_Reset(void);
void KeyEvent_Update(uint32_t now_ms);
uint8_t KeyEvent_IsPressed(KeyEvent_KeyTypeDef key);
uint8_t KeyEvent_GetPressedEvent(KeyEvent_KeyTypeDef key);
uint8_t KeyEvent_GetReleasedEvent(KeyEvent_KeyTypeDef key);
uint8_t KeyEvent_GetRepeatEvent(KeyEvent_KeyTypeDef key, uint32_t delay_ms, uint32_t period_ms);

#endif
