#include "key_event.h"

#define KEY_EVENT_DEBOUNCE_SAMPLES 4U

typedef struct {
    uint8_t stable;
    uint8_t previous_raw;
    uint8_t same_count;
    uint8_t pressed_event;
    uint8_t released_event;
    uint32_t repeat_time;
} KeyEvent_State;

static uint32_t key_event_now_ms;

static KeyEvent_State key_states[KEY_EVENT_COUNT];

static Input_Key_TypeDef input_key_from_event(KeyEvent_KeyTypeDef key)
{
    switch (key) {
        case KEY_EVENT_DOWN:  return INPUT_KEY_DOWN;
        case KEY_EVENT_UP:    return INPUT_KEY_UP;
        case KEY_EVENT_RIGHT: return INPUT_KEY_RIGHT;
        case KEY_EVENT_LEFT:  return INPUT_KEY_LEFT;
        default:              return INPUT_KEY_UP;
    }
}

void KeyEvent_Reset(void)
{
    key_event_now_ms = 0U;

    for (uint8_t i = 0U; i < KEY_EVENT_COUNT; i++) {
        key_states[i].stable = 0U;
        key_states[i].previous_raw = 0U;
        key_states[i].same_count = 0U;
        key_states[i].pressed_event = 0U;
        key_states[i].released_event = 0U;
        key_states[i].repeat_time = 0U;
    }
}

void KeyEvent_Update(uint32_t now_ms)
{
    key_event_now_ms = now_ms;

    for (uint8_t i = 0U; i < KEY_EVENT_COUNT; i++) {
        uint8_t raw = Input_Get(input_key_from_event((KeyEvent_KeyTypeDef)i));
        KeyEvent_State *state = &key_states[i];

        state->pressed_event = 0U;
        state->released_event = 0U;

        if (raw == state->previous_raw) {
            if (state->same_count < KEY_EVENT_DEBOUNCE_SAMPLES) {
                state->same_count++;
            }
        } else {
            state->previous_raw = raw;
            state->same_count = 1U;
        }

        if ((state->same_count >= KEY_EVENT_DEBOUNCE_SAMPLES) && (state->stable != raw)) {
            state->stable = raw;
            if (raw != 0U) {
                state->pressed_event = 1U;
            } else {
                state->released_event = 1U;
                state->repeat_time = 0U;
            }
        }
    }
}

uint8_t KeyEvent_IsPressed(KeyEvent_KeyTypeDef key)
{
    if ((uint8_t)key >= KEY_EVENT_COUNT) {
        return 0U;
    }

    return key_states[(uint8_t)key].stable;
}

uint8_t KeyEvent_GetPressedEvent(KeyEvent_KeyTypeDef key)
{
    if ((uint8_t)key >= KEY_EVENT_COUNT) {
        return 0U;
    }

    return key_states[(uint8_t)key].pressed_event;
}

uint8_t KeyEvent_GetReleasedEvent(KeyEvent_KeyTypeDef key)
{
    if ((uint8_t)key >= KEY_EVENT_COUNT) {
        return 0U;
    }

    return key_states[(uint8_t)key].released_event;
}

uint8_t KeyEvent_GetRepeatEvent(KeyEvent_KeyTypeDef key, uint32_t delay_ms, uint32_t period_ms)
{
    KeyEvent_State *state;

    if ((uint8_t)key >= KEY_EVENT_COUNT) {
        return 0U;
    }

    state = &key_states[(uint8_t)key];

    if (state->pressed_event != 0U) {
        state->repeat_time = key_event_now_ms + delay_ms;
        return 0U;
    }

    if (state->stable == 0U) {
        state->repeat_time = 0U;
        return 0U;
    }

    if (state->repeat_time == 0U) {
        state->repeat_time = key_event_now_ms + delay_ms;
        return 0U;
    }

    if ((int32_t)(key_event_now_ms - state->repeat_time) >= 0) {
        state->repeat_time = key_event_now_ms + period_ms;
        return 1U;
    }

    return 0U;
}
