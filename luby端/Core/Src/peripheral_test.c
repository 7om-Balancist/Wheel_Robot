#include "peripheral_test.h"
#include "luby_adc.h"
#include "luby_lcd.h"
#include "key_event.h"
#include "drive.h"
#include "uart_debug.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

#define TEST_MOTOR_SPEED       100
#define TEST_MOTOR_TIME_MS     300U
#define TEST_LCD_PERIOD_MS     100U
#define TEST_PRINT_PERIOD_MS   200U
#define TEST_EDGE_LIMIT        3600U
#define TEST_KEY_REPEAT_DELAY_MS   300U
#define TEST_KEY_REPEAT_PERIOD_MS  120U
#define TEST_LCD_CONTRAST_DEFAULT 0x0CU
#define TEST_LCD_CONTRAST_MIN     0x00U
#define TEST_LCD_CONTRAST_MAX     0x1FU
#define TEST_LCD_CONTRAST_STEP    1U

typedef enum {
    TEST_PAGE_STATUS = 0,
    TEST_PAGE_ADC_0_7,
    TEST_PAGE_ADC_8_15,
    TEST_PAGE_COUNT
} TestPage;

typedef enum {
    TEST_MODE_VIEW = 0,
    TEST_MODE_MOTOR
} TestMode;

static TestPage current_page;
static TestMode current_mode;
static uint8_t left_motor_running;
static uint8_t right_motor_running;
static uint32_t motor_stop_time;
static uint8_t lcd_contrast;
static uint32_t contrast_repeat_time[2];
static char last_lcd_lines[LUBY_LCD_ROWS][LUBY_LCD_COLUMNS + 1U];

static void print_event_serial(const char *message)
{
    UartDebug_Print(message);
}

static void stop_motors(void)
{
    (void)Drive_Stop();
    left_motor_running = 0U;
    right_motor_running = 0U;
    motor_stop_time = 0U;
}

static void toggle_mode(void)
{
    if (current_mode == TEST_MODE_MOTOR) stop_motors();
    current_mode = (current_mode == TEST_MODE_VIEW) ? TEST_MODE_MOTOR : TEST_MODE_VIEW;
    current_page = TEST_PAGE_STATUS;
    print_event_serial(current_mode == TEST_MODE_MOTOR
        ? "KEY DOWN MODE MOTOR\r\n" : "KEY DOWN MODE VIEW\r\n");
}

static void next_page(void)
{
    current_page = (TestPage)(((uint8_t)current_page + 1U) % (uint8_t)TEST_PAGE_COUNT);
    if (current_page == TEST_PAGE_STATUS) print_event_serial("KEY UP PAGE STATUS\r\n");
    else if (current_page == TEST_PAGE_ADC_0_7) print_event_serial("KEY UP PAGE A0-A7\r\n");
    else print_event_serial("KEY UP PAGE A8-A15\r\n");
}

static void adjust_lcd_contrast(int8_t direction)
{
    char message[32];

    if (direction > 0) {
        if (lcd_contrast <= (uint8_t)(TEST_LCD_CONTRAST_MAX - TEST_LCD_CONTRAST_STEP)) {
            lcd_contrast = (uint8_t)(lcd_contrast + TEST_LCD_CONTRAST_STEP);
        } else {
            lcd_contrast = TEST_LCD_CONTRAST_MAX;
        }
    } else {
        if (lcd_contrast >= (uint8_t)(TEST_LCD_CONTRAST_MIN + TEST_LCD_CONTRAST_STEP)) {
            lcd_contrast = (uint8_t)(lcd_contrast - TEST_LCD_CONTRAST_STEP);
        } else {
            lcd_contrast = TEST_LCD_CONTRAST_MIN;
        }
    }
    LubyLCD_SetContrast(lcd_contrast);
    last_lcd_lines[3][0] = '\0';
    (void)snprintf(message, sizeof(message), "LCD C=%02X DIR=%c\r\n",
                   (unsigned int)lcd_contrast,
                   (direction > 0) ? '+' : '-');
    print_event_serial(message);
}

static void start_motor_test(uint8_t left)
{
    if (current_mode != TEST_MODE_MOTOR) return;

    if (left != 0U) {
        (void)Drive_SetSpeed(TEST_MOTOR_SPEED, 0);
        left_motor_running = 1U;
        right_motor_running = 0U;
    } else {
        (void)Drive_SetSpeed(0, TEST_MOTOR_SPEED);
        left_motor_running = 0U;
        right_motor_running = 1U;
    }
    motor_stop_time = HAL_GetTick() + TEST_MOTOR_TIME_MS;
}

static void process_key_events(void)
{
    /* INPUT_KEY_DOWN / K1 -> toggle_mode, now provided through KEY_EVENT_DOWN. */
    if (KeyEvent_GetPressedEvent(KEY_EVENT_DOWN) != 0U) { toggle_mode(); }
    /* INPUT_KEY_UP / K2 -> next_page, now provided through KEY_EVENT_UP. */
    if (KeyEvent_GetPressedEvent(KEY_EVENT_UP) != 0U) { next_page(); }
    if (KeyEvent_GetPressedEvent(KEY_EVENT_RIGHT) != 0U) {
        if (current_mode == TEST_MODE_MOTOR) {
            print_event_serial("KEY RIGHT MOTOR TEST\r\n");
            start_motor_test(0U);
        } else {
            adjust_lcd_contrast(1);
        }
    }
    if (KeyEvent_GetPressedEvent(KEY_EVENT_LEFT) != 0U) {
        if (current_mode == TEST_MODE_MOTOR) {
            print_event_serial("KEY LEFT MOTOR TEST\r\n");
            start_motor_test(1U);
        } else {
            adjust_lcd_contrast(-1);
        }
    }
}

static void process_contrast_keys(uint32_t now)
{
    (void)now;
    /* released_event is now owned by key_event.c and accessed via KeyEvent_GetReleasedEvent(). */

    if (current_mode != TEST_MODE_VIEW) {
        return;
    }

    if (KeyEvent_GetReleasedEvent(KEY_EVENT_RIGHT) != 0U) {
        print_event_serial("KEY RIGHT RELEASE\r\n");
    } else if (KeyEvent_GetRepeatEvent(KEY_EVENT_RIGHT,
                                       TEST_KEY_REPEAT_DELAY_MS,
                                       TEST_KEY_REPEAT_PERIOD_MS) != 0U) {
        adjust_lcd_contrast(1);
    }

    if (KeyEvent_GetReleasedEvent(KEY_EVENT_LEFT) != 0U) {
        print_event_serial("KEY LEFT RELEASE\r\n");
    } else if (KeyEvent_GetRepeatEvent(KEY_EVENT_LEFT,
                                       TEST_KEY_REPEAT_DELAY_MS,
                                       TEST_KEY_REPEAT_PERIOD_MS) != 0U) {
        adjust_lcd_contrast(-1);
    }
}

static void show_line_if_changed(uint8_t row, const char *text)
{
    char formatted[LUBY_LCD_COLUMNS + 1U];

    if ((row >= LUBY_LCD_ROWS) || (text == NULL)) return;
    (void)snprintf(formatted, sizeof(formatted), "%-16.16s", text);
    if (strncmp(last_lcd_lines[row], formatted, LUBY_LCD_COLUMNS) != 0) {
        (void)memcpy(last_lcd_lines[row], formatted, sizeof(formatted));
        LubyLCD_ShowLine(row, formatted);
    }
}

static void render_status_page(void)
{
    char line[17];
    (void)snprintf(line, sizeof(line), "K1:%u K2:%u",
                   KeyEvent_IsPressed(KEY_EVENT_DOWN),
                   KeyEvent_IsPressed(KEY_EVENT_UP));
    show_line_if_changed(0U, line);
    (void)snprintf(line, sizeof(line), "K3:%u K4:%u",
                   KeyEvent_IsPressed(KEY_EVENT_RIGHT),
                   KeyEvent_IsPressed(KEY_EVENT_LEFT));
    show_line_if_changed(1U, line);
    (void)snprintf(line, sizeof(line), "M1:%s M2:%s",
                   left_motor_running ? "RUN" : "STOP",
                   right_motor_running ? "RUN" : "STOP");
    show_line_if_changed(2U, line);
    (void)snprintf(line, sizeof(line), "MD:%c P:%u C:%02X",
                   current_mode == TEST_MODE_MOTOR ? 'M' : 'V',
                   (unsigned int)current_page, (unsigned int)lcd_contrast);
    show_line_if_changed(3U, line);
}

static void write_two_digits(char *out, uint8_t value)
{
    out[0] = (char)('0' + ((value / 10U) % 10U));
    out[1] = (char)('0' + (value % 10U));
}

static void write_four_digits(char *out, uint16_t value)
{
    if (value > 4095U) value = 4095U;
    out[0] = (char)('0' + ((value / 1000U) % 10U));
    out[1] = (char)('0' + ((value / 100U) % 10U));
    out[2] = (char)('0' + ((value / 10U) % 10U));
    out[3] = (char)('0' + (value % 10U));
}

static void render_adc_page(const uint16_t adc[16], uint8_t first_channel)
{
    char line[16] = "00:0000 00:0000";
    for (uint8_t row = 0U; row < 4U; row++) {
        uint8_t first = (uint8_t)(first_channel + row * 2U);
        uint8_t second = (uint8_t)(first + 1U);
        write_two_digits(&line[0], first);
        write_four_digits(&line[3], adc[first]);
        write_two_digits(&line[8], second);
        write_four_digits(&line[11], adc[second]);
        line[15] = '\0';
        show_line_if_changed(row, line);
    }
}

static void render_lcd(const uint16_t adc[16])
{
    if (current_page == TEST_PAGE_STATUS) render_status_page();
    else if (current_page == TEST_PAGE_ADC_0_7) render_adc_page(adc, 0U);
    else render_adc_page(adc, 8U);
}

static void print_adc_serial(const uint16_t adc[16])
{
    char line[192];
    int length = snprintf(line, sizeof(line),
        "GRAY A0-A3:%u,%u,%u,%u | IR A4-A11:%u,%u,%u,%u,%u,%u,%u,%u | UNUSED A12-A15:%u,%u,%u,%u\r\n",
        adc[0], adc[1], adc[2], adc[3], adc[4], adc[5], adc[6], adc[7],
        adc[8], adc[9], adc[10], adc[11], adc[12], adc[13], adc[14], adc[15]);
    if (length > 0) {
        UartDebug_Write((const uint8_t *)line, (uint16_t)length);
    }
}

void PeripheralTest_Run(void)
{
    uint16_t adc[16];
    uint32_t last_lcd = 0U;
    uint32_t last_print = 0U;

    current_page = TEST_PAGE_STATUS;
    current_mode = TEST_MODE_VIEW;
    left_motor_running = 0U;
    right_motor_running = 0U;
    motor_stop_time = 0U;
    lcd_contrast = TEST_LCD_CONTRAST_DEFAULT;
    contrast_repeat_time[0] = 0U;
    contrast_repeat_time[1] = 0U;
    LubyLCD_SetContrast(lcd_contrast);
    LubyLCD_SetInverse(1U);
    KeyEvent_Reset();
    (void)memset(last_lcd_lines, 0xFF, sizeof(last_lcd_lines));

    for (;;) {
        uint32_t now = HAL_GetTick();
        ADC_GetAllValues(adc);
        KeyEvent_Update(now);
        process_key_events();
        process_contrast_keys(now);

        if ((adc[0] > TEST_EDGE_LIMIT) || (adc[1] > TEST_EDGE_LIMIT) ||
            (adc[2] > TEST_EDGE_LIMIT) || (adc[3] > TEST_EDGE_LIMIT) ||
            (KeyEvent_IsPressed(KEY_EVENT_RIGHT) && KeyEvent_IsPressed(KEY_EVENT_LEFT))) {
            stop_motors();
        } else if ((motor_stop_time != 0U) && ((int32_t)(now - motor_stop_time) >= 0)) {
            stop_motors();
        }

        if ((now - last_lcd) >= TEST_LCD_PERIOD_MS) {
            last_lcd = now;
            render_lcd(adc);
        }
        if ((now - last_print) >= TEST_PRINT_PERIOD_MS) {
            last_print = now;
            print_adc_serial(adc);
        }
        HAL_Delay(5U);
    }
}
