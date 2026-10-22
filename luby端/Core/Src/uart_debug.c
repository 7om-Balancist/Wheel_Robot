#include "uart_debug.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define UART_DEBUG_TIMEOUT_MS 100U
#define UART_DEBUG_BUFFER_SIZE 192U

void UartDebug_Write(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)data, len, UART_DEBUG_TIMEOUT_MS);
}

void UartDebug_Print(const char *text)
{
    if (text == NULL) {
        return;
    }

    UartDebug_Write((const uint8_t *)text, (uint16_t)strlen(text));
}

void UartDebug_Printf(const char *fmt, ...)
{
    char buffer[UART_DEBUG_BUFFER_SIZE];
    va_list args;
    int length;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    length = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (length <= 0) {
        return;
    }

    if (length > (int)sizeof(buffer)) {
        length = (int)sizeof(buffer);
    }

    UartDebug_Write((const uint8_t *)buffer, (uint16_t)length);
}
