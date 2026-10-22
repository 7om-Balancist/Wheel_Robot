#ifndef __UART_DEBUG_H
#define __UART_DEBUG_H

#include <stdint.h>

void UartDebug_Write(const uint8_t *data, uint16_t len);
void UartDebug_Print(const char *text);
void UartDebug_Printf(const char *fmt, ...);

#endif
