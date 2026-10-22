#ifndef __LUBY_LCD_H
#define __LUBY_LCD_H

#include <stdint.h>

#define LUBY_LCD_ROWS 4U
#define LUBY_LCD_COLUMNS 16U

typedef void (*LubyLCD_TraceCallback)(const char *message);

void LubyLCD_Init(void);
void LubyLCD_InitWithTrace(LubyLCD_TraceCallback trace);
void LubyLCD_SetContrast(uint8_t contrast);
void LubyLCD_SetInverse(uint8_t enable);
void LubyLCD_Clear(void);
void LubyLCD_ShowLine(uint8_t row, const char *text);

#endif
