#include "luby_lcd.h"
#include "main.h"
#include "stm32f1xx_hal.h"

/* Same six GPIO signals and serial protocol as the original UPLib driver. */
#define LCD_CS_HIGH() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET)
#define LCD_CS_LOW() HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET)
#define LCD_RS_HIGH() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET)
#define LCD_RS_LOW() HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET)
#define LCD_SDA_HIGH() HAL_GPIO_WritePin(LCD_SDA_GPIO_Port, LCD_SDA_Pin, GPIO_PIN_SET)
#define LCD_SDA_LOW() HAL_GPIO_WritePin(LCD_SDA_GPIO_Port, LCD_SDA_Pin, GPIO_PIN_RESET)
#define LCD_CLK_HIGH() HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_SET)
#define LCD_CLK_LOW() HAL_GPIO_WritePin(LCD_CLK_GPIO_Port, LCD_CLK_Pin, GPIO_PIN_RESET)
#define LCD_RST_HIGH() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET)
#define LCD_RST_LOW() HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET)
#define LCD_BK_ON() HAL_GPIO_WritePin(LCD_BK_GPIO_Port, LCD_BK_Pin, GPIO_PIN_SET)

static void lcd_delay_us(uint32_t us)
{
    volatile uint32_t loops = (SystemCoreClock / 8000000U) * us;
    while (loops-- != 0U)
    {
        __NOP();
    }
}

static void lcd_write_byte(uint8_t value)
{
    /* ⚠ 每 bit 前后各 1µs: bit 率降到 ~500kHz。原版裸翻转 ~5MHz,
     * 远超 RA8816 内部 RC 振荡器的采样能力, 偶发 bit 采样错位 ——
     * 症状: 字符变成相邻字形(内容变形) / 模式字节被采错(整行位图)。
     * 代价: ShowLine 每行 +0.26ms, 无感。 */
    for (uint8_t bit = 0U; bit < 8U; bit++)
    {
        LCD_CLK_LOW();
        if ((value & 0x80U) != 0U)
            LCD_SDA_HIGH();
        else
            LCD_SDA_LOW();
        lcd_delay_us(1U);
        LCD_CLK_HIGH();
        lcd_delay_us(1U);
        value <<= 1U;
    }
    LCD_CLK_HIGH();
    LCD_SDA_HIGH();
}

static void lcd_write_command(uint8_t address, uint8_t command)
{
    LCD_CS_LOW();
    LCD_RS_LOW();
    LCD_CLK_LOW();
    LCD_SDA_LOW();
    LCD_CLK_HIGH();
    lcd_write_byte(address);
    lcd_write_byte(command);
    LCD_CS_HIGH();
    /* ⚠ 命令后必须延时: RA8816 内部 RC 振荡器慢且抖, 连续写寄存器时下一条
     * 命令会被吞掉(尤其 ShowLine 开头的 0x03=0x42 文字模式设置 —— 被吞后
     * 本行 ASCII 落进图形模式, 显示成"转置位图")。故障是概率性的:
     * 50us 实测仍偶发被吞, 500us 给足 30 倍余量。
     * 代价: ShowLine 每行 ~7ms, Clear ~0.5s, 全部可接受。 */
    lcd_delay_us(500U);
}

static void lcd_write_data(uint8_t data)
{
    LCD_CS_LOW();
    LCD_RS_HIGH();
    LCD_CLK_LOW();
    LCD_SDA_LOW();
    LCD_CLK_HIGH();
    lcd_write_byte(data);
    LCD_CS_HIGH();
    lcd_delay_us(350U);
}

void LubyLCD_Clear(void)
{
    lcd_write_command(0x03U, 0x40U);
    for (uint8_t row = 0U; row < 64U; row++)
    {
        lcd_write_command(0x05U, 0x00U);
        lcd_write_command(0x06U, row);
        for (uint8_t column = 0U; column < 16U; column++)
        {
            lcd_write_data(0x00U);
        }
    }
}

void LubyLCD_InitWithTrace(LubyLCD_TraceCallback trace)
{
    LCD_CS_HIGH();
    LCD_CLK_HIGH();
    LCD_SDA_HIGH();
    LCD_BK_ON();
    if (trace != NULL)
        trace("D02.1 LCD GPIO IDLE\r\n");

    LCD_RST_LOW();
    if (trace != NULL)
        trace("D02.2 LCD RESET LOW\r\n");
    HAL_Delay(300U); /* 100→300: 控制器内部 RC 振荡器起振需要时间, 拉长复位低电平 */
    LCD_RST_HIGH();
    if (trace != NULL)
        trace("D02.3 LCD RESET HIGH\r\n");
    HAL_Delay(300U); /* 同上: 复位释放后等振荡稳定再发命令 */

    if (trace != NULL)
        trace("D02.4 LCD COMMANDS\r\n");
    /* 初始化命令序列发两遍: 命令之间没有间隔延时, 控制器慢一拍时
     * 某条关键命令(如 0x01 方向)可能被吞, 发两遍兜底 */
    for (uint8_t pass = 0U; pass < 2U; pass++)
    {
        lcd_write_command(0x00U, 0x00U);
        lcd_write_command(0x01U, 0x00U);
        lcd_write_command(0x02U, 0x7BU);
        lcd_write_command(0x03U, 0x00U);
        lcd_write_command(0x04U, 0x00U);
        lcd_write_command(0x05U, 0x00U);
        lcd_write_command(0x06U, 0x00U);
        lcd_write_command(0x07U, 0x00U);
        lcd_write_command(0x08U, 0x00U);
        lcd_write_command(0x09U, 0x00U);
        lcd_write_command(0x0AU, 0x00U);
        lcd_write_command(0x0BU, 0x00U);
        lcd_write_command(0x0CU, 0x00U);
        lcd_write_command(0x0DU, 0x00U);
        lcd_write_command(0x0EU, 0x00U);
        lcd_write_command(0x0FU, 0x00U);
        lcd_write_command(0x13U, 0x00U);
        lcd_write_command(0x14U, 0x00U);
        lcd_write_command(0x15U, 0x00U);
        lcd_write_command(0x16U, 0x00U);
        lcd_write_command(0x17U, 0x00U);
        lcd_write_command(0x18U, 0x00U);
        lcd_write_command(0x10U, 0x2CU);
        lcd_write_command(0x12U, 0x24U);
        lcd_write_command(0x11U, 0xF8U);
        lcd_write_command(0x01U, 0x02U);
        lcd_write_command(0x11U, 0xF8U); /* Original UP_LCD_Turn(0) call. */
    } /* pass 循环闭合: 序列共发两遍 */

    if (trace != NULL)
        trace("D02.5 LCD CLEAR BEGIN\r\n");
    LubyLCD_Clear();
    if (trace != NULL)
        trace("D02.6 LCD CLEAR END\r\n");
    LCD_BK_ON();
}

void LubyLCD_Init(void)
{
    LubyLCD_InitWithTrace(NULL);
}

void LubyLCD_SetContrast(uint8_t contrast)
{
    if (contrast > 0x1FU)
        contrast = 0x1FU;
    /* RA8816 CSTR[10h]: keep original 1/6 bias (001b), adjust CT[4:0]. */
    lcd_write_command(0x10U, (uint8_t)(0x20U | contrast));
}

void LubyLCD_SetInverse(uint8_t enable)
{
    lcd_write_command(0x13U, (enable != 0U) ? 0x10U : 0x00U);
}

void LubyLCD_ShowLine(uint8_t row, const char *text)
{
    uint8_t column = 0U;

    if ((row >= LUBY_LCD_ROWS) || (text == NULL))
        return;

    /* 文字模式设置写两遍 + 命令后已有 500us 延时: 确保 0x42 一定被控制器
     * 接住, 否则本行 16 个 ASCII 字节会被当图形位图渲染出来 */
    lcd_write_command(0x03U, 0x42U);
    lcd_write_command(0x03U, 0x42U);
    lcd_write_command(0x05U, 0x00U);
    lcd_write_command(0x06U, (uint8_t)(row * 16U));

    while ((column < LUBY_LCD_COLUMNS) && (text[column] != '\0'))
    {
        lcd_write_data((uint8_t)text[column]);
        column++;
    }
    while (column < LUBY_LCD_COLUMNS)
    {
        lcd_write_data((uint8_t)' ');
        column++;
    }
}
