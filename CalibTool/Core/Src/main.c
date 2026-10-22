/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   灰度标定工程(CalibTool) — 独立固件, 采完写进外部 SPI flash
 *
 * ================================================================
 * 它是干什么的 / 怎么用
 * ================================================================
 *   主工程 car7.15 从 2026-09-17 起不再现场采集灰度, 改为上电从外部 SPI
 *   flash 读回。**采集这一步搬到了本工程**。
 *
 *   用法: 把本工程的 bin 通过 Luby 的 U 盘模式烧进去 → 上电自动跑三步采集
 *         (车头压前边缘 / 车尾压后边缘 / 整车放台下地面) → 屏幕显示四个
 *         数值 → 按 KEY1 存进 flash → 断电, 换回主工程的 bin。
 *
 * ================================================================
 * 本工程不用看门狗
 * ================================================================
 *   全程只有"三步采集(约 18.4 秒) → 显示 → 按键存盘", 没有任何会卡死的
 *   等待, 加狗只是白添一层复杂度。
 *   前提是这块板的**硬件看门狗没有被选项字节使能**: 若 WDG_SW=0, 上电时
 *   IWDG 会自动开始计数(~0.41 秒), 连 LCD 初始化都跑不完, 表现是
 *   "屏幕半清、无限重启"。用 FlashTest 工程打一次 FLASH_OBR 就能确认。
 *
 * ================================================================
 * 为什么 LCD 是唯一判据
 * ================================================================
 *   本工程不跑 FreeRTOS、不用蓝牙。屏幕亮了并显示 flash 自检结果 =
 *   程序跑起来了; 全黑 = 没跑到 LCD 初始化(bin 没被载入)。
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "luby_adc.h"
#include "luby_lcd.h"
#include "calibrate.h"
#include "cal_store.h"
#include "sst25.h"
#include "sst25_port.h"
#include "input.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* 本工程不链接 sensors.c(那套 200Hz 滤波在这里用不到), 但 calibrate.c 的
   calc_thresholds() 要往 g_thresh 里写结果, 所以在这里定义这个全局。 */
SensorThresholds g_thresh;

static uint8_t lcd_ready = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void ShowNum(uint8_t row, const char *label, uint16_t v);
static uint8_t WaitKey(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void ShowNum(uint8_t row, const char *label, uint16_t v)
{
  char b[24];
  snprintf(b, sizeof(b), "%s=%-5u", label, (unsigned)v);
  LubyLCD_ShowLine(row, b);
}

/**
 * @brief 等一个按键
 * @return 1 = KEY1(下键, 存盘)   2 = KEY2(上键, 重测)
 */
static uint8_t WaitKey(void)
{
  for (;;)
  {
    if (Input_Get(INPUT_KEY_DOWN))
    {
      HAL_Delay(40);
      return 1U;
    } /* KEY1 */
    if (Input_Get(INPUT_KEY_UP))
    {
      HAL_Delay(40);
      return 2U;
    } /* KEY2 */
    HAL_Delay(20);
  }
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* 本工程不用看门狗 —— 理由见文件头 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* ---- 本工程不用看门狗 ----
   * 采集/显示/存盘全是顺序执行的短路径, 没有可能卡死的等待。
   * 唯一要确认的是硬件狗没被选项字节使能(见文件头说明)。 */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  /* ---- 0. LCD ---- */
  LubyLCD_Init();
  LubyLCD_SetContrast(0x07);
  LubyLCD_SetInverse(1);
  LubyLCD_Clear();
  lcd_ready = 1U;
  LubyLCD_ShowLine(0, "CALIB TOOL      ");
  LubyLCD_ShowLine(1, "starting...     ");

  /* ---- 1. 外部 flash 自检 ---- */
  {
    Sst25Status_t st = SST25_Init();
    char b[24];

    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "SPI FLASH CHECK ");
    snprintf(b, sizeof(b), "ID %02X%02X%02X %s",
             g_sst25_info.jedec[0], g_sst25_info.jedec[1], g_sst25_info.jedec[2],
             g_sst25_info.id_ok ? "OK" : "!!");
    LubyLCD_ShowLine(1, b);
    snprintf(b, sizeof(b), "FAM:%-5s SR:%02X",
             (g_sst25_info.family == SST25_FAMILY_SST) ? "SST" : (g_sst25_info.family == SST25_FAMILY_WINBOND) ? "WB"
                                                                                                               : "?",
             g_sst25_info.status);
    LubyLCD_ShowLine(2, b);
    LubyLCD_ShowLine(3, Cal_Crc32SelfTest() ? "CRC32 SELF OK" : "CRC32 FAIL!!");
    HAL_Delay(2500);

    if (st != SST25_OK)
    {
      /* ID 都读不到: 采集了也没地方存, 不做无意义的 18 秒采集。
         最常见的原因是 HOLD# 被拉低 / 芯片没焊 / 供电没有。 */
      LubyLCD_Clear();
      LubyLCD_ShowLine(0, "FLASH NOT FOUND!");
      LubyLCD_ShowLine(1, SST25_StatusStr(st));
      LubyLCD_ShowLine(2, "check U2 solder");
      LubyLCD_ShowLine(3, "and HOLD# pin   ");
      while (1)
      {
        HAL_Delay(200);
      }
    }
  }

  /* ---- 2. ADC 启动 ---- */
  if (ADC_Init() != HAL_OK)
  {
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "ADC INIT FAIL!  ");
    while (1)
    {
      HAL_Delay(200);
    }
  }

  /* ---- 3. 主流程: 采集 → 显示 → 存盘 ---- */
  for (;;)
  {
    /* 3.1 三步自动采集(LCD 引导, 阻塞约 18.4 秒) */
    Gray_LoadThresholds(NULL); /* 传 NULL = 采集模式(这是标定工程的特权) */

    {
      const GrayBaseline_t *bl = Gray_GetLastBaseline();

      /* 3.2 显示采集结果, 供人工抄录到 g_gray_fallback */
      LubyLCD_Clear();
      LubyLCD_ShowLine(0, "MEASURED VALUES ");
      ShowNum(1, "F_EDGE", bl->front_edge);
      ShowNum(2, "R_EDGE", bl->rear_edge);
      ShowNum(3, "GND_F ", bl->ground[GRAY_FRONT]);
      HAL_Delay(2500);

      LubyLCD_Clear();
      LubyLCD_ShowLine(0, "MEASURED VALUES ");
      ShowNum(1, "GND_R ", bl->ground[GRAY_REAR]);
      LubyLCD_ShowLine(2, "KEY1=SAVE       ");
      LubyLCD_ShowLine(3, "KEY2=RE-MEASURE ");

      /* 3.3 等按键 */
      if (WaitKey() == 1U)
      {
        Cal_Status_t st = CalStore_Save(bl);

        LubyLCD_Clear();
        LubyLCD_ShowLine(0, "SAVE RESULT     ");
        LubyLCD_ShowLine(1, Cal_StatusStr(st));
        HAL_Delay(2000);

        if (st == CAL_OK)
        {
          /* 3.4 回读验证 —— 证明真的落盘了, 而不是只写了 RAM */
          GrayBaseline_t rb;
          Cal_Status_t st2 = CalStore_Load(&rb);

          LubyLCD_Clear();
          LubyLCD_ShowLine(0, "READBACK TEST   ");
          if (st2 == CAL_OK && rb.front_edge == bl->front_edge &&
              rb.rear_edge == bl->rear_edge &&
              rb.ground[GRAY_FRONT] == bl->ground[GRAY_FRONT] &&
              rb.ground[GRAY_REAR] == bl->ground[GRAY_REAR])
          {
            LubyLCD_ShowLine(1, "VERIFY OK!      ");
            LubyLCD_ShowLine(2, "POWER OFF NOW   ");
            LubyLCD_ShowLine(3, "reflash car7.15 ");
            /* 验证成功 = 记录已落盘: 停在这幅画面等断电,
             * 绝不回 3.1 重新采集(再采一次没意义, 还可能被误按
             * KEY1 把新值覆盖进去)。想重标定就断电重启。 */
            while (1)
            {
              HAL_Delay(200);
            }
          }
          else
          {
            LubyLCD_ShowLine(1, "VERIFY FAIL!!   ");
            LubyLCD_ShowLine(2, Cal_StatusStr(st2));
            LubyLCD_ShowLine(3, "flash may be bad");
            HAL_Delay(4000);
            /* 验证失败: 回 3.1 重新采集再试 */
          }
        }
      }
      /* 按 KEY2 重测 / 存盘失败或验证失败: 回到 3.1 重新采集
       * (存盘+验证成功走不到这里 —— 上面已死循环停机等断电) */
    }
  }
  /* USER CODE END 2 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  TIM2 时基回调 —— HAL_GetTick / HAL_Delay 全靠它走时
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  if (lcd_ready)
  {
    LubyLCD_ShowLine(2, "I'm dead!       ");
  }
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
