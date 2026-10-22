/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 主程序入口文件 - 测试标定专用版
  *                   上电自动执行三步灰度采集标定，阈值当场生效
  *                   用于换新场地时测量灰度基准值
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "luby_adc.h"
#include "sensors.h"
#include "decision.h"
#include "drive.h"
#include "pi_comm.h"
#include "uart_debug.h"
#include <stdio.h>
#include "calibrate.h"
#include "luby_lcd.h"
#include "string.h"
#include "bluetooth.h"
/* ---- 2026-09-17 新增: 外部 SPI flash 标定存储 + 看门狗 ---- */
#include "sst25.h"
#include "cal_store.h"
#include "iwdg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/**
 * @brief  全局决策状态实例
 * 生命周期：全局静态存储，程序运行全程持久化
 * 访问：本文件初始化，freertos.c 中 SenseDecideTask 每帧读写
 */
DecisionState g_decision_state;

/* LCD 是否已初始化: Error_Handler 可能在 LCD_Init 之前被调用
   (比如 SystemClock_Config 里), 那时不能往未初始化的 LCD 写。 */
static uint8_t lcd_ready = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
/**
 * @brief  蓝牙接收回调函数
 * @param  data  接收到的数据缓冲区指针
 * @param  len   接收到的数据字节长度
 */
void MyBluetoothHandler(const unsigned char *data, unsigned short len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* ---- 复位原因锁存: 必须在 HAL_Init() 之前 ----
   * 没有这个就分不清"固件被看门狗咬复位了"(需求达成)与"电池瞬间掉电"
   * (硬件问题)。同时会清掉标志, 否则下次读到的永远是上一次的结果。 */
  Wdg_LatchResetCause();

  /* 无条件重载一次 IWDG: bootloader 若留了个未知周期的狗, 这一行救命。
     IWDG 没启动时只是空操作。 */
  IWDG->KR = IWDG_KR_RELOAD;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* ---- 看门狗: 尽早启动, 用已知的 5 秒配置覆盖继承来的未知周期 ----
   * 放在 SystemClock_Config **之前**有两个原因:
   *   1. IWDG 由独立的 LSI 时钟驱动, 寄存器访问不需要系统时钟, 这里就能配;
   *   2. SystemClock_Config 自己会调 Error_Handler(HSE 起不来 / PLL 不锁),
   *      若那时狗还没开, Error_Handler 里"停止喂狗 → 自动复位"的兜底就是一句
   *      空话, 板子会黑着屏死在那儿等人手动断电。
   * IWDG 一旦启动软件就关不掉, 所以必须在这里一次配好。 */
  Wdg_ConfigAndStart();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  /* ---- 0. LCD 初始化 ---- */
  LubyLCD_Init();                 /* 内部有 2x HAL_Delay(100), 不喂也远不到 5s */
  LubyLCD_SetContrast(0x07);
  LubyLCD_SetInverse(1);
  LubyLCD_Clear();
  lcd_ready = 1U;                 /* Error_Handler 的守卫 */
  WDG_FEED();

  /* ---- 1. 外部 SPI flash 初始化 ----
   * 主工程对配置区**永远是只读的**: 这里只做器件的抢救序列(补 WRDI + 读 ID
   * + 读状态), 一个字都不写。所以车在任何情况下都不可能把自己的标定写坏 ——
   * 所有写入路径只存在于工作台上的标定工程里。 */
  (void)SST25_Init();
  WDG_FEED();

  /* ---- 2. ADC 初始化 + 启动 DMA 循环采样 ---- */
  if (ADC_Init() != HAL_OK)
  {
      Error_Handler();
  }
  WDG_FEED();

  /* ---- 3. 传感器滑动窗口滤波初始化 ---- */
  Sensors_Init();

  /* ---- 4. 决策状态机复位 ----
   * 若本次复位是 IWDG 咬的(比赛中固件卡死被狗重启), 跳过软启动与硬编码冲台:
   *   看门狗的完整价值不只是"把板子救回来", 还要让它立刻回到能打的状态。
   * 判因只能用 g_reset_csr —— main 开头 Wdg_LatchResetCause() 已清掉 RCC->CSR 的标志,
   *   那之后再去读 RCC->CSR 永远是 0。 */
  uint8_t resume_after_wdg = Wdg_ResetWasWatchdog();
  Decision_Init(&g_decision_state, resume_after_wdg);

  /* ---- 5. 灰度阈值: 从外部 SPI flash 读回(不再现场采集) ----
   * 采集流程已搬到独立工程 CalibTool, 采完写进 flash 的 A/B 双槽。
   *
   * 读不到时**用编译期兜底值继续跑**:
   *   · 绝不停机、绝不进 Error_Handler —— flash 坏了不等于车坏了,
   *     比赛现场死循环等于直接弃权;
   *   · 绝不退回"自动采集"那条路 —— 那是 18 秒阻塞 + 等人压边缘,
   *     在起跑线上发生同样等于弃权。
   * 故障本身由 LCD + 蓝牙遥测暴露出来, 让操作者看得见。 */
  {
      uint8_t cal_ok = Gray_LoadFromFlash() ? 1U : 0U;

      LubyLCD_Clear();
      if (cal_ok) {
          const GrayBaseline_t *v = Gray_GetLastBaseline();
          char b[20];
          LubyLCD_ShowLine(0, "CAL: FLASH OK   ");
          /* 第 1/2 行显示读回的 4 个原始基准值(前/后边缘, 前/后台下),
           * 上电即核对 flash 里存的是哪一组, 不用接上位机。 */
          snprintf(b, sizeof(b), "EDGE %4u/%4u",
                   (unsigned)v->front_edge, (unsigned)v->rear_edge);
          LubyLCD_ShowLine(1, b);
          snprintf(b, sizeof(b), "GND  %4u/%4u",
                   (unsigned)v->ground[0], (unsigned)v->ground[1]);
          LubyLCD_ShowLine(2, b);
      } else {
          LubyLCD_ShowLine(0, "CAL MISSING!    ");
          LubyLCD_ShowLine(1, Cal_StatusStr(Cal_LastStatus()));
          LubyLCD_ShowLine(2, "USE DEFAULT VAL ");
      }
      {   /* 第 3 行: 复位原因; 若是看门狗咬的, 额外标出"已跳过软启动" */
          char rst_line[17];
          snprintf(rst_line, sizeof(rst_line), "%-16s",
                   resume_after_wdg ? "RST:WDG SKIP-SS" : Wdg_ResetCauseStr(g_reset_csr));
          LubyLCD_ShowLine(3, rst_line);
      }
      /* OK 路径 500→2500ms: 多出的两行数值要来得及看 */
      Wdg_DelayMs(cal_ok ? 2500U : 1500U);   /* 自喂延时, 不用裸 HAL_Delay */
  }
  WDG_FEED();

  /* ---- 6. 电机驱动初始化 ----
   * ID1/ID2 电机模式 + 扭矩开启 + 零速待命 */
  if (Drive_Init() != HAL_OK)
  {
      Error_Handler();
  }
  WDG_FEED();

  /* ---- 7. 树莓派通信初始化（UART5 中断接收）---- */
  PiComm_Init();
  WDG_FEED();

  /* ---- 8. 调试串口输出启动信息 ---- */
  UartDebug_Print(resume_after_wdg
                  ? "CAR7.15 BOOT OK (WDG-RESUME, SKIP SOFT-START)"
                  : "CAR7.15 FLASH-CAL BOOT OK");

  /* ---- 9. 蓝牙初始化 + 注册接收回调 ---- */
  BT_Init();
  BT_SetRxCallback(MyBluetoothHandler);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();
  WDG_FEED();            /* 最后一次裸机喂狗 —— 之后就交给任务存活位图 */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief  蓝牙接收回调函数：蓝牙模块收到数据后自动调用
 * @param  data  输入参数：接收到的数据首地址
 * @param  len   输入参数：数据字节长度
 * 支持指令：auto / manual / forward / back / left / right / stop
 */

/* 外部全局变量：定义在 decision.c，此处用于蓝牙手动控制 */
extern volatile uint8_t  g_manual_mode;
extern volatile int16_t  g_manual_left_spd;
extern volatile int16_t  g_manual_right_spd;

void MyBluetoothHandler(const unsigned char *data, unsigned short len)
{
    const char *cmd_str = (const char *)data;
    // 强制类型转换：将字节指针转为字符串指针用于比较

    // 切换自动 AI 模式
    if (strcmp(cmd_str, "auto") == 0)
    {
        g_manual_mode = 0;
        BT_SendString("[OK] 切换AI自动模式\r\n");
        return;
    }

    // 切换蓝牙手动控制模式
    if (strcmp(cmd_str, "manual") == 0)
    {
        g_manual_mode = 1;
        g_manual_left_spd = 0;
        g_manual_right_spd = 0;
        BT_SendString("[OK] 切换蓝牙手动模式\r\n");
        return;
    }

    // 未开启手动模式，拒绝运动指令
    if (!g_manual_mode)
    {
        BT_SendString("[ERR] 请先发送manual开启手动模式\r\n");
        return;
    }

    // 运动指令解析
    /* 方向用 drive.c 的统一约定: 左轮/右轮, 正=前进 负=后退。
       两轮同号 = 直线; 左正右负 = 右转; 左负右正 = 左转。
       ⚠ 这里原先四个方向**全写反了**(forward 发 (500,-500) 实际是右转、
       left 发 (-300,-300) 实际是直线后退), 2026-09-11 一并修正。 */
    if (strcmp(cmd_str, "forward") == 0)
    {
        g_manual_left_spd = 500;
        g_manual_right_spd = 500;
        BT_SendString("[OK] 前进\r\n");
    }
    else if (strcmp(cmd_str, "back") == 0)
    {
        g_manual_left_spd = -500;
        g_manual_right_spd = -500;
        BT_SendString("[OK] 后退\r\n");
    }
    else if (strcmp(cmd_str, "left") == 0)
    {
        g_manual_left_spd = -300;   /* 左轮后退 */
        g_manual_right_spd = 300;   /* 右轮前进 → 车头左转 */
        BT_SendString("[OK] 左转\r\n");
    }
    else if (strcmp(cmd_str, "right") == 0)
    {
        g_manual_left_spd = 300;    /* 左轮前进 */
        g_manual_right_spd = -300;  /* 右轮后退 → 车头右转 */
        BT_SendString("[OK] 右转\r\n");
    }
    else if (strcmp(cmd_str, "stop") == 0)
    {
        g_manual_left_spd = 0;
        g_manual_right_spd = 0;
        BT_SendString("[OK] 停车\r\n");
    }
    else
    {
        BT_Printf("[ERR] 未知指令:%s\r\n", cmd_str);
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM2)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* 走到这里说明启动阶段出了不可恢复的错(最常见: Drive_Init 电机总线没通)。

     与看门狗的配合(这是"死机自动重启"需求最直接的落点):
       __disable_irq() 只关 CPU 中断, **挡不住 IWDG** —— 它由独立的 LSI 时钟
       驱动, 照常计数。所以这里先主动喂 10 秒, 让致命提示留在屏幕上(这是
       初学者唯一的诊断窗口), 10 秒后停止喂狗, IWDG 会在 3.3~5 秒内把整板
       复位 —— 车自动重启, 不需要人去按复位键。 */
  if (lcd_ready) {
      LubyLCD_Clear();
      LubyLCD_ShowLine(0, "I'm dead!       ");
      LubyLCD_ShowLine(1, "auto reset in~10");
      LubyLCD_ShowLine(2, "sec (watchdog)  ");
  }
  /* 先喂 10 秒, 让致命提示留在屏幕上(初学者唯一的诊断窗口),
     然后停止喂狗, IWDG 会在 3.3~6.7 秒内把整板复位。
     用 DWT 计时而不是空转循环 —— 循环耗时会随优化等级变化,
     照 luby_lcd 的换算, "900000 次 x 10" 实际只有约 1.1 秒而非 10 秒。
     详见 iwdg.h 里 Wdg_HoldFeedingMs 的说明。 */
  __disable_irq();
  Wdg_HoldFeedingMs(10000U);

  while (1)
  {
      /* 停止喂狗 → IWDG 到点自动复位整板 */
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
