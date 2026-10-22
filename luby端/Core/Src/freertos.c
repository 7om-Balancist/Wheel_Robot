/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "luby_adc.h"
#include "sensors.h"
#include "decision.h"
#include "cds55xx.h"
#include "drive.h"
#include "pi_comm.h"
#include "gpio.h"
#include<stdio.h>
#include "luby_lcd.h"
#include <string.h>
#include "bluetooth.h"
#include "iwdg.h"        /* 看门狗: 任务存活位图喂狗 */


extern DecisionState g_decision_state;
extern volatile uint8_t  g_manual_mode;
extern volatile int16_t  g_manual_left_spd;
extern volatile int16_t  g_manual_right_spd;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* ================================================================
 * 起步同步(直线纠偏):
 *   左右轮电机批次不同 → 双轮从 0 起步时物理左轮(ID2, Drive_SetSpeed
 *   的 left 参数)比右轮快约 180ms, 左轮先动导致车头右偏、无法直线起步。
 *   修正: 双轮从静止同步起步的瞬间, 物理左轮保持 0 速等待, 右轮先启动,
 *   START_SYNC_DELAY_MS 之后左轮再起步, 让两轮"开始响应"的时刻对齐。
 * ================================================================ */
#define START_SYNC_DELAY_MS   100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* ================================================================
 * 看门狗: 任务存活位图
 * ----------------------------------------------------------------
 * 每个任务在自己的循环里"报到"(把自己的位置 1); 只有三个任务都报到过,
 * 才喂一次狗并把位图清零, 开始下一轮。
 *
 * 为什么不用别的方案:
 *   · idle hook —— 只在所有任务都阻塞时才跑, 只能发现"调度器整体死了",
 *     发现不了"某个任务卡死";
 *   · 单独一个 WatchdogTask —— 只看得到它自己, 高优先级任务空转把低优先级
 *     任务饿死这种情况完全测不出来。
 * 位图同时覆盖这两类故障。
 *
 * **置位侧和清零侧都必须进临界区**:
 *   · 清零侧 —— "检查通过"与"清零"之间如果刚好有任务置位, 那一位会被吃掉,
 *     下一轮就永远凑不齐 → 周期性假复位;
 *   · 置位侧 —— `g_wd_alive |= bit` 是"读-改-写", 三条指令之间若被高优先级
 *     任务抢占, 低优先级任务会用陈旧的副本把高优先级刚置的位**写回去**覆盖掉
 *     (或反过来复活一个已经被消费掉的满位图, 让下一轮多喂一次)。
 *     临界区只有 3 条指令, 相对控制任务本来就要开的临界区可以忽略。
 * ================================================================ */
#define WD_BIT_SENSE     (1U << 0)
#define WD_BIT_DISPLAY   (1U << 1)
#define WD_BIT_DEFAULT   (1U << 2)
#define WD_BIT_ALL       (WD_BIT_SENSE | WD_BIT_DISPLAY | WD_BIT_DEFAULT)

volatile uint32_t g_wd_alive = 0U;
static uint8_t    g_wd_armed = 0U;   /* 三个任务都创建成功后才置 1 */

osThreadId_t senseDecideTaskHandle;
const osThreadAttr_t senseDecideTask_attributes = {
  .name = "SenseDecideTask",
  .stack_size = 512 * 4,  // 2KB栈空间，满足滤波+决策的结构体开销
  .priority = (osPriority_t) osPriorityHigh, // 高优先级，保证200Hz准时执行
};

/* ---- P0 重构⑤: 控制任务 → 显示任务 共享快照 ----
   控制任务每拍(临界区)更新 g_disp; DisplayTask 每 750ms 读快照。
   这样 LCD/BT 上报的阻塞(LCD ~37ms + BT ~21ms)不再冻结控制任务。 */
typedef struct {
    uint8_t  gray_on_stage;   /* 台上/台下原始判定 */
    uint8_t  bomb_warn;       /* 树莓派炸弹警告 */
    uint8_t  on_stage;        /* 决策状态机阶段 */
    uint8_t  climb_state;     /* 爬台状态 */
    int16_t  left_speed;      /* 物理左轮指令速度 */
    int16_t  right_speed;     /* 物理右轮指令速度 */
    char     desc[16];        /* 当前动作名(长度15+'\0') */
    uint16_t gray_slow[2];    /* 前后灰度慢窗值(校准/调参显示用) */
    /* ---- 8 路红外 IR ADC 示数(围栏/敌人阈值调参显示用) ---- */
    uint16_t ir_normal[8];    /* 4帧窗口: 围栏检测 + 敌人确认 */
    uint16_t ir_fast[8];      /* 2帧窗口: 快速敌人检测 */
    uint8_t  ir_digital;      /* 二值掩码(阈值100): bit=1 该路无遮挡 */
    uint8_t  ir_enemy_mask;   /* 敌人掩码: bit=1 该方向判到敌人 */
    uint8_t  wall_idx;        /* 8路中值最大的通道 0..7 */
    uint16_t wall_max;        /* 8路最大值 */
    /* P0 计时统计(滚动窗口, 由 DisplayTask 上报后清零) */
    uint32_t chain_min, chain_max, chain_sum, chain_cnt;
    uint32_t dec_min,   dec_max,   dec_sum;
    uint32_t tick_gap_max;
} DisplaySnapshot;
volatile DisplaySnapshot g_disp;   /* 静态清零: cnt/min 在任务内初始化 */

osThreadId_t displayTaskHandle;
const osThreadAttr_t displayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,  // 2KB栈空间，LCD+BT 上报(含 vsnprintf)用
  .priority = (osPriority_t) osPriorityLow, // 最低优先：绝不影响控制任务
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void SenseDecideTask(void *argument);
void DisplayTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* 栈溢出 = 某任务把分配的栈写穿了, 内存已被破坏, 继续跑只会随机误动作。
    * hook 在已损坏的任务上下文里执行, 绝不能调用 FreeRTOS API 或阻塞驱动
    * (任务栈已不可信, 复用会二次溢出)。
    * 策略与 main.c 的 Error_Handler 相同: LCD 给一个可见的致命提示(含哪个任务
    * 溢出, 开发调参直接看到是谁) → 关中断 → 死循环等复位。
    * LubyLCD_ShowLine 是纯 GPIO 位带写, 不依赖 HAL_Delay/FreeRTOS, 此处安全。
    * 若实车见到此画面: 用蓝牙 [STACK] 遥测校准那个任务的 .stack_size。 */
   (void)xTask;
   LubyLCD_Clear();
   LubyLCD_ShowLine(0, "STACK OVERFLOW!");
   if (pcTaskName != NULL) {
       /* 溢出任务名, 内核保证以'\0'结尾, 只读安全 */
       LubyLCD_ShowLine(1, (const char *)pcTaskName);
   }
   LubyLCD_ShowLine(2, "auto reset in~10");
   __disable_irq();

   /* 与 main.c 的 Error_Handler 同样的策略: 先喂 10 秒让提示留在屏幕上,
     然后停止喂狗, IWDG 会自动把整板复位。
     只用寄存器访问(IWDG->KR / DWT / LCD 位带写), 不依赖 FreeRTOS 与
     HAL_Delay, 在被写穿的任务栈上下文里也是安全的。 */
   Wdg_HoldFeedingMs(10000U);

   for (;;) {
       /* 停止喂狗 → 到点自动复位 */
   }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  senseDecideTaskHandle = osThreadNew(SenseDecideTask, NULL, &senseDecideTask_attributes);
  displayTaskHandle = osThreadNew(DisplayTask, NULL, &displayTask_attributes);

  /* ---- 看门狗位图武装 ----
   * 只有三个任务都创建成功, 这个位图才有意义: 少一个任务, 它的位永远不会
   * 被置, 喂狗就永远不成立 → 每 5 秒复位一次。
   * FreeRTOS heap 只有 15360 B, 三个任务栈 512*4 + 512*4 + 128*4 = 4608 B
   * 加上 TCB, 创建失败是真会发生的, 必须查。 */
  if (senseDecideTaskHandle && displayTaskHandle && defaultTaskHandle) {
      g_wd_armed = 1U;
  } else {
      /* 创建失败: 不武装位图。IWDG 仍在跑, 于是启动路径最后一次喂狗之后的
         5 秒窗口内会自然复位 —— 屏幕上先给一个可见提示。 */
      LubyLCD_Clear();
      LubyLCD_ShowLine(0, "WDG TASK FAIL!  ");
      LubyLCD_ShowLine(1, "auto reset soon ");
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    /* 看门狗报到(每 1ms 一次)。置位必须进临界区, 理由见文件上方位图说明 */
    taskENTER_CRITICAL();
    g_wd_alive |= WD_BIT_DEFAULT;
    taskEXIT_CRITICAL();
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/**
 * @brief  传感器采集+决策+电机输出 核心任务，周期5ms(200Hz)
 * * 数据流：ADC_GetAllValues → Sensors_Process → Decision_Run → Drive_SetSpeed
 * 所有硬件抽象均通过工程原生 API：luby_adc / sensors / decision / drive
 */
void SenseDecideTask(void *argument)
{
    /* ========== 初始化阶段 ========== */
    SensorState sensor_state;
    MotionCmd cmd;
    uint16_t adc_raw[LUBY_ADC_CHANNEL_COUNT];
    /* ---- P0: DWT 计时(每拍) 与 上一拍时刻(测真实周期) ---- */
    uint32_t cyc0 = 0, cyc_dec = 0, cyc1 = 0;
    uint32_t tick_prev = 0;
    /* ---- LCD 初始化（已经在main里初始化过了，这里暂时关一下）---- */
    // LubyLCD_Init();
    // LubyLCD_SetContrast(0x07);
    // LubyLCD_SetInverse(1);
    // LubyLCD_Clear();

    /* ---- DWT 循环计数器使能（72MHz：1 cycle = 1/72 µs）---- */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 快照统计初始化: min 起点=0xFFFFFFFF(否则 min 恒为 0) */
    g_disp.chain_min = 0xFFFFFFFF;
    g_disp.dec_min   = 0xFFFFFFFF;

    /* ========== 主循环 5ms周期 ========== */
    for(;;)
    {
        uint32_t now = HAL_GetTick();

        /* 看门狗报到: 本拍活着。置位必须进临界区, 理由见文件上方位图说明 */
        taskENTER_CRITICAL();
        g_wd_alive |= WD_BIT_SENSE;
        taskEXIT_CRITICAL();

        /* ---- P0: 真实周期间隔（>5ms = 本拍被阻塞/被打断）  属于调试部分---- */
        if (tick_prev) {
            uint32_t gap = now - tick_prev;
            taskENTER_CRITICAL();
            if (gap > g_disp.tick_gap_max) g_disp.tick_gap_max = gap;
            taskEXIT_CRITICAL();
        }
        tick_prev = now;

        /* ---- P0: 核心链计时起点（ADC 之前）（此部分用作计算决策总耗时，属于调试部分）---- */
        cyc0 = DWT->CYCCNT;

        /* 1. 获取ADC所有通道排序后的数据（A0~A15） */
        ADC_GetAllValues(adc_raw);


        /* 2. 传感器滤波处理，输出结构化数据 */
        Sensors_Process(adc_raw, &sensor_state);


        /* 3. 消费树莓派串口帧，更新全局标签信息 */

        PiComm_ConsumeFrame(&g_tag_info);

        /* 4. 核心决策运算 */
        cyc_dec = DWT->CYCCNT;                  /* P0: 决策计时起点 */
        cmd = Decision_Run(&g_decision_state, &sensor_state, &g_tag_info, now);

        // 蓝牙手动模式优先级高于自动AI
        if(g_manual_mode)
        {
            cmd.left_speed = g_manual_left_spd;
            cmd.right_speed = g_manual_right_spd;
        }

        /* 5. 电机输出 — 起步同步整形:
           物理左轮(ID2)起步比右轮快约 START_SYNC_DELAY_MS,
           双轮从静止起步时左轮先保持 0 速, 等右轮启动后再起步 */
        int16_t out_l = cmd.left_speed;    /* 物理左轮 → 舵机 ID2 (快轮) */
        int16_t out_r = cmd.right_speed;   /* 物理右轮 → 舵机 ID1 */
        static uint32_t lw_delay_until = 0;
        static int16_t  prev_out_l = 0, prev_out_r = 0;

        if (prev_out_l == 0 && prev_out_r == 0 && out_l != 0 && out_r != 0) {
            lw_delay_until = now + START_SYNC_DELAY_MS;   /* 双轮从静止同步起步 */
        }
        if (now < lw_delay_until) {
            out_l = 0;   /* 物理左轮保持静止, 等待右轮先启动 */
        }
        prev_out_l = out_l;
        prev_out_r = out_r;
        Drive_SetSpeed(out_l, out_r);

        /* ---- P0: 统计本拍耗时 + 刷新显示快照 ----
           (临界区保护; 显示任务只读快照, 永不触碰控制路径) */
        cyc1 = DWT->CYCCNT;
        {
            uint32_t chain_us = (cyc1 - cyc0) / (SystemCoreClock / 1000000U);
            uint32_t dec_us   = (cyc1 - cyc_dec) / (SystemCoreClock / 1000000U);
            /* 8 路红外二值打包成位掩码, 方便显示(bit[i]=1 → 该路高于阈值100=无遮挡) */
            uint8_t  ir_dig = 0U;
            for (uint8_t i = 0U; i < 8U; i++) {
                if (sensor_state.ir_digital[i]) ir_dig |= (uint8_t)(1U << i);
            }
            taskENTER_CRITICAL();
            if (chain_us < g_disp.chain_min) g_disp.chain_min = chain_us;
            if (chain_us > g_disp.chain_max) g_disp.chain_max = chain_us;
            g_disp.chain_sum += chain_us; g_disp.chain_cnt++;
            if (dec_us < g_disp.dec_min) g_disp.dec_min = dec_us;
            if (dec_us > g_disp.dec_max) g_disp.dec_max = dec_us;
            g_disp.dec_sum += dec_us;
            g_disp.gray_on_stage = sensor_state.gray_on_stage_raw;
            g_disp.bomb_warn     = g_tag_info.bomb_warn;
            g_disp.on_stage      = g_decision_state.on_stage;
            g_disp.climb_state   = g_decision_state.climb_state;
            g_disp.left_speed    = cmd.left_speed;
            g_disp.right_speed   = cmd.right_speed;
            /* g_disp 为 volatile: 快照写入用强转去掉限定(临界区内单写者, 安全) */
            strncpy((char *)g_disp.desc, cmd.desc, 15);
            g_disp.desc[15] = '\0';
            memcpy((void *)g_disp.gray_slow, sensor_state.gray_slow, sizeof(g_disp.gray_slow));
            /* 8 路红外示数快照(调参用; 同样在临界区内整块拷贝) */
            memcpy((void *)g_disp.ir_normal, sensor_state.ir_normal, sizeof(g_disp.ir_normal));
            memcpy((void *)g_disp.ir_fast,   sensor_state.ir_fast,   sizeof(g_disp.ir_fast));
            g_disp.ir_digital    = ir_dig;
            g_disp.ir_enemy_mask = sensor_state.ir_enemy_mask;
            g_disp.wall_idx      = sensor_state.wall_sensor_idx;
            g_disp.wall_max      = sensor_state.wall_max_value;
            taskEXIT_CRITICAL();
        }

        /* ---- 显示/遥测已移到低优先级 DisplayTask，控制任务只做控制 ---- */

        /* ---- 看门狗: 三个任务都报到过才喂一次 ----
           放在本拍最后: 这一拍该干的活全干完了才算"这个任务活着"。
           清零必须在临界区里, 否则"检查通过"与"清零"之间刚好有别的任务
           置位会丢一位, 下一轮永远凑不齐 → 周期性假复位。 */
        if (g_wd_armed) {
            uint32_t snap;
            taskENTER_CRITICAL();
            snap = g_wd_alive;
            if ((snap & WD_BIT_ALL) == WD_BIT_ALL) { g_wd_alive = 0U; }
            taskEXIT_CRITICAL();
            if ((snap & WD_BIT_ALL) == WD_BIT_ALL) { WDG_FEED(); }
        }

        /* 5ms延时，严格保证200Hz执行频率 */
        osDelay(5);
    }
}

/**
 * @brief  显示/遥测任务（低优先级）: 每 750ms 刷 LCD + 蓝牙上报
 * 数据全部来自 g_disp 快照(控制任务每拍更新), 本任务绝不碰控制/驱动路径。
 * 重构⑤: 原 SenseDecideTask 内的上报块堵塞电机控制(LCD ~37ms + BT ~21ms)整体搬到此任务,
 * 控制任务因此不再被阻塞, 真实周期恒为 5ms。
 */
void DisplayTask(void *argument)
{
    char buf[17];
    uint32_t prev = 0;

    for (;;) {
        uint32_t now = HAL_GetTick();

        /* 看门狗报到 —— 必须放在 for(;;) 最外层(每 10ms 一次),
           不能塞进下面 750ms 的 if 块里, 否则整轮周期被拉到 1.5s。
           置位必须进临界区, 理由见文件上方位图说明。 */
        taskENTER_CRITICAL();
        g_wd_alive |= WD_BIT_DISPLAY;
        taskEXIT_CRITICAL();

        if ((now - prev) >= 750) {
            prev = now;

            /* 临界区整块拷贝快照(显示值抖动可接受, 统计值需一致) */
            DisplaySnapshot s;
            taskENTER_CRITICAL();
            memcpy(&s, (const void *)&g_disp, sizeof(s));
            taskEXIT_CRITICAL();

            /* ---- LCD 4 行 ---- */
            if (s.gray_on_stage) {
                snprintf(buf, sizeof(buf), "ON PLATFORM    ");
            } else {
                snprintf(buf, sizeof(buf), "OFF PLATFORM!! ");
            }
            LubyLCD_ShowLine(0, buf);
            if (s.bomb_warn) {
                snprintf(buf, sizeof(buf), "BOMB WARN!!     ");
            } else {
                snprintf(buf, sizeof(buf), "BOMB: SAFE      ");
            }
            LubyLCD_ShowLine(1, buf);
            snprintf(buf, sizeof(buf), "L=%+4d R=%+4d   ", s.left_speed, s.right_speed);
            LubyLCD_ShowLine(2, buf);
            snprintf(buf, sizeof(buf), "ST:%d ON:%d     ", s.climb_state, s.on_stage);
            LubyLCD_ShowLine(3, buf);

            /* ---- 蓝牙遥测 ---- */
            BT_Printf("[%s] ACT=%s ST=%d L=%+d R=%+d ON=%d BOMB=%d\r\n",
                      g_manual_mode ? "MANUAL" : "AI",
                      s.desc, s.climb_state,
                      s.left_speed, s.right_speed,
                      s.on_stage, s.bomb_warn);

            /* ---- 校准/调参: 前后灰度实时值与标定阈值 ---- */
            BT_Printf("[SENSOR]\nGRAY_SLOW(F/R): %d, %d\nEDGE_TH(F/R): %d, %d\nONSTAGE_TH(F/R): %d, %d\nGRAY_ON_RAW: %d\r\n",
                      s.gray_slow[0], s.gray_slow[1],
                      g_thresh.edge_thresh[0], g_thresh.edge_thresh[1],
                      g_thresh.on_stage[0], g_thresh.on_stage[1],
                      s.gray_on_stage);

            /* ---- 8 路红外 ADC 示数(调 wall_threshold / enemy_threshold 用) ----
               通道角度: 0=0°正前 1=45°右前 2=90°正右 3=135°右后
                         4=180°正后 5=225°左后 6=270°正左 7=315°左前
               ADC 引脚 : AD4  AD5  AD6  AD7  AD8  AD9  AD10  AD14
               N = 4帧窗口(围栏 wall_detected / 敌人确认用), F = 2帧窗口(快速敌人检测用)
               DIG   = 二值掩码(阈值100): bit=1 该路无遮挡
               ENEMY = 敌人判定掩码: bit=1 该方向 ir_fast > enemy_threshold
                       (这两个掩码按位读: 0x01=bit0=ch0 ... 0x80=bit7=ch7)
               注: 分两次 BT_Printf 是刻意的——BT_Printf 内部缓冲 256B,
                   一次塞太多行会被截断 */
            BT_Printf("[IR] ch(deg):  0   45   90  135  180  225  270  315   (AD4..AD10, AD14)\r\n"
                      "[IR]        N: %4u %4u %4u %4u %4u %4u %4u %4u\r\n"
                      "[IR]        F: %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                      (unsigned)s.ir_normal[0], (unsigned)s.ir_normal[1],
                      (unsigned)s.ir_normal[2], (unsigned)s.ir_normal[3],
                      (unsigned)s.ir_normal[4], (unsigned)s.ir_normal[5],
                      (unsigned)s.ir_normal[6], (unsigned)s.ir_normal[7],
                      (unsigned)s.ir_fast[0], (unsigned)s.ir_fast[1],
                      (unsigned)s.ir_fast[2], (unsigned)s.ir_fast[3],
                      (unsigned)s.ir_fast[4], (unsigned)s.ir_fast[5],
                      (unsigned)s.ir_fast[6], (unsigned)s.ir_fast[7]);
            BT_Printf("[IR] ENEMY_TH: %4u %4u %4u %4u %4u %4u %4u %4u\r\n"
                      "[IR] WALL_TH : %4u %4u %4u %4u %4u %4u %4u %4u\r\n",
                      (unsigned)g_thresh.enemy_threshold[0], (unsigned)g_thresh.enemy_threshold[1],
                      (unsigned)g_thresh.enemy_threshold[2], (unsigned)g_thresh.enemy_threshold[3],
                      (unsigned)g_thresh.enemy_threshold[4], (unsigned)g_thresh.enemy_threshold[5],
                      (unsigned)g_thresh.enemy_threshold[6], (unsigned)g_thresh.enemy_threshold[7],
                      (unsigned)g_thresh.wall_threshold[0], (unsigned)g_thresh.wall_threshold[1],
                      (unsigned)g_thresh.wall_threshold[2], (unsigned)g_thresh.wall_threshold[3],
                      (unsigned)g_thresh.wall_threshold[4], (unsigned)g_thresh.wall_threshold[5],
                      (unsigned)g_thresh.wall_threshold[6], (unsigned)g_thresh.wall_threshold[7]);
            BT_Printf("[IR] DIG=%02X ENEMY=%02X   WALL: idx=%u max=%u\r\n",
                      (unsigned)s.ir_digital, (unsigned)s.ir_enemy_mask,
                      (unsigned)s.wall_idx, (unsigned)s.wall_max);

            /* ---- P0: 每 750ms 上报本窗口耗时统计 ---- */
            if (s.chain_cnt) {
                BT_Printf("[TIMING] chain_us min=%lu max=%lu avg=%lu | dec_us min=%lu max=%lu avg=%lu | tickgap_ms_max=%lu\r\n",
                          s.chain_min, s.chain_max, s.chain_sum / s.chain_cnt,
                          s.dec_min, s.dec_max, s.dec_sum / s.chain_cnt,
                          s.tick_gap_max);
                /* 重置滚动窗口统计(临界区, 防控制任务同时写) */
                taskENTER_CRITICAL();
                g_disp.chain_min = 0xFFFFFFFF; g_disp.chain_max = 0;
                g_disp.chain_sum = 0;           g_disp.chain_cnt = 0;
                g_disp.dec_min   = 0xFFFFFFFF; g_disp.dec_max = 0;
                g_disp.dec_sum   = 0;
                g_disp.tick_gap_max = 0;
                taskEXIT_CRITICAL();
            }

            /* ---- P0: 栈余量上报(High-Water Mark, 校准任务栈大小用) ----
               uxTaskGetStackHighWaterMark(handle): 任务运行以来最少剩余栈,
               返回值单位 = 字(word, 32位机 = 4字节) → ×4 得字节。
               每个任务的栈总量 512字×4 = 2048字节; 数字越小越接近溢出。
               用 BT 串口助手观察此值, 若某任务恒 < ~200B 就应加大它的 .stack_size。 */
            BT_Printf("[STACK] sense_free=%uB disp_free=%uB (each total 2048B)\r\n",
                      (unsigned)(uxTaskGetStackHighWaterMark(senseDecideTaskHandle) * 4U),
                      (unsigned)(uxTaskGetStackHighWaterMark(displayTaskHandle) * 4U));
        }
        osDelay(10);
    }
}
/* USER CODE END Application */

