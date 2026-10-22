/**
 * @file    iwdg.h
 * @brief   独立看门狗(IWDG) — 寄存器级, 死机自动重启
 *
 * ================================================================
 * 为什么走寄存器
 * ================================================================
 *   本工程 Drivers/ 里没有 stm32f1xx_hal_iwdg.c(见 sst25_port.h 的同类说明),
 *   而且 IWDG 一共就 4 个寄存器, 寄存器级比 HAL 更短更可控。
 *
 * ================================================================
 * 超时预算(必须按最坏值算)
 * ================================================================
 *   ÷64 预分频 + RLR=3124 → (3124+1) × 64 / F_LSI
 *     LSI 标称 40kHz → 5.00 s      ← 设计目标(用户指定 5 秒)
 *     LSI 最坏 60kHz → 3.33 s      ← 所有余量必须按这个卡
 *     LSI 最坏 30kHz → 6.67 s
 *
 *   启动路径上最长的"无喂狗"间隔其实是 **LCD 那一段, 约 0.7~1.0 秒** ——
 *   不是 flash 擦写(那才 200ms):
 *     LubyLCD_Init 内部两张 HAL_Delay(100) 加两次整屏 Clear
 *     (每次 64x16 = 1024 字节位带写, 每字节约 175µs), 这一段中间没有喂狗点。
 *   即使按最坏 LSI 的 3.33 秒算, 对 ~1.0 秒也还有 3 倍以上余量。
 *   (如果要进一步收紧, 可以在 main.c 的两次整屏 Clear 之间补一个 WDG_FEED。)
 *
 * ================================================================
 * 喂狗策略: 任务存活位图(见 freertos.c)
 * ================================================================
 *   只有三个任务都在最近一轮里报到过才喂狗。这样既能抓"某个任务卡死",
 *   也能抓"高优先级任务空转饿死低优先级任务" —— 这两类故障用 idle hook
 *   或者单独一个看门狗任务都测不出来。
 */
#ifndef __IWDG_H
#define __IWDG_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* IWDG 键值 */
#define IWDG_KR_RELOAD   0xAAAAU   /* 重载计数器(喂狗) */
#define IWDG_KR_UNLOCK   0x5555U   /* 解锁 PR/RLR 写保护 */
#define IWDG_KR_START    0xCCCCU   /* 启动(启动后软件再也关不掉) */

/** 喂狗。任何上下文都能调用, 只是往 KR 写一个常量 */
#define WDG_FEED()       (IWDG->KR = IWDG_KR_RELOAD)

/* ===== 全局状态 ===== */
extern volatile uint32_t g_reset_csr;   /* 开机时锁存的 RCC->CSR, 用于判断复位原因 */
extern volatile uint8_t  g_wdg_ready;   /* 1 = IWDG 已配置好, 可以安全喂狗 */

/**
 * @brief 本次复位是否由 IWDG 超时(固件卡死)导致
 *
 * 数据来自开机时 Wdg_LatchResetCause() 锁存的 g_reset_csr —— 那之后
 * RCC->CSR 的标志已被清掉, 再去读 RCC->CSR 永远是 0。
 * 用途: 判断"这次是看门狗把车咬重启的" → 可以跳过软启动直接续跑。
 */
static inline uint8_t Wdg_ResetWasWatchdog(void)
{
    return (g_reset_csr & RCC_CSR_IWDGRSTF) ? 1U : 0U;
}

/**
 * @brief 锁存本次复位原因, **必须在 HAL_Init() 之前调用**
 *
 * 没有这个就分不清"固件卡死后看门狗把它重启了"(需求达成)和"电池瞬间掉电"
 * (硬件问题)。同时会清掉复位标志, 否则下一次读到的永远是上一次的结果。
 */
void Wdg_LatchResetCause(void);

/**
 * @brief 主工程: 改写 IWDG 配置并启动
 *
 * 必须在 SystemClock_Config() 之后立刻调用。为什么不等启动快结束再开:
 * 如果 bootloader 已经开了狗(周期未知), 一次重载只能买到"一个未知周期",
 * 而主工程启动路径远长于一个未知周期 —— 必须在时钟配好后立刻用我们已知的
 * 5 秒配置把它覆盖掉。(IWDG 的 PR/RLR 在启动后写 0x5555 解锁仍可修改。)
 */
void Wdg_ConfigAndStart(void);

/**
 * @brief 自喂式延时 —— 长阻塞时必须用它替代 HAL_Delay, 否则会被自己开的狗咬
 * @note  含 g_wdg_ready 守卫: IWDG 尚未配置时绝不喂狗
 */
void Wdg_DelayMs(uint32_t ms);

/**
 * @brief 在**中断已关**的情况下限时喂狗 ms 毫秒, 然后返回
 *
 * 用在 Error_Handler / 栈溢出钩子里: 先喂够时间让 LCD 上的致命提示能被看清,
 * 再停止喂狗让 IWDG 自动复位整板。
 *
 * ⚠ 这里不能用 HAL_GetTick() —— 中断已关, TIM2 不再累加, tick 永远不动。
 *   也不能用"空转 N 次"来计时: 循环耗时随优化等级和 flash 等待周期变化,
 *   曾经写成"约 10 秒"实际只有 1.1 秒(差了近一个数量级)。
 *   所以用 DWT 周期计数器, 按 SystemCoreClock 换算, 与优化等级无关。
 */
void Wdg_HoldFeedingMs(uint32_t ms);

/** 复位原因 → 一行可读字符串(正好 7 字符, 塞得进 LCD) */
const char *Wdg_ResetCauseStr(uint32_t csr);

#endif /* __IWDG_H */
