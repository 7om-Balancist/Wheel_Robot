/**
 * @file    iwdg.c
 * @brief   IWDG 实现 — 寄存器级, 5 秒窗口
 * @see     iwdg.h 顶部的超时预算与喂狗策略
 */
#include "iwdg.h"

volatile uint32_t g_reset_csr = 0U;
volatile uint8_t  g_wdg_ready = 0U;

/* 预分频: 0=÷4 1=÷8 2=÷16 3=÷32 4=÷64 5=÷128 6=÷256 7=÷256 */
#define IWDG_PR_DIV64    0x04U
/* (3124+1) × 64 / 40000Hz = 5.00 s @ LSI 标称 40kHz
 * LSI 实际 30~60kHz → 真实窗口 3.33 ~ 6.67 s */
#define IWDG_RLR_5S      3124U
#define IWDG_SR_BUSY_MASK  (IWDG_SR_PVU | IWDG_SR_RVU)

void Wdg_LatchResetCause(void)
{
    g_reset_csr = RCC->CSR;
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

/* 起 LSI 并等它就绪。IWDG 的寄存器同步靠 LSI 时钟, 不先起 LSI 写进去也没用 */
static void lsi_start(void)
{
    uint32_t guard = 200000U;

    RCC->CSR |= RCC_CSR_LSION;
    while (!(RCC->CSR & RCC_CSR_LSIRDY) && --guard) { }
}

/* 等 PR/RLR 的同步完成, 否则参数会被静默丢弃 */
static void iwdg_wait_sync(void)
{
    uint32_t guard = 200000U;
    while ((IWDG->SR & IWDG_SR_BUSY_MASK) != 0U && --guard) { }
}

void Wdg_ConfigAndStart(void)
{
    /* 先重载一次: bootloader 若留了个短周期的狗, 这一行救命。
       (IWDG 没开时它是空操作。) */
    IWDG->KR = IWDG_KR_RELOAD;

    lsi_start();

    /* 断点停住 CPU 时不让狗继续数, 否则单步调试每 5 秒复位一次。
       注意 STM32F1 的 DBGMCU 没有独立时钟门控, 不需要(也**不存在**
       __HAL_RCC_DBGMCU_CLK_ENABLE 这个宏), 直接写 CR 即可。 */
    __HAL_DBGMCU_FREEZE_IWDG();

    IWDG->KR  = IWDG_KR_UNLOCK;      /* 解锁 PR/RLR */
    IWDG->PR  = IWDG_PR_DIV64;
    IWDG->RLR = IWDG_RLR_5S;
    iwdg_wait_sync();

    IWDG->KR = IWDG_KR_START;        /* 启动 —— 此后软件再也关不掉 */
    IWDG->KR = IWDG_KR_RELOAD;       /* 立刻装一次, 从满窗口开始数 */

    g_wdg_ready = 1U;
}

void Wdg_DelayMs(uint32_t ms)
{
    while (ms > 0U) {
        uint32_t chunk = (ms > 100U) ? 100U : ms;
        if (g_wdg_ready) {
            WDG_FEED();              /* 关键: IWDG 未配置前不能喂 */
        }
        HAL_Delay(chunk);
        ms -= chunk;
    }
}

void Wdg_HoldFeedingMs(uint32_t ms)
{
    uint32_t start, span;

    /* DWT 周期计数器: 与优化等级无关, 也不需要中断 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 72MHz 下 10000ms -> 720000000, 未溢出 uint32(上限 42.9 亿) */
    span  = (uint32_t)((SystemCoreClock / 1000U) * ms);
    start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < span) {
        IWDG->KR = IWDG_KR_RELOAD;
    }
}

const char *Wdg_ResetCauseStr(uint32_t csr)
{
    if (csr & RCC_CSR_IWDGRSTF) return "RST:WDG";   /* 看门狗把它咬复位了 */
    if (csr & RCC_CSR_SFTRSTF)  return "RST:SW";
    if (csr & RCC_CSR_PORRSTF)  return "RST:PWR";
    if (csr & RCC_CSR_PINRSTF)  return "RST:PIN";
    return "RST:?";
}
