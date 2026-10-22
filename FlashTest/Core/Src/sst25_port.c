/**
 * @file    sst25_port.c
 * @brief   SST25 端口层实现 — SPI2 寄存器级初始化 + CS/字节收发原语
 * @see     sst25_port.h 顶部的硬件引脚与设计理由
 */
#include "sst25_port.h"

/* ================================================================
 * 时序纪律
 *
 * SST25 要求:
 *   · CS# 下降沿到第一个 SCK 沿 >= tSLCH (5 ns)
 *   · 两条独立命令之间 CS# 保持高 >= tSHSL (50 ns)
 * 72MHz 下 50ns 只有 4 个时钟周期, 而高优化等级会把背靠背的存储操作
 * 合并/重排 —— 必须用 __NOP() 显式撑开。少了这几条 NOP 的典型症状是
 * "偶尔一次写不进去"这种极难复现的间歇故障。
 * ================================================================ */
#define SST25_NOP()   do { __NOP(); __NOP(); } while (0)

void SST25_Port_CsLow(void)
{
    GPIOB->BRR = GPIO_PIN_12;
    SST25_NOP();
}

void SST25_Port_CsHigh(void)
{
    GPIOB->BSRR = GPIO_PIN_12;
    SST25_NOP();
    __NOP();          /* 命令间 CS 高电平多留一点余量 */
}

/**
 * @brief SPI2 + PB12..PB15 寄存器级初始化
 *
 * 注意这里刻意没用 HAL_SPI_Init: Drivers/ 里没有 hal_spi.c(见头文件说明),
 * 而且 HAL_SPI_TransmitReceive() 在 pRxData==NULL 时直接返回 HAL_ERROR
 * 一个字节都不发 —— 这类陷阱在寄存器级完全不存在。
 */
void SST25_Port_Init(void)
{
    /* ---- 1. 时钟使能 ---- */
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;   /* SPI2 在 APB1 */
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   /* GPIOB */
    (void)RCC->APB1ENR;                   /* 读回一次: 保证时钟已生效再写寄存器 */

    /* ---- 2. 硬复位 SPI2 ----
     * bootloader 跳转过来时 SPI2 可能停在半途(传输未完成/CR1 残留),
     * 先清 SPE 再走一次外设复位, 拿到确定的初始状态。 */
    SPI2->CR1 = 0U;
    RCC->APB1RSTR |=  RCC_APB1RSTR_SPI2RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_SPI2RST;

    /* ---- 3. CS 先拉高再配成输出 ----
     * 顺序很重要: 先写 ODR 再切模式, 避免配置瞬间产生一个低电平毛刺,
     * 那个毛刺会被 flash 当成一次片选起始。 */
    GPIOB->BSRR = GPIO_PIN_12   /* CS 初始高 */
                | GPIO_PIN_14;  /* MISO 的 ODR=1 表示选"上拉"(见下) */

    /* ---- 4. GPIOB CRH: PB12..PB15 ---- */
    GPIOB->CRH &= ~(0xFFFFUL << 16);          /* 只清 PB12..PB15, 保留 PB8..PB11 */
    GPIOB->CRH |=  (0x3UL << 16)              /* PB12 CS   : 通用推挽输出 50MHz */
                |  (0xBUL << 20)              /* PB13 SCK  : 复用推挽输出 50MHz */
                |  (0x8UL << 24)              /* PB14 MISO : 输入 + 上拉 */
                |  (0xBUL << 28);             /* PB15 MOSI : 复用推挽输出 50MHz */

    /* 为什么 MISO 是"输入+上拉"而不是复用推挽:
     * RM0008 的 SPI 主机模式引脚表把 MISO 列为 Input floating / pull-up。
     * CS 无效时 flash 的 SO 脚是三态, 悬空输入会被相邻的 SCK 串扰,
     * 内部上拉让空闲电平确定为 1。 */

    /* ---- 5. SPI2 主机模式 ---- */
    SPI2->CR2     = 0U;                       /* 不用中断, 不用 DMA */
    SPI2->I2SCFGR = 0U;                       /* 保留寄存器, 写 0 即"非 I2S 模式" */
    SPI2->CR1     = SPI_CR1_MSTR              /* 主机 */
                  | SPI_CR1_SSM               /* 软件 NSS 管理 */
                  | SPI_CR1_SSI               /* 内部 NSS 拉高, 防止 MODF */
                  | (uint16_t)SST25_SPI_BR    /* 波特率分频 */
                  | SPI_CR1_SPE;              /* 使能 SPI */

    /* CR1 期望值速查(含 SPE): /2=0x0344  /4=0x034C  /8=0x0354  /16=0x035C
     * 为什么 NSS 必须软件(SSM=1 且 SSI=1):
     *   (a) SST25 要求"命令字节 + 3 字节地址 + 数据/状态"在同一次 CS 有效期内
     *       连续送出。硬件 NSS 的 NSSP 模式会每字节拉高一次片选, 把 RDSR 和
     *       擦除地址切碎。
     *   (b) F1 主机模式的硬件 NSS 需要外部上拉(本板没有), 一旦被采到低电平
     *       就进 MODF 状态, 硬件自动清 SPE, 此后所有传输静默失败。
     *       SSM=1/SSI=1 让 MODF 状态机根本进不去。 */

    /* ---- 6. 冲掉可能残留的 RXNE ---- */
    while (SPI2->SR & SPI_SR_RXNE) {
        (void)*(volatile uint8_t *)&SPI2->DR;
    }
}

/**
 * @brief 写一字节并读回一字节
 *
 * 必须成对读写: STM32F1 的 SPI 没有 OVR(溢出)标志位, 只写不读会让 RXNE
 * 一直挂着, 后续字节全体错位。命令阶段的读回值可以丢弃, 但"读"这个动作
 * 不能省 —— 典型 bug 就是发 0x03 + 3 字节地址时不读 DR, 数据阶段整体错位。
 *
 * 用计数 guard 而不是 tick 超时: tick 依赖 TIM2, 而本驱动有可能在
 * HAL_Init() 之前被调用(那时 tick 永远不走, 会死等)。
 */
uint8_t SST25_Port_Xfer(uint8_t tx)
{
    uint32_t guard;

    guard = 300000U;
    while (!(SPI2->SR & SPI_SR_TXE) && --guard) { }
    *(volatile uint8_t *)&SPI2->DR = tx;

    guard = 300000U;
    while (!(SPI2->SR & SPI_SR_RXNE) && --guard) { }
    return *(volatile uint8_t *)&SPI2->DR;
}

/**
 * @brief 等 SPI 移位寄存器彻底空
 *
 * 必须在拉高 CS 之前调用: 函数返回时最后一个字节可能还在移位寄存器里,
 * 此刻拉高 CS 会把最后一字节截断。F1 的 BSY 位还有"晚一个 SCK 才清"的
 * 已知毛病, 所以这里用轮询而不是指望 HAL。
 */
void SST25_Port_WaitBusy(void)
{
    uint32_t guard = 300000U;
    while ((SPI2->SR & SPI_SR_BSY) && --guard) { }
}

/**
 * @brief 地址是否允许擦写
 *
 * 第二道闸。第一道是 cal_store 的扇区分类(看内容), 第三道在协议层
 * (ProgramByte 写之前再断言目标确实处于擦除态)。三层都过才动手。
 */
bool SST25_Port_AddrWritable(uint32_t addr)
{
    if (addr >= SST25_DEV_SIZE) return false;              /* 器件物理边界 */
    if (addr < SST25_ERASE_WIN_BASE) return false;         /* 本工程窗口下限 */
    if (addr >= SST25_ERASE_WIN_END) return false;         /* 本工程窗口上限 */
    return true;
}
