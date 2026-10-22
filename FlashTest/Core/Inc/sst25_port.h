/**
 * @file    sst25_port.h
 * @brief   SST25 外部 SPI Flash 的「端口层」— 全工程唯一与硬件绑定的一层
 *
 * ================================================================
 * 硬件事实(来自 LUBY 控制器原理图, 设计号 U2)
 * ================================================================
 *   芯片: SST25VF016 (16 Mbit = 2 MiB SPI NOR)
 *   CS   (pin1 CE)   -> PB12   软件控制(不是硬件 NSS)
 *   SCK  (pin6)      -> PB13   SPI2_SCK
 *   MISO (pin2 SO)   -> PB14   SPI2_MISO
 *   MOSI (pin5 SI)   -> PB15   SPI2_MOSI
 *   WP   (pin3)      -> 经 R4 10K 上拉 3.3V(推断)
 *   HOLD (pin7)      -> 原理图文字层看不到连线, 待实机确认
 *   SPI2 为标准引脚映射, **不需要 AFIO remap**。
 *
 * ================================================================
 * 为什么全走寄存器而不用 HAL
 * ================================================================
 *   本工程的 Drivers/ 里**没有** stm32f1xx_hal_spi.c(只有 19 个 HAL 驱动,
 *   SPI 不在其中), 而 car7.12.ioc 里 LibraryCopy=1 ——
 *   一旦有人在 CubeMX 里重新 Generate Code, 手工补进去的 HAL 驱动会被当成
 *   "用不到的文件"删掉。寄存器级初始化一共二十来行, 彻底绕开这个风险。
 *
 * ================================================================
 * 三个工程共用本文件, 差异全部收敛到下面这几个宏
 * ================================================================
 */
/* ================================================================
 * ★ 本工程(FlashTest)专用配置 ★
 *
 * 三个工程共用这一份代码, 差异就收敛在这几行。主工程与标定工程
 * 保持默认("只允许动最后 8KB"); 而本工程的任务正是**探测哪些扇区
 * 能安全读写**, 所以必须把窗口放宽到整片 —— 否则连扇区 0 都碰不到。
 *
 * 放宽窗口不等于放弃保护: 真正的防线是 cal_store.c 的**扇区内容分类**
 * (文件系统 / 固件镜像一律硬拒绝) 加上"必须按住 KEY2 开机的人工武装"。
 * ================================================================ */
#define SST25_ERASE_WIN_BASE   0x000000UL   /* 整片 */
#define SST25_ERASE_WIN_END    0x200000UL

#ifndef __SST25_PORT_H
#define __SST25_PORT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== 各工程差异宏(可在工程侧的编译选项里覆盖) ===== */

/* SPI2 时钟分频。APB1 = 36MHz, BR[2:0] 编码见 RM0008:
 *
 *   BR[2:0]   分频   SPI2 时钟    C 宏写法
 *   000       /2     18    MHz    #define SST25_SPI_BR 0               <- SPI2 上限
 *   001       /4     9     MHz    SPI_CR1_BR_0
 *   010       /8     4.5   MHz    SPI_CR1_BR_1                         <- 默认
 *   011       /16    2.25  MHz    SPI_CR1_BR_0|SPI_CR1_BR_1
 *   100       /32    1.125 MHz    SPI_CR1_BR_2
 *
 * 等示波器确认过 SCK/MOSI 边沿之后, 改这一个宏就能提速。
 * (注意 #ifndef 判的是"有没有定义"而不是值, 所以想用 /2 要写 `SST25_SPI_BR 0`。) */
#ifndef SST25_SPI_BR
#define SST25_SPI_BR          SPI_CR1_BR_1
#endif

/* 本工程允许擦写的地址窗口(左闭右开)。
 * 这是第二道闸 —— 第一道是器件物理边界(2MiB), 第三道是 cal_store 的扇区分类。
 * 主工程是最严的: 只允许动最后 8KB(两个记录槽)。
 * 工具工程才会放宽窗口(否则 flash 测试工程连扇区 0 都碰不到)。 */
#ifndef SST25_ERASE_WIN_BASE
#define SST25_ERASE_WIN_BASE  0x1FE000UL
#endif
#ifndef SST25_ERASE_WIN_END
#define SST25_ERASE_WIN_END   0x200000UL
#endif

/* ===== 器件几何 ===== */
#define SST25_DEV_SIZE        0x200000UL   /* 2 MiB 器件物理边界 */
#define SST25_SECTOR_SIZE     0x1000UL     /* 4 KiB 扇区(SST25 的擦除单位) */

/* ===== 端口原语: 协议层只调用这 5 个 + 1 个守卫 ===== */
void     SST25_Port_Init(void);       /* SPI2 + GPIO 寄存器级初始化 */
void     SST25_Port_CsLow(void);
void     SST25_Port_CsHigh(void);
uint8_t  SST25_Port_Xfer(uint8_t tx); /* 写一字节并读回一字节(必须成对) */
void     SST25_Port_WaitBusy(void);   /* 等 SPI 移位寄存器空(拉高 CS 之前必须调) */

/**
 * 地址是否允许擦写: 同时检查器件物理边界 [0, 2MiB) 与工程窗口。
 * 擦除和编程两条路径**都要**先过这一关。
 */
bool     SST25_Port_AddrWritable(uint32_t addr);

#endif /* __SST25_PORT_H */
