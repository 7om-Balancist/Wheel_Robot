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
 *   本工程的 Drivers/ 里**没有** stm32f1xx_hal_spi.c / hal_iwdg.c(只有 19 个
 *   HAL 驱动, SPI 和 IWDG 都不在其中), 而 car7.12.ioc 里 LibraryCopy=1 ——
 *   一旦有人在 CubeMX 里重新 Generate Code, 手工补进去的 HAL 驱动会被当成
 *   "用不到的文件"删掉。寄存器级初始化一共二十来行, 彻底绕开这个风险。
 *
 * ================================================================
 * 本工程只有只读原语
 * ================================================================
 *   主工程对 flash 只读。写成才需要的两个东西**不在本文件里**:
 *     · SST25_ERASE_WIN_BASE/END —— 允许擦写的地址窗口
 *     · SST25_Port_AddrWritable() —— 窗口检查
 *   CalibTool 工程的同名文件才有它们(那里的窗口放宽到整片)。
 *   这样"主固件越界擦掉 flash 内容"连编译都过不去。
 *
 *   可调项只剩一个: SST25_SPI_BR(见下)。
 */
#ifndef __SST25_PORT_H
#define __SST25_PORT_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

/* ===== SPI2 时钟分频(三个工程共用的唯一可调宏) =====
 * APB1 = 36MHz, BR[2:0] 编码见 RM0008:
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

/* ===== 器件几何 =====
 * 2 MiB 分 512 个 4 KiB 扇区; 擦除单位是扇区, 读没有单位限制。
 * (标定记录放在最后两个扇区: 0x1FF000 / 0x1FE000, 见 cal_store.h) */
#define SST25_DEV_SIZE        0x200000UL   /* 器件物理边界 */
#define SST25_SECTOR_SIZE     0x1000UL     /* 4 KiB 扇区 */

/* ===== 端口原语: 协议层只调用这 5 个 ===== */
void     SST25_Port_Init(void);       /* SPI2 + GPIO 寄存器级初始化 */
void     SST25_Port_CsLow(void);
void     SST25_Port_CsHigh(void);
uint8_t  SST25_Port_Xfer(uint8_t tx); /* 写一字节并读回一字节(必须成对) */
void     SST25_Port_WaitBusy(void);   /* 等 SPI 移位寄存器空(拉高 CS 之前必须调) */

#endif /* __SST25_PORT_H */
