/**
 * @file    sst25.h
 * @brief   SST25 系列 SPI NOR Flash 的「协议层」
 *
 * 本层只管操作码、地址拼装、擦写读算法和状态轮询;
 * 不 include 任何工程头文件, 不碰 GPIO, 只调用 sst25_port.h 的 6 个原语。
 * 这样三个工程(main / calib_test / flash_test)可以逐字节共用同一份代码。
 *
 * 指令集(已按数据手册核对):
 *   WREN 0x06   WRDI 0x04   RDSR 0x05   WRSR 0x01   EWSR 0x50
 *   READ 0x03   FAST_READ 0x0B(地址后跟 1 个 dummy 字节)
 *   JEDEC_ID 0x9F(3 字节)   RDID 0x90(地址 0x000000, 2 字节)
 *   BYTE_PROGRAM 0x02       AAI_WORD_PROGRAM 0xAD
 *   SECTOR_ERASE_4K 0x20    BLOCK_ERASE_32K 0x52   BLOCK_ERASE_64K 0xD8
 *   片擦除 0x60/0xC7 —— **固件里绝对不允许出现**
 *
 * 状态寄存器位: bit0 BUSY / bit1 WEL / bit2 BP0 / bit3 BP1 / bit4 BP2 /
 *               bit5 BP3 / bit6 AAI / bit7 BPL     (AAI 是 bit6, 不是 bit4)
 */
#ifndef __SST25_H
#define __SST25_H

#include "sst25_port.h"

/* ===== 协议层返回码 ===== */
typedef enum {
    SST25_OK = 0,
    SST25_ERR_PARAM,        /* 入参非法 */
    SST25_ERR_ID,           /* 读不到器件 ID(全 0x00/全 0xFF) —— 最可能是 HOLD# 被拉低或没焊 */
    SST25_ERR_WINDOW,       /* 地址越过允许擦写的窗口 */
    SST25_ERR_PROTECTED,    /* 被块保护(BP/BPL)挡住 —— 注意: WREN 照样成功, 只有写后回读才能发现 */
    SST25_ERR_WEL,          /* WREN 后回读 WEL 仍为 0 */
    SST25_ERR_NOT_ERASED,   /* 目标区不是全 0xFF, 拒绝编程(不替调用者擅自擦除) */
    SST25_ERR_TIMEOUT,      /* 等 BUSY 超时 */
    SST25_ERR_VERIFY,       /* 写后回读不一致 —— 唯一的真凭实据 */
} Sst25Status_t;

/* ===== 器件类别(状态寄存器位定义不同, 解保护序列也不同) ===== */
typedef enum {
    SST25_FAMILY_UNKNOWN = 0,
    SST25_FAMILY_SST,       /* 厂商码 0xBF */
    SST25_FAMILY_WINBOND,   /* 厂商码 0xEF (W25Q16 等兼容件) */
} Sst25Family_t;

/* ===== 器件信息(SST25_Init 填充, 供 LCD/串口上报) ===== */
typedef struct {
    uint8_t       jedec[3];   /* 0x9F 读到的 3 字节 */
    uint8_t       status;     /* RDSR 原始值 */
    Sst25Family_t family;
    bool          id_ok;
} Sst25Info_t;

extern Sst25Info_t g_sst25_info;

/* ===== API ===== */

/**
 * @brief 初始化: SPI2 + 器件抢救序列 + 读 ID + 读状态
 *
 * 抢救序列顺序不能变(万一上次掉电打断了 AAI 序列, 器件会停在 AAI 模式,
 * 此时除 RDSR/WRDI 外所有命令都被忽略, 表现为"芯片突然死了"):
 *   1. 端口层初始化   2. 无条件补一次 WRDI   3. 等 BUSY 清零
 *   4. 读 JEDEC ID(连读两次要求一致)   5. 读状态寄存器并原样上报
 *   6. 若 BP != 0 → 尝试解保护
 *
 * @return SST25_OK / SST25_ERR_ID / SST25_ERR_TIMEOUT
 */
Sst25Status_t SST25_Init(void);

/** 读 JEDEC ID(0x9F), 3 字节。失败返回 false */
bool          SST25_ReadJedecId(uint8_t out[3]);

/** 读状态寄存器(0x05) */
uint8_t       SST25_ReadStatus(void);

/** 解除块保护。SST 族用 EWSR+WRSR, Winbond 族直接 WRSR */
Sst25Status_t SST25_Unprotect(void);

/** 擦除 4KB 扇区。addr 会自动向下对齐到扇区边界 */
Sst25Status_t SST25_Erase4K(uint32_t addr);

/**
 * @brief 读任意长度(内部用 0x03 Read, 全程 CS 不翻)
 * @note  CS 必须一直保持低 —— 中途拉高再拉低等于重新发起一次读命令,
 *        数据会整体错位。这是最容易写错的地方。
 */
Sst25Status_t SST25_Read(uint32_t addr, uint8_t *dst, uint32_t len);

/**
 * @brief 编程任意长度(0x02 单字节循环, 每字节前重发 WREN)
 *
 * 刻意不用 AAI(0xAD): AAI 快但引入三个失效模式(必须 WRDI 收尾否则芯片
 * "变砖"、起始地址必须偶对齐、中途断电永久停在 AAI)。24 字节用单字节
 * 编程总耗时不到 1 ms, 用这点时间换掉三个坑非常划算。
 *
 * 入口会先检查目标区是否全 0xFF, 不是就返回 SST25_ERR_NOT_ERASED,
 * **绝不自作主张替调用者擦除**。
 */
Sst25Status_t SST25_ProgramBytes(uint32_t addr, const uint8_t *src, uint32_t len);

/** 目标区是否全 0xFF */
bool          SST25_IsErased(uint32_t addr, uint32_t len);

/** 把返回码变成可读字符串(LCD/串口上报用) */
const char   *SST25_StatusStr(Sst25Status_t st);

#endif /* __SST25_H */
