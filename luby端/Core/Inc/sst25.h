/**
 * @file    sst25.h
 * @brief   SST25 系列 SPI NOR Flash 的「协议层」—— **只读子集**
 *
 * ================================================================
 * 本工程只有只读部分
 * ================================================================
 *   主工程对 flash 只做一件事: 上电把标定记录读回来。
 *   擦除(SECTOR_ERASE 0x20)与编程(BYTE_PROGRAM 0x02)两条路径**在这份
 *   头文件里根本不存在** —— 这样"主固件误写 flash"在编译期就不可能发生。
 *   写入路径完整实现在 CalibTool 工程的同名文件里(sst25.c/h 会多出
 *   SST25_Erase4K / SST25_ProgramBytes / SST25_IsErased / SST25_Unprotect)。
 *
 *   读取用到的操作码:
 *     WRDI 0x04 (抢救 AAI 残留)   RDSR 0x05   READ 0x03   JEDEC_ID 0x9F
 *
 *   状态寄存器位: bit0 BUSY / bit1 WEL / bit2 BP0 / bit3 BP1 / bit4 BP2 /
 *                 bit5 BP3 / bit6 AAI / bit7 BPL
 *
 * 本层只管操作码、地址拼装和状态轮询; 不 include 任何工程头文件,
 * 不碰 GPIO, 只调用 sst25_port.h 的原语。
 */
#ifndef __SST25_H
#define __SST25_H

#include "sst25_port.h"

/* ===== 协议层返回码 ===== */
typedef enum {
    SST25_OK = 0,
    SST25_ERR_PARAM,        /* 入参非法 */
    SST25_ERR_ID,           /* 读不到器件 ID(全 0x00/全 0xFF) —— 最可能是 HOLD# 被拉低或没焊 */
    SST25_ERR_TIMEOUT,      /* 等 BUSY 超时 */
} Sst25Status_t;

/* ===== 器件类别(状态寄存器位定义不同) ===== */
typedef enum {
    SST25_FAMILY_UNKNOWN = 0,
    SST25_FAMILY_SST,       /* 厂商码 0xBF */
    SST25_FAMILY_WINBOND,   /* 厂商码 0xEF (W25Q16 等兼容件) */
} Sst25Family_t;

/* ===== 器件信息(SST25_Init 填充, 供上报) ===== */
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
 * 此时除 RDSR/WRDI 外**所有命令都被忽略 —— 连读都不行**, 表现为"芯片突然
 * 死了", 而主工程只需要读, 所以这一步是必需的, 不是可选的):
 *   1. 端口层初始化   2. 无条件补一次 WRDI   3. 等 BUSY 清零
 *   4. 读 JEDEC ID(连读两次要求一致)   5. 读状态寄存器
 *
 * @return SST25_OK / SST25_ERR_ID / SST25_ERR_TIMEOUT
 */
Sst25Status_t SST25_Init(void);

/** 读 JEDEC ID(0x9F), 3 字节。失败返回 false */
bool          SST25_ReadJedecId(uint8_t out[3]);

/** 读状态寄存器(0x05) */
uint8_t       SST25_ReadStatus(void);

/**
 * @brief 读任意长度(内部用 0x03 Read, 全程 CS 不翻)
 * @note  CS 必须一直保持低 —— 中途拉高再拉低等于重新发起一次读命令,
 *        数据会整体错位。这是最容易写错的地方。
 */
Sst25Status_t SST25_Read(uint32_t addr, uint8_t *dst, uint32_t len);

/** 把返回码变成可读字符串(LCD/串口上报用) */
const char   *SST25_StatusStr(Sst25Status_t st);

#endif /* __SST25_H */
