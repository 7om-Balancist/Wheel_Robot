/**
 * @file    cal_store.h
 * @brief   灰度标定值在外部 SPI Flash 上的「存储层」— A/B 双槽 + CRC32
 *
 * ================================================================
 * 本工程只读, 不写
 * ================================================================
 *   主工程只做一件事: 上电 CalStore_Load() 把记录读回来。
 *   写入(CalStore_Save)与它的擦除窗口守卫(扇区分类 Cal_ClassifySector)
 *   在 CalibTool 工程的同名文件里 —— 写入是现场标定动作, 不属于飞行固件。
 *   好处: 主固件里**没有任何一条能改动 flash 的代码路径**。
 *
 * ================================================================
 * 布局(2 MiB 器件, 用最后两个 4KB 扇区)
 * ================================================================
 *   0x1FF000  扇区 511  槽 A                      0x1FFFFF
 *   0x1FE000  扇区 510  槽 B                      0x1FEFFF
 *   0x1F0000  硬地板 —— 低于此地址一律拒绝擦写
 *
 *   为什么必须占**两个独立扇区**: 0x20 的最小擦除单位就是 4KB, 两条记录放
 *   同一个扇区里擦一个必然连坐, 冗余就失去意义了。代价 8KB / 2MiB。
 *
 * ================================================================
 * 记录格式(24 字节, 小端, 放在扇区偏移 0x000)
 * ================================================================
 *   偏移  长度  字段
 *   0x00   4    magic    = 0x314C4143  (小端 = 'C','A','L','1')
 *   0x04   2    version  = 1
 *   0x06   2    length   = 8  (sizeof(GrayBaseline_t), 不是 6)
 *   0x08   4    seq      单调递增, 从 1 起; 0 与 0xFFFFFFFF 保留不用
 *   0x0C   8    payload  front_edge / rear_edge / ground[0] / ground[1] (各 u16)
 *   0x14   4    crc32    CRC-32/ISO-HDLC, 覆盖 [0x00..0x13] 共 20 字节
 *   ---- 合计 0x18 = 24 字节
 *   0x18..0xFFF 保持擦除态 0xFF(将来 v2 原地加长不用搬家)
 *
 *   ⚠ payload 里存的是**原始基准值**(实测的四个 AD 读数), 不是乘过系数
 *     的判定阈值 —— 阈值由 calibrate.c 读回后再算。这样改系数不必重标定。
 */
#ifndef __CAL_STORE_H
#define __CAL_STORE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "calibrate.h"    /* GrayBaseline_t */
#include "sst25.h"

/* ===== 地址分配 =====
 * 这三个地址**只允许出现在这里**。将来确认了哪些扇区安全要迁移时,
 * 只改这三行, cal_store.c 和驱动层一行都不用动。 */
#define CAL_SLOT_A_ADDR   0x1FF000UL   /* 扇区 511, 主槽 */
#define CAL_SLOT_B_ADDR   0x1FE000UL   /* 扇区 510, 备槽 */
#define CAL_SAFE_FLOOR    0x1F0000UL   /* 硬地板(写路径用, 见 CalibTool) */

#define CAL_REC_MAGIC     0x314C4143UL /* 'C','A','L','1' */
#define CAL_REC_VERSION   1U
#define CAL_REC_SIZE      24U
#define CAL_PAYLOAD_LEN   8U           /* sizeof(GrayBaseline_t) */

/* GrayBaseline_t 必须正好 8 字节 —— 记录布局是按它硬编码的。
 * 这个断言是防"某天有人给结构体加了个字段, 标定工程和主工程分别编译,
 * 静默错位而 CRC 还照样通过"这种情况。 */
#if defined(__GNUC__)
_Static_assert(sizeof(GrayBaseline_t) == CAL_PAYLOAD_LEN,
               "GrayBaseline_t 大小变了, cal_store 的记录布局必须同步修改");
#endif

/* ===== 返回码 ===== */
typedef enum {
    CAL_OK = 0,
    CAL_NOT_FOUND,        /* 两个槽都没有有效记录(首次上电 / 没烧过标定工程) */
    CAL_CORRUPT,          /* 有记录但 CRC 或字段不合法 */
    CAL_VERSION_MISMATCH, /* 版本号不是本固件认识的 */
    CAL_IMPLAUSIBLE,      /* CRC 正确但数值退化 —— 见下方"有效性判定"第 5 条 */
    CAL_IO_ERROR,         /* flash 读取失败 */
} Cal_Status_t;

/**
 * @brief 从 flash 读取标定值
 *
 * 有效性判定(全部满足才算有效):
 *   1. magic == CAL_REC_MAGIC   2. version == 1   3. length == 8
 *   4. crc32 正确
 *   5. **数值合理性**: 四个 u16 都在 [1,4094], 且按其重算出来的
 *      on_stage 与 edge_thresh 也都在 [1,4094]
 *
 * 第 5 条拦的是最危险的情况: 一条 CRC 正确但数值退化的记录会让 on_stage
 * 变成 0 或 4095, 于是"台上判定"永久恒真或恒假 —— 车会**永远以为自己
 * 在台上**, 直接冲下擂台。这比读不到记录危险得多。
 *
 * 两个槽都有效时取 seq 大的那个(seq 比较用有符号差, 天然处理回绕)。
 */
Cal_Status_t CalStore_Load(GrayBaseline_t *out);

/** 最近一次 Load 的状态(供遥测上报, 不必再存一份) */
Cal_Status_t Cal_LastStatus(void);

/** 返回码 → 可读字符串(LCD 一行 16 列以内) */
const char  *Cal_StatusStr(Cal_Status_t st);

/** CRC-32/ISO-HDLC。参数逐位锁定: poly=0xEDB88320(reflected), init=0xFFFFFFFF,
 *  refin=true, refout=true, xorout=0xFFFFFFFF */
uint32_t Cal_Crc32(const uint8_t *data, uint32_t len);

/** CRC 自检: crc32("123456789") 必须 == 0xCBF43926。
 *  这是"两个分别编译的固件逐位一致"的唯一可验证证据 —— 不自检的 CRC 等于没有 CRC。 */
bool Cal_Crc32SelfTest(void);

#endif /* __CAL_STORE_H */
