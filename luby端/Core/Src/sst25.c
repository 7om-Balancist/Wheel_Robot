/**
 * @file    sst25.c
 * @brief   SST25 协议层实现 — **只读子集**: 操作码 / 读 / 状态轮询
 * @see     sst25.h 顶部的指令集与状态位表
 *
 * 这里没有 WREN/WRSR/EWSR/BYTE_PROGRAM/SECTOR_ERASE —— 主工程不写 flash。
 * 需要擦写的场合请用 CalibTool 工程, 那份 sst25.c 是完整版。
 */
#include "sst25.h"
#include <string.h>

/* ===== 操作码 ===== */
#define OP_WRDI          0x04U   /* 只用于上电抢救 AAI 残留, 不是写操作 */
#define OP_RDSR          0x05U
#define OP_READ          0x03U
#define OP_JEDEC_ID      0x9FU

/* ===== 状态寄存器位 ===== */
#define SR_BUSY          0x01U

/* ===== 轮询 guard =====
 * 刻意不用 HAL_GetTick 做超时: tick 依赖 TIM2, 而本驱动有可能在
 * HAL_Init() 之前被调用(那时 tick 永远不走, 用 tick 会死等)。
 * 改用一个足够大的迭代计数, 既能兜住最坏耗时, 又永远不可能挂死。
 *   一次 RDSR 轮询 ≈ 5µs(4.5MHz 下 2 字节 + GPIO 开销)
 *   上电抢救时若器件正卡在一次 4KB 擦除里, 最坏 200ms → 约 40000 次,
 *   guard 取 40 万, 留 10 倍余量。
 */
#define GUARD_POLL       400000U

Sst25Info_t g_sst25_info;

/* ================================================================
 * 内部基础操作
 * ================================================================ */

/* 发一条无参数命令(只用于 WRDI 抢救) */
static void send_cmd(uint8_t op)
{
    SST25_Port_CsLow();
    (void)SST25_Port_Xfer(op);
    SST25_Port_WaitBusy();
    SST25_Port_CsHigh();
}

/* 读状态寄存器 —— 命令与状态字节必须在同一次 CS 有效期内 */
static uint8_t rdsr(void)
{
    uint8_t v;
    SST25_Port_CsLow();
    (void)SST25_Port_Xfer(OP_RDSR);
    v = SST25_Port_Xfer(0xFFU);
    SST25_Port_WaitBusy();
    SST25_Port_CsHigh();
    return v;
}

/* 等器件内部自定时操作结束(BUSY 清零)。
 * 主工程只在下电抢救时可能撞上一次正在进行的擦除, 等它落地。 */
static bool wait_ready(uint32_t guard_max)
{
    while ((rdsr() & SR_BUSY) != 0U) {
        if (--guard_max == 0U) return false;
    }
    return true;
}

/* 读 JEDEC ID(0x9F, 3 字节) */
static bool read_jedec(uint8_t out[3])
{
    SST25_Port_CsLow();
    (void)SST25_Port_Xfer(OP_JEDEC_ID);
    out[0] = SST25_Port_Xfer(0xFFU);
    out[1] = SST25_Port_Xfer(0xFFU);
    out[2] = SST25_Port_Xfer(0xFFU);
    SST25_Port_WaitBusy();
    SST25_Port_CsHigh();
    return true;
}

/* ================================================================
 * 对外 API
 * ================================================================ */

bool SST25_ReadJedecId(uint8_t out[3])
{
    if (out == NULL) return false;
    read_jedec(out);
    /* 全 0x00 / 全 0xFF 都当读不到: 前者是 MISO 被拉低, 后者是 MISO 悬空 */
    if ((out[0] == 0x00U && out[1] == 0x00U && out[2] == 0x00U) ||
        (out[0] == 0xFFU && out[1] == 0xFFU && out[2] == 0xFFU)) {
        return false;
    }
    return true;
}

uint8_t SST25_ReadStatus(void)
{
    return rdsr();
}

Sst25Status_t SST25_Init(void)
{
    uint8_t a[3] = {0};
    uint8_t b[3] = {0};
    bool ok;

    SST25_Port_Init();

    /* ---- 抢救 1: 无条件补一次 WRDI ----
     * 万一上次掉电正好打断了 AAI 序列, 器件会停在 AAI 模式(状态 bit6=1),
     * 此时除 RDSR/WRDI 之外**所有命令都被忽略, 连读都不行**, 现象就是
     * "芯片突然死了"。这一条 WRDI 成本几乎为零, 却能救回这种状态。
     * 主工程虽然不写, 但照样会被这个状态挡住读取, 所以这步不能省。 */
    send_cmd(OP_WRDI);

    /* ---- 抢救 2: 等上一次自定时操作结束 ---- */
    if (!wait_ready(GUARD_POLL)) {
        g_sst25_info.id_ok = false;
        g_sst25_info.family = SST25_FAMILY_UNKNOWN;
        return SST25_ERR_TIMEOUT;
    }

    /* ---- 读 ID, 连读两次要求一致(bootloader 跳转瞬间 SPI2 可能停在半途) ---- */
    read_jedec(a);
    read_jedec(b);
    ok = SST25_ReadJedecId(a) && (a[0] == b[0]) && (a[1] == b[1]) && (a[2] == b[2]);

    memcpy(g_sst25_info.jedec, a, sizeof(a));
    g_sst25_info.family = (a[0] == 0xBFU) ? SST25_FAMILY_SST
                        : (a[0] == 0xEFU) ? SST25_FAMILY_WINBOND
                                          : SST25_FAMILY_UNKNOWN;
    g_sst25_info.id_ok  = ok;
    g_sst25_info.status = rdsr();

    /* 块保护位(BP3..0/BPL)在只读路径上无需处理 —— 保护只拦擦写, 不拦读。
     * 值仍记在 g_sst25_info.status 里, 供上报/排查。 */

    if (!ok) return SST25_ERR_ID;
    return SST25_OK;
}

Sst25Status_t SST25_Read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    if ((dst == NULL) || (len == 0U)) return SST25_ERR_PARAM;
    if ((addr + len) > SST25_DEV_SIZE) return SST25_ERR_PARAM;

    SST25_Port_CsLow();
    (void)SST25_Port_Xfer(OP_READ);
    (void)SST25_Port_Xfer((uint8_t)(addr >> 16));
    (void)SST25_Port_Xfer((uint8_t)(addr >> 8));
    (void)SST25_Port_Xfer((uint8_t)(addr));

    /* 全程 CS 不能翻: 中途拉高再拉低等于重新发一次读命令, 数据整体错位 */
    for (uint32_t i = 0U; i < len; i++) {
        dst[i] = SST25_Port_Xfer(0xFFU);
    }
    SST25_Port_WaitBusy();
    SST25_Port_CsHigh();
    return SST25_OK;
}

const char *SST25_StatusStr(Sst25Status_t st)
{
    switch (st) {
    case SST25_OK:          return "OK";
    case SST25_ERR_PARAM:   return "BAD PARAM";
    case SST25_ERR_ID:      return "NO FLASH ID";
    case SST25_ERR_TIMEOUT: return "FLASH TIMEOUT";
    default:                return "?";
    }
}
