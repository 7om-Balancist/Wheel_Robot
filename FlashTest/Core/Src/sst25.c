/**
 * @file    sst25.c
 * @brief   SST25 协议层实现 — 操作码 / 擦写读 / 状态轮询
 * @see     sst25.h 顶部的指令集与状态位表
 */
#include "sst25.h"
#include <string.h>

/* ===== 操作码 ===== */
#define OP_WREN          0x06U
#define OP_WRDI          0x04U
#define OP_RDSR          0x05U
#define OP_WRSR          0x01U
#define OP_EWSR          0x50U
#define OP_READ          0x03U
#define OP_JEDEC_ID      0x9FU
#define OP_RDID          0x90U
#define OP_BYTE_PROGRAM  0x02U
#define OP_SECTOR_ERASE  0x20U

/* ===== 状态寄存器位 ===== */
#define SR_BUSY          0x01U
#define SR_WEL           0x02U
#define SR_BP_MASK       0x3CU   /* BP0..BP3 = bit2..bit5 */
#define SR_BPL           0x80U   /* bit7                    */
#define SR_PROTECT_MASK  (SR_BP_MASK | SR_BPL)

/* ===== 轮询 guard =====
 * 刻意不用 HAL_GetTick 做超时: tick 依赖 TIM2, 而本驱动有可能在
 * HAL_Init() 之前被调用(那时 tick 永远不走, 用 tick 会死等)。
 * 改用一个足够大的迭代计数, 既能兜住最坏耗时, 又永远不可能挂死。
 *   一次 RDSR 轮询 ≈ 5µs(4.5MHz 下 2 字节 + GPIO 开销)
 *   4KB 擦除手册最坏 200ms → 约 40000 次 → guard 给 40 万, 10x 余量
 */
#define GUARD_ERASE      400000U
#define GUARD_PROGRAM     40000U

Sst25Info_t g_sst25_info;

/* ================================================================
 * 内部基础操作
 * ================================================================ */

/* 发一条无参数命令(WREN/WRDI 这种) */
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

/**
 * @brief 写使能。**每一次 Program/Erase 之前都必须重发**
 * @note  回读 WEL 是必须的, 但它**不是充分条件** —— 块保护(BP 位)只阻断
 *        Program/Erase, 不阻断 WREN, 所以 WREN 照样能把 WEL 置 1, 后续擦写
 *        被静默丢弃。唯一能发现这种情况的手段是写完之后逐字节回读比对。
 */
static bool wren(void)
{
    send_cmd(OP_WREN);
    return (rdsr() & SR_WEL) != 0U;
}

/* 等器件内部自定时操作结束(BUSY 清零) */
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
     * 此时除 RDSR/WRDI 之外**所有命令都被忽略**, 现象就是"芯片突然死了"。
     * 这一条 WRDI 成本几乎为零, 却能救回这种状态。 */
    send_cmd(OP_WRDI);

    /* ---- 抢救 2: 等上一次自定时操作结束 ---- */
    if (!wait_ready(GUARD_ERASE)) {
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

    if (!ok) return SST25_ERR_ID;

    /* ---- 若被块保护, 尝试解保护 ----
     * SST25 上电时 BP3..0 可能非零, 而保护表里 001 恰好映射到
     * 1F0000H..1FFFFFH(上 1/32) —— 正是本方案用的两个记录槽。
     * 不解保护的话 WREN 会成功但擦写静默失效。 */
    if ((g_sst25_info.status & SR_PROTECT_MASK) != 0U) {
        (void)SST25_Unprotect();
        g_sst25_info.status = rdsr();
    }
    return SST25_OK;
}

Sst25Status_t SST25_Unprotect(void)
{
    /* SST 族要先用 EWSR 打开状态寄存器写使能; Winbond 族不需要 */
    if (g_sst25_info.family == SST25_FAMILY_SST) {
        send_cmd(OP_EWSR);
    }
    send_cmd(OP_WREN);
    {
        SST25_Port_CsLow();
        (void)SST25_Port_Xfer(OP_WRSR);
        (void)SST25_Port_Xfer(0x00U);     /* BP3..0=0, BPL=0 */
        SST25_Port_WaitBusy();
        SST25_Port_CsHigh();
    }
    if (!wait_ready(GUARD_ERASE)) return SST25_ERR_TIMEOUT;

    /* 回读确认 */
    if ((rdsr() & SR_PROTECT_MASK) != 0U) {
        return SST25_ERR_PROTECTED;       /* BPL=1 且 WP# 被拉低时无法解除 */
    }
    return SST25_OK;
}

bool SST25_IsErased(uint32_t addr, uint32_t len)
{
    uint8_t buf[64];
    if (addr + len > SST25_DEV_SIZE) return false;

    while (len > 0U) {
        uint32_t chunk = (len > sizeof(buf)) ? (uint32_t)sizeof(buf) : len;
        if (SST25_Read(addr, buf, chunk) != SST25_OK) return false;
        for (uint32_t i = 0U; i < chunk; i++) {
            if (buf[i] != 0xFFU) return false;
        }
        addr += chunk;
        len  -= chunk;
    }
    return true;
}

Sst25Status_t SST25_Erase4K(uint32_t addr)
{
    /* 向下对齐: 0x20 命令的 A11..A0 是 don't care, 传未对齐地址不会报错,
     * 而是**静默擦掉它所在的那个 4KB 扇区** —— 必须先自己对齐再检查窗口。 */
    addr &= ~(SST25_SECTOR_SIZE - 1U);

    if (!SST25_Port_AddrWritable(addr)) return SST25_ERR_WINDOW;
    if (!wren()) return SST25_ERR_WEL;

    SST25_Port_CsLow();
    (void)SST25_Port_Xfer(OP_SECTOR_ERASE);
    (void)SST25_Port_Xfer((uint8_t)(addr >> 16));
    (void)SST25_Port_Xfer((uint8_t)(addr >> 8));
    (void)SST25_Port_Xfer((uint8_t)(addr));
    SST25_Port_WaitBusy();
    SST25_Port_CsHigh();

    if (!wait_ready(GUARD_ERASE)) return SST25_ERR_TIMEOUT;
    return SST25_OK;
}

Sst25Status_t SST25_Read(uint32_t addr, uint8_t *dst, uint32_t len)
{
    if ((dst == NULL) || (len == 0U)) return SST25_ERR_PARAM;
    if ((addr + len) > SST25_DEV_SIZE) return SST25_ERR_PARAM;

    /* 读不设窗口限制 —— 读是只读操作, 没有破坏性, 工具工程要能读全片 */
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

Sst25Status_t SST25_ProgramBytes(uint32_t addr, const uint8_t *src, uint32_t len)
{
    uint8_t rb[64];

    if ((src == NULL) || (len == 0U)) return SST25_ERR_PARAM;
    if ((addr + len) > SST25_DEV_SIZE) return SST25_ERR_WINDOW;
    if (!SST25_Port_AddrWritable(addr)) return SST25_ERR_WINDOW;

    /* 编程只能把 1 变成 0。目标区没擦干净就写, 结果是两次数据按位与的垃圾。
     * 这里**拒绝写入而不是替调用者擦除** —— 擅自擦除会连带毁掉同扇区的其他数据。 */
    if (!SST25_IsErased(addr, len)) return SST25_ERR_NOT_ERASED;

    for (uint32_t i = 0U; i < len; i++) {
        uint32_t a = addr + i;

        if (!wren()) return SST25_ERR_WEL;      /* 每一字节都要重发 WREN */

        SST25_Port_CsLow();
        (void)SST25_Port_Xfer(OP_BYTE_PROGRAM);
        (void)SST25_Port_Xfer((uint8_t)(a >> 16));
        (void)SST25_Port_Xfer((uint8_t)(a >> 8));
        (void)SST25_Port_Xfer((uint8_t)(a));
        (void)SST25_Port_Xfer(src[i]);
        SST25_Port_WaitBusy();
        SST25_Port_CsHigh();

        if (!wait_ready(GUARD_PROGRAM)) return SST25_ERR_TIMEOUT;   /* 单字节典型 7µs */
    }

    /* ---- 写后回读比对 ----
     * 这是唯一能发现"块保护把擦写静默丢掉了"/"HOLD 虚焊"/"WP 异常"的证据。
     * WEL 回读和 BUSY 轮询都看不出来。 */
    while (len > 0U) {
        uint32_t chunk = (len > sizeof(rb)) ? (uint32_t)sizeof(rb) : len;
        if (SST25_Read(addr, rb, chunk) != SST25_OK) return SST25_ERR_VERIFY;
        if (memcmp(rb, src, chunk) != 0) return SST25_ERR_VERIFY;
        addr += chunk;
        src  += chunk;
        len  -= chunk;
    }
    return SST25_OK;
}

const char *SST25_StatusStr(Sst25Status_t st)
{
    switch (st) {
    case SST25_OK:              return "OK";
    case SST25_ERR_PARAM:       return "BAD PARAM";
    case SST25_ERR_ID:          return "NO FLASH ID";
    case SST25_ERR_WINDOW:      return "OUT OF WINDOW";
    case SST25_ERR_PROTECTED:   return "BLOCK PROTECTED";
    case SST25_ERR_WEL:         return "WREN FAILED";
    case SST25_ERR_NOT_ERASED:  return "NOT ERASED";
    case SST25_ERR_TIMEOUT:     return "FLASH TIMEOUT";
    case SST25_ERR_VERIFY:      return "VERIFY FAIL";
    default:                    return "?";
    }
}
