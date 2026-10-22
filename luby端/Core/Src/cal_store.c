/**
 * @file    cal_store.c
 * @brief   存储层实现 — 24 字节记录 / CRC32 / A-B 双槽**读取择优**
 * @see     cal_store.h 顶部的布局与判定规则
 *
 * 本工程只读: 组装记录(rec_pack)、写槽(CalStore_Save)和擦除前的扇区分类
 * 守卫(Cal_ClassifySector)都属于现场标定动作, 在 CalibTool 工程里。
 */
#include "cal_store.h"

/* 最近一次 Load 的状态, 供遥测上报 */
static Cal_Status_t s_last_status = CAL_NOT_FOUND;

/* ================================================================
 * CRC-32/ISO-HDLC (反射, poly 0xEDB88320)
 * 逐位实现, 不用查表 —— 只算 20 字节, 省下 1KB 表空间
 * ================================================================ */
uint32_t Cal_Crc32(const uint8_t *data, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;

    if (data == NULL) return 0U;

    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint32_t)data[i];
        for (uint8_t b = 0U; b < 8U; b++) {
            if (crc & 1U) crc = (crc >> 1) ^ 0xEDB88320UL;
            else          crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFFUL;
}

bool Cal_Crc32SelfTest(void)
{
    /* "123456789" 的 CRC-32/ISO-HDLC 标准校验值 */
    static const uint8_t probe[9] = {'1','2','3','4','5','6','7','8','9'};
    return Cal_Crc32(probe, 9U) == 0xCBF43926UL;
}

/* ================================================================
 * 记录的解析
 * ================================================================
 * 字段必须**逐字段显式取值**, 绝不 memcpy 到结构体:
 * 标定工程与主工程是两个分别编译的固件, 结构体一旦加字段或改对齐,
 * memcpy 会静默错位, 而 CRC 还照样通过 —— 那种 bug 极难查。
 */
static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

/**
 * @brief 解析并校验一条记录
 * @param seq_out 可选, 传出记录里的序号
 * @return CAL_OK 表示这条记录完全可用
 */
static Cal_Status_t rec_parse(const uint8_t *rec, GrayBaseline_t *out, uint32_t *seq_out)
{
    uint16_t fe, re_, g0, g1;
    uint32_t crc_stored, crc_calc;

    if (get_u32(&rec[0x00]) != CAL_REC_MAGIC)  return CAL_NOT_FOUND;
    if (get_u16(&rec[0x04]) != CAL_REC_VERSION) return CAL_VERSION_MISMATCH;
    if (get_u16(&rec[0x06]) != CAL_PAYLOAD_LEN) return CAL_CORRUPT;

    crc_stored = get_u32(&rec[0x14]);
    crc_calc   = Cal_Crc32(rec, 0x14U);
    if (crc_stored != crc_calc) return CAL_CORRUPT;

    fe = get_u16(&rec[0x0C]);
    re_ = get_u16(&rec[0x0E]);
    g0 = get_u16(&rec[0x10]);
    g1 = get_u16(&rec[0x12]);

    /* ---- 有效性第 5 条: 数值合理性 ----
     * 拦的是"CRC 正确但数值退化"。ADC 是 12 位, 所以合理范围 [1,4094]。
     * 数值退化会让 on_stage 变成 0 或 4095 → 台上判定恒真/恒假 → 车永远
     * 以为自己站在台上, 直接冲下擂台。这比读不到记录危险得多。 */
    if (fe < 1U || fe > 4094U) return CAL_IMPLAUSIBLE;
    if (re_ < 1U || re_ > 4094U) return CAL_IMPLAUSIBLE;
    if (g0 < 1U || g0 > 4094U) return CAL_IMPLAUSIBLE;
    if (g1 < 1U || g1 > 4094U) return CAL_IMPLAUSIBLE;

    /* 按实际公式(0.995 / 0.9)重算一遍, 确认派生阈值也不会退化 */
    {
        uint32_t on_f = (uint32_t)((float)g0 * 0.995f);
        uint32_t on_r = (uint32_t)((float)g1 * 0.995f);
        uint32_t ed_f = (uint32_t)((float)fe * 0.9f);
        uint32_t ed_r = (uint32_t)((float)re_ * 0.9f);
        if (on_f < 1U || on_f > 4094U) return CAL_IMPLAUSIBLE;
        if (on_r < 1U || on_r > 4094U) return CAL_IMPLAUSIBLE;
        if (ed_f < 1U || ed_f > 4094U) return CAL_IMPLAUSIBLE;
        if (ed_r < 1U || ed_r > 4094U) return CAL_IMPLAUSIBLE;
    }

    if (out != NULL) {
        out->front_edge     = fe;
        out->rear_edge      = re_;
        out->ground[GRAY_FRONT] = g0;
        out->ground[GRAY_REAR]  = g1;
    }
    if (seq_out != NULL) *seq_out = get_u32(&rec[0x08]);
    return CAL_OK;
}

/* 返回码的"具体程度"排序 —— 两个槽都坏时, 上报最具体的那个原因 */
static uint32_t cal_status_rank(Cal_Status_t s)
{
    switch (s) {
    case CAL_IO_ERROR:         return 5U;
    case CAL_IMPLAUSIBLE:      return 4U;
    case CAL_VERSION_MISMATCH: return 3U;
    case CAL_CORRUPT:          return 2U;
    case CAL_NOT_FOUND:        return 1U;
    default:                   return 0U;
    }
}

/* ================================================================
 * Load
 * ================================================================ */
Cal_Status_t CalStore_Load(GrayBaseline_t *out)
{
    uint8_t rec[CAL_REC_SIZE];
    GrayBaseline_t bl_a, bl_b;
    uint32_t seq_a = 0U, seq_b = 0U;
    Cal_Status_t sa, sb;

    if (out == NULL) return CAL_IO_ERROR;

    if (SST25_Read(CAL_SLOT_A_ADDR, rec, CAL_REC_SIZE) != SST25_OK) {
        s_last_status = CAL_IO_ERROR;
        return CAL_IO_ERROR;
    }
    sa = rec_parse(rec, &bl_a, &seq_a);

    if (SST25_Read(CAL_SLOT_B_ADDR, rec, CAL_REC_SIZE) != SST25_OK) {
        s_last_status = CAL_IO_ERROR;
        return CAL_IO_ERROR;
    }
    sb = rec_parse(rec, &bl_b, &seq_b);

    if (sa == CAL_OK && sb == CAL_OK) {
        /* 两个都有效 → 取 seq 大的。用有符号差比较, 天然处理 seq 回绕 */
        *out = ((int32_t)(seq_a - seq_b) >= 0) ? bl_a : bl_b;
    } else if (sa == CAL_OK) {
        *out = bl_a;
    } else if (sb == CAL_OK) {
        *out = bl_b;
    } else {
        /* 两个槽都不可用: 上报**最具体**的那个原因, 而不是一律压成 CORRUPT。
         * "数值退化"和"固件版本不匹配"的处置完全不同, 混报成 CORRUPT 会让
         * 现场去查根本没坏的硬件。 */
        s_last_status = (cal_status_rank(sa) >= cal_status_rank(sb)) ? sa : sb;
        return s_last_status;
    }

    s_last_status = CAL_OK;
    return CAL_OK;
}

Cal_Status_t Cal_LastStatus(void) { return s_last_status; }

const char *Cal_StatusStr(Cal_Status_t st)
{
    switch (st) {
    case CAL_OK:               return "OK";
    case CAL_NOT_FOUND:        return "NO CAL RECORD";
    case CAL_CORRUPT:          return "CAL CORRUPT";
    case CAL_VERSION_MISMATCH: return "CAL BAD VER";
    case CAL_IMPLAUSIBLE:      return "CAL BAD VALUE";
    case CAL_IO_ERROR:         return "FLASH ERROR";
    default:                   return "?";
    }
}
