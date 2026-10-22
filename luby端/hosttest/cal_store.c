/**
 * @file    cal_store.c
 * @brief   存储层实现 — 24 字节记录 / CRC32 / A-B 双槽 / 扇区分类守卫
 * @see     cal_store.h 顶部的布局与判定规则
 */
#include "cal_store.h"
#include <string.h>

/* 最近一次 Load/Save 的状态, 供遥测上报 */
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
 * 记录的组装 / 解析
 * ================================================================
 * payload 必须**逐字段显式序列化**, 绝不 memcpy 整个结构体:
 * 标定工程与主工程是两个分别编译的固件, 结构体一旦加字段或改对齐,
 * memcpy 会静默错位, 而 CRC 还照样通过 —— 那种 bug 极难查。
 */
static void put_u16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void put_u32(uint8_t *p, uint32_t v)
{ p[0]=(uint8_t)v; p[1]=(uint8_t)(v>>8); p[2]=(uint8_t)(v>>16); p[3]=(uint8_t)(v>>24); }

static uint16_t get_u16(const uint8_t *p) { return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }
static uint32_t get_u32(const uint8_t *p)
{ return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24); }

static void rec_pack(uint8_t out[CAL_REC_SIZE], const GrayBaseline_t *bl, uint32_t seq)
{
    memset(out, 0, CAL_REC_SIZE);
    put_u32(&out[0x00], CAL_REC_MAGIC);
    put_u16(&out[0x04], CAL_REC_VERSION);
    put_u16(&out[0x06], CAL_PAYLOAD_LEN);
    put_u32(&out[0x08], seq);
    put_u16(&out[0x0C], bl->front_edge);
    put_u16(&out[0x0E], bl->rear_edge);
    put_u16(&out[0x10], bl->ground[GRAY_FRONT]);
    put_u16(&out[0x12], bl->ground[GRAY_REAR]);
    put_u32(&out[0x14], Cal_Crc32(out, 0x14U));   /* 覆盖 [0x00..0x13] 共 20 字节 */
}

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

    /* 按实际公式(0.995 / 0.88)重算一遍, 确认派生阈值也不会退化 */
    {
        uint32_t on_f = (uint32_t)((float)g0 * 0.995f);
        uint32_t on_r = (uint32_t)((float)g1 * 0.995f);
        uint32_t ed_f = (uint32_t)((float)fe * 0.88f);
        uint32_t ed_r = (uint32_t)((float)re_ * 0.88f);
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

/* ================================================================
 * Save
 * ================================================================ */
Cal_Status_t CalStore_Save(const GrayBaseline_t *in)
{
    uint8_t rec[CAL_REC_SIZE];
    uint8_t probe[CAL_REC_SIZE];
    GrayBaseline_t dummy;
    uint32_t seq_a = 0U, seq_b = 0U;
    Cal_Status_t sa, sb;
    uint32_t target, max_seq;
    bool     have_valid = false;
    Sst25Status_t half;

    if (in == NULL) { s_last_status = CAL_CORRUPT; return CAL_CORRUPT; }

    /* ---- 0. 先过一遍**读取端那道合理性门** ----
     * 标定的四个数来自 ADC: 通道悬空 / 传感器没插会读到 0 或 4095。
     * 这种记录写进去之后读取端一定拒收, 可保存却会报 CAL_OK ——
     * 人会以为标定成功了, 车上却一直在用兜底值。
     * 所以在擦除之前就用同一个 rec_parse 验一遍, 不过就直接返回。 */
    {
        uint8_t chk_buf[CAL_REC_SIZE];
        Cal_Status_t chk;
        rec_pack(chk_buf, in, 1U);          /* seq 用什么都行, 只验字段合法性 */
        chk = rec_parse(chk_buf, NULL, NULL);
        if (chk != CAL_OK) { s_last_status = chk; return chk; }
    }

    /* ---- 1. 先读一次: 拿到 max_seq 与当前主槽。读失败就一个字都不写 ---- */
    if (SST25_Read(CAL_SLOT_A_ADDR, rec, CAL_REC_SIZE) != SST25_OK) {
        s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR;
    }
    sa = rec_parse(rec, &dummy, &seq_a);
    if (SST25_Read(CAL_SLOT_B_ADDR, rec, CAL_REC_SIZE) != SST25_OK) {
        s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR;
    }
    sb = rec_parse(rec, &dummy, &seq_b);

    if (sa == CAL_OK && sb == CAL_OK) {
        have_valid = true;
        max_seq = ((int32_t)(seq_a - seq_b) >= 0) ? seq_a : seq_b;
        target  = ((int32_t)(seq_a - seq_b) >= 0) ? CAL_SLOT_B_ADDR : CAL_SLOT_A_ADDR;
    } else if (sa == CAL_OK) {
        have_valid = true; max_seq = seq_a; target = CAL_SLOT_B_ADDR;
    } else if (sb == CAL_OK) {
        have_valid = true; max_seq = seq_b; target = CAL_SLOT_A_ADDR;
    } else {
        /* 两槽都空: 写 A, 从 seq = 1 起(0 与 0xFFFFFFFF 保留不用) */
        max_seq = 0U; target = CAL_SLOT_A_ADDR;
    }

    /* ---- 2. 地址硬地板 + 扇区分类守卫(擦除前) ----
     * 两层独立防线:
     *   a. CAL_SAFE_FLOOR: 槽地址宏被误改到低地址(如 0x08000000 一带)时,
     *      在碰任何擦除命令之前兜底拒绝 —— 那片是固件自己, 擦了直接变砖;
     *   b. 分类守卫: 只允许动"整片擦除"或"我们自己的记录"两种扇区。
     * 文件系统与固件镜像硬拒绝 —— FAT 的 slack 空间/未分配簇天生是成片
     * 0xFF, 只判目标 24 字节会让首次保存静默写进 bootloader 的数据里,
     * 而且回读还报"成功"。 */
    if (target < CAL_SAFE_FLOOR) {
        s_last_status = CAL_CORRUPT;
        return CAL_CORRUPT;
    }
    {
        SecClass_t cls = Cal_ClassifySector(target);   /* 只调一次: 内部要读满 4KB */
        if (cls != SEC_ERASED && cls != SEC_OURS) {
            s_last_status = CAL_CORRUPT;
            return CAL_CORRUPT;
        }
    }

    /* ---- 3. 擦除并回读确认 ----
     * 此刻起, 另一槽(若有有效记录)一动不动 —— 任意时刻掉电都还有一份好的 */
    half = SST25_Erase4K(target);
    if (half != SST25_OK) { s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR; }
    if (!SST25_IsErased(target, CAL_REC_SIZE)) { s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR; }

    /* ---- 4. 写记录 ---- */
    rec_pack(rec, in, have_valid ? (max_seq + 1U) : 1U);
    half = SST25_ProgramBytes(target, rec, CAL_REC_SIZE);
    if (half != SST25_OK) { s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR; }

    /* ---- 5. 回读校验 ---- */
    if (SST25_Read(target, probe, CAL_REC_SIZE) != SST25_OK) {
        s_last_status = CAL_IO_ERROR; return CAL_IO_ERROR;
    }
    if (memcmp(probe, rec, CAL_REC_SIZE) != 0) {
        s_last_status = CAL_CORRUPT; return CAL_CORRUPT;
    }

    /* 写完**不擦旧槽**: 它 seq 更小天然被忽略, 留着多一层保险 */
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

/* ================================================================
 * 扇区内容分类
 *
 * 用 512 字节的静态缓冲流式读, 不把整个 4KB 拉进 RAM ——
 * 主工程光 FreeRTOS 就占了 24KB RAM, 4KB 栈上/静态缓冲太奢侈。
 * ================================================================ */
SecClass_t Cal_ClassifySector(uint32_t sector_addr)
{
    static uint8_t buf[512];      /* 必须是 static: 放栈上会撞任务栈 */
    uint32_t base = sector_addr & ~(SST25_SECTOR_SIZE - 1U);
    uint32_t non_ff_total = 0U;
    uint32_t max_non_ff_off = 0U;
    uint32_t non_ff_head = 0U;
    int      run = 0, max_run = 0;
    uint8_t  rec[CAL_REC_SIZE];

    for (uint32_t blk = 0U; blk < (SST25_SECTOR_SIZE / sizeof(buf)); blk++) {
        uint32_t off = blk * (uint32_t)sizeof(buf);
        if (SST25_Read(base + off, buf, sizeof(buf)) != SST25_OK) {
            return SEC_FOREIGN_UNKNOWN;
        }
        for (uint32_t i = 0U; i < sizeof(buf); i++) {
            if (buf[i] != 0xFFU) {
                non_ff_total++;
                if ((off + i) > max_non_ff_off) max_non_ff_off = off + i;
                if ((off + i) < 512U) non_ff_head++;
            }
            /* 连续可打印 ASCII 检测 */
            if (buf[i] >= 0x20U && buf[i] <= 0x7EU) { run++; if (run > max_run) max_run = run; }
            else run = 0;
        }
    }

    /* 1. 整片全 0xFF */
    if (non_ff_total == 0U) return SEC_ERASED;

    /* 2. 完整记录: 魔数精确匹配 */
    if (SST25_Read(base, rec, CAL_REC_SIZE) == SST25_OK) {
        if (get_u32(&rec[0x00]) == CAL_REC_MAGIC) return SEC_OURS;
    }

    /* 3/4. 文件系统与固件镜像 —— 必须放在"撕毁的记录"判定**之前**。
       否则一个只有十几字节的外来数据(某个稀疏镜像头、某个小文件)会先被
       "非 0xFF 字节都落在记录区内"吃掉, 判成 OURS。 */
    if (SST25_Read(base, buf, 512U) == SST25_OK) {
        if (buf[510] == 0x55U && buf[511] == 0xAAU) return SEC_FOREIGN_FS;   /* MBR/FAT 引导签名 */
        if (buf[0] == 0xEBU && buf[2] == 0x90U)     return SEC_FOREIGN_FS;   /* x86 跳转指令 */
        if (memcmp(&buf[3],  "NTFS",  4U) == 0)     return SEC_FOREIGN_FS;
        if (memcmp(&buf[3],  "EXFAT", 5U) == 0)     return SEC_FOREIGN_FS;
        if (memcmp(&buf[54], "FAT12", 5U) == 0)     return SEC_FOREIGN_FS;
        if (memcmp(&buf[54], "FAT16", 5U) == 0)     return SEC_FOREIGN_FS;
        if (memcmp(&buf[82], "FAT32", 5U) == 0)     return SEC_FOREIGN_FS;

        /* 4. STM32 固件镜像: 开局是"栈顶指针 + 复位向量" */
        {
            uint32_t sp = get_u32(&buf[0]);
            uint32_t pc = get_u32(&buf[4]) & ~1UL;
            if (sp >= 0x20000000UL && sp <= 0x2000C000UL &&
                pc >= 0x08000000UL && pc <= 0x08040000UL) {
                return SEC_FOREIGN_IMAGE;
            }
        }
    }

    /* 5. 撕毁的记录: 所有非 0xFF 字节都落在记录区 [0x00,0x18) 内。
     *
     * 这一条**必须宽松**, 不能要求"看起来像我们记录的前缀"。
     * 因为除了"写入写一半", 还有一种更隐蔽的撕毁方式: **擦除擦一半**。
     * SECTOR_ERASE 被打断时, 记录那几个字节会被擦成**任意位翻转的值**
     * (例如 0x43 变成 0x63), 完全不符合任何前缀形态。若此时判成 FOREIGN,
     * 那一槽就**再也擦不掉** —— 而 CalStore_Save 每次都写"非最新"的那一槽,
     * 于是以后每一次保存都会失败, 标定功能永久死锁, 车上没有任何恢复手段
     * (0x60 片擦被设计禁用了, 主工程只读, FlashTest 也只动 ERASED 的扇区)。
     *
     * 放宽它带来的误判风险由第 3/4 步挡在前面: 真正的文件系统扇区和固件
     * 镜像在那两步就已被硬拒, 走不到这里。剩下能被误判的只有"整个 4KB 里
     * 只有前 24 字节有数据、且既不像文件系统也不像镜像"的扇区, 极其罕见,
     * 且远轻于"标定永久死锁"。 */
    if (max_non_ff_off < CAL_REC_SIZE) return SEC_OURS;

    /* 6. 其余: 有数据但认不出来 */
    (void)non_ff_head;
    (void)max_run;
    (void)rec;
    return SEC_FOREIGN_UNKNOWN;
}

const char *Cal_SecClassStr(SecClass_t c)
{
    switch (c) {
    case SEC_ERASED:          return "ERASED";
    case SEC_OURS:            return "OURS";
    case SEC_FOREIGN_FS:      return "FILESYSTEM";
    case SEC_FOREIGN_IMAGE:   return "FW IMAGE";
    case SEC_FOREIGN_UNKNOWN: return "UNKNOWN";
    default:                  return "?";
    }
}
