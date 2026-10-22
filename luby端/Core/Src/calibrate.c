/**
 * @file    calibrate.c
 * @brief   灰度阈值计算(新车: 前后 2 路灰度) — 由一组基准值算出 g_thresh
 *
 * 主工程只做一件事: 把基准值换算成判定阈值。
 *   ① 台上/台下判定 on_stage[i]     = 台下地面值 × 0.995
 *   ② 台上边缘区判定 edge_thresh[i] = 前/后边缘校准值 × 0.9
 *
 * 基准值从哪来: **外部 SPI flash** —— CalibTool 工程现场标定后写进去的,
 * 上电由 Gray_LoadFromFlash() 读回。
 *
 * ⚠ LCD 引导的三步现场采集(以及它需要的 ADC 采样、LCD 提示)已经在
 *   2026-09-17 **整体搬到了 CalibTool 工程**, 主工程里不再有那段代码。
 *   所以本文件没有采集、不碰 LCD、不碰 ADC, 是纯计算 + 读 flash。
 */
#include "calibrate.h"
#include "cal_store.h"
#include <string.h>

/* 内部静态变量: 保存当前基准值, 仅本文件可访问 */
static GrayBaseline_t s_current_baseline;

/**
 * 编译期兜底基准值 —— flash 读不到时用它。
 *
 * ⚠ 必须是**真实场地上用 CalibTool 实测出来**的一组值, 不是占位符。
 *   这是"flash 读失败"与"车带着编造的阈值冲下擂台"之间唯一的东西。
 *   (四项若全为 0, Gray_LoadFromFlash 会认出"还没填", 那时它宁可保持
 *    sensors.c 的编译期默认值, 也不拿全 0 去过一遍台上判定。)
 */
const GrayBaseline_t g_gray_fallback = {
    .front_edge = 1700,
    .rear_edge  = 3400,
    .ground     = {2900, 3600},
};

/**
 * @brief  由基准值计算阈值并写入全局 g_thresh
 * @param  baseline  基准值指针(只读)
 */
static void calc_thresholds(const GrayBaseline_t *baseline)
{
    /* 台上/台下判定阈值: 台下地面值 × 0.995 */
    g_thresh.on_stage[GRAY_FRONT] = (uint16_t)(baseline->ground[GRAY_FRONT] * 0.995);
    g_thresh.on_stage[GRAY_REAR]  = (uint16_t)(baseline->ground[GRAY_REAR] * 0.995);

    /* 边缘区判定阈值: 边缘校准值 × 0.9 (与 cal_store.c 里的合理性复算同系数) */
    g_thresh.edge_thresh[GRAY_FRONT] = (uint16_t)(baseline->front_edge * 0.9f);
    g_thresh.edge_thresh[GRAY_REAR]  = (uint16_t)(baseline->rear_edge  * 0.9f);
}

/**
 * @brief  统一入口: 由一组基准值算出灰度阈值
 * @param  baseline  基准值指针, **主工程只传非空**
 *
 * 传 NULL 在旧版里意味着"启动 LCD 引导的三步自动采集"。那段采集(连带它
 * 需要的 ADC 采样与 LCD 提示)已经整体搬到 CalibTool 工程, 主工程里不再
 * 包含, 所以这里对 NULL 直接返回 —— 保持阈值现状, 什么都不做。
 * 主工程唯一的调用者是 Gray_LoadFromFlash(), 只会传有效指针。
 */
void Gray_LoadThresholds(const GrayBaseline_t *baseline)
{
    if (baseline == NULL) return;   /* 采集模式属于 CalibTool, 见文件头说明 */

    memcpy(&s_current_baseline, baseline, sizeof(GrayBaseline_t));
    calc_thresholds(&s_current_baseline);
}

/* ================================================================
 * 外部 SPI flash 相关(2026-09-17 新增)
 * ================================================================ */

const GrayBaseline_t *Gray_GetLastBaseline(void)
{
    return &s_current_baseline;
}

bool Gray_LoadFromFlash(void)
{
    GrayBaseline_t bl;

    if (CalStore_Load(&bl) == CAL_OK) {
        Gray_LoadThresholds(&bl);      /* 用 flash 里的实测值 */
        return true;
    }

    /* ---- 读不到 / 读坏了 ----
     * 绝不传 NULL: NULL 在这个 API 里意味着"阻塞 18 秒、引导人工压边缘",
     * 正式比赛固件在起跑线上卡 18 秒等人操作是不可接受的。
     * 故障状态由 Cal_LastStatus() 提供给遥测上报。 */

    /* 兜底值还没填(四项全 0)时**不要拿它覆盖 g_thresh**, 保持 sensors.c
       里那组编译期默认值(3400/3100)。
       为什么: 台上判定是 `gray < on_stage`, 阈值给 0 会让它**恒为假** ——
       车会以为自己永远在台下, 无限跑爬台状态机。那比"先用一组别的场地的
       阈值"糟糕得多, 后者至少量级是对的。 */
    if (g_gray_fallback.front_edge == 0U && g_gray_fallback.rear_edge == 0U &&
        g_gray_fallback.ground[GRAY_FRONT] == 0U && g_gray_fallback.ground[GRAY_REAR] == 0U) {
        return false;                  /* 保持 sensors.c 的编译期默认阈值 */
    }

    Gray_LoadThresholds(&g_gray_fallback);
    return false;
}
