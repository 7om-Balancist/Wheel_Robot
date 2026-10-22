/**
 * @file    calibrate.c
 * @brief   灰度校准模块(新车: 前后 2 路灰度) — 三步自动采集 / 固化加载
 *
 * 模式切换: Gray_LoadThresholds(baseline)
 *   - baseline == NULL : LCD 引导的三步自动采集(新场地调车用)
 *   - baseline != NULL : 直接按预设基准计算阈值(正式比赛固化用)
 *
 * 新车灰度只承担两个角色(不再防掉台):
 *   ① 台上/台下判定 on_stage[i]   = 台下地面值×0.995
 *   ② 台上边缘区判定 edge_thresh[i] = 前/后边缘校准值×0.88
 */
#include "calibrate.h"
#include "luby_adc.h"
#include "luby_lcd.h"
#include "cal_store.h"
#include <string.h>
#include <stdio.h>

/* 内部静态变量: 保存当前基准值, 仅本文件可访问 */
static GrayBaseline_t s_current_baseline;

/**
 * 编译期兜底基准值 —— flash 读不到时用它。
 *
 * ⚠⚠ 必须填**真实场地上用本工程实测出来**的一组典型值, 不是占位符。
 *     每次重新标定后, 把 LCD 上显示的四个数抄到这里。
 *     真正会用到兜底值的是主工程 car7.15 的 calibrate.c(flash 读不到时才
 *     生效), 本工程不链接 sensors.c; 这份留着是为了和主工程保持一致。
 */
const GrayBaseline_t g_gray_fallback = {
    .front_edge = 2100,
    .rear_edge  = 3400,
    .ground     = {3200, 3700},
};

/**
 * @brief  由基准值计算阈值并写入全局 g_thresh
 * @param  baseline  基准值指针(只读)
 */
static void calc_thresholds(const GrayBaseline_t *baseline)
{
    /* 台上/台下判定阈值: 台下地面值 × 0.995 */
    g_thresh.on_stage[GRAY_FRONT] = (uint16_t)(baseline->ground[GRAY_FRONT] * 0.995f);
    g_thresh.on_stage[GRAY_REAR]  = (uint16_t)(baseline->ground[GRAY_REAR]  * 0.995f);

    /* 边缘区判定阈值: 边缘校准值 × 0.88 (与 cal_store.c 里的合理性复算同系数) */
    g_thresh.edge_thresh[GRAY_FRONT] = (uint16_t)(baseline->front_edge * 0.88f);
    g_thresh.edge_thresh[GRAY_REAR]  = (uint16_t)(baseline->rear_edge  * 0.88f);
}

/* ================================================================
 * 采集滤波: 组内中值 + 组间截尾均值
 * ================================================================
 * 旧版 10 点简单平均的短板: 单次 ADC 尖刺(电源纹波/静电/采样毛刺)会
 * 直接混进平均值, 把标定阈值整体带偏 —— 标定值是一次定终身, 运行端
 * 还有 decision.c 的连续确认兜底, 标定端没有任何兜底。
 *
 * 两级滤波:
 *   组内: 连采 5 次取中值 —— 单次尖刺被中值直接剔除;
 *   组间: 16 组中值(每组隔 10ms)排序后去掉首尾各 1 组再平均 ——
 *         抗持续性干扰(如附近设备启停引起的基线漂移)。
 * 车静止时均值无偏, 组数只影响总耗时(16×10ms=160ms, 流程里无感)。
 */
#define MED_OF       5   /* 组内采样次数: 取中值(奇数) */
#define MEAN_GROUPS  16  /* 组数 */
#define TRIM_EACH    1   /* 截尾: 首尾各去掉几组 */

static uint16_t sample_mean(uint8_t ch)
{
    uint16_t adc_raw[16];
    uint16_t grp[MEAN_GROUPS];
    uint16_t s[MED_OF];
    uint32_t acc = 0;

    for (int g = 0; g < MEAN_GROUPS; g++) {
        for (int i = 0; i < MED_OF; i++) {
            ADC_GetAllValues(adc_raw);
            s[i] = adc_raw[ch];
        }
        /* 插入排序取中值: MED_OF 是奇数, 中值必落在单一元素上 */
        for (int i = 1; i < MED_OF; i++) {
            uint16_t k = s[i];
            int j = i - 1;
            while (j >= 0 && s[j] > k) { s[j + 1] = s[j]; j--; }
            s[j + 1] = k;
        }
        grp[g] = s[MED_OF / 2];
        HAL_Delay(10);
    }

    /* 组间截尾均值: 排序后去掉首尾各 TRIM_EACH 组再平均 */
    for (int i = 1; i < MEAN_GROUPS; i++) {
        uint16_t k = grp[i];
        int j = i - 1;
        while (j >= 0 && grp[j] > k) { grp[j + 1] = grp[j]; j--; }
        grp[j + 1] = k;
    }
    for (int i = TRIM_EACH; i < MEAN_GROUPS - TRIM_EACH; i++) acc += grp[i];
    return (uint16_t)(acc / (MEAN_GROUPS - 2 * TRIM_EACH));
}

/**
 * @brief  LCD 引导的三步自动采集(步骤见 calibrate.h 顶部)
 */
static void auto_measure_baseline(GrayBaseline_t *out)
{
    char buf[17];

    /* ========== 0. 开场提示 ========== */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "GRAY CALIB 1/3");
    LubyLCD_ShowLine(1, "FRONT WHEEL ON");
    LubyLCD_ShowLine(2, "THE FRONT EDGE ");
    LubyLCD_ShowLine(3, "WAIT 3 SEC...   ");
    HAL_Delay(3000);
    LubyLCD_ShowLine(3, "SAMPLING...     ");
    out->front_edge = sample_mean(0);   /* 前灰度 AD0 */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "F_EDGE DONE     ");
    snprintf(buf, sizeof(buf), "FRONT=%-5d    ", out->front_edge);
    LubyLCD_ShowLine(1, buf);
    HAL_Delay(4000);

    /* ========== 1. 车尾压后边缘 ========== */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "GRAY CALIB 2/3");
    LubyLCD_ShowLine(1, "REAR WHEEL ON  ");
    LubyLCD_ShowLine(2, "THE REAR EDGE  ");
    LubyLCD_ShowLine(3, "WAIT 3 SEC...   ");
    HAL_Delay(3000);
    LubyLCD_ShowLine(3, "SAMPLING...     ");
    out->rear_edge = sample_mean(1);    /* 后灰度 AD1 */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "R_EDGE DONE     ");
    snprintf(buf, sizeof(buf), "REAR=%-5d     ", out->rear_edge);
    LubyLCD_ShowLine(1, buf);
    HAL_Delay(4000);

    /* ========== 2. 整车放台外地面 ========== */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "GRAY CALIB 3/3");
    LubyLCD_ShowLine(1, "PUT ON GROUND  ");
    LubyLCD_ShowLine(2, "(OFF PLATFORM) ");
    LubyLCD_ShowLine(3, "WAIT 3 SEC...   ");
    HAL_Delay(3000);
    LubyLCD_ShowLine(3, "SAMPLING...     ");
    out->ground[GRAY_FRONT] = sample_mean(0);   /* 前灰度台下值 */
    out->ground[GRAY_REAR]  = sample_mean(1);   /* 后灰度台下值 */

    /* ========== 结果翻页显示, 供记录 ========== */
    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "CALIB FINISHED! ");
    LubyLCD_ShowLine(1, "F_EDGE & R_EDGE ");
    snprintf(buf, sizeof(buf), "%4d / %-4d", out->front_edge, out->rear_edge);
    LubyLCD_ShowLine(2, buf);
    HAL_Delay(2500);

    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "GROUND FWD / R  ");
    snprintf(buf, sizeof(buf), "%4d / %-4d",
             out->ground[GRAY_FRONT], out->ground[GRAY_REAR]);
    LubyLCD_ShowLine(1, buf);
    HAL_Delay(2500);
}

/**
 * @brief  统一入口: 加载灰度阈值
 * @param  baseline  NULL=自动采集标定, 非空=固化加载
 */
void Gray_LoadThresholds(const GrayBaseline_t *baseline)
{
    if (baseline == NULL) {
        auto_measure_baseline(&s_current_baseline);
    } else {
        memcpy(&s_current_baseline, baseline, sizeof(GrayBaseline_t));
    }
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

    /* 读不到 / 读坏了: 退到编译期兜底值。
       注意这里**绝不传 NULL** —— NULL 会去跑 18 秒的 LCD 引导采集,
       正式比赛固件在起跑线上卡 18 秒等人压边缘是不可接受的。
       故障状态由 Cal_LastStatus() 提供给遥测上报。 */
    Gray_LoadThresholds(&g_gray_fallback);
    return false;
}
