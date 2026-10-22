/**
 * @file    calibrate.h
 * @brief   灰度阈值 — 由一组基准值算出判定阈值(新车: 前后 2 路灰度)
 *
 * 阈值计算(用户指定):
 *   边缘区阈值 edge_thresh[0/1] = 边缘校准值 × 0.9   (灰度高于此 → in_edge_zone)
 *   台上判定阈值 on_stage[0/1]  = 台下地面值 × 0.995  (灰度低于此 → 该端在台上)
 *
 * ----------------------------------------------------------------
 * 2026-09-17 起主工程不再现场采集:
 *   采集流程搬到了独立的标定工程(CalibTool), 采完写进**外部 SPI flash**;
 *   主工程上电只从 flash 读回(见 Gray_LoadFromFlash)。
 *   传 NULL 的自动采集语义**从此只属于标定工程**, 主工程永远不传 NULL。
 * ----------------------------------------------------------------
 */
#ifndef __CALIBRATE_H
#define __CALIBRATE_H

#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include "sensors.h"

/**
 * @brief  灰度基准值(新车 前后 2 路)
 *         ground[0] = 前灰度台下值, ground[1] = 后灰度台下值
 */
typedef struct {
    uint16_t front_edge;   /* 步骤1: 前灰度(AD0)压擂台前边缘读数 */
    uint16_t rear_edge;    /* 步骤2: 后灰度(AD1)压擂台后边缘读数 */
    uint16_t ground[2];    /* 步骤3: 整车台下地面读数 [0]=前灰度 [1]=后灰度 */
} GrayBaseline_t;

/**
 * @brief  由一组基准值算出灰度阈值, 直接写入全局 g_thresh
 * @param  baseline  基准值结构体指针, **主工程永远传非空**(传 NULL 直接返回)
 */
void Gray_LoadThresholds(const GrayBaseline_t *baseline);

/* ================================================================
 * 外部 SPI flash 相关(2026-09-17 新增)
 * ================================================================ */

/**
 * @brief 只读访问器 — 取最近一次用过的基准值
 *
 * s_current_baseline 是文件内 static, 标定工程采集完拿不到结果, 故开这个口子。
 */
const GrayBaseline_t *Gray_GetLastBaseline(void);

/**
 * @brief 主工程上电路径: 从外部 SPI flash 读回标定值
 *
 * 读成功 → 用读到的值算阈值, 返回 true
 * 读失败 → 用编译期兜底值 g_gray_fallback 算阈值, 返回 false
 *
 * ⚠ 绝不调用 Error_Handler, 也绝不传 NULL 给 Gray_LoadThresholds:
 *   · flash 坏了不等于车坏了 —— 竞赛现场死循环等于弃权;
 *   · NULL 在这个 API 里意味着"阻塞 18 秒、引导人工压边缘", 正式固件因
 *     读不到记录就走那条路, 会在起跑线上卡 18 秒等人操作。
 */
bool Gray_LoadFromFlash(void);

/**
 * 编译期兜底基准值 —— flash 读不到时用它。
 *
 * ⚠ 必须填**真实场地上用标定工程实测**的一组典型值, 不是占位符。
 *   这是"flash 读失败"与"车带着编造的阈值冲下擂台"之间唯一的东西。
 */
extern const GrayBaseline_t g_gray_fallback;

#endif /* __CALIBRATE_H */
