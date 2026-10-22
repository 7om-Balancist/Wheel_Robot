/**
 * @file    sensors.h
 * @brief   传感器数据结构与滤波处理 API
 *
 * 架构定位: 位于硬件抽象层(luby_adc)之上、决策引擎(decision)之下。
 * 职责: 把 16 路 ADC 原始值变成干净的结构化传感器数据。
 *
 * 注意: 上电校准功能在独立的 calibrate.h/calibrate.c 中。
 *
 * —— 新车(2026-09)传感器拓扑 ——
 *   灰度只剩 前后 2 路(前=AD0, 后=AD1), 用于:
 *     ① 台上/台下判定(gray_on_stage_raw, gray_normal)
 *     ② 台上是否进入边缘区 in_edge_zone(gray_slow, 任意一路超 edge_thresh)
 *     ★ 灰度不再参与防掉台 —— 防掉台由 4 路光电接管。
 *   防掉台光电 4 路, 分布四角: 左前(AD11) 右前(AD15) 左后(AD2) 右后(AD3)。
 *   爬台对准 2 路(AD12/AD13, 左前/右前): 2026-10 由反射式光电**换成红外测距**,
 *     同一个 ADC 口、极性反转(数值越大越靠近障碍), 见 ir_climb_lf/rf。
 *
 * 传感器通道 → ADC buffer 索引(全量 16 路, raw[n]=ADn):
 *   AD0  = raw[0]  = 前灰度
 *   AD1  = raw[1]  = 后灰度
 *   AD2  = raw[2]  = 左后防掉台光电     (原左后灰度口)
 *   AD3  = raw[3]  = 右后防掉台光电     (原右后灰度口)
 *   AD4..AD10      = IR 敌人/围栏 0..6 (0°~270°)
 *   AD11 = raw[11] = 左前防掉台光电
 *   AD12 = raw[12] = 左前爬台对准红外测距 (原反射式光电)
 *   AD13 = raw[13] = 右前爬台对准红外测距 (原反射式光电)
 *   AD14 = raw[14] = IR 7(315° 左前方向, 前身原为 AD11 的路改到了这里)
 *   AD15 = raw[15] = 右前防掉台光电
 *
 * 滤波窗口:
 *   灰度 normal(4帧/20ms): 上台/下台判定
 *   灰度 slow (16帧/80ms): in_edge_zone 判定(强调平滑)
 *   IR   fast/normal(2/4帧): 敌人快速检测 / 围栏+确认
 *   正前红外 / 爬台侧向红外(均 5 帧): 爬台对准围栏判定(更钝, 防单帧抖动误判)
 */
#ifndef __SENSORS_H
#define __SENSORS_H
#define ADC_DIGITAL_THRESH    100    /* 高于此值判为高电平(光电原始值) */

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include "pi_comm.h"

/* 灰度索引别名: [0]=前 [1]=后 */
#define GRAY_FRONT  0
#define GRAY_REAR   1

/* 防掉台悬空掩码(decision 层按位判断): 由 photo_* 反相而来, photo_*=0(悬空) → 该位=1
 * ★ 与 photo_* 成员语义相反: photo_* 里 1=在台上安全; 本掩码里 1=该角悬空(危险) */
#define PHOTO_BIT_FL   0x01   /* 左前 */
#define PHOTO_BIT_FR   0x02   /* 右前 */
#define PHOTO_BIT_RL   0x04   /* 左后 */
#define PHOTO_BIT_RR   0x08   /* 右后 */

/* ================================================================
 * 滑动窗口滤波器
 * ================================================================ */
typedef struct {
    uint16_t *buf;          /* 环形缓冲指针 */
    uint8_t  size;          /* 窗口大小(帧数) */
    uint8_t  idx;           /* 当前写入位置(环形) */
    uint32_t sum;           /* 窗口内所有值的总和 */
    uint8_t  filled;        /* 缓冲区是否已填满 */
} SlidingWindow;

/* ================================================================
 * SensorState — sensors.c 滤波函数的输出
 *
 * 字段索引约定:
 *   gray_*[GRAY_FRONT=0] 前灰度, gray_*[GRAY_REAR=1] 后灰度
 *   ir_*[0..7] 红外角度见下方说明
 *
 *   红外角度 → 索引映射(均为水平安装, 底盘高度 ~6cm):
 *     AD4  = ir[0] =   0° 正前
 *     AD5  = ir[1] =  45° 右前
 *     AD6  = ir[2] =  90° 正右
 *     AD7  = ir[3] = 135° 右后
 *     AD8  = ir[4] = 180° 正后
 *     AD9  = ir[5] = 225° 左后
 *     AD10 = ir[6] = 270° 正左
 *     AD14 = ir[7] = 315° 左前
 * ================================================================ */
typedef struct {
    /* --- 2路灰度(前后) --- */
    uint16_t gray_normal[2];     /* 4帧窗口: in_edge_zone 判定用 */
    uint16_t gray_slow[2];       /* 16帧窗口:  上台/下台判定用 */

    /* --- 8路红外测距 --- */
    uint16_t ir_fast[8];         /* 2帧窗口: 快速敌人检测 */
    uint16_t ir_normal[8];       /* 4帧窗口: 围栏检测 + 敌人确认 */
    uint8_t  ir_digital[8];      /* 二值化: 0=有遮挡 1=无遮挡(阈值100) */
    uint8_t  ir_enemy_mask;      /* 位掩码: bit[i]=1 该方向检测到敌人 */

    /* --- 正前红外(ir[0] / AD4 / 0°)的 5 帧均值 ---
     * 目前唯一消费方 = 爬台对准围栏判定(decision.c 的 ROUTE_DECISION 三选二)。
     * 为什么另开一个 5 帧窗口, 不直接用 ir_fast[0]/ir_normal[0]: 那两个是通用值
     * (2 帧太抖、4 帧是给敌人/围栏用的), 这里要的是一个更钝的正前读数。
     * ⚠ 窗口尺寸(5)直接决定 decision.c 里 CLIMB_FRONT_IR_THRESH 那个阈值该怎么标 ——
     *   改这里的窗口长度, 必须重新标定那个阈值。 */
    uint16_t ir_front_slow;

    /* --- 围栏检测(台下爬台时复用 8 路 IR) --- */
    uint8_t  wall_detected;      /* 至少一路 > wall_threshold */
    uint8_t  wall_sensor_idx;    /* 8 路中值最大的那一路(0~7) */
    uint16_t wall_max_value;     /* 最大那路的 ADC 原始值 */

    /* --- 爬台对准红外测距(AD12 左前 / AD13 右前) ---
     * 2026-10 硬件把这两个口上的**反射式光电换成了红外测距模块**(光电实车不好用),
     * ADC 口没变, 但极性反了 —— 用之前务必记住:
     *   旧光电: raw < ADC_DIGITAL_THRESH(100) → 检测到遮挡(低电平有效);
     *   新红外: raw **越大 = 越靠近障碍** —— 所以下面直接存 5 帧均值原始值,
     *           不要再拿去和 ADC_DIGITAL_THRESH 比。
     * 消费方两处, 阈值都在 decision.c:
     *   ① 爬台"对准围栏"三选二 → CLIMB_SIDE_IR_L_THRESH / _R_THRESH
     *   ② 台上"正前贴脸"判定 front_close → 同样用这两个阈值
     * ⚠ 5 帧窗口的长度直接决定那个阈值该怎么标 —— 改窗口必须重新标定阈值。 */
    uint16_t ir_climb_lf;        /* AD12 左前, 5 帧均值(越大越近) */
    uint16_t ir_climb_rf;        /* AD13 右前, 5 帧均值(越大越近) */

    /* --- 四角防掉台光电 ---
     * 低电平有效(反射式): 1=下方是擂台表面(安全), 0=悬空(危险) */
    uint8_t  photo_fl;           /* AD11 左前 */
    uint8_t  photo_fr;           /* AD15 右前 */
    uint8_t  photo_rl;           /* AD2  左后 */
    uint8_t  photo_rr;           /* AD3  右后 */

    /* --- 台上/台下单帧判定 --- */
    uint8_t  gray_on_stage_raw;  /* 前后灰度均 < on_stage 阈值 → 1(在台上) */

    uint32_t timestamp_ms;       /* 本帧传感器数据的时间戳(HAL_GetTick) */
} SensorState;

/* ================================================================
 * SensorThresholds — 全局阈值结构体(灰度只保留 前后 2 路)
 *
 * 校准后由 calibrate.c 的 calc_thresholds() 按用户现场测量覆盖。
 * ================================================================ */
typedef struct {
    uint16_t on_stage[2];       /* 台上判定阈值 = 台下地面值×0.995 [0]=前 [1]=后
                                   灰度读值低于此 → 该端仍压在擂台上 */
    uint16_t edge_thresh[2];    /* 边缘区判定阈值 = 边缘校准值×0.9 [0]=前 [1]=后
                                   灰度读值高于此 → 该端已近/压台边(in_edge_zone) */
    uint16_t wall_threshold[8]; /* 围栏 ADC 阈值(逐路, 同 enemy_threshold 的索引约定)
                                   decision.c 按 ir_normal[i] > wall_threshold[i] 逐路判定
                                   (P1 前后对角、爬台角落包围检测) */
    uint16_t enemy_threshold[8];/* 敌人检测阈值 */
} SensorThresholds;

/* 全局阈值实例 — 定义在 sensors.c, 由 calibrate.c 填充 */
extern SensorThresholds g_thresh;

/* ================================================================
 * API — 传感器滤波(200Hz, 由 SenseDecideTask 每帧调用)
 * ================================================================ */

/* 初始化所有滑动窗口滤波器(清零) */
void Sensors_Init(void);

/* 核心滤波: 传入 16 路 ADC, 输出 SensorState
 * @param raw_adc  luby_adc 提供的 16 路 ADC 值(A0~A15)
 * @param out      输出结构体指针(调用者栈上分配)
 * 执行时间: ~50μs @ 72MHz */
void Sensors_Process(const uint16_t *raw_adc, SensorState *out);

/* 从 16 路全量 ADC 中提取 2 灰度 + 8 红外
 * 灰度: raw[0..1] = A0~A1(前/后)
 * 红外: raw[4..10] + raw[14] = A4~A10,A14 */
void Sensors_ExtractRaw(const uint16_t *all16, uint16_t *gray2, uint16_t *ir8);

#endif /* __SENSORS_H */
