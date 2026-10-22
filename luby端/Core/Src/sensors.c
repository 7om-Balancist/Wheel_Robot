/**
 * @file    sensors.c
 * @brief   传感器滤波处理 — 新车(前后2灰度 + 8红外 + 爬台红外2 + 防掉台光电4)
 *
 * 灰度不再输出 fast/max 两套窗口: 只有 台上判定用 normal + 边缘区判定用 slow。
 * 防掉台不再使用灰度, 改由四角光电(photo_fl/fr/rl/rr)提供二值信号。
 * 校准功能在独立的 calibrate.c 中。
 */
#include "sensors.h"
#include "luby_adc.h"
#include <string.h>
#include "gpio.h"
#include "pi_comm.h"


/* ================================================================
 * 全局阈值 — 初始默认值, 校准后由 calibrate.c 覆盖（仅灰度）
 * ================================================================ */
SensorThresholds g_thresh = {
        .on_stage      = {3400, 3400},
        .edge_thresh   = {3100, 3100},
        .wall_threshold = {448, 380, 436, 363, 497, 421, 500, 430},
        .enemy_threshold = {480, 500, 520, 650, 650, 600, 600, 480},
};

/* ================================================================
 * 滑动窗口滤波器缓冲区(静态分配, 避免栈溢出)
 * 2灰度×(4+16) + 8红外×(2+4) + 正前红外×5 + 爬台侧向红外2×5 = 103 个 uint16_t
 * ================================================================ */
static uint16_t gbuf_norm[2][4],  gbuf_slow[2][16];
static uint16_t ibuf_fast[8][2],  ibuf_norm[8][4];
static uint16_t ibuf_front[5];      /* 正前红外(ir[0])专用: 爬台对准判定 */
static uint16_t ibuf_climb[2][5];   /* 爬台侧向红外(AD12 左前 / AD13 右前): 5帧均值 */
static uint8_t  s_initialized = 0;

/* 滤波器内部类型 */
typedef struct {
    uint16_t *buf; uint8_t size, idx; uint32_t sum; uint8_t filled;
} Sw;

static void sw_init(Sw *s, uint16_t *b, uint8_t sz) {
    s->buf=b; s->size=sz; s->idx=0; s->sum=0; s->filled=0; memset(b,0,sz*2);
}
static uint16_t sw_up(Sw *s, uint16_t v) {
    s->sum-=s->buf[s->idx]; s->sum+=v; s->buf[s->idx]=v;
    s->idx=(s->idx+1)%s->size;
    if(!s->filled && s->idx==0) s->filled=1;
    return s->filled?(uint16_t)(s->sum/s->size):v;
}

static Sw gn[2], gs[2], i_f[8], i_n[8], i_front, i_climb[2];

/* ================================================================
 * API
 * ================================================================ */
void Sensors_Init(void) {
    for(int i=0;i<2;i++){
        sw_init(&gn[i],gbuf_norm[i],4);
        sw_init(&gs[i],gbuf_slow[i],16);
    }
    for(int i=0;i<8;i++){sw_init(&i_f[i],ibuf_fast[i],2);sw_init(&i_n[i],ibuf_norm[i],4);}
    sw_init(&i_front, ibuf_front, 5);   /* 正前红外: 爬台对准判定用(5帧均值) */
    for(int i=0;i<2;i++) sw_init(&i_climb[i], ibuf_climb[i], 5); /* 爬台侧向红外 AD12/AD13 */
    s_initialized=1;
}

void Sensors_ExtractRaw(const uint16_t *all16, uint16_t *gray2, uint16_t *ir8) {
    gray2[GRAY_FRONT] = all16[0];   /* 前灰度 */
    gray2[GRAY_REAR]  = all16[1];   /* 后灰度 */
    for(int i=0;i<7;i++) ir8[i]=all16[i+4];   /* ir0..6 = AD4..AD10 */
    ir8[7] = all16[14];             /* ir7(315° 左前)=AD14(原 AD11 已被左前防掉台光电占用) */
}

void Sensors_Process(const uint16_t *raw, SensorState *o) {
    if(!s_initialized) Sensors_Init();
    uint16_t gr[2],ir[8]; Sensors_ExtractRaw(raw,gr,ir);

    /* 灰度滤波(前后两路) */
    for(int i=0;i<2;i++) {
        o->gray_normal[i]=sw_up(&gn[i],gr[i]);
        o->gray_slow[i]  =sw_up(&gs[i],gr[i]);
    }
    /* 红外滤波 */
    for(int i=0;i<8;i++) {
        o->ir_fast[i]=sw_up(&i_f[i],ir[i]);
        o->ir_normal[i]=sw_up(&i_n[i],ir[i]);
        o->ir_digital[i]=(o->ir_normal[i]<100)?0:1;
    }
    /* 正前红外(ir[0]/AD4)单独一路 5 帧均值: 只给爬台对准判定用, 不参与敌人/围栏 */
    o->ir_front_slow = sw_up(&i_front, ir[0]);

    /* 爬台侧向红外测距(AD12 左前 / AD13 右前): 2026-10 由反射式光电换成红外测距模块。
     * 同一个 ADC 口、极性反转 —— 旧光电是 "raw<100 = 压到东西", 新红外是
     * "raw 越大 = 越近"。各走一路 5 帧均值, "算不算对墙"由 decision.c 的
     * CLIMB_SIDE_IR_L_THRESH / _R_THRESH 判, 这里只出数值。 */
    o->ir_climb_lf = sw_up(&i_climb[0], raw[12]);
    o->ir_climb_rf = sw_up(&i_climb[1], raw[13]);

    /* 四角防掉台光电(AD11/AD15/AD2/AD3): 低电平有效=下方是擂台表面(安全)
     * 1=擂台上方, 0=悬空 */
    o->photo_fl = (raw[11] < ADC_DIGITAL_THRESH) ? 1 : 0;
    o->photo_fr = (raw[15] < ADC_DIGITAL_THRESH) ? 1 : 0;
    o->photo_rl = (raw[2]  < ADC_DIGITAL_THRESH) ? 1 : 0;
    o->photo_rr = (raw[3]  < ADC_DIGITAL_THRESH) ? 1 : 0;

    /* 敌人检测: 位掩码 */
    o->ir_enemy_mask=0;
    for(int i=0;i<8;i++) if(o->ir_fast[i]>g_thresh.enemy_threshold[i]) o->ir_enemy_mask|=(1<<i);

    /* 围栏检测: 8路取最大值 */
    uint16_t wm=0; uint8_t wi=0;
    for(int i=0;i<8;i++) if(o->ir_normal[i]>wm){wm=o->ir_normal[i];wi=i;}
    o->wall_max_value=wm; o->wall_sensor_idx=wi;
    /* 逐路与自己那一路的阈值比: 任一路超出即检出围栏(与 sensors.h 的
     * "至少一路 > wall_threshold" 一致)。8 路阈值全相等时, 此式与原来的
     * "最大值 > 单一阈值" 完全等价 —— 默认 500 下行为逐位相同。 */
    o->wall_detected=0;
    for(int i=0;i<8;i++) if(o->ir_normal[i]>g_thresh.wall_threshold[i]){o->wall_detected=1;break;}

    /* 台上判定(单帧, 连续确认在 decision.c): 前后灰度均在台上阈值内 → 在台上 */
    o->gray_on_stage_raw =
        (o->gray_slow[GRAY_FRONT] < g_thresh.on_stage[GRAY_FRONT] &&
         o->gray_slow[GRAY_REAR]  < g_thresh.on_stage[GRAY_REAR]) ? 1 : 0;
    o->timestamp_ms=HAL_GetTick();
}
