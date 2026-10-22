/**
 * @file    canary_good.c
 * @brief   反证桩: **必须编译通过**。如果这个文件报错, 说明装置本身坏了
 *          (例如 -I 顺序错、真实头文件没被找到), 而不是项目代码有问题。
 *
 * 这里用的全是**真实存在**的字段/类型, 且与 canary_bad_*.c 用的是同一批头文件。
 * 它与 bad 系列成对出现, 才能证明装置"该过的过、该挂的挂", 而不是永远通过
 * 或永远失败。
 */
#include "decision.h"
#include "sensors.h"
#include "pi_comm.h"
#include "cds55xx.h"

static DecisionState canary_st;
static SensorState   canary_sensor;
static TagInfo       canary_tag;

/* 全部为 decision.h / sensors.h 中真实存在的成员 */
void canary_good(void)
{
    canary_st.bomb_escape_active = 1;
    canary_st.bomb_escape_fwd    = 1;   /* 真实存在: uint8_t */
    canary_st.bomb_turning       = 0;   /* 真实存在: uint8_t */
    canary_st.patrol_turn_active = 0;   /* 真实存在: uint8_t */
    canary_st.patrol_turn_dir    = -1;  /* 真实存在: int8_t  */
    canary_st.patrol_run_start_ms = 0;  /* 真实存在: uint32_t */
    canary_st.patrol_rebound_same = 0;
    canary_st.patrol_last_wall_idx = 0;
    canary_st.climb_state = CLIMB_RUSH; /* 真实枚举 */
    canary_st.on_stage = 1;

    g_thresh.wall_threshold[0] = 500;   /* 真实是 uint16_t[8], 合法下标 */
    g_thresh.wall_threshold[7] = 500;
    g_thresh.enemy_threshold[3] = 500;
    g_thresh.on_stage[GRAY_FRONT] = 100;

    canary_sensor.photo_fl = 1;
    canary_sensor.ir_normal[0] = 0;

    canary_tag.bomb_frame = 0;
    canary_tag.type = PI_TAG_NONE;

    (void)canary_sensor;
    (void)canary_tag;
    (void)PiComm_ConsumeFrame;
    (void)Decision_Run;
    (void)CDS55XX_InitDriveMotors;
}
