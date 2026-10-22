/**
 * @file    drive.c
 * @brief   电机输出层 — 左右轮速 → 两个 CDS55xx 总线舵机(轮模式) 的物理帧
 *
 * ================================================================
 * 全系统唯一与"电机装在哪、朝哪转"相关的文件
 *   改转向 / 换电机, 只看这一个文件就够了。
 * ================================================================
 *
 * ★ 方向约定(全工程唯一标准, decision.c / freertos.c / 蓝牙手动 一致):
 *
 *     cmd.left_speed   = 实际物理左轮速度
 *     cmd.right_speed  = 实际物理右轮速度
 *     正值 = 前进, 负值 = 后退, 0 = 停
 *
 *     两轮同号 → 走直线:  (+, +) 前进        (-, -) 后退
 *     两轮反号 → 原地转:  (+, -) 右转        (-, +) 左转
 *                        左 右               左 右
 *
 *   (2026-09-11 实车验证: turn-R 发 (+70, -70) → 物理左轮往前、物理右轮
 *    往后 → 整车右转 ✓; patrol 发 (+,+) → 直线前进 ✓)
 *
 * ----------------------------------------------------------------
 * 物理层: 两个 CDS55xx(轮模式) 舵机, 速度寄存器 0x20
 *         速度字 = 幅值(0~1023) | 方向位 bit10(0x400)
 *
 *   物理左轮 → 舵机 ID2 → DRIVE_SIGN_LEFT
 *   物理右轮 → 舵机 ID1 → DRIVE_SIGN_RIGHT
 *
 *   两个轮子是**镜像安装**的: 两边舵机收到同一个方向位时, 在地面上会
 *   朝相反方向转。所以下面两个 SIGN 必须反号, "同号指令"才能跑成直线。
 *
 * ★ 2026-09-11 实车结论(试了两次才定下来):
 *   ① 左右两轮的极性必须**相反**(SIGN_LEFT 与 SIGN_RIGHT 反号)——
 *      否则直行指令下两轮各转一边, 车原地打转;
 *   ② 满足①之后整车前后还是反的 → 把两个 SIGN **同时取反**, 得到现在的值。
 *
 *   判据(以后换车/换电机照这个试):
 *     · 发同号指令(left == right), 两轮必须朝同一地面方向转;
 *     · 两轮同向了但整车前后反 → 两个 SIGN 一起取反(仅此两行);
 *     · 两轮各转一边(打转) → 两个 SIGN 的值对调(仅此两行)。
 *
 * ⚠ 历史坑(已修): 本文件早先沿用过一套"逻辑左轮 ↔ 物理右轮交叉"的说法,
 *   那是错的 —— 按它推算 turn-R 会得到"左转", 与实车相反。现已全部删除,
 *   统一为上面这套"left_speed 就是物理左轮"。
 * ----------------------------------------------------------------
 */
#include "drive.h"
#include "cds55xx.h"
#include "callbacks.h"

#define DRIVE_FAULT_THRESHOLD       3U
#define DRIVE_RECOVERY_THRESHOLD    3U

/* ===== 物理轮 ↔ 舵机 ID(接线既定事实) ===== */
#define DRIVE_ID_LEFT               2U   /* 物理左轮(快轮): 起步比右轮快 ~100ms */
#define DRIVE_ID_RIGHT              1U   /* 物理右轮 */

/* ===== 每个物理轮一个方向开关 =====
 *  +1 = 轮速为正 → 舵机 CCW(不带 0x400 位)
 *  -1 = 轮速为正 → 舵机 CW (带 0x400 位)
 *
 *  判据(实车 1 分钟验证): 发直行指令(left == right)时,
 *  两个轮子必须朝同一地面方向转:
 *    · 若某轮转反 → 把该轮的 SIGN 取反;
 *    · 若两个轮子朝同一方向转、但整车是后退 → 说明两侧都装反,
 *      两个 SIGN 一起取反。
 *  实车结论: 左右轮镜像安装、极性相反 → 两个 SIGN 反号。 */
#define DRIVE_SIGN_LEFT             (-1)  /* 物理左轮(ID2) */
#define DRIVE_SIGN_RIGHT            (+1)  /* 物理右轮(ID1) */

static uint8_t failure_count;
static uint8_t success_count;

/**
 * @brief 轮速 → CDS55xx 速度字(幅值 | 方向位 0x400)
 * @param speed 该轮速度(正=该轮前进)
 * @param sign  该轮的安装极性, +1 或 -1(见上)
 * 速度为 0 时不带方向位, 保证"停"就是干净的 0x0000
 */
static uint16_t encode_speed(int16_t speed, int sign)
{
    int32_t v = (int32_t)speed * sign;
    uint16_t magnitude;

    if (v < 0) {
        magnitude = (uint16_t)(-v);
        if (magnitude > 1023U) magnitude = 1023U;
        return (uint16_t)(magnitude | 0x0400U);
    }

    magnitude = (uint16_t)v;
    if (magnitude > 1023U) magnitude = 1023U;
    return magnitude;
}

/**
 * @brief 发送一对轮速: left=物理左轮, right=物理右轮
 *        内部完成"物理轮 → 舵机ID → 方向极性", 调用者不碰硬件细节。
 */
static HAL_StatusTypeDef send_pair(int16_t left, int16_t right)
{
    uint8_t entries[6];
    uint16_t val_left  = encode_speed(left,  DRIVE_SIGN_LEFT);   /* 物理左轮 → 舵机 ID2 */
    uint16_t val_right = encode_speed(right, DRIVE_SIGN_RIGHT);  /* 物理右轮 → 舵机 ID1 */

    entries[0] = DRIVE_ID_LEFT;
    entries[1] = (uint8_t)val_left;
    entries[2] = (uint8_t)(val_left >> 8);
    entries[3] = DRIVE_ID_RIGHT;
    entries[4] = (uint8_t)val_right;
    entries[5] = (uint8_t)(val_right >> 8);

    return CDS55XX_SyncWrite(0x20U, 2U, entries, 2U);
}

HAL_StatusTypeDef Drive_Init(void)
{
    failure_count = 0U;
    success_count = 0U;
    g_comms_fault = 0U;
    return CDS55XX_InitDriveMotors();
}

HAL_StatusTypeDef Drive_SetSpeed(int16_t left, int16_t right)
{
    HAL_StatusTypeDef status = send_pair(left, right);

    if (status != HAL_OK) {
        (void)send_pair(0, 0);
        success_count = 0U;
        if (failure_count < 0xFFU) failure_count++;
        if (failure_count >= DRIVE_FAULT_THRESHOLD) g_comms_fault = 1U;
        return status;
    }

    failure_count = 0U;
    if (success_count < DRIVE_RECOVERY_THRESHOLD) success_count++;
    if (success_count >= DRIVE_RECOVERY_THRESHOLD) g_comms_fault = 0U;
    return HAL_OK;
}

HAL_StatusTypeDef Drive_Stop(void)
{
    return Drive_SetSpeed(0, 0);
}
