/**
 * @file    callbacks.h
 * @brief   HAL 全局回调统一分派 + 通信故障标志
 *
 * HAL 的回调(如 HAL_UART_RxCpltCallback)是全局唯一的(弱符号 → 强符号覆盖),
 * 各模块不能自行定义, 只能由本文件提供统一入口, 再按
 * huart->Instance / hadc->Instance 分派到对应模块的处理函数。
 *
 * 本文件原先还承载了「ISR 快速边缘检测 + MotionTask 紧急通道」:
 *   旧车在 ADC 转换完成中断里直接检查四角灰度是否超硬阈值(3600),
 *   命中就置 g_emergency_stop, 再由 MotionTask 执行 emergency_backoff。
 *   该通道属于已删除的 MotionTask 架构, 且灰度传感器方案已被光电接管,
 *   现整体删除——防掉台统一交给 decision 层的光电防掉台(200Hz 逐帧决策)。
 *   因此 g_emergency_stop / g_pending_cmd / g_cmd_ready / apply_cmd /
 *   emergency_backoff / Callbacks_Init 一并移除。
 */
#ifndef __CALLBACKS_H
#define __CALLBACKS_H

#include "stm32f1xx_hal.h"

/* 通信故障标志 — drive.c 写(连续发送失败置位, 连续成功清除)。
 * 目前仅作记录/预留(无消费者), 故只保留一个字节。 */
extern volatile uint8_t g_comms_fault;   /* 1=CDS55xx 电机总线连续发送失败 */

#endif /* __CALLBACKS_H */
