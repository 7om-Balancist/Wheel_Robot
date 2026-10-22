/**
 * @file    callbacks.c
 * @brief   HAL 全局回调唯一入口 — 按外设 Instance 分派到对应模块
 *
 * 回调清单:
 *   HAL_UART_RxCpltCallback    — UART4=zigbee, UART5=pi_comm, USART1=蓝牙
 *   HAL_ADC_ConvCpltCallback   — ADC DMA 一轮 16 通道扫描完成 → luby_adc
 *   HAL_ADC_ConvHalfCpltCallback — ADC DMA 前半完成(双缓冲切换) → luby_adc
 */
#include "callbacks.h"
#include "pi_comm.h"
#include "zigbee.h"
#include "bluetooth.h"
#include "luby_adc.h"

/* 通信故障标志 — drive.c 维护, 暂为预留 */
volatile uint8_t g_comms_fault = 0;

/* ================================================================
 * UART 接收完成回调 — 每收到一字节触发一次
 * 执行时间必须短(<100μs, 只做字节接收+状态机推进, 不做复杂解析)
 * ================================================================ */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        ZB_RxByteHandler(huart);      /* zigbee.c 这部分暂时没有使用 */
    } else if (huart->Instance == UART5) {
        PiComm_RxByteCallback(huart); /* pi_comm.c */
    } else if (huart->Instance == USART1) {
        BT_RxByteHandler(huart);      /* 蓝牙 */
    }
}

/* ================================================================
 * ADC DMA 传输完成回调(完整一轮 16 通道扫描完成)
 * 仅通知 luby_adc 切换 DMA 双缓冲读取端。
 * ================================================================ */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ADC_OnDmaComplete(hadc);
    }
}

/* ================================================================
 * ADC DMA 半传输完成回调(前半 8 通道转换完毕)
 * 通知 luby_adc 前半缓冲可用, 保证读到的 16 值属同一轮扫描。
 * ================================================================ */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        ADC_OnDmaHalfComplete(hadc);
    }
}
