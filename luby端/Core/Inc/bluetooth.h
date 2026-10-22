/**
 * @file    bluetooth.h
 * @brief   HC-05/06 蓝牙模块驱动 (HAL + FreeRTOS)
 *
 * 硬件接口:
 *   USART1: PA9(TX), PA10(RX), 115200-8N1
 *   KEY:    PD7 (低电平=AT模式, 高电平=数据透传)
 *
 * 依赖:
 *   usart.c (MX_USART1_UART_Init 已完成)
 *   callbacks.c (HAL_UART_RxCpltCallback 分派)
 *
 * FreeRTOS 集成:
 *   所有发送函数可在任务上下文中安全调用(HAL_UART_Transmit 阻塞发送)。
 *   接收通过回调机制: 回调在 ISR 上下文中被调用 → 用户应尽快将数据拷贝走,
 *   或在回调中释放信号量/写队列通知任务处理。
 *   若需在回调外消费数据, 建议配合 FreeRTOS stream buffer 或 queue 使用。
 */

#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ──────────────────────────────────────────────
 * 回调类型
 * ────────────────────────────────────────────── */

/**
 * @brief 蓝牙接收回调
 * @param data  接收到的数据指针
 * @param len   数据长度
 *
 * 触发条件: 收到 \n 时, 将行缓冲(不含 \r\n)通过此回调送出。
 * 注意: 此回调在 UART ISR 上下文中执行, 应尽量短(<100μs),
 * 仅做数据拷贝或信号量释放, 不要在回调中做阻塞操作。
 */
typedef void (*BT_RxCallback)(const uint8_t *data, uint16_t len);

/* ──────────────────────────────────────────────
 * 初始化与配置
 * ────────────────────────────────────────────── */

/**
 * @brief 初始化蓝牙模块
 *
 * 行为:
 *   1. 配置 PD7 为推挽输出, 初始拉高(数据透传模式)
 *   2. 重置接收缓冲和行解析状态
 *   3. 启动 USART1 单字节中断接收
 *
 * 前置条件: MX_USART1_UART_Init() 已调用
 * 中断要求: USART1 全局中断必须在 CubeMX NVIC 中使能, 否则只能发不能收
 */
void BT_Init(void);

/**
 * @brief 注册接收回调
 * @param cb  回调函数指针, 传 NULL 取消
 */
void BT_SetRxCallback(BT_RxCallback cb);

/* ──────────────────────────────────────────────
 * 模式切换
 * ────────────────────────────────────────────── */

/**
 * @brief 进入 AT 命令模式 (PD7 拉低)
 *
 * HC-05 模块: KEY 拉低后上电进入 AT 模式, 波特率固定 38400
 * HC-06 模块: 无需 KEY, 直接发 AT 命令即可
 * 本函数仅在 HC-05 上电后需要配置时有用。
 */
void BT_EnterATMode(void);

/**
 * @brief 进入数据透传模式 (PD7 拉高)
 */
void BT_EnterDataMode(void);

/**
 * @brief 查询当前是否在数据模式
 * @return 1=数据模式, 0=AT模式
 */
uint8_t BT_IsDataMode(void);

/* ──────────────────────────────────────────────
 * 发送
 * ────────────────────────────────────────────── */

void BT_SendByte(uint8_t byte);
void BT_SendString(const char *str);
void BT_SendData(const uint8_t *data, uint16_t len);

/**
 * @brief 格式化发送 (printf 风格)
 * @param fmt  格式字符串
 * @param ...  可变参数
 *
 * 内部使用 vsnprintf, 缓冲区 256 字节, 超出截断。
 */
void BT_Printf(const char *fmt, ...);

/* ──────────────────────────────────────────────
 * 调试日志缓冲
 * ────────────────────────────────────────────── */

/**
 * @brief 将调试信息暂存到内部环形缓冲
 * @param msg  调试字符串(自动追加换行)
 *
 * 行为:
 *   将 msg 拷贝到调试缓冲, 等待 BT_FlushDebugLog() 集中发送。
 *   缓冲满时最旧的消息被覆盖。
 *   适合在 ISR 或高频任务中快速记录, 避免阻塞发送。
 */
void BT_LogDebug(const char *msg);

/**
 * @brief 将格式化调试信息暂存到内部环形缓冲 (printf 风格)
 * @param fmt  格式字符串
 * @param ...  可变参数
 *
 * 内部使用 vsnprintf 格式化后调用 BT_LogDebug(), 单条消息最长 128 字节, 超出截断。
 */
void BT_LogDebugf(const char *fmt, ...);

/**
 * @brief 取出缓冲中的所有调试信息, 用 \n 连接后调用 BT_SendString 发送
 *
 * 行为:
 *   1. 按存入顺序取出每条调试信息
 *   2. 用 \n 连接
 *   3. 调用 BT_SendString() 一次性发送
 *   4. 清空调试缓冲
 */
void BT_FlushDebugLog(void);

/* ──────────────────────────────────────────────
 * AT 命令
 * ────────────────────────────────────────────── */

/**
 * @brief 发送 AT 命令并等待响应
 * @param cmd        AT 命令字符串(不含 \r\n, 自动追加)
 * @param response   响应缓冲区(调用者提供)
 * @param resp_len   缓冲区大小
 * @param timeout_ms 超时(ms)
 * @return HAL_OK = 收到响应, HAL_TIMEOUT = 超时, HAL_ERROR = 参数错误
 *
 * 注意: 此函数会临时接管接收回调, 阻塞等待直到收到 "OK" / "ERROR"
 *       或超时。不要在 ISR 或接收回调中调用。
 */
HAL_StatusTypeDef BT_SendATCommand(const char *cmd, char *response,
                                   uint16_t resp_len, uint32_t timeout_ms);

/* ──────────────────────────────────────────────
 * 中断分派(仅供 callbacks.c 调用)
 * ────────────────────────────────────────────── */

void BT_RxByteHandler(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* __BLUETOOTH_H */
