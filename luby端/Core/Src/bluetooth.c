/**
 * @file    bluetooth.c
 * @brief   HC-05/06 蓝牙模块驱动实现
 *
 * 架构:
 *   发送: 阻塞 HAL_UART_Transmit(), 简单可靠
 *   接收: 单字节 HAL_UART_Receive_IT() → 逐字节中断接收 →
 *         callbacks.c HAL_UART_RxCpltCallback → BT_RxByteHandler() →
 *         行缓冲 → 用户回调
 *
 *   行缓冲: 最多缓存 BT_LINE_BUF_SIZE-1 个字符,
 *           收到 \n 时触发回调, 然后复位缓冲。
 *
 * FreeRTOS 注意事项:
 *   BT_RxCallback 在 ISR 上下文中调用。
 *   如需在任务中消费数据, 推荐在回调中向 FreeRTOS 队列发送指针,
 *   或使用 stream buffer。示例见文件末尾注释。
 */

#include "bluetooth.h"
#include "usart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ──────────────────────────────────────────────
 * 硬件宏
 * ────────────────────────────────────────────── */

#define BT_KEY_PORT             GPIOD
#define BT_KEY_PIN              GPIO_PIN_7
#define BT_KEY_LOW()            (BT_KEY_PORT->BRR = BT_KEY_PIN)
#define BT_KEY_HIGH()           (BT_KEY_PORT->BSRR = BT_KEY_PIN)

/* ──────────────────────────────────────────────
 * 缓冲区常量
 * ────────────────────────────────────────────── */
#define BT_LINE_BUF_SIZE        256U
#define BT_PRINTF_BUF_SIZE      256U
#define BT_AT_RESP_BUF_SIZE     256U

/* 调试日志缓冲 */
#define BT_DEBUG_MAX_MSGS       16U     /* 最多缓存调试信息条数 */
#define BT_DEBUG_MSG_LEN        128U    /* 每条调试信息最大长度 */
#define BT_DEBUG_FLUSH_SIZE     ((BT_DEBUG_MAX_MSGS) * ((BT_DEBUG_MSG_LEN) + 1U))

/* ──────────────────────────────────────────────
 * 模块级变量
 * ────────────────────────────────────────────── */

static BT_RxCallback bt_rx_cb = NULL;       /* 用户注册的接收回调 */
static uint8_t        bt_data_mode = 1;     /* 1=数据模式, 0=AT模式 */

/* 接收: 单字节中断缓冲 + 行缓冲 */
static uint8_t  bt_rx_byte;                 /* HAL_UART_Receive_IT 目标 */
static uint8_t  bt_line_buf[BT_LINE_BUF_SIZE];
static uint16_t bt_line_idx = 0;

/* AT 命令应答(阻塞模式) */
static char           bt_at_resp[BT_AT_RESP_BUF_SIZE];
static uint16_t       bt_at_resp_idx;
static volatile uint8_t bt_at_done;

/* 调试日志环形缓冲 */
static char    bt_debug_msgs[BT_DEBUG_MAX_MSGS][BT_DEBUG_MSG_LEN];
static uint8_t bt_debug_head  = 0;  /* 下一条写入位置 */
static uint8_t bt_debug_count = 0;  /* 当前缓冲中的消息条数 */
static char    bt_debug_flush[BT_DEBUG_FLUSH_SIZE];

/* ──────────────────────────────────────────────
 * 内部函数声明
 * ────────────────────────────────────────────── */

static void BT_StartRx(void);
static void BT_ProcessByte(uint8_t byte);

/* ──────────────────────────────────────────────
 * 初始化
 * ────────────────────────────────────────────── */

void BT_Init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PD7 推挽输出, 置低电平 → 数据透传模式 */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    gpio.Pin   = BT_KEY_PIN;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(BT_KEY_PORT, &gpio);
    BT_KEY_LOW();

    /* 接收状态 */
    bt_line_idx = 0;
    bt_data_mode = 1;
    bt_rx_cb = NULL;

    /* 启动单字节中断接收 */
    BT_StartRx();
}

void BT_SetRxCallback(BT_RxCallback cb)
{
    bt_rx_cb = cb;
}

/* ──────────────────────────────────────────────
 * 模式切换
 * ────────────────────────────────────────────── */

void BT_EnterATMode(void)
{
	BT_KEY_HIGH();
    bt_data_mode = 0;
}

void BT_EnterDataMode(void)
{
    BT_KEY_LOW();
    bt_data_mode = 1;
}

uint8_t BT_IsDataMode(void)
{
    return bt_data_mode;
}

/* ──────────────────────────────────────────────
 * 发送
 * ────────────────────────────────────────────── */

void BT_SendByte(uint8_t byte)
{
    (void)HAL_UART_Transmit(&huart1, &byte, 1U, 100U);
}

void BT_SendString(const char *str)
{
    if (str != NULL) {
        (void)HAL_UART_Transmit(&huart1, (const uint8_t *)str,
                                (uint16_t)strlen(str), 500U);
    }
}

void BT_SendData(const uint8_t *data, uint16_t len)
{
    if (data != NULL && len > 0U) {
        (void)HAL_UART_Transmit(&huart1, data, len, 500U);
    }
}

void BT_Printf(const char *fmt, ...)
{
    char buf[BT_PRINTF_BUF_SIZE];
    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0) {
        uint16_t len = (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf) - 1);
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 500U);
    }
}

/* ──────────────────────────────────────────────
 * 调试日志缓冲
 * ────────────────────────────────────────────── */

/**
 * @brief 将调试信息暂存到环形缓冲
 *
 * 缓冲满时覆盖最旧的消息, 保证总能记录最新调试信息。
 * 仅做内存拷贝, 不阻塞发送, 适合 ISR / 高频任务。
 */
void BT_LogDebug(const char *msg)
{
    uint16_t len;
    uint16_t copy_len;

    if (msg == NULL) {
        return;
    }

    len = (uint16_t)strlen(msg);
    if (len == 0U) {
        return;
    }

    /* 截断过长消息 */
    copy_len = (len < BT_DEBUG_MSG_LEN - 1U) ? len : (BT_DEBUG_MSG_LEN - 1U);

    /* 写入当前 head 位置 */
    memcpy(bt_debug_msgs[bt_debug_head], msg, copy_len);
    bt_debug_msgs[bt_debug_head][copy_len] = '\0';

    /* 环形推进 */
    bt_debug_head = (bt_debug_head + 1U) % BT_DEBUG_MAX_MSGS;

    /* 计数递增, 满后不再增长(覆盖旧数据) */
    if (bt_debug_count < BT_DEBUG_MAX_MSGS) {
        bt_debug_count++;
    }
}

/**
 * @brief 将格式化调试信息暂存到环形缓冲 (printf 风格)
 */
void BT_LogDebugf(const char *fmt, ...)
{
    char buf[BT_DEBUG_MSG_LEN];
    va_list args;
    int n;

    if (fmt == NULL) {
        return;
    }

    va_start(args, fmt);
    n = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (n > 0) {
        BT_LogDebug(buf);
    }
}

/**
 * @brief 取出缓冲中的所有调试信息, 用 \n 连接后通过 BT_SendString 发送
 *
 * 按存入顺序(FIFO)取出, 每条消息之间插入 \n。
 * 发送完成后清空调试缓冲。
 */
void BT_FlushDebugLog(void)
{
    uint8_t  i;
    uint8_t  idx;
    uint16_t total = 0U;
    uint16_t msg_len;

    if (bt_debug_count == 0U) {
        return;
    }

    /* 计算第一条消息的索引(最旧) */
    if (bt_debug_count < BT_DEBUG_MAX_MSGS) {
        /* 缓冲未满, 最旧在 [0] */
        idx = 0U;
    } else {
        /* 缓冲已满, 最旧在 head(被覆盖的下一个) */
        idx = bt_debug_head;
    }

    /* 拼接所有消息到 flush 缓冲 */
    for (i = 0U; i < bt_debug_count; i++) {
        msg_len = (uint16_t)strlen(bt_debug_msgs[idx]);

        /* 预留 \n 和 \0 的空间 */
        if (total + msg_len + 2U > BT_DEBUG_FLUSH_SIZE) {
            break;  /* 超出 flush 缓冲, 截断 */
        }

        if (i > 0U) {
            bt_debug_flush[total++] = '\n';
        }

        memcpy(&bt_debug_flush[total], bt_debug_msgs[idx], msg_len);
        total += msg_len;

        idx = (idx + 1U) % BT_DEBUG_MAX_MSGS;
    }

    bt_debug_flush[total] = '\0';

    /* 一次性发送 */
    if (total > 0U) {
        BT_SendString(bt_debug_flush);
    }

    /* 清空缓冲 */
    bt_debug_head  = 0U;
    bt_debug_count = 0U;
}

/* ──────────────────────────────────────────────
 * AT 命令 (阻塞)
 * ────────────────────────────────────────────── */

HAL_StatusTypeDef BT_SendATCommand(const char *cmd, char *response,
                                   uint16_t resp_len, uint32_t timeout_ms)
{
    BT_RxCallback old_cb;
    uint32_t start;

    if (cmd == NULL) {
        return HAL_ERROR;
    }

    /* 保存旧回调, 临时接管 */
    old_cb = bt_rx_cb;
    bt_rx_cb = NULL;

    /* 重置 AT 响应缓冲 */
    memset(bt_at_resp, 0, sizeof(bt_at_resp));
    bt_at_resp_idx = 0;
    bt_at_done = 0;

    /* 发送命令 + \r\n */
    BT_SendString(cmd);
    BT_SendString("\r\n");

    /* 等待响应 */
    start = HAL_GetTick();
    while (!bt_at_done) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            bt_rx_cb = old_cb;
            return HAL_TIMEOUT;
        }
        /* 让出 CPU(FreeRTOS 下 taskYIELD, 裸机下空转) */
        __NOP();
    }

    /* 拷贝响应 */
    if (response != NULL && resp_len > 0U) {
        uint16_t copy_len = (bt_at_resp_idx < resp_len) ? bt_at_resp_idx : (resp_len - 1U);
        memcpy(response, bt_at_resp, copy_len);
        response[copy_len] = '\0';
    }

    /* 恢复旧回调 */
    bt_rx_cb = old_cb;
    return HAL_OK;
}

/* ──────────────────────────────────────────────
 * 接收中断链
 * ────────────────────────────────────────────── */

/**
 * @brief 启动下一字节中断接收
 */
static void BT_StartRx(void)
{
    HAL_UART_Receive_IT(&huart1, &bt_rx_byte, 1U);
}

/**
 * @brief 逐字节处理(在 ISR 上下文中)
 */
static void BT_ProcessByte(uint8_t byte)
{
    /* ── AT 命令阻塞模式: 直接填充 at_resp ── */
    if (bt_rx_cb == NULL && bt_at_resp_idx < BT_AT_RESP_BUF_SIZE - 1U) {
        bt_at_resp[bt_at_resp_idx++] = (char)byte;

        /* 检测 "OK\r\n" 或 "ERROR\r\n" 结尾 */
        if (bt_at_resp_idx >= 2U &&
            bt_at_resp[bt_at_resp_idx - 2] == 'O' &&
            bt_at_resp[bt_at_resp_idx - 1] == 'K') {
            /* 再等 \r\n 确认, 等收到 \n 时判定 */
        }
        if (bt_at_resp_idx >= 4U &&
            strstr(bt_at_resp, "OK\r\n") != NULL) {
            bt_at_done = 1;
            return;
        }
        if (strstr(bt_at_resp, "ERROR\r\n") != NULL ||
            strstr(bt_at_resp, "ERROR\r") != NULL) {
            bt_at_done = 1;
            return;
        }
        if (byte == '\n' &&
            bt_at_resp_idx >= 2U &&
            bt_at_resp[bt_at_resp_idx - 2] == '\r') {
            /* 以 \r\n 结尾但未匹配 OK/ERROR, 可能是多行响应中间 */
            /* 继续收集, OK/ERROR 会在后续行出现 */
        }
        return;
    }

    /* ── 数据模式: 行缓冲 → 回调 ── */
    if (bt_rx_cb == NULL) {
        return;
    }

    if (byte == '\n') {
        /* 去掉尾部的 \r */
        uint16_t line_len = bt_line_idx;
        if (line_len > 0U && bt_line_buf[line_len - 1U] == '\r') {
            line_len--;
        }

        /* 空行也回调(方便做超时/心跳检测) */
        bt_rx_cb(bt_line_buf, line_len);

        bt_line_idx = 0;
    } else if (bt_line_idx < BT_LINE_BUF_SIZE - 1U) {
        bt_line_buf[bt_line_idx++] = byte;
    }
    /* 超出缓冲: 丢弃, 等待下一个 \n 复位 */
}

/**
 * @brief 中断分派入口(由 callbacks.c HAL_UART_RxCpltCallback 调用)
 *
 * 每收到一个字节调用一次。处理完后立即重启下一字节的接收。
 * 执行时间: < 10μs (一次函数调用 + 状态更新)
 */
void BT_RxByteHandler(UART_HandleTypeDef *huart)
{
    (void)huart;
    BT_ProcessByte(bt_rx_byte);
    BT_StartRx();
}

/* ──────────────────────────────────────────────
 * FreeRTOS 集成示例(不编译, 供参考)
 * ──────────────────────────────────────────────
 *
 * 示例 1: 用 Queue 将数据从 ISR 传到任务
 *
 *   #include "cmsis_os.h"
 *   static osMessageQueueId_t bt_queue;
 *
 *   void BT_ISR_ToTask(const uint8_t *data, uint16_t len) {
 *       // ISR 中只发通知, 不拷贝大数据
 *       BaseType_t woken = pdFALSE;
 *       xTaskNotifyFromISR(bt_task_handle, len, eSetValueWithOverwrite, &woken);
 *       portYIELD_FROM_ISR(woken);
 *   }
 *
 * 示例 2: 在任务中周期性检查
 *
 *   void BTTask(void *arg) {
 *       for (;;) {
 *           uint32_t notified;
 *           if (xTaskNotifyWait(0, 0xFFFFFFFF, &notified, pdMS_TO_TICKS(100))) {
 *               // notified = 上次收到的字节数, 数据可从全局缓冲读
 *           }
 *       }
 *   }
 */

/*
 * 集成到 callbacks.c 的方法:
 *   在 HAL_UART_RxCpltCallback() 中添加:
 *
 *   } else if (huart->Instance == USART1) {
 *       BT_RxByteHandler(huart);
 *   }
 *
 * 在 main.c 初始化序列中添加:
 *
 *   #include "bluetooth.h"
 *   ...
 *   MX_USART1_UART_Init();  // 已有
 *   BT_Init();
 *   BT_SetRxCallback(MyBluetoothHandler);
 *
 * CubeMX 配置:
 *   在 NVIC 选项卡中使能 USART1 global interrupt
 *   (否则 HAL_UART_Receive_IT 不产生中断, 只能发不能收)
 */
