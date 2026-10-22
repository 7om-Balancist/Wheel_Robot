/**
 * @file    pi_comm.c
 * @brief   树莓派视觉通信实现 — UART5 @ 115200-8N1
 *
 * 通信协议:
 *   Pi 端(独立 Python 脚本): USB 摄像头 → YOLO11 检测 → 串口发送 12 字节帧
 *   STM32 端(本文件): UART5 中断逐字节接收 → 帧同步 → 校验 → 环形缓冲
 *
 * 帧同步策略:
 *   检测 0xAA 帧头 → 严格收后续 10 字节 + 1 字节 CHECKSUM。
 *   数据中可能出现的 0xAA 不会干扰: 验不过 CHECKSUM 就丢弃,
 *   从下一个 0xAA 重新开始。最多丢 1 帧即可恢复同步。
 *
 * 环形缓冲(深度 3):
 *   ISR 写入(wr_idx++), SenseDecideTask 读出(rd_idx++)。
 *   单写者单读者 → 无需互斥锁, count 用临界区保护。
 *   当 count 达到深度上限时 ISR 丢弃新帧(不阻塞 ISR)。
 *
 * 离线检测:
 *   超过 PI_TIMEOUT_MS(500ms)未收到有效帧 → 标签 valid 清零。
 *   车回到纯传感器模式继续战斗 — 不会因为 Pi 挂了就瘫痪。
 */
#include "pi_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include <string.h>

PiFrameFIFO g_pi_fifo;      /* ISR → 任务的环形缓冲 */
TagInfo     g_tag_info;     /* 当前消费的标签(全局, SenseDecideTask 更新) */

/* UART5 接收状态机(ISR 上下文) */
static uint8_t  rx_byte;       /* HAL_UART_Receive_IT 的单字节缓冲 */
static uint8_t  rx_buf[PI_FRAME_SIZE];   /* 当前接收中的帧数据 */
static uint8_t  rx_idx;        /* 已接收字节数(0~11) */
static uint8_t  rx_state;      /* 0=等待帧头, 1=接收数据 */
static uint8_t  rx_checksum;   /* 累积 XOR 校验值 */


volatile uint32_t g_pi_rx_byte_count = 0;
volatile uint32_t g_pi_good_frames = 0;
volatile uint32_t g_pi_bad_frames = 0;
/**
 * 初始化: 清零环形缓冲 + 启动 UART5 逐字节中断接收
 * 在 SenseDecideTask 启动时调用一次
 */
void PiComm_Init(void)
{
    memset(&g_pi_fifo, 0, sizeof(g_pi_fifo));
    memset(&g_tag_info, 0, sizeof(g_tag_info));
    g_tag_info.valid = 0;
    rx_state = 0; rx_idx = 0;
    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

/**
 * UART5 逐字节接收回调(ISR 上下文, NVIC 优先级 5)
 *
 * 帧同步状态机:
 *   WAIT_AA:  等待 0xAA 帧头, 忽略所有其他字节
 *   RECV_DATA: 接收后续 10 字节(TYPE+ID+X+Y+W+H) + 1 字节 CHECKSUM
 *             收到第 11 字节后验证 XOR, 通过则写入环形缓冲
 *             无论通过与否都回到 WAIT_AA 状态
 *
 * 环形缓冲满时: 丢弃当前帧, 不阻塞 ISR
 */
void PiComm_RxByteCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5) return;

    g_pi_rx_byte_count++;

    uint8_t byte = rx_byte;

    if (rx_state == 0) {
        /* 等待帧头 0xAA */
        if (byte == PI_FRAME_HEADER) {
            rx_state = 1;
            rx_idx = 0;
            rx_checksum = 0;
        }
    } else {
        /* 接收数据 */
        if (rx_idx < PI_FRAME_SIZE - 1) {
            rx_buf[rx_idx] = byte;          /* 存 TYPE..H_L + CHECKSUM 共 11 字节 */
        }
        if (rx_idx < PI_FRAME_SIZE - 2) {
            rx_checksum ^= byte;            /* TYPE..H_L 共 10 个数据字节参与 XOR */
        }
        rx_idx++;

        if (rx_idx >= PI_FRAME_SIZE - 1) {
            /* 收完 12 字节(帧头+10数据+checksum), 验证 CHECKSUM */
            if (byte == rx_checksum) {
                g_pi_good_frames++;
                uint8_t next_wr = (g_pi_fifo.wr_idx + 1) % PI_FIFO_DEPTH;
                if (next_wr != g_pi_fifo.rd_idx) {
                    TagInfo *f = &g_pi_fifo.frames[g_pi_fifo.wr_idx];
                    f->type   = rx_buf[0];   /* TYPE: PI_TAG_NONE/BOMB/ENEMY/NEUTRAL/BOOST/PENALTY */
                    f->tag_id = rx_buf[1];   /* ID: 0=空帧 或检测类别编号 */
                    /* X/Y/W/H 大端组装: X=buf[2..3] Y=buf[4..5] W=buf[6..7] H=buf[8..9] */
                    f->rel_x  = (int16_t)(((uint16_t)rx_buf[2] << 8) | rx_buf[3]);
                    f->rel_y  = (int16_t)(((uint16_t)rx_buf[4] << 8) | rx_buf[5]);
                    f->width  = (uint16_t)(((uint16_t)rx_buf[6] << 8) | rx_buf[7]);
                    f->height = (uint16_t)(((uint16_t)rx_buf[8] << 8) | rx_buf[9]);
                    f->last_update_ms = HAL_GetTick();
                    /* valid 只表示"链路活着": NONE(0x00) 心跳帧也算。
                       目标消失后 Pi 仍以 50ms 持续发心跳(pi_detect.py), 所以只要 Pi
                       还在跑, valid 就一直是 1 —— 判断"有没有目标"必须看 type/target_det,
                       不要用 valid(详见 pi_comm.h 里 TagInfo 的说明)。 */
                    f->valid  = 1;
                    f->target_det = (uint8_t)(f->type != PI_TAG_NONE);
                    g_pi_fifo.wr_idx = next_wr;
                    g_pi_fifo.count++;
                }
            } else {
                g_pi_bad_frames++;
            }
            rx_state = 0;
        }
    }

    /* 重新启动下一字节接收(HAL 中断接收需要每字节手动重启) */
    HAL_UART_Receive_IT(&huart5, &rx_byte, 1);
}

/**
 * SenseDecideTask 每帧调用: 消费环形缓冲中的一帧
 *
 * @param out  输出 TagInfo 指针
 * @return     1=消费成功(有新帧), 0=缓冲为空
 *
 * 内部自动检查超时: 如果最近一帧的时间戳距今超过 PI_TIMEOUT_MS, 强制 valid=0。
 * 注意: Pi 正常工作时"没目标"也会发 NONE 心跳帧(pi_detect.py), 所以 count 不会为 0,
 * 这条只在 **Pi 真挂了 / 串口断了** 时生效 —— 它判的是链路, 不是目标。
 */
uint8_t PiComm_ConsumeFrame(TagInfo *out)
{
    uint32_t now = HAL_GetTick();

    /* 边沿标志先清 0: 本拍若没消费到炸弹帧, 下面的 early-return 会让
       decision.c 看不到触发 —— 边沿只维持这一拍。 */
    g_tag_info.bomb_frame = 0;

    /* 无新帧: 仍检查超时, 防止 Pi 离线后 bomb_warn 僵死 */
    if (g_pi_fifo.count == 0) {
        if (g_tag_info.valid && (now - g_tag_info.last_update_ms) > PI_TIMEOUT_MS) {
            g_tag_info.valid = 0;
            g_tag_info.bomb_warn = 0;
        }
        return 0;
    }

    taskENTER_CRITICAL();
    *out = g_pi_fifo.frames[g_pi_fifo.rd_idx];
    g_pi_fifo.rd_idx = (g_pi_fifo.rd_idx + 1) % PI_FIFO_DEPTH;
    g_pi_fifo.count--;
    taskEXIT_CRITICAL();

    /* 炸弹状态**只看帧的 TYPE**, 不看帧新鲜度(原来只要帧新鲜就置 bomb_warn,
     * 导致心跳帧/敌人帧也会让小车逃跑)。
     *   bomb_warn  = 电平: 这一帧是不是炸弹帧(给遥测看)
     *   bomb_frame = 边沿: 只在真正消费到炸弹帧的这一拍为 1
     * PENALTY(减益能量块)行为同 BOMB: 撞下去给对方加分, 必须躲。 */
    if ((now - out->last_update_ms) <= PI_TIMEOUT_MS) {
        out->bomb_warn  = (out->type == PI_TAG_BOMB ||
                           out->type == PI_TAG_PENALTY) ? 1U : 0U;
        out->bomb_frame = out->bomb_warn;
    } else {
        out->valid      = 0;
        out->bomb_warn  = 0;
        out->bomb_frame = 0;
    }

    g_tag_info = *out;
    return 1;
}
