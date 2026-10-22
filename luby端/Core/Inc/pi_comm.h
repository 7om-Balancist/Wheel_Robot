/**
 * @file    pi_comm.h
 * @brief   树莓派视觉通信 — UART5 @ 115200-8N1
 *
 * 架构定位: 独立于传感器链, 通过旁路注入 decision.c。
 * Pi 只做摄像头 → AprilTag 检测 → 串口发结果, 不管控制。
 * STM32 只收结果 → 融进决策, 不管 Pi 怎么拍的。
 *
 * 帧格式(12 字节定长, 大端):
 *   0xAA | TYPE | ID | X_H | X_L | Y_H | Y_L | W_H | W_L | H_H | H_L | CHECKSUM
 *   CHECKSUM = TYPE..H_L 共 10 个数据字节的 XOR(防止数据中含 0xAA 导致帧同步漂移)
 *   X: 目标框中心相对画面中心的水平偏移, -1000~+1000, 单位 0.1%(负=左 正=右)
 *   Y: 垂直偏移, 同单位(负=上 正=下)
 *   W/H: 目标框宽/高占画面比例, 0~1000, 单位 0.1%(越大越近)
 *   无目标的心跳帧/躲避帧可把 X/Y/W/H 全填 0。
 *
 * 环形缓冲(深度 3): ISR 写入, SenseDecideTask 读出。
 * 即使 SenseDecideTask 偶尔执行超时, 也不会丢帧。
 */

#include<stdint.h>
#ifndef __PI_COMM_H
#define __PI_COMM_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* 帧常量 */
#define PI_FRAME_SIZE       12      /* 每帧 12 字节: HEADER+TYPE+ID+X+Y+W+H+CHECKSUM */
#define PI_FRAME_HEADER     0xAA     /* 帧头标记 */
#define PI_FIFO_DEPTH       3        /* 环形缓冲深度(ISR 与任务之间的解耦缓冲) */

/* 标签类型 — 对应 Pi 端检测分类结果 */
#define PI_TAG_NONE         0x00     /* 无标签(心跳帧, 本帧未检测到任何目标) */
#define PI_TAG_BOMB         0x01     /* 减益能量块/危险物 — 必须紧急躲避, 否则违规扣分 */
#define PI_TAG_ENEMY        0x02     /* 敌方机器人 — 可攻击 */
#define PI_TAG_NEUTRAL      0x03     /* 中立标签 — 视同敌方处理 */
#define PI_TAG_BOOST        0x04     /* 增益能量块 — 空闲时追踪并撞下擂台得分 */
#define PI_TAG_PENALTY      0x05     /* (预留)减益能量块独立编码, 行为同 BOMB */

/* Pi 离线超时: 超过 500ms 未收到有效帧 → 标签清零 → 纯传感器模式继续跑 */
#define PI_TIMEOUT_MS       500    /* Pi 离线超时: 500ms 无帧 → 炸弹告警清零 */

/**
 * TagInfo — 单帧标签数据
 *
 * rel_x/rel_y: 标签中心相对于画面中心的偏移, 单位 0.1%(范围 -1000~+1000)
 *   负值=左/上, 正值=右/下(图像坐标系, y 向下为正), 0=正中
 * width/height: 标签在画面中的占比, 单位 0.1%(0~1000)
 *   值越大=标签越近。**目前没有任何消费方**(Pi 仍在发, STM32 侧未读)。
 *
 * ⚠ valid 的语义(2026-09-27 起): **只代表"链路活着", 不代表"有目标"**。
 *   目标消失时 Pi 会持续发 NONE(0x00) 心跳帧(见 pi_detect.py), 心跳帧同样会刷新
 *   last_update_ms 并置 valid=1 → 只要 Pi 在跑, valid 就一直是 1, 于是本文件里那条
 *   ">PI_TIMEOUT_MS 强制 valid=0" 的逻辑再也不会触发。判断"有没有目标"请看
 *   type != PI_TAG_NONE 或 target_det; valid==0 现在只在"Pi 真挂了"时出现。
 */
typedef struct {
    uint8_t  valid;              /* 1=链路活着(含 NONE 心跳帧); 0=Pi 离线超时 */
    uint8_t  type;               /* PI_TAG_NONE/BOMB/ENEMY/NEUTRAL/BOOST/PENALTY */
    uint8_t  tag_id;             /* 帧里的 ID 字节(Pi 固定发 0, 无消费方) */
    int16_t  rel_x;              /* -1000(左)~+1000(右) */
    int16_t  rel_y;              /* -1000(上)~+1000(下), 无消费方 */
    uint16_t width;              /* 0~1000, 无消费方 */
    uint16_t height;             /* 0~1000, 无消费方 */
    uint32_t last_update_ms;     /* 上次收到校验通过帧的 HAL_GetTick(NONE 帧也算) */

    /* --- 炸弹(己方能量块) ---
     * bomb_warn : 电平量 = "最近消费到的那一帧就是炸弹帧"。只给 LCD/蓝牙遥测看。
     * bomb_frame: **边沿量** = "本拍刚刚消费到一帧炸弹帧"。只在真正消费到炸弹帧
     *             的那一拍为 1, 下一次 PiComm_ConsumeFrame() 会自动清 0。
     *
     *   ⚠ decision.c 的躲避逻辑必须用 bomb_frame 这个边沿, 不能用 bomb_warn。
     *     炸弹帧是持续到来的(树莓派 50ms 一发), 用电平触发会让躲避被无限续期
     *     → 小车一直后退停不下来(这正是旧实现的 bug)。 */
    uint8_t  bomb_warn;
    uint8_t  bomb_frame;
    /* target_det: 本帧是否**真有目标**(type != PI_TAG_NONE)。
     * 2026-09-27 起由 ISR 写入。目前仍无消费方 —— 留着是为了让"Pi 活着但没目标"
     * 和"Pi 挂了"能分开, 只看 valid 是分不出来的(心跳帧也会让它为 1)。
     * 想用的话: 在遥测里暴露(蓝牙/LCD), 或做"空场超时"逻辑。 */
    uint8_t  target_det;
} TagInfo;

extern volatile uint32_t g_pi_rx_byte_count;   /* 收到的总字节数 */
extern volatile uint32_t g_pi_good_frames;     /* 校验通过的帧数 */
extern volatile uint32_t g_pi_bad_frames;      /* 校验失败的帧数 */

/**
 * PiFrameFIFO — ISR 安全的环形缓冲
 *
 * ISR 只操作 wr_idx 和 count(递增), SenseDecideTask 只操作 rd_idx(递增)。
 * 单写者(ISR)单读者(任务) → 不需要互斥锁, 只需要 volatile + 临界区保护 count。
 */
typedef struct {
    TagInfo frames[PI_FIFO_DEPTH];
    volatile uint8_t wr_idx;     /* ISR 写入位置, 由 ISR 递增 */
    volatile uint8_t rd_idx;     /* 任务读取位置, 由 SenseDecideTask 递增 */
    volatile uint8_t count;      /* 待消费帧数, ISR++/任务--, 临界区保护 */
} PiFrameFIFO;

/* 全局实例 */
extern PiFrameFIFO g_pi_fifo;
extern TagInfo       g_tag_info;    /* 当前消费的标签(SenseDecideTask 更新) */

/* ================================================================
 * API
 * ================================================================ */

/* 初始化环形缓冲为零, 启动 UART5 逐字节中断接收 */
void PiComm_Init(void);

/* UART5 逐字节接收回调(由 callbacks.c 的 HAL_UART_RxCpltCallback 分派调用)
 * 内部维护帧同步状态机: WAIT_AA → 收 11 字节 → 验 CHECKSUM → 写入环形缓冲 */
void PiComm_RxByteCallback(UART_HandleTypeDef *huart);

/* SenseDecideTask 每帧调用: 从环形缓冲消费一帧 → 更新全局 g_tag_info
 * @param out  输出 TagInfo 指针(调用者栈上分配)
 * @return     1=消费成功(有新帧), 0=缓冲为空
 * 内部自动检查超时(>PI_TIMEOUT_MS → valid 清零) */
uint8_t PiComm_ConsumeFrame(TagInfo *out);

#endif /* __PI_COMM_H */
