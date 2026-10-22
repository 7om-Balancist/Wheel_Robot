/**
 * @file    decision.h
 * @brief   核心决策引擎 — 状态机 + 优先级仲裁
 *
 * 架构定位: 系统的"大脑"。输入 SensorState(传感器) + TagInfo(Pi视觉),
 *           输出 MotionCmd(运动指令)。唯一的副作用是修改 DecisionState。
 *
 * 两大工作模式:
 *   台上(ON_STAGE): 优先级仲裁(从高到低, 命中即返回)
 *     1. 四角光电防掉台 — 任一悬空即斜向躲避(生存第一): 触发后至少退缩
 *        300ms(回台仍沿用方向退缩, 防台边往复); 退缩中逐帧按悬空组合重算方向
 *        (组合一变方向立刻变, 只在躲避前 300ms 内有效); 躲避满 300ms **且已无悬空**
 *        时, 前侧悬空的三种组合再接 300ms 原地旋转调转巡台方向; 旋转中一旦再
 *        出现悬空立刻打断旋转、重新躲避(见 decision.c P1)
 *     2. 躲避己方能量块 — 不违规
 *     3. 攻敌 — 红外发现敌人
 *     4. 直线巡台 — 简单直线前进, 直到被更高优先级动作打断
 *
 *   台下(OFF_STAGE): 双轮非阻塞爬台状态机
 *     IDLE → CHECK_CORNER → ROUTE_DECISION → PRE_RUSH
 *       → APPROACH → PAUSE → RUSH
 *       → SLOW_ADVANCE → SETTLE → IDLE
 *     未对墙的等待/缓进状态每帧扫描围栏(8路 IR): 墙检出立即打断重新决策;
 *     自转找墙(ROUTE_DECISION)与已对墙冲刺段(PRE_RUSH~RUSH)不打断。
 */
#ifndef __DECISION_H
#define __DECISION_H

#include "sensors.h"
#include "pi_comm.h"
#include <stdint.h>

/* 卡台边检测通道数: 左后 + 正左 + 正右(与 decision.c 中 STUCK_IR_IDX[] 对应, 或逻辑)
 * ★ 正后方(180°/ir[4])**不在**这个数组里 —— 它的动作与这三路相反(前进 vs 后退旋转),
 *   状态字段也单独一组(stuck_rear_* / stuck_fwd_until), 见 DecisionState。 */
#define STUCK_IR_IDX_NUM  3

/* ================================================================
 * ClimbState — 仅使用左右轮的爬台状态机
 *
 * 每个状态有固定超时, 超时后自动推进到下一状态。
 * 所有状态(除 RUSH/APPROACH 这2个高速运动态)中
 * 持续扫描围栏, 检出墙即立即打断 → 进入 ROUTE_DECISION 重新对齐。
 * ================================================================ */
typedef enum {
    CLIMB_IDLE = 0,          /* 空闲(台上状态或刚完成爬台) */
    CLIMB_CHECK_CORNER,      /* 角落后退 0.3s: 角落掉台时先后退脱离墙面 */
    CLIMB_ROUTE_DECISION,    /* 路线决策: 光电未对墙时自转+卡台边检测(见 decision.c) */
    CLIMB_PRE_RUSH,          /* 反向冲刺 1.3s: 对准围栏后先倒车(负速=后退)拉开距离 */
    CLIMB_APPROACH,          /* 前进 1.5s: 低速接近擂台壁(正速) */
    CLIMB_PAUSE,             /* 停顿 0.25s: 两轮归 0 速, 冲台前从静止同步起步 */
    CLIMB_RUSH,              /* 冲台 1.8s: 全速倒车冲上擂台(车是**倒着**上台的, 负速) */
    CLIMB_SLOW_ADVANCE,      /* 缓进 0.2s: 冲台后缓进稳定车身 */
    CLIMB_SETTLE             /* 稳定 1.5s: 等待车身稳定, 灰度确认上台后进入战斗 */
} ClimbState;

/**
 * DecisionState — 决策引擎内部状态(全局唯一, SenseDecideTask 独占访问)
 *
 * 状态切换保护期: ON_STAGE ↔ OFF_STAGE 切换后 200ms 内忽略灰度新变化,
 * 防止车在擂台边缘时状态反复横跳。
 * 例外: 爬台 SETTLE 阶段灰度单帧确认上台 → 主动提前结束保护期。
 */
typedef struct {
    /* --- 上台/下台状态 --- */
    uint8_t  on_stage;           /* 0=台下 1=台上 */
    uint8_t  stage_confirm_cnt;  /* 连续同向确认帧数(需≥STAGE_CONFIRM_FRAMES=50 帧/250ms 才切换) */
    uint8_t  stage_protection;   /* 1=保护期内, 忽略灰度变化 */
    uint32_t stage_change_ms;    /* 上次状态切换的 HAL_GetTick 值 */

    /* --- 爬台状态机 --- */
    uint8_t  climb_state;        /* ClimbState 枚举当前值 */
    uint8_t  climb_state_prev;   /* 上一帧 climb_state(用于检测重新进入自转) */
    uint8_t  climb_corner_detected; /* IDLE 时检测: 是否角落掉台 */
    uint32_t climb_state_start_ms;  /* 进入当前状态的 tick */

    /* --- 卡台边检测(台下自转扫描时): 左后/正左/正右 多通道, 或逻辑 --- */
    uint32_t stuck_window_start[STUCK_IR_IDX_NUM]; /* 各通道波动窗口起点 tick */
    uint16_t stuck_lr_min[STUCK_IR_IDX_NUM];       /* 各通道窗口内最小值 */
    uint16_t stuck_lr_max[STUCK_IR_IDX_NUM];       /* 各通道窗口内最大值 */
    uint32_t stuck_rev_until;    /* 卡边反向自转截止 tick, 0=未反向 */

    /* --- 正后方挡板脱困(优先级**高于**上面的反向自转, 见 decision.c ROUTE_DECISION) ---
     * 判据: 正后方红外(ir[4] = 180° = AD8)在 STUCK_WINDOW_MS 窗口内
     *       **波动 ≤ STUCK_VARIATION** 且 **窗口内每帧都 > STUCK_REAR_MIN_VALUE(1000)**
     *       → 车尾顶着挡板。
     * 动作: 以 STUCK_FWD_SPD(600) **前进** STUCK_FWD_MS(300ms) 脱离, 走完回自转找台。
     * 为什么要 >1000 这个绝对门槛: 只看"波动小"会把"正后方是空场"也判成卡边
     *   (空场读数低且平), 那样车会在空地上莫名其妙往前冲。
     * 为什么优先级高于反向自转: 反向自转是靠后退脱困; 车尾已经顶着挡板时根本退不动,
     *   越转越蹭, 只能先往前开一段。
     * "一旦已经正对挡板就继续爬台流程"由代码顺序保证: 三选二对准判定排在脱困段之前,
     *   脱困途中一旦对准就立刻转 PRE_RUSH, 不会傻冲满 300ms。 */
    uint32_t stuck_rear_window_start; /* 正后方波动窗口起点 tick, 0=未起窗 */
    uint16_t stuck_rear_min;          /* 窗口内最小值(判"每帧都 >1000"用) */
    uint16_t stuck_rear_max;          /* 窗口内最大值(判波动用) */
    uint32_t stuck_fwd_until;         /* 前进脱困截止 tick, 0=不在脱困段 */

    /* --- 攻击状态机 --- */
    uint8_t  attack1_active;     /* 攻击第一段(推进)进行中: 锁住 3s 轮计时起点 */
    uint32_t attack_time_start_ms; /* 本轮攻击开始 tick */

    /* --- 防消极比赛 --- */
    uint32_t last_action_ms;     /* 上次"有动作"的时间戳, 超 9s 触发防消极微动 */

    /* --- 围栏连续确认 --- */
    uint8_t  wall_last_idx;      /* 上一帧围栏传感器 idx(用于判断方向是否稳定) */
    uint8_t  wall_confirm_cnt;   /* (预留, 当前未启用: 墙检出直接打断, 本计数仅复位不累加) */

    /* --- 校准 --- */
    uint8_t  calibration_done;   /* 校准是否已完成(上电后执行一次) */

    uint8_t  started;               // 软启动标志：0=等待启动，1=已启动运行
    uint16_t start_confirm_cnt;     // 启动条件连续确认计数器

    /* --- 看门狗复位续跑(见 Decision_Init / Decision_Run) --- */
    uint8_t  resume_pending;        // 1=续跑待决: 等灰度滤波窗口填满后再定台上/台下
    uint8_t  resume_cnt;            // resume_pending 期间的稳定帧计数

    /* --- P2 躲避己方能量块(炸弹) ---
     * 一轮 = 一次性 BOMB_ESCAPE_MS(250ms) 后退 + (非"前进躲避"时的) BOMB_TURN_MS(700ms)
     * 原地旋转。只有收到**炸弹帧的边沿**、**且整轮已结束**(既没在躲也没在转)才起一轮;
     * 一轮进行中新来的炸弹帧既不续期也不重开 —— 漏掉 bomb_turning 那一项的话, 50ms
     * 一发的炸弹帧会把旋转段反复打断重来, 表现为"躲完 250ms 后根本不转"(见 §18)。
     * 整轮走完后再收到炸弹帧则再起一轮。
     * bomb_escape_start_ms 在旋转段被**复用**为旋转计时起点(见 decision.c 步骤 4 P2)。 */
    uint8_t  bomb_escape_active;      /* 1=这一轮躲避进行中 */
    uint32_t bomb_escape_start_ms;    /* 本轮躲避的起始 tick; 进入旋转段后改记旋转起点 */
    uint8_t  bomb_escape_fwd;         /* 1=本轮是"前进躲避": 起躲瞬间后方是台边, 反向前进防冲出台沿 */
    uint8_t  bomb_turning;            /* 1=后退躲满后的原地旋转段进行中(前进躲避不接旋转) */

    /* --- 冒险模式(限制区行为开关; 入口条件保持原样) ---
     * 冒险仅影响台上攻敌速度, 不参与防掉台:
     * 四角防掉台统一由优先级1的光电防掉台承担(不论是否冒险)。
     * 悬空触发 P1 逃逸时会把 adventure_active 清掉, 防止在台边反复进出冒险。 */
    uint8_t  adventure_active;        /* 1=冒险模式(限制区激进执行中) */
    uint32_t adventure_exit_ms;       /* 上次退出冒险的时间戳(防抖重入冷却) */

    /* --- P1 光电防掉台: 躲避保持 + 躲避满 300ms 后的原地旋转(见 decision.c 步骤4 P1) ---
     * 悬空触发后: 记录退缩方向与计时起点; 全部回台后仍沿用该方向
     * 退缩 ≥ ESCAPE_HOLD_MS=300ms, 防止"轮子刚擦回台→下一帧满速前冲"的台边往复。
     * 退缩期间每帧按当前悬空组合重算方向: 出现新的悬空组合会立即覆盖 el/er。
     * 本轮躲避满 ESCAPE_SPIN_TRIGGER_MS=300ms **且当帧已无任何光电悬空** 时,
     * 若本轮悬空组合属于"前侧"三种(仅左前/仅右前/双前), 再接一段
     * ESCAPE_SPIN_MS=300ms 原地旋转调转巡台方向(躲避只退让、不改车头朝向,
     * 转个身是为了脱险后不再原路撞回同一条台边)。
     * 旋转方向在进入旋转段的瞬间定死, 期间不再跟随悬空组合; 但**旋转中每帧
     * 仍看光电, 一旦又出现悬空立刻打断旋转、重新起一轮躲避**。 */
    uint8_t  photo_esc_active;        /* 1=退缩窗口进行中 */
    uint32_t photo_esc_start_ms;      /* 最近一次悬空帧的 tick(窗口起点) */
    uint32_t photo_esc_begin_ms;      /* 本轮躲避的第一帧 tick(悬空期间不刷新, 判"满300ms"用) */
    uint8_t  photo_esc_hang;          /* 本轮躲避最近的悬空组合(回台后仍保留, 供旋转判定) */
    int16_t  photo_esc_l;             /* 当前退缩左轮速(悬空时按组合重算, 回台后沿用) */
    int16_t  photo_esc_r;             /* 当前退缩右轮速 */
    uint32_t photo_spin_until;        /* 原地旋转截止 tick, 0=不在旋转段 */
    int16_t  photo_spin_l;            /* 本次旋转左轮速(进入旋转段时定死) */
    int16_t  photo_spin_r;            /* 本次旋转右轮速 */

    /* --- 防绕柱(侧面/后方敌人打破同步旋转) --- */
    uint8_t  anti_circle_phase;       /* 0=空闲 1=直线后退中 2=旋转找敌中 */
    uint32_t anti_circle_start;       /* 防绕柱阶段起始 tick */
    uint8_t  anti_circle_target;      /* 防绕柱目标敌人方向(0~7) */
    /* --- 软启动冲台参数 --- */
    uint8_t  force_climb;         /* 1=硬编码冲台进行中 */
    uint8_t  force_climb_step;    /* 当前步骤: 0=冲台, 1=停止稳定, 2=左转靠拢 */
    uint32_t force_climb_start;   /* 当前步骤起始时间 */
    uint8_t  fc_stage_on_cnt;     /* 左转前台上确认: 连续台上帧计数 */
    uint8_t  fc_stage_off_cnt;    /* 左转中台下确认: 连续台外帧计数 */
    uint8_t  fc_turning;          /* 1=左转确认完成, 正在左转 */

    /* --- 防抖: best_i 滞回 + 转向方向确认 --- */
    uint8_t  best_i_last;         /* 上一帧选中的 best_i */
    uint8_t  best_i_last_valid;   /* 1=已初始化 */
    uint32_t best_i_last_ratio;   /* 上一帧 best_i 对应的 ratio */
    int8_t   turn_dir_last;       /* 上一帧 45° 转向方向: -1=左转 +1=右转 0=未初始化 */
    uint8_t  turn_dir_same_cnt;   /* 转向方向连续相同帧数 */

    /* --- P4 巡台状态机(★未接线: 预留) ---
     * ⚠ 当前 P4 只做**直线前进**(decision.c 末尾 "patrol" 一条), 下面这组字段只在
     * Decision_Init 里被清零、没有任何地方读; decision.c 里的 PATROL_* 宏也未被引用。
     * 它们是"巡台卡住→按 PATROL_TURN_BASE_MS 转向"这套状态机的预留, 尚未实现 ——
     * 调 PATROL_* 宏目前不会有任何效果。要么把状态机补完, 要么连同宏一起删掉。 */
    uint8_t  patrol_turn_active;   /* 1=巡台转向段进行中(未实现) */
    int8_t   patrol_turn_dir;      /* 巡台转向方向: -1=左 +1=右(未实现) */
    uint32_t patrol_run_start_ms;  /* 本轮直行起点 tick(未实现) */
    uint8_t  patrol_rebound_same;  /* 连续同侧撞墙次数, ≥2 触发大角度转向(未实现) */
    uint8_t  patrol_last_wall_idx; /* 上一次围栏通道 idx, 判方向是否稳定(未实现) */

} DecisionState;

/* ================================================================
 * MotionCmd — 决策输出: 一条运动指令
 *
 * Decision_Run 每帧直接返回本结构, SenseDecideTask 写入电机驱动。
 * duration_ms/desc 供遥测显示, 实际控制只读 left_speed/right_speed。
 * (use_gray_speed 曾表示"速度由灰度巡台覆盖", 灰度巡台已删除, 字段一并移除)
 *
 * ★ 方向约定(全工程唯一标准, 详见 drive.c):
 *   left_speed / right_speed 就是**实际物理左轮/右轮**, 正=前进 负=后退。
 *   两轮同号 = 直线((+,+)前进 / (-,-)后退);
 *   两轮反号 = 原地转((+,-)右转 / (-,+)左转)。
 * ================================================================ */
typedef struct {
    char     desc[16];          /* 调试描述(可打印到串口) */
    int16_t  left_speed;        /* 物理左轮: 正=前进 负=后退 (-1023~+1023) */
    int16_t  right_speed;       /* 物理右轮: 正=前进 负=后退 */
    uint16_t duration_ms;       /* 指令持续时长(ms), 0=持续到被更高优先级抢占 */
} MotionCmd;

/* ================================================================
 * API
 * ================================================================ */

/* 初始化决策状态: 默认判为**台下**(由首帧灰度再纠正), 爬台状态 IDLE, 校准未完成
 *
 * @param resume_after_wdg  1 = 本次复位由 IWDG 超时导致(比赛中固件卡死被狗咬重启),
 *                          跳过软启动与硬编码冲台直接续跑; 传 0 是正常上电/手动复位,
 *                          仍要等人压双红外起跑。详见 Decision_Init 实现里的说明。
 *
 * @note 本函数刻意**不自己读 RCC->CSR** —— 决策层要保持"不依赖硬件的纯逻辑"
 *       (可在 PC 上脱离硬件验证), 复位原因由调用者 main.c 判好后当参数传进来。 */
void Decision_Init(DecisionState *state, uint8_t resume_after_wdg);

/**
 * 核心决策函数 — SenseDecideTask 每帧调用一次(200Hz)
 *
 * @param state   决策内部状态(由调用者持有, 持久化)
 * @param sensor  本帧传感器数据(已滤波)
 * @param tag     本帧 Pi 标签数据(可能 valid=0)
 * @param now_ms  当前 HAL_GetTick() 值
 * @return        本帧应执行的 MotionCmd
 *
 * 内部流程:
 *   1. 灰度上台/下台判断: 连续 50 帧(250ms)同向确认 + 200ms 切换保护期
 *   2. 防消极检查: 超过 9s 无动作 → 微动
 *   3. 若 on_stage=0 → 爬台状态机(非阻塞, 见 ClimbState)
 *   4. 若 on_stage=1 → 4 级优先级仲裁(见文件头注释)
 */
MotionCmd Decision_Run(DecisionState *state, const SensorState *sensor,
                       const TagInfo *tag, uint32_t now_ms);

#endif /* __DECISION_H */

/* 蓝牙手动控制全局变量 */
extern volatile uint8_t  g_manual_mode;       // 1=蓝牙手动控制，0=AI自动模式
extern volatile int16_t  g_manual_left_spd;   // 手动模式物理左轮速度(+前进 -后退)
extern volatile int16_t  g_manual_right_spd;  // 手动模式物理右轮速度(+前进 -后退)
