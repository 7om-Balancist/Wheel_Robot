/**
 * @file    decision.c
 * @brief   核心决策引擎 — 上台 4 级优先级仲裁 + 台下多状态非阻塞爬台
 *
 * 调用频率: SenseDecideTask 200Hz (每 5ms 调用一次 Decision_Run)
 *
 * 上台逻辑(ON_STAGE):
 *   优先级从高到低依次判断, 命中即返回:
 *     1. 光电防掉台: 四角防掉台光电任一悬空 → 斜向躲避, 触发后至少退缩
 *        300ms(回台后仍沿用方向退缩, 防台边往复); 退缩中方向逐帧按悬空组合重算,
 *        躲避满 300ms 后若前侧悬空(仅左前/仅右前/双前)再接 300ms 原地旋转调头
 *     2. 躲避己方能量块: Pi 报告标签类型为 BOMB/PENALTY → 紧急躲避
 *     3. 攻敌: 8 路 IR 任一路 > enemy_threshold → 转向敌方或推进
 *     3.5 增益块追踪: 无敌情且视觉看到增益能量块(BOOST) → 对准并撞下擂台（目前尝试过 但是感觉开环控制下效果不好 并且容易被优先级3的敌人检测干扰，可以后续改进，出发点是对的）
 *         (比赛规则: 撞增益块己方加分; 排在攻敌之后 = "没有其他任务"才执行)
 *     4. 直线巡台: 简单直线前进(默认行为)
 *   (灰度仅在台上判 in_edge_zone, 不再防掉台)
 *
 * 台下逻辑(OFF_STAGE):
 *   多状态非阻塞状态机, 每个状态靠 elapsed >= 阈值推进。
 *   未对墙的爬台状态每帧扫描围栏(见步骤 3), 已对墙冲刺段不打断。
 *   状态切换保护期 200ms(ON↔OFF), 防止反复横跳。
 *
 * 设计原则:
 *   - 不调用 HAL_Delay, 不阻塞, 所有计时用 HAL_GetTick 差值
 *   - 单帧执行时间 < 200μs(实测以 BT 遥测 [TIMING] 的 dec_us 为准)
 *   - 所有常量在文件头 #define, 无魔法数字
 */
#include "decision.h"
#include "cds55xx.h"
#include <string.h>
#include <stdio.h>
#include "pi_comm.h"
#include "bluetooth.h"

/* ================================================================
 * 运动速度常量(CDS55xx 速度标度, 范围 -1023~+1023)
 *
 * ★ 方向约定(全工程唯一标准, 详见 drive.c):
 *     正值 = 前进, 负值 = 后退, 0 = 停
 *     两轮同号 = 直线 ((+,+) 前进 / (-,-) 后退)
 *     两轮反号 = 原地转 ((+,-) 右转 / (-,+) 左转), 即"左 > 右 = 右转"
 *   下面所有 SPD_* 都是这个约定下的左右轮速度。
 * ================================================================ */

/* 紧急后退 — 双轮同速后退(负值) */
#define SPD_FULL_REV_L -600
#define SPD_FULL_REV_R -600

/* 爬台序列 */
#define SPD_CLIMB_L -945   /* 冲台左轮: 负=后退(车倒着上擂台) */
#define SPD_CLIMB_R -1000  /* 冲台右轮: 负=后退; 比左轮略大做微调 */
#define SPD_APPROACH_L 600 /* 接近围栏左轮: 正=前进 */
#define SPD_APPROACH_R 600 /* 接近围栏右轮: 正=前进 */
#define SPD_RUSH_L -300    /* 冲台后缓进左轮: 负=后退 */
#define SPD_RUSH_R -360    /* 冲台后缓进右轮: 负=后退 */

/* 对准围栏后的反向冲刺: 与 approach 反向(= 后退 = 冲台方向), 先拉开距离再低速接近 */
#define CLIMB_PRE_RUSH_MS 1300 /* 冲刺时长(ms) */
#define SPD_PRE_RUSH_L -600    /* 反向冲刺左轮(与 approach 相反: 负=后退) */
#define SPD_PRE_RUSH_R -600    /* 反向冲刺右轮 */

/* 爬台"对准围栏"判定: 左前红外 + 右前红外 + 正前红外(ir[0]) 三者**三选二**。
 * 三路各自算"触发" = 该路 5 帧均值 > 对应阈值:
 *   左前 = sensor->ir_climb_lf    (AD12, 原爬台对准光电, 2026-10 换成红外测距)
 *   右前 = sensor->ir_climb_rf    (AD13, 同上)
 *   正前 = sensor->ir_front_slow  (ir[0] = AD4)
 * ⚠ 两个阈值故意分开, **不要图省事合成一个**: AD12/AD13 是新换的独立红外模块,
 *   AD4 是 8 路 IR 阵列里的一路 —— 反射率、安装角、朝向都不同, 必须各自标定。 */
#define CLIMB_FRONT_IR_THRESH 580 /* 正前 ir[0](AD4) 阈值 */
#define CLIMB_SIDE_IR_L_THRESH 540  /* 左前 AD12阈值 */
#define CLIMB_SIDE_IR_R_THRESH 580  /* 右前 AD13 阈值 */
/* 原地旋转(左右轮等速反向) */
#define SPD_ROTATE_L 650
#define SPD_ROTATE_R 650

/* 巡台基础速度 */
#define SPD_PATROL 470

/* 攻击 */
#define SPD_ATTACK_L 850
#define SPD_ROTATE_L_large 620 /* 后方敌人先转侧的大角度旋转速度 */

/* 四角光电防掉台(优先级1) — 斜向躲避速度 */
#define ESCAPE_BASE_SPD 650 /* 躲避基础轮速 */
#define ESCAPE_STEER 280    /* 左右差速: 决定斜向弧线(转弯)幅度 */
/* 悬空触发后最少退缩时长: 全部回台后仍沿用退缩方向走够这段时间,
 * 防止轮子刚擦回台面下一帧就满速前冲 → 台边往复振荡。 */
#define ESCAPE_HOLD_MS 300

/* 躲避满 ESCAPE_SPIN_TRIGGER_MS → 原地旋转 ESCAPE_SPIN_MS 调转巡台方向。
 *   · 前 ESCAPE_SPIN_TRIGGER_MS 内仍逐帧按悬空组合重算方向(组合一变方向立刻变);
 *   · 触发旋转还要求**当帧没有任何光电悬空**(已脱险才转);
 *   · 只有"前侧悬空"的三种组合才旋转(仅左前/仅右前/双前, 见 photo_spin_cmd),
 *     其余组合(后侧/侧边/三角/全悬)不旋转;
 *   · 旋转中每帧继续看光电: 一旦又悬空 → **立刻打断**旋转, 重新起一轮躲避。
 * 为什么需要它: 躲避是"斜向退让", 退让本身不改变车头朝向; 在台边反复退让时,
 * 车头始终对着同一条台边, 下一帧又冲回去。转个身把巡台方向换掉,
 * 是为了覆盖巡台范围 */
#define ESCAPE_SPIN_TRIGGER_MS 400 /* 躲避持续满多久后触发旋转 */
#define ESCAPE_SPIN_MS 500         /* 旋转时长(ms), ≈39°(与 bomb-turn 620 同速) */
#define ESCAPE_SPIN_SPD 620        /* 旋转轮速(左右等速反向) */

/* ================================================================
 * 决策参数
 * ================================================================ */
#define STAGE_CONFIRM_FRAMES 50 /* 上台/下台需连续同向确认的帧数 */
#define STAGE_PROTECT_MS 200    /* 状态切换后保护期(ms) */
#define ATTACK_DURATION_MS 3000 /* 攻击推进最长持续时间(3s) */
#define IDLE_TIMEOUT_MS 9000    /* 防消极超时(9s无动作触发微动) */
#define CORNER_WALL_COUNT 5     /* 角落判断: ≥5路同时触发认为被墙包围 */

/* 比赛软启动参数 */
#define START_CONFIRM_FRAMES 100 /* 启动需连续确认帧数：100帧×5ms = 500ms */
#define START_IR_RIGHT_IDX 2     /* 右侧触发用的红外索引：正右方向 */
#define START_IR_LEFT_IDX 6      /* 左侧触发用的红外索引：正左方向 */

/* 看门狗复位续跑: 采信灰度判台上/台下之前要先等的稳定帧数。
 *
 * 必须 ≥ sensors.c Sensors_Init() 里 gray_slow 的窗口长度(16 帧): 窗口填满之前
 * sw_up() 返回的是**瞬时单帧值**, 那时 gray_on_stage_raw 建立在单次 ADC 采样上,
 * 正是步骤1 的 50 帧确认本来要滤掉的那种噪声。16 帧 = 80ms, 比赛里感知不到,
 * 但拿到手的灰度与稳态同质。
 * ⚠ 若改动 sensors.c 的灰度窗口尺寸, 这里要同步改。 */
#define RESUME_SETTLE_FRAMES 16
/* 躲避己方能量块(炸弹)参数
 * 时长/速度均已收紧(200→120ms, 550→400): 单轮退距减半, 宁可多轮连躲
 * 也不要一记冲太远 —— 防止朝台边退时一步冲出台沿 */
#define BOMB_ESCAPE_MS 250   /* 一轮后退躲避时长(ms): 收到炸弹帧起算, 期间不续期 */
#define BOMB_ESCAPE_BASE 520 /* 躲避纵向基础速度 */
#define BOMB_TURN_MS 700     /* 后退躲满后的原地方向旋转时长(ms), ≈65° */
#define BOMB_TURN_SPD 620    /* 旋转段轮速(左右等速反向, 固定右转) */

/* 增益能量块追踪(优先级3.5)参数: 无敌情/无躲避时, 追踪视觉报告的增益块并撞下擂台 */
#define BOOST_DEADZONE 150   /* 对准死区: |rel_x|≤150(15%画面宽)视为已对准 → 直进撞击 */
#define BOOST_TURN_SPD 620  /* 对准阶段原地转向轮速 */
#define BOOST_CHASE_SPD 560 /* 已对准后的直进撞击速度(高于巡台, 保证撞得动) */

/* 冒险模式(限制区激进执行)参数: 仅影响台上攻敌速度, 不参与防掉台 */
#define ADVENTURE_SPD 650                 /* 冒险模式攻敌降速值(后续按表现调整) */
#define ADVENTURE_REENTRY_COOLDOWN_MS 300 /* 悬空逃逸退出冒险后禁止重入的冷却(ms) */

/* 防绕柱(侧面/后方敌人打破同步旋转)参数 */
#define ANTI_CIRCLE_BACK_MS 420  /* 后退打破同步时长(ms) */
#define ANTI_CIRCLE_BACK_SPD 580 /* 后退速度 */
#define ANTI_CIRCLE_TURN_MS 2500 /* 后退后旋转找敌最大时长(ms) */

/* 硬编码冲台时序 */
#define FORCE_CLIMB_RUSH_MS 1500
#define FORCE_CLIMB_SETTLE_MS 400
/* 冲台+停止结束后左原地转靠拢台中间(550ms): 左轮-600 右轮+600 */
#define FORCE_CLIMB_LEFT_MS 550
#define FORCE_CLIMB_LEFT_L -600
#define FORCE_CLIMB_LEFT_R 600
/* 左转前台上确认: 连续5帧灰度台上才执行左转, 左转中连续5帧台外则中断 */
#define FORCE_CLIMB_STAGE_CONFIRM_FRAMES 5
/* 冲台前两轮停顿时长: 让两轮从 0 速同时起步, 减小左右轮启动时间差 */
#define CLIMB_RUSH_PAUSE_MS 250

/* 卡台边检测: 台下自转扫描围栏时, 左后/正左/正右任一 IR 窗口内波动 ≤STUCK_VARIATION → 判定卡台边, 反向自转脱离 */
static const uint8_t STUCK_IR_IDX[STUCK_IR_IDX_NUM] = {
    5, /* [0] 左后红外(225°, AD9) */
    6, /* [1] 正左红外(270°, AD10) */
    2  /* [2] 正右红外(90°, AD6) */
};
#define STUCK_WINDOW_MS 2000 /* 检测窗口时长(ms) */
#define STUCK_VARIATION 90   /* 窗口内波动阈值(ADC 差值) */
#define STUCK_REV_MS 3500    /* 卡边后反向自转时长(ms), 结束后回到正常扫描 */

/* 正后方挡板脱困: 与上面三路共用窗口时长/波动阈值, 但**动作相反**(前进而不是后退),
 * 且多一道"绝对读数"门槛。见 DecisionState.stuck_rear_* 的说明。 */
#define STUCK_REAR_IR_IDX 4        /* 正后方红外: 180° / AD8 / ir[4] */
#define STUCK_REAR_MIN_VALUE 1000  /* 窗口内每帧都要 > 此值, 才认"真的顶着挡板" */
#define STUCK_FWD_MS 300           /* 脱困前进时长(ms) */
#define STUCK_FWD_SPD 600          /* 脱困前进轮速(正=前进) */

volatile uint8_t g_manual_mode = 0;
volatile int16_t g_manual_left_spd = 0;
volatile int16_t g_manual_right_spd = 0;

/* 注: 本文件不再直接引用全局 g_tag_info —— Pi 标签数据统一走 Decision_Run()
 * 的 tag 形参, 便于测试时换成别的数据源。 */
/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/** 填充 MotionCmd 结构体(避免每处重复赋值)。
 * gray 形参为历史遗留(灰度巡台已删除), 现统一忽略。 */
static void make_cmd(MotionCmd *c, const char *desc, int16_t l, int16_t r,
                     uint16_t dur, uint8_t gray)
{
    (void)gray;
    strncpy(c->desc, desc, 15);
    c->desc[15] = '\0';
    c->left_speed = l;
    c->right_speed = r;
    c->duration_ms = dur;
}

/** 限幅: 确保速度在 -1023~+1023 范围内(CDS55xx 协议限制) */
static int16_t clamp_speed(int32_t v)
{
    if (v > 1023)
        return 1023;
    if (v < -1023)
        return -1023;
    return (int16_t)v;
}

/**
 * 光电防掉台选向 — 按四角防掉台光电的悬空掩码给出斜向躲避轮速
 * @param hang  悬空掩码: bit0=左前 bit1=右前 bit2=左后 bit3=右后
 * @param l, r  输出物理左右轮速(+前进 -后退, 见 drive.c 方向约定)
 *
 * 设计原则:
 *   · 只用"斜向/转向"躲避、不做长时间直退直进 —— 直退直进会让车在台边
 *     前后往复振荡(一退到安全就继续前进又压边, 如此循环)。
 *   · 每帧(5ms)按当前悬空掩码重算, 状态一变动作立即更新(不盲走固定时长)。
 */
static void photo_escape_cmd(uint8_t hang, int16_t *l, int16_t *r)
{
    const int16_t B = ESCAPE_BASE_SPD; /* 基础轮速 */
    const int16_t T = ESCAPE_STEER;    /* 差速(转弯幅度) */

    switch (hang)
    {
    /* ---- 单角悬空: 往对角斜退/斜进, 让该轮回到台面 ---- */
    case PHOTO_BIT_FL: /* 仅左前悬空 → 向右后斜退 */
        *l = -(B + T);
        *r = -B;
        break;
    case PHOTO_BIT_FR: /* 仅右前悬空 → 向左后斜退 */
        *l = -B;
        *r = -(B + T);
        break;
    case PHOTO_BIT_RL: /* 仅左后悬空 → 向右前斜进 */
        *l = +(B + T);
        *r = +B;
        break;
    case PHOTO_BIT_RR: /* 仅右后悬空 → 向左前斜进 */
        *l = +B;
        *r = +(B + T);
        break;

    /* ---- 双角相邻悬空: 整条前/后/侧边越过台沿 ---- */
    case (PHOTO_BIT_FL | PHOTO_BIT_FR): /* 前边双悬空 → 直退 */
        *l = -B;
        *r = -B;
        break;
    case (PHOTO_BIT_RL | PHOTO_BIT_RR): /* 后边双悬空 → 直进 */
        *l = +B;
        *r = +B;
        break;
    case (PHOTO_BIT_FL | PHOTO_BIT_RL): /* 左侧双边悬空 → 右前斜进挤回台 */
        *l = +(B + T);
        *r = +B;
        break;
    case (PHOTO_BIT_FR | PHOTO_BIT_RR): /* 右侧双边悬空 → 左前斜进挤回台 */
        *l = +B;
        *r = +(B + T);
        break;

    /* ---- 对角双悬空(车骑在擂台角上): 兜底直退, 依实车再调 ---- */
    case (PHOTO_BIT_FL | PHOTO_BIT_RR):
    case (PHOTO_BIT_FR | PHOTO_BIT_RL):
        *l = -B;
        *r = -B;
        break;

    /* ---- 三角悬空(仅一角仍在台上): 朝唯一安全角方向移动, 把其余轮拉回台 ---- */
    case (PHOTO_BIT_FR | PHOTO_BIT_RL | PHOTO_BIT_RR): /* 仅左前(FL)安全 → 朝 FL */
        *l = +B;
        *r = +(B + T);
        break;
    case (PHOTO_BIT_FL | PHOTO_BIT_RL | PHOTO_BIT_RR): /* 仅右前(FR)安全 → 朝 FR */
        *l = +(B + T);
        *r = +B;
        break;
    case (PHOTO_BIT_FL | PHOTO_BIT_FR | PHOTO_BIT_RR): /* 仅左后(RL)安全 → 朝 RL */
        *l = -B;
        *r = -(B + T);
        break;
    case (PHOTO_BIT_FL | PHOTO_BIT_FR | PHOTO_BIT_RL): /* 仅右后(RR)安全 → 朝 RR */
        *l = -(B + T);
        *r = -B;
        break;

    /* ---- 四角全悬空(近乎已掉台): 兜底全速直退 ---- */
    case 0x0F:
    default:
        *l = -(B + T);
        *r = -(B + T);
        break;
    }
}

/**
 * @brief 躲避满 300ms 后的原地旋转选向 — 决定"要不要转、往哪转"
 * @param hang  本轮躲避的悬空掩码: bit0=左前 bit1=右前 bit2=左后 bit3=右后
 * @param l, r  输出轮速(+前进 -后退; 左>右=右转, 见 drive.c 方向约定)
 * @return 1=需要旋转(已填好 l/r), 0=不需要旋转
 *
 * 注意: 本函数只看"上一轮的悬空组合"选方向; **能不能真的开始转**由调用处判
 * (还要满足"当帧没有任何光电悬空", 见 Decision_Run 步骤 4 优先级 1)。
 *
 * 只有**车头方向出沿**的三种组合才转身:
 *   · 仅左前悬空 → 原地右转: 右转把左前轮往台内摆(朝安全侧转身);
 *   · 仅右前悬空 → 原地左转: 同理镜像;
 *   · 双前悬空   → 原地右转: 整条前边过沿, 右转把车头调开这个方向。
 * 后侧/侧边/三角/全悬空一律不转 —— 那些组合的躲避本来就是"直进/斜进把轮拉回台"
 * 速度固定 ESCAPE_SPIN_SPD=620(左右等速反向), 时长 ESCAPE_SPIN_MS=300ms。
 */
static uint8_t photo_spin_cmd(uint8_t hang, int16_t *l, int16_t *r)
{
    switch (hang)
    {
    case PHOTO_BIT_FL:                             /* 仅左前悬空 → 原地右转 */
    case (PHOTO_BIT_FL | PHOTO_BIT_FR):            /* 前边双悬空 → 原地右转 */
        *l = +ESCAPE_SPIN_SPD;
        *r = -ESCAPE_SPIN_SPD;
        return 1U;

    case PHOTO_BIT_FR:                             /* 仅右前悬空 → 原地左转 */
        *l = -ESCAPE_SPIN_SPD;
        *r = +ESCAPE_SPIN_SPD;
        return 1U;

    default:                                       /* 其他组合不旋转 */
        return 0U;
    }
}

/** 围栏传感器 idx → 车头需转的角度(°), 正=右转, 负=左转 */
static int16_t wall_angle(uint8_t idx)
{
    int16_t deg = (int16_t)idx * 45; /* 0°=正前, 45°=右前 ... 315°=左前 */
    if (deg > 180)
        deg -= 360; /* 转为 -180°~+180°: 正=右转, 负=左转 */
    return deg;
}

/* ================================================================
 * 决策核心 — Decision_Run
 *
 * 这是整个系统最核心的函数, SenseDecideTask 每 5ms 调用一次。
 * 内部流程:
 *   1. 灰度上台/下台判断(连续50帧确认 + 200ms保护期)
 *   2. 防消极检查(9s 无动作 → 微动)几乎不会触发 除非死机 但死机会看门狗复位
 *   3. 若 on_stage=0 → 爬台状态机(非阻塞)
 *   4. 若 on_stage=1 → 4 级优先级仲裁
 * ================================================================ */

void Decision_Init(DecisionState *st, uint8_t resume_after_wdg)
{
    memset(st, 0, sizeof(DecisionState));
    st->on_stage = 0; /* 初始: 认为在台下 */
    st->climb_state = CLIMB_IDLE;
    st->climb_state_prev = CLIMB_IDLE;
    for (int i = 0; i < STUCK_IR_IDX_NUM; i++)
    {
        st->stuck_window_start[i] = 0;
        st->stuck_lr_min[i] = 0;
        st->stuck_lr_max[i] = 0;
    }
    st->stuck_rev_until = 0;
    /* 正后方挡板脱困(与上面三路分开的一组状态) */
    st->stuck_rear_window_start = 0;
    st->stuck_rear_min = 0;
    st->stuck_rear_max = 0;
    st->stuck_fwd_until = 0;
    st->calibration_done = 0;
    st->last_action_ms = HAL_GetTick();
    // 软启动初始状态
    st->started = 0;
    st->start_confirm_cnt = 0;
    st->force_climb = 0;
    st->force_climb_step = 0;
    st->force_climb_start = 0;
    st->fc_stage_on_cnt = 0;
    st->fc_stage_off_cnt = 0;
    st->fc_turning = 0;
    /* 反弹式巡台初始状态(当前停用, 字段保留) */
    st->patrol_turn_active = 0;
    st->patrol_turn_dir = 1;
    st->patrol_run_start_ms = HAL_GetTick();
    st->patrol_rebound_same = 0;
    st->patrol_last_wall_idx = 0;

    /* ---- 看门狗复位后的自动续跑 ----
     * 语义: 本次复位是 IWDG 咬的 → 认定"比赛进行中被打断", 不等人工起跑。
     *
     * ⚠ 这几句必须写在上面 memset 和所有显式复位**之后** —— 任何一句挪到 memset
     *   前面都会被静默清零, 而且不会有编译错误。
     *
     * 为什么连硬编码冲台一起跳过: 冲台是"从台外冲上擂台"的动作(SPD_CLIMB 是负速,
     *   车是倒着上台的)。而复位时车多半已经在台上, 再冲一次就是朝台缘倒车冲刺。
     *   跳过之后, 台上/台下完全交给步骤1 的灰度判断 + 步骤3 的正常爬台状态机。 */
    if (resume_after_wdg)
    {
        st->started        = 1;   /* 越过步骤0 的 500ms 双红外软启动闸门 */
        st->force_climb    = 0;   /* 跳过硬编码冲台 */
        st->resume_pending = 1;   /* 等灰度窗口填满再定台上/台下, 见 Decision_Run */
        st->resume_cnt     = 0;
    }
}

MotionCmd Decision_Run(DecisionState *st, const SensorState *sensor,
                       const TagInfo *tag, uint32_t now)
{
    MotionCmd cmd = {0}; /* 本帧输出命令; 兜底为零速, 防止个别分支未设 cmd 读到栈垃圾 */
    memset(&cmd, 0, sizeof(cmd));
    /* ================================================================
     * 步骤 0：比赛软启动检测（最高优先级）
     * 左右两侧红外同时遮挡持续500ms → 解锁全部功能，自主运行
     * ================================================================ */
    if (!st->started)
    {
        // 判断左右两路红外是否同时被遮挡（数值超过阈值=近距离遮挡）
        uint8_t left_blocked = (sensor->ir_normal[START_IR_LEFT_IDX] > g_thresh.wall_threshold[START_IR_LEFT_IDX]);
        uint8_t right_blocked = (sensor->ir_normal[START_IR_RIGHT_IDX] > g_thresh.wall_threshold[START_IR_RIGHT_IDX]);

        if (left_blocked && right_blocked)
        {
            st->start_confirm_cnt++;
            // 连续确认达标，正式启动
            if (st->start_confirm_cnt >= START_CONFIRM_FRAMES)
            {
                st->started = 1;
                st->last_action_ms = now;
                st->climb_state = CLIMB_RUSH;
                // 记录冲台起始时间，状态机以此计算冲台时长
                st->climb_state_start_ms = now;
                // 清零围墙检测的残留计数，避免历史值干扰后续状态
                st->wall_confirm_cnt = 0;
                st->wall_last_idx = 0;
                // 清零角落检测标记，避免误触发角落后退
                st->climb_corner_detected = 0;

                st->force_climb = 1;
                st->force_climb_step = 0;
                st->force_climb_start = now;
            }
        }
        else
        {
            // 不同时遮挡，重置计数，防误触
            st->start_confirm_cnt = 0;
        }

        // 未启动状态：电机完全静止，直接返回
        make_cmd(&cmd, "wait-start", 0, 0, 0, 0);
        return cmd;
    }
    /* ================================================================
     * 看门狗复位续跑: 等灰度滤波窗口填满, 再用它定台上/台下, 跳过那 50 帧确认。
     *
     * 为什么不能靠默认 on_stage=0 让步骤1 自己确认 250ms: 车复位时多半还在台上,
     *   默认"台下"会让步骤3 的爬台状态机在台上空转 250ms(原地自转找墙 / 倒退冲刺),
     *   有可能把车推下台。
     * 为什么要等 RESUME_SETTLE_FRAMES 帧而不是首帧就用: 见该宏的说明 —— 滤波窗口
     *   填满前 gray_on_stage_raw 建立在单帧 ADC 采样上, 不能采信。
     * ================================================================ */
    if (st->resume_pending)
    {
        st->last_action_ms = now;   /* 防消极的 9s 从"续跑"起算, 而不是从开机起算 */

        if (st->resume_cnt < RESUME_SETTLE_FRAMES)
        {
            st->resume_cnt++;
            make_cmd(&cmd, "resume-wait", 0, 0, 0, 0);   /* 稳定窗口内两轮归零 */
            return cmd;
        }

        st->resume_pending       = 0;
        st->resume_cnt           = 0;
        st->on_stage             = sensor->gray_on_stage_raw;
        st->climb_state          = CLIMB_IDLE;
        st->climb_state_prev     = CLIMB_IDLE;   /* 必须与 climb_state 保持一致 */
        st->climb_state_start_ms = now;
        st->stage_change_ms      = now;
        st->stage_protection     = 0;
        st->stage_confirm_cnt    = 0;
        /* 这里**不 return**: 本帧继续往下走正常的步骤1~4。
           此刻 on_stage 已等于 gray_on_stage_raw, 步骤1 会判"没有变化"而不二次切换。 */
    }
    /* 硬编码冲台：软启动后强制执行，不依赖传感器（续跑路径 force_climb 恒为 0） */
    if (st->force_climb)
    {
        uint32_t elapsed = now - st->force_climb_start;

        switch (st->force_climb_step)
        {
        case 0:
            if (elapsed >= FORCE_CLIMB_RUSH_MS)
            {
                st->force_climb_step = 1;
                st->force_climb_start = now;
            }
            else
            {
                make_cmd(&cmd, "fc-rush", SPD_CLIMB_L, SPD_CLIMB_R, 0, 0);
                st->last_action_ms = now;
                return cmd;
            }
            break;

        case 1:
            if (elapsed >= FORCE_CLIMB_SETTLE_MS)
            {
                /* 冲台+停止完成 → 进入左转靠拢(step 2), 清零台上确认计数 */
                st->force_climb_step = 2;
                st->force_climb_start = now;
                st->fc_stage_on_cnt = 0;
                st->fc_stage_off_cnt = 0;
                st->fc_turning = 0;
            }
            else
            {
                make_cmd(&cmd, "fc-settle", 0, 0, 0, 0);
                st->last_action_ms = now;
                return cmd;
            }
            break;

        case 2:
            /* 左转靠拢台中间: 左轮600 右轮600, 持续550ms, 让车快速往台中间靠拢
             * 台上5帧确认: 连续 FORCE_CLIMB_STAGE_CONFIRM_FRAMES 帧灰度台上才执行左转;
             * 左转中连续5帧台外则中断退出; 冲台失败(未上台)不左转, 交回爬台逻辑 */
            if (sensor->gray_on_stage_raw)
            {
                if (st->fc_stage_on_cnt < 255)
                    st->fc_stage_on_cnt++;
                st->fc_stage_off_cnt = 0;
            }
            else
            {
                st->fc_stage_on_cnt = 0;
                if (st->fc_stage_off_cnt < 255)
                    st->fc_stage_off_cnt++;
            }

            if (!st->fc_turning)
            {
                /* 未确认台上: 连续5帧台外 → 直接跳过左转退出(如冲台失败) */
                if (st->fc_stage_off_cnt >= FORCE_CLIMB_STAGE_CONFIRM_FRAMES)
                {
                    st->force_climb = 0;
                    st->force_climb_step = 0;
                    make_cmd(&cmd, "fc-turn-skip", 0, 0, 0, 0);
                    st->last_action_ms = now;
                    return cmd;
                }
                /* 连续5帧台上 → 确认左转, 从确认点重新计时550ms */
                if (st->fc_stage_on_cnt >= FORCE_CLIMB_STAGE_CONFIRM_FRAMES)
                {
                    st->fc_turning = 1;
                    st->force_climb_start = now;
                }
                else
                {
                    make_cmd(&cmd, "fc-turn-wait", 0, 0, 0, 0);
                    st->last_action_ms = now;
                    return cmd;
                }
            }

            /* 左转中: 连续5帧台外 → 中断退出 */
            if (st->fc_stage_off_cnt >= FORCE_CLIMB_STAGE_CONFIRM_FRAMES)
            {
                st->fc_turning = 0;
                st->force_climb = 0;
                st->force_climb_step = 0;
                make_cmd(&cmd, "fc-turn-skip", 0, 0, 0, 0);
                st->last_action_ms = now;
                return cmd;
            }

            if (elapsed >= FORCE_CLIMB_LEFT_MS)
            {
                st->fc_turning = 0;
                st->force_climb = 0;
                st->force_climb_step = 0;
                st->on_stage = 1;
                make_cmd(&cmd, "fc-done", 0, 0, 0, 0);
                st->last_action_ms = now;
                return cmd;
            }
            else
            {
                make_cmd(&cmd, "fc-turnL", FORCE_CLIMB_LEFT_L, FORCE_CLIMB_LEFT_R, 0, 0);
                st->last_action_ms = now;
                return cmd;
            }
            break;
        }
    }

    /* ================================================================
     * 步骤 1: 灰度上台/下台判断(连续 50 帧(250ms)同向确认 + 200ms 保护期)
     *
     * 为什么需要连续 50 帧(250ms)? 单帧可能因传感器噪声或车轮压到边缘而误判。
     * 为什么需要保护期? 车在擂台边缘时灰度可能来回跳, 导致状态来回切,
     *   每次切换触发爬台/战斗模式切换 → 车失控。
     * 例外: 爬台 SETTLE 阶段灰度单帧确认上台 → 主动提前结束保护期,
     *   避免车已上台却被保护期卡在爬台状态。
     * ================================================================ */
    uint8_t gray_on_stage = sensor->gray_on_stage_raw;

    if (st->stage_protection && (now - st->stage_change_ms) < STAGE_PROTECT_MS)
    {
        /* 保护期内: 保持当前状态不变 */
        if (st->climb_state == CLIMB_SETTLE && gray_on_stage)
        {
            st->stage_protection = 0; /* 提前结束保护 */
        }
    }
    else
    {
        st->stage_protection = 0;
        if (gray_on_stage == st->on_stage)
        {
            st->stage_confirm_cnt = 0; /* 没有变化, 重置计数器 */
        }
        else
        {
            st->stage_confirm_cnt++; /* 同向变化, 递增确认计数 */
            if (st->stage_confirm_cnt >= STAGE_CONFIRM_FRAMES)
            {
                /* 连续 50 帧(250ms)确认: 执行状态切换 */
                st->on_stage = gray_on_stage;
                st->stage_change_ms = now;
                st->stage_protection = 1;
                st->stage_confirm_cnt = 0;
            }
        }
    }

    /* ================================================================
     * 步骤 2: 防消极比赛检查(台上超过 9s 无动作 → 微动)
     * ================================================================ */
    if ((now - st->last_action_ms) > IDLE_TIMEOUT_MS && st->on_stage)
    {
        make_cmd(&cmd, "anti-idle", SPD_PATROL, SPD_PATROL, 200, 0);
        st->last_action_ms = now;
        return cmd;
    }

    /* ================================================================
     * 步骤 3: 台下 — 爬台状态机(非阻塞, 状态见 decision.h ClimbState)
     *
     * 未对墙的爬台状态每帧扫描围栏(IR 8路): 墙检出立即打断(见下)。
     * 角落包围检测: ≥5路同时 > wall_threshold → 后退脱离而非取平均方向。
     * ================================================================ */
    if (!st->on_stage)
    {
        uint32_t elapsed = now - st->climb_state_start_ms;

        /* 下台后清掉 P1 退缩/旋转状态, 防止爬回台上时残留的 300ms 窗口误触发 */
        st->photo_esc_active = 0;
        st->photo_spin_until = 0;

        /* --- 围栏打断(未对墙的爬台状态才生效) ---
         * 自转找墙(ROUTE_DECISION)与已对墙冲刺段(PRE_RUSH~RUSH)
         * 不做墙打断, 避免冲刺途中因传感器抖动而中断。 */
        if (sensor->wall_detected &&
            st->climb_state != CLIMB_ROUTE_DECISION &&
            st->climb_state != CLIMB_PRE_RUSH &&
            st->climb_state != CLIMB_APPROACH &&
            st->climb_state != CLIMB_PAUSE &&
            st->climb_state != CLIMB_RUSH)
        {
            /* 角落包围检测: ≥5路同时触发 → 是角落, 后退 
            注意防卡角落的这部分效果并不好 需要重写*/
            uint8_t cnt = 0;
            for (int i = 0; i < 8; i++)
                if (sensor->ir_normal[i] > g_thresh.wall_threshold[i])
                    cnt++;
            if (cnt >= CORNER_WALL_COUNT)
            {
                make_cmd(&cmd, "corner-escape",
                         SPD_FULL_REV_L / 2, SPD_FULL_REV_R / 2, 300, 0);
                st->climb_state = CLIMB_IDLE;
                st->climb_state_start_ms = now;
                st->last_action_ms = now;
                return cmd;
            }
            /* 正常围栏检出 → 打断当前状态, 进入路线决策重新对齐 */
            st->climb_state = CLIMB_ROUTE_DECISION;
            st->climb_state_start_ms = now;
            st->wall_confirm_cnt = 0;
            elapsed = 0;
        }

        /* 记录上一帧状态: 卡台边检测用它判断是否"重新进入自转"(切换后重置波动窗口) */
        uint8_t prev_climb_state = st->climb_state_prev;
        st->climb_state_prev = st->climb_state;
        /* 爬台"对准围栏"三选二计数(ROUTE_DECISION 用)。
         * 声明放在 switch 外面而不是 case 里: C 的标号(route_decision:)后面只能跟
         * 语句, 不能直接跟声明 —— 写进 case 里会报 "a label can only be part of
         * a statement"。 */
        uint8_t align_cnt;

        switch (st->climb_state)
        {

        /* ---- IDLE: 刚检测到掉台, 启动爬台流程 ---- */
        case CLIMB_IDLE:
            /* 检查是否角落掉台(前两路 IR + 任一侧路 IR 同时触发) */
            st->climb_corner_detected =
                (sensor->ir_digital[0] == 0 && sensor->ir_digital[1] == 0 &&
                 (sensor->ir_digital[6] == 0 || sensor->ir_digital[2] == 0));
            st->climb_state_start_ms = now;
            if (st->climb_corner_detected)
            {
                st->climb_state = CLIMB_CHECK_CORNER;
                make_cmd(&cmd, "corner-back",
                         SPD_CLIMB_L / 2, SPD_CLIMB_R / 2, 300, 0);
                break;
            }
            st->climb_state = CLIMB_ROUTE_DECISION;
            st->climb_state_start_ms = now;
            st->wall_confirm_cnt = 0;
            goto route_decision;

        /* ---- CHECK_CORNER: 角落后退(0.3s) ---- */
        case CLIMB_CHECK_CORNER:
            if (elapsed >= 300)
            {
                st->climb_state = CLIMB_ROUTE_DECISION;
                st->climb_state_start_ms = now;
                goto route_decision;
            }
            else
            {
                make_cmd(&cmd, "corner-back-wait", 0, 0, 0, 0);
            }
            break;

        /* ---- ROUTE_DECISION: 对准围栏判定 + 自转/卡边检测 ----
         * 对墙判据 = 左前红外(AD12) + 右前红外(AD13) + 正前红外(AD4),
         * **三者中 ≥2 触发** (见下方 align_cnt; 旧版要求"两路光电必须同时触发")。
         * 对准 → 冲刺序列(PRE_RUSH); 未对准 → 原地自转继续找。
         * 卡台边检测(自转时做), 优先顺序:
         *   ① 正后方挡板: ir[4] 的 2s 窗口内波动 ≤阈值 **且** 每帧都 >1000
         *      → 前进 300ms @600 脱离(车尾顶着挡板时后退自转没法脱困) 但后面发现找台旋转速度够大的时候 好像不会卡住;
         *   ② 左后/正左/正右(STUCK_IR_IDX[]) 任一 IR 的 2s 窗口内波动 ≤阈值
         *      → 判定卡台边, 反向自转脱离。
         * ② 的波动判据会被"正后方是空场"满足, 所以 ① 才需要那道绝对读数门槛。 */
        case CLIMB_ROUTE_DECISION:
        route_decision:
            /* 三选二对准判定 → 车头已对上围栏: 先倒车碰台(负速=后退, 即冲台方向), 再接近挡板。
             * 为什么两侧从光电换成红外(2026-10): 反射式光电实车不好用 —— 有效距离太短、
             * 对墙面颜色/反光敏感、装歪一点就一侧亮一侧不亮。红外测距是**模拟量**,
             * 离墙越近读数越高（注意但是太近的话会反而降低）, 天然带"距离信息", 判据比二值光电稳。 */
            align_cnt = 0;
            if (sensor->ir_climb_lf > CLIMB_SIDE_IR_L_THRESH) align_cnt++;
            if (sensor->ir_climb_rf > CLIMB_SIDE_IR_R_THRESH) align_cnt++;
            if (sensor->ir_front_slow > CLIMB_FRONT_IR_THRESH) align_cnt++;
            if (align_cnt >= 2)
            {
                st->climb_state = CLIMB_PRE_RUSH;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "pre-rush", SPD_PRE_RUSH_L, SPD_PRE_RUSH_R, 0, 0);
                break;
            }
            /* 未对准 → 自转扫描, 同时做卡台边检测 */
            if (prev_climb_state != CLIMB_ROUTE_DECISION || st->stuck_window_start[0] == 0)
            {
                /* 从其他状态进入自转 → 重置各通道波动窗口 */
                for (int i = 0; i < STUCK_IR_IDX_NUM; i++)
                {
                    st->stuck_window_start[i] = now;
                    st->stuck_lr_min[i] = st->stuck_lr_max[i] =
                        sensor->ir_normal[STUCK_IR_IDX[i]];
                }
                st->stuck_rev_until = 0;
                /* 正后方挡板窗口一并重开; 上一轮留下的脱困段也清掉 */
                st->stuck_rear_window_start = now;
                st->stuck_rear_min = st->stuck_rear_max =
                    sensor->ir_normal[STUCK_REAR_IR_IDX];
                st->stuck_fwd_until = 0;
            }

            /* ---- 正后方挡板脱困(优先级**高于**下面的反向自转脱离) ----
             * 车尾顶着挡板时反向自转是无效动作: 轮子往后退、车身被挡板顶住,
             * 只会原地蹭, 脱不了困。这里改成直线前进 STUCK_FWD_MS, 先离开挡板。
             *
             * 判据(与三路卡边共用窗口时长/波动阈值, 只多一道绝对门槛):
             *   ① 窗口满 STUCK_WINDOW_MS;
             *   ② 窗口内波动 ≤ STUCK_VARIATION  → "读数稳", 说明不是路过;
             *   ③ 窗口内**每帧都 > STUCK_REAR_MIN_VALUE(1000)** → "离得近"。
             * 只靠 ② 会把"正后方是空场"也判成卡边(空场读数低且平),
             * 那样车会在空地上莫名其妙往前冲 —— ③ 就是用来区分"挡板"和"空场"的。
             *
             * ⚠ "一旦已经正对挡板就继续爬台流程"由**代码顺序**保证, 不是额外加的判断:
             *   三选二对准判定在本 case 最上面, 脱困途中一旦对准就直接 PRE_RUSH,
             *   走不到这里, 也就不会傻冲满 300ms。改动本段时别把顺序调乱。 */
            if (st->stuck_fwd_until != 0)
            {
                if (now < st->stuck_fwd_until)
                {
                    make_cmd(&cmd, "rear-stuck-fwd", STUCK_FWD_SPD, STUCK_FWD_SPD, 0, 0);
                    break;
                }
                /* 脱困段刚走完: 清标记并把正后方窗口整段重开,
                 * 让之后的判定只基于"脱困后"的读数(否则窗口会横跨这 300ms 前进过程) */
                st->stuck_fwd_until = 0;
                st->stuck_rear_window_start = now;
                st->stuck_rear_min = st->stuck_rear_max =
                    sensor->ir_normal[STUCK_REAR_IR_IDX];
            }
            /* 维护正后方窗口 */
            {
                uint16_t rv = sensor->ir_normal[STUCK_REAR_IR_IDX];
                if (rv < st->stuck_rear_min)
                    st->stuck_rear_min = rv;
                if (rv > st->stuck_rear_max)
                    st->stuck_rear_max = rv;
                if (now - st->stuck_rear_window_start >= STUCK_WINDOW_MS-400)
                {
                    if ((uint16_t)(st->stuck_rear_max - st->stuck_rear_min) <= STUCK_VARIATION &&
                        st->stuck_rear_min > STUCK_REAR_MIN_VALUE)
                    {
                        /* 顶着挡板 → 起前进脱困段。
                         * 同时把四路窗口全部重开、并取消反向自转:
                         *   · 取消反向自转 = 本脱困优先级更高的直接体现;
                         *   · 重开窗口 = 免得脱困刚结束, 旧窗口立刻又判一次卡边来回抖。 */
                        st->stuck_fwd_until = now + STUCK_FWD_MS;
                        st->stuck_rev_until = 0;
                        for (int j = 0; j < STUCK_IR_IDX_NUM; j++)
                        {
                            st->stuck_window_start[j] = now;
                            st->stuck_lr_min[j] = st->stuck_lr_max[j] =
                                sensor->ir_normal[STUCK_IR_IDX[j]];
                        }
                        st->stuck_rear_window_start = now;
                        st->stuck_rear_min = st->stuck_rear_max = rv;
                        make_cmd(&cmd, "rear-stuck-fwd", STUCK_FWD_SPD, STUCK_FWD_SPD, 0, 0);
                        break;
                    }
                    /* 波动大(在动) 或 读数不够高(后面是空场) → 不是挡板, 开新窗口继续看 */
                    st->stuck_rear_window_start = now;
                    st->stuck_rear_min = st->stuck_rear_max = rv;
                }
            }

            if (now < st->stuck_rev_until)
            {
                /* 卡边反向脱离期: 保持反向自转, 直到 STUCK_REV_MS 结束 */
                make_cmd(&cmd, "stuck-rev", -SPD_ROTATE_L, SPD_ROTATE_R, 0, 0);
                break;
            }
            /* 维护各通道波动窗口; 任一通道 窗口满 且 波动≤阈值 → 卡台边(或逻辑) */
            for (int i = 0; i < STUCK_IR_IDX_NUM; i++)
            {
                uint16_t lr = sensor->ir_normal[STUCK_IR_IDX[i]];
                if (lr < st->stuck_lr_min[i])
                    st->stuck_lr_min[i] = lr;
                if (lr > st->stuck_lr_max[i])
                    st->stuck_lr_max[i] = lr;
                if (now - st->stuck_window_start[i] >= STUCK_WINDOW_MS)
                {
                    if ((uint16_t)(st->stuck_lr_max[i] - st->stuck_lr_min[i]) <= STUCK_VARIATION)
                    {
                        /* 该通道窗口内波动很小 → 判定卡台边: 反向自转脱离, 并重开所有窗口 */
                        st->stuck_rev_until = now + STUCK_REV_MS;
                        for (int j = 0; j < STUCK_IR_IDX_NUM; j++)
                        {
                            st->stuck_window_start[j] = now;
                            st->stuck_lr_min[j] = st->stuck_lr_max[j] =
                                sensor->ir_normal[STUCK_IR_IDX[j]];
                        }
                        break; /* 检测到卡边, 跳出维护循环 */
                    }
                    /* 波动正常 → 该通道未卡边, 开新窗口继续扫描 */
                    st->stuck_window_start[i] = now;
                    st->stuck_lr_min[i] = st->stuck_lr_max[i] = lr;
                }
            }
            /* 刚判定卡边(上面循环设置了反向计时) → 输出反向自转 */
            if (now < st->stuck_rev_until)
            {
                make_cmd(&cmd, "stuck-rev", -SPD_ROTATE_L, SPD_ROTATE_R, 0, 0);
                break;
            }
            /* 还未对准: 自转继续扫描 */
            make_cmd(&cmd, "rotate-find", SPD_ROTATE_L , -SPD_ROTATE_R , 0, 0);
            break;

        /* ---- PRE_RUSH: 光电对准围栏后的反向冲刺(1.3s) ----
         * 方向与 approach 相反(负速=后退, 即冲台方向), 先冲刺一段再接近挡板。 */
        case CLIMB_PRE_RUSH:
            if (elapsed >= CLIMB_PRE_RUSH_MS)
            {
                st->climb_state = CLIMB_APPROACH;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "approach", SPD_APPROACH_L , SPD_APPROACH_R , 2000, 0);
            }
            else
            {
                make_cmd(&cmd, "pre-rush", SPD_PRE_RUSH_L , SPD_PRE_RUSH_R , 0, 0);
            }
            break;

        /* ---- APPROACH: 前进接近擂台壁(1.50s) ---- */
        case CLIMB_APPROACH:
            if (elapsed >= 1500)
            {
                st->climb_state = CLIMB_PAUSE;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "rush-pause", 0, 0, CLIMB_RUSH_PAUSE_MS, 0);
            }
            else
            {
                make_cmd(&cmd, "approach-go", SPD_APPROACH_L , SPD_APPROACH_R , 0, 0);
            }
            break;

        /* ---- PAUSE: 冲台前两轮停顿 250ms ----
         * 让两轮从低速回 0, 再从 0 速同步起步冲台, 复现软启动的起步条件,
         * 减小 CLIMB_RUSH 时左右轮启动时间差(低速直踩全速比静止起步差异更大)。 */
        case CLIMB_PAUSE:
            if (elapsed >= CLIMB_RUSH_PAUSE_MS)
            {
                st->climb_state = CLIMB_RUSH;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "rush", SPD_CLIMB_L, SPD_CLIMB_R, 1000, 0);
            }
            else
            {
                make_cmd(&cmd, "rush-pause", 0, 0, 0, 0);
            }
            break;

        /* ---- RUSH: 全速冲台(1.8s) ---- */
        case CLIMB_RUSH:
            if (elapsed >= 1800)
            {
                st->climb_state = CLIMB_SLOW_ADVANCE;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "slow-adv", 0, 0, 200, 0);
            }
            else
            {
                make_cmd(&cmd, "rush-go", SPD_CLIMB_L, SPD_CLIMB_R, 0, 0);
            }
            break;

        /* ---- SLOW_ADVANCE: 冲台后缓进稳定(0.2s) ---- */
        case CLIMB_SLOW_ADVANCE:
            if (elapsed >= 200)
            {
                st->climb_state = CLIMB_SETTLE;
                st->climb_state_start_ms = now;
                make_cmd(&cmd, "settle", 0, 0, 1500, 0);
            }
            else
            {
                make_cmd(&cmd, "slow-go", 0, 0, 0, 0);
            }
            break;

        /* ---- SETTLE: 等待车身稳定(1.5s), 灰度确认上台后切入战斗 ---- */
        case CLIMB_SETTLE:
            if (elapsed >= 1500)
            {
                st->climb_state = CLIMB_IDLE;
                /* 二次上台: 车身稳定后复用 force_climb step2 强制左转550ms靠拢台中间 */
                // st->force_climb = 1;
                // st->force_climb_step = 2;
                // st->force_climb_start = now;
                st->fc_stage_on_cnt = 0;
                st->fc_stage_off_cnt = 0;
                st->fc_turning = 0;
                make_cmd(&cmd, "settle-turn", 0, 0, 0, 0);
            }
            else
            {
                make_cmd(&cmd, "settle-wait", 0, 0, 0, 0);
            }
            break;
        }

        st->last_action_ms = now;
        return cmd;
    }

    /* ================================================================
     * 步骤 4: 台上 — 优先级仲裁(灰度防掉台已整体改为光电防掉台)
     *
     * 灰度在台上的唯一作用: 判定 in_edge_zone(前后任一路超 edge_thresh)。
     * 防掉台由四角光电(photo_fl/fr/rl/rr)全权承担, 与冒险模式是否开启无关:
     *   · 悬空触发即斜向躲避, 触发后至少退缩 500ms(回台仍沿用方向, 防台边往复);
     *   · 退缩中每帧按当前悬空组合重算方向, 组合一变立即改向(见 photo_escape_cmd)
     * ================================================================ */

    /* ---- 边缘区判断(多级共用): 前后灰度任一路超阈值 → 在边缘区 ---- */
    uint8_t in_edge_zone =
        (sensor->gray_normal[GRAY_FRONT] > g_thresh.edge_thresh[GRAY_FRONT]) ||
        (sensor->gray_normal[GRAY_REAR] > g_thresh.edge_thresh[GRAY_REAR]);

    /* ---- 冒险模式管理(入口条件保持原样) ---- */
    if (!st->adventure_active)
    {
        /* 条件②: 有限制区任务(有敌人 或 有可推能量块) */
        uint8_t has_task = (sensor->ir_enemy_mask & 0xC7); /* 仅前侧 bit0/1/2/6/7 */
        /* 条件③: 双前轮均在擂台上方(光电双安全, 任一悬空禁止进入) */
        uint8_t wheels_safe = sensor->photo_fl && sensor->photo_fr;
        /* 条件④: 防抖冷却期已过(防止悬空退出后立即重入振荡) */
        uint8_t cooldown_ok = (now - st->adventure_exit_ms) >= ADVENTURE_REENTRY_COOLDOWN_MS;
        if (in_edge_zone && has_task && wheels_safe && cooldown_ok)
        {
            st->adventure_active = 1;
        }
    }
    /* 退出判断: 任务完成 或 离开边缘区(防止冒险扩散到中场)。
     * 轮悬空触发 P1 逃逸时也会在此文件上层清掉 adventure_active。 */
    if (st->adventure_active)
    {
        uint8_t task_done = (sensor->ir_enemy_mask == 0);
        if (task_done || !in_edge_zone)
        {
            st->adventure_active = 0;
            st->adventure_exit_ms = now; /* 所有退出均记录时间戳(冷却重入) */
        }
    }

    /* --- 优先级 1: 四角光电防掉台(生存第一, 不再区分是否冒险) ---
     * 分三层, 命中即返回:
     *   ① 躲避(0 ~ ESCAPE_SPIN_TRIGGER_MS=400ms): 悬空触发 → 按当前悬空组合斜向躲避,
     *      且触发后至少退缩 ESCAPE_HOLD_MS(300ms):
     *        · 悬空期间: 每帧(5ms)按当前组合重算方向 —— 退缩途中若出现**新的**悬空
     *          组合(如后退时另一角也出去), 方向立即随之切换, 绝不盲退。
     *        · **这条"组合一变方向立刻变"只在本层有效**(即躲避的前 300ms);
     *          进入下面的旋转层后方向就定死了。
     *        · 全部回台后: 仍沿用最后方向退缩, 防止"轮子刚擦回台面、下一帧就
     *          前冲又压出去"的台边往复。
     *   ② 旋转(ESCAPE_SPIN_MS=300ms): 本轮躲避满 300ms 且**当帧没有任何光电悬空**
     *      (已脱险)时, 若本轮悬空组合是"仅左前 / 仅右前 / 双前"三者之一
     *      (见 photo_spin_cmd), 原地转 300ms 调转巡台方向。
     *      ⚠ 旋转**不是**"继续脱险": 躲避只是斜向退让, 本身不改变车头朝向 ——
     *        退让后车头还对着同一条台边, 下一帧又冲回去; 转个身把巡台方向换掉,
     *        图的是**脱险后不再原路撞回来**。所以仍有悬空时不许旋转, 继续躲避。
     *   ③ 旋转中: 每帧继续看光电 —— **一旦又出现悬空, 立刻打断旋转**,
     *      清掉旋转段并当帧重新起一轮躲避(重新计 300ms)。期间方向不再跟组合变。
     *   ④ 旋转结束: 若悬空 → 下帧重新起一轮躲避(重新计 300ms), 否则交还正常逻辑。
     * 全程持续清掉攻击/冒险残留, 防台边反复进出。 */
    {
        int16_t el = 0, er = 0;
        uint8_t spinning = 0; /* 本帧是否处于旋转层 */
        uint8_t hang = 0;     /* 本帧悬空掩码(旋转层也要看它) */
        if (!sensor->photo_fl)
            hang |= PHOTO_BIT_FL; /* 左前悬空 */
        if (!sensor->photo_fr)
            hang |= PHOTO_BIT_FR; /* 右前悬空 */
        if (!sensor->photo_rl)
            hang |= PHOTO_BIT_RL; /* 左后悬空 */
        if (!sensor->photo_rr)
            hang |= PHOTO_BIT_RR; /* 右后悬空 */

        /* 旋转层的两个退出条件(每帧都判, 本帧不 return —— 继续往下判躲避):
         *   · 计时到点 → 旋转结束;
         *   · 旋转中又出现悬空 → 立刻打断旋转, 掉回躲避层重新起一轮。 */
        if (st->photo_spin_until != 0 &&
            (now >= st->photo_spin_until || hang))
        {
            st->photo_spin_until = 0;
            st->photo_esc_active = 0;
        }
        if (st->photo_spin_until != 0)
        {
            spinning = 1;
        }

        if (!spinning)
        {
            if (hang)
            {
                /* 悬空: 按当前组合取方向(组合一变方向就变), 并重记退缩计时起点 */
                if (!st->photo_esc_active)
                {
                    /* 新一轮躲避的第一帧: "满 300ms 才旋转"以此刻起算。
                     * 注意 photo_esc_start_ms 是每帧刷新的"最近悬空帧",
                     * 判断 300ms 必须用这个不刷新的 photo_esc_begin_ms。 */
                    st->photo_esc_begin_ms = now;
                }
                photo_escape_cmd(hang, &el, &er);
                st->photo_esc_l = el;
                st->photo_esc_r = er;
                st->photo_esc_active = 1;
                st->photo_esc_start_ms = now;
                st->photo_esc_hang = hang; /* 本轮组合, 回台后仍保留着判旋转 */
            }
            else if (st->photo_esc_active)
            {
                /* 已回台(无悬空)。本轮躲避满 300ms 且组合属前侧三种 → 起旋转;
                 * 没满 / 组合不转 → 在 300ms 退缩窗口内沿用方向, 窗口过了就交还。 */
                int16_t sl = 0, sr = 0;
                if ((now - st->photo_esc_begin_ms) >= ESCAPE_SPIN_TRIGGER_MS &&
                    photo_spin_cmd(st->photo_esc_hang, &sl, &sr))
                {
                    st->photo_spin_l = sl;
                    st->photo_spin_r = sr;
                    st->photo_spin_until = now + ESCAPE_SPIN_MS;
                    spinning = 1;
                }
                else if ((now - st->photo_esc_start_ms) < ESCAPE_HOLD_MS)
                {
                    /* 退缩窗口内: 沿用上次方向继续退缩 */
                    el = st->photo_esc_l;
                    er = st->photo_esc_r;
                }
                else
                {
                    st->photo_esc_active = 0; /* 窗口已过 → 交还正常逻辑 */
                }
            }
        }

        if (spinning || st->photo_esc_active)
        {
            if (st->adventure_active)
            { /* 台边悬空/退缩/调头中 → 退出冒险 */
                st->adventure_active = 0;
                st->adventure_exit_ms = now;
            }
            st->attack1_active = 0;
            st->anti_circle_phase = 0;
            if (spinning)
            {
                make_cmd(&cmd, "photo-spin", st->photo_spin_l, st->photo_spin_r, 0, 0);
            }
            else
            {
                make_cmd(&cmd, "photo-escape", el, er, 0, 0);
            }
            st->last_action_ms = now;
            return cmd;
        }
    }

    /* --- 优先级 2: 躲避己方能量块(炸弹, 不违规) ---
     * 触发条件 = **本拍真的消费到一帧炸弹帧**(tag->bomb_frame), 而不是
     * "最近有帧到达"。bomb_frame 由 pi_comm.c 按帧 TYPE==PI_TAG_BOMB 生成,
     * 只维持一拍(边沿), 和帧的处理速度/新鲜度无关。
     *
     * 时序(一轮 = 一次性 BOMB_ESCAPE_MS=250ms 后退, 期间不续期; 后退躲满且非
     * "前进躲避"时, 再接一段 BOMB_TURN_MS=700ms 原地旋转):
     *   t=0        本拍收到炸弹帧, 且**整轮已结束**(没在躲也没在转) → 起一轮躲避
     *   0~250ms    期间再收到多少炸弹帧都**不续期**, 老老实实躲完 250ms
     *   t=250ms    本轮后退结束; 若非"前方躲避"则接 700ms 原地旋转(bomb_turn)
     *   t=950ms    旋转结束, 整轮才算走完; 之后的炸弹帧才会起下一轮
     *
     * ⚠ 起新一轮必须**同时**排除 bomb_turning, 不能只看 bomb_escape_active:
     *   炸弹帧是树莓派 50ms 一发持续来的, 而旋转段里 escape_active 恒为 0 —— 漏掉
     *   这个判断的话, 旋转刚转 4~5° 就被下一帧炸弹顶回后退、再从头计时, 表现出来
     *   就是"躲完 250ms 之后根本不转"
     *
     * 排在 P1 之后: 倒车途中任一角落悬空会被 P1 光电防掉台下帧立即接管,
     * 不会因此掉台。 */
    if (tag->bomb_frame && !st->bomb_escape_active && !st->bomb_turning)
    {
        /* 起躲瞬间感知台边: 后任一角光电悬空 → 后方是台边,
         * 本轮反转为前进躲避(escape-bomb-F), 防止朝台边一记后退冲出台沿 */
        uint8_t rear_danger =
            (!sensor->photo_rl) || (!sensor->photo_rr);
        st->bomb_escape_fwd = rear_danger;
        st->bomb_escape_active = 1;
        st->bomb_escape_start_ms = now;
    }

    if (st->bomb_escape_active)
    {
        if ((now - st->bomb_escape_start_ms) >= BOMB_ESCAPE_MS)
        {
            /* 后退躲满 → 接一段原地方向旋转(bomb-turn), 打破直线后退轨迹;
             * 前进躲避(起躲时后方是台边)不接旋转, 直接结束 */
            st->bomb_escape_active = 0;
            st->bomb_turning = !st->bomb_escape_fwd;
            st->bomb_escape_start_ms = now;
        }
        else
        {
            st->attack1_active = 0;
            if (st->bomb_escape_fwd)
            {
                make_cmd(&cmd, "escape-bomb-F", +BOMB_ESCAPE_BASE + 150, +BOMB_ESCAPE_BASE, 0, 0);
            }
            else
            {
                make_cmd(&cmd, "escape-bomb", -BOMB_ESCAPE_BASE - 150, -BOMB_ESCAPE_BASE, 0, 0);
            }
            st->last_action_ms = now;
            return cmd;
        }
    }

    if (st->bomb_turning)
    {
        if ((now - st->bomb_escape_start_ms) >= BOMB_TURN_MS)
        {
            st->bomb_turning = 0; /* 转满 BOMB_TURN_MS, 本轮躲避全部结束 */
        }
        else
        {
            st->attack1_active = 0;
            make_cmd(&cmd, "bomb-turn", +BOMB_TURN_SPD, -BOMB_TURN_SPD, 0, 0);
            st->last_action_ms = now;
            return cmd;
        }
    }
    /* --- 优先级 3: 攻敌 ---
     * 8 路 IR 命中敌人(高于 enemy_threshold) → 按最强方向行动:
     *   · 正前(0)      : 直线推进硬顶(3s 一轮)
     *   · 前侧(1/7)    : 原地旋转对准(转向方向带防抖)
     *   · 侧/后(2/6/3/4/5): 防绕柱状态机(后退打破同步 → 旋转面向)
     * 边缘区(in_edge_zone)内降速推进或原地转向, 避免高速冲出擂台。 */
    /* 正前贴脸扩展: 双前向红外(AD12/AD13)同时超阈值 → 视为正前目标,
     * 即使 8 路 IR 未超阈值也直接进攻。
     * ⚠ 2026-10 这两个口由反射式光电换成红外测距后, 这条判据的含义变了:
     *   光电时代它等价于"车头两边都贴到东西了"(距离≈0);
     *   现在它只是"两侧读数都 > CLIMB_SIDE_IR_L/R_THRESH", 而这组阈值是给**爬台对围栏**
     *   用的、不代表"贴脸"。若实车发现台上(巡台/攻敌时)误触发冲锋, 优先怀疑这里 ——
     *   离线标定后应给"贴脸"单独定一个更高的阈值, 而不是继续借用爬台那个。 */
    uint8_t front_close = (sensor->ir_climb_lf > CLIMB_SIDE_IR_L_THRESH &&
                           sensor->ir_climb_rf > CLIMB_SIDE_IR_R_THRESH);
    if (sensor->ir_enemy_mask || front_close)
    {
        uint8_t best_i = 0;
        uint32_t best_ratio = 0; /* 超出阈值的倍数 × 100，用整数避免浮点 */
        for (int i = 0; i < 8; i++)
        {
            if (sensor->ir_enemy_mask & (1 << i))
            {
                uint32_t ratio = (uint32_t)sensor->ir_fast[i] * 100 / g_thresh.enemy_threshold[i];
                if (ratio > best_ratio)
                {
                    best_ratio = ratio;
                    best_i = i;
                }
            }
        }

        /* 滞回: 上一帧的 best_i 仍然在位时, 新候选必须显著更强(>10%)才切换,
         * 防止相邻 IR 通道因噪声来回跳变导致 ±470/-470 反复翻转抽搐 */
        if (st->best_i_last_valid && best_i != st->best_i_last)
        {
            uint8_t old_still_present = (sensor->ir_enemy_mask & (1 << st->best_i_last)) != 0;
            if (old_still_present)
            {
                uint32_t old_ratio = (uint32_t)sensor->ir_fast[st->best_i_last] * 100 / g_thresh.enemy_threshold[st->best_i_last];
                if (best_ratio <= old_ratio * 110 / 100)
                {
                    best_i = st->best_i_last;
                    best_ratio = old_ratio;
                }
            }
        }
        st->best_i_last = best_i;
        st->best_i_last_ratio = best_ratio;
        st->best_i_last_valid = 1;
        if (best_i == 0 || front_close)
        {
            /* 正前方(0°): 直接攻击推进 */
            st->anti_circle_phase = 0;
            if (!st->attack1_active)
            {
                st->attack1_active = 1;
                st->attack_time_start_ms = now;
            }
            if ((now - st->attack_time_start_ms) >= ATTACK_DURATION_MS)
            {
                /* 攻击满 3s → 复位本轮攻击: 下帧重新起一轮 3s 直推
                 * (旧 design 的 attack2 摇摆堵截从未实现, 已随 attack2_sway 删除) */
                st->attack1_active = 0;
                st->attack_time_start_ms = now;
            }
            else
            {
                /* 光电红外触发 = 极近距离贴脸, 满速硬顶 */
                int16_t atk_spd;
                if (st->adventure_active)
                {
                    atk_spd = ADVENTURE_SPD; /* 冒险模式降速 */
                }
                else if (in_edge_zone)
                {
                    atk_spd = SPD_PATROL; /* 边缘区内降速，避免自掉 */
                }
                else
                {
                    atk_spd = SPD_ATTACK_L;
                }
                const char *desc_str = "attack";
                make_cmd(&cmd, desc_str, atk_spd, atk_spd, 300, 0);
            }
        }
        else if (best_i == 1 || best_i == 7)
        {
            /* 前侧±45°: 先调整身位，转向对准敌人，对准后自动进入正前攻击 */
            st->attack1_active = 0;
            /* 前侧±45°: 直接原地旋转对准，不后退
             * 转向方向防抖: 需连续15帧同向才切换, 防止 noise 导致 ±470/-470 翻转 */
            st->anti_circle_phase = 0;

            int16_t ang = wall_angle(best_i);
            int16_t turn_spd = SPD_ROTATE_L;
            int8_t des_dir = (ang < 0) ? -1 : 1; /* -1=左转 +1=右转 */

            if (st->turn_dir_last != 0 && des_dir != st->turn_dir_last)
            {
                st->turn_dir_same_cnt++;
                if (st->turn_dir_same_cnt < 15)
                {
                    des_dir = st->turn_dir_last; /* 未达阈值，保持旧方向 */
                }
                else
                {
                    st->turn_dir_last = des_dir; /* 连续15帧: 确认切换 */
                    st->turn_dir_same_cnt = 0;
                }
            }
            else if (des_dir == st->turn_dir_last)
            {
                st->turn_dir_same_cnt = 0;
            }
            else
            {
                st->turn_dir_last = des_dir; /* 首次初始化 */
            }

            if (des_dir < 0)
            {
                make_cmd(&cmd, "align-turn-L", -turn_spd - 100, turn_spd + 100, 0, 0);
            }
            else
            {
                make_cmd(&cmd, "align-turn-R", turn_spd + 100, -turn_spd - 100, 0, 0);
            }
            st->last_action_ms = now;
            return cmd;
        }
        else
        {
            /* 非正前/非前侧敌人
             * 后方(3/4/5): 先旋转到侧面，再走侧面逻辑
             * 侧面(2/6):   直接后退+旋转 */
            st->attack1_active = 0;
            /* ★ 边缘区不退，直接原地旋转面向敌人 */
            if (in_edge_zone)
            {
                /* 转向方向防抖(同上): 需连续15帧同向才切换 */
                st->anti_circle_phase = 0;
                int16_t ang = wall_angle(best_i);
                int16_t turn_spd = SPD_ROTATE_L;
                int8_t des_dir = (ang < 0) ? -1 : 1;

                if (st->turn_dir_last != 0 && des_dir != st->turn_dir_last)
                {
                    st->turn_dir_same_cnt++;
                    if (st->turn_dir_same_cnt < 15)
                    {
                        des_dir = st->turn_dir_last;
                    }
                    else
                    {
                        st->turn_dir_last = des_dir;
                        st->turn_dir_same_cnt = 0;
                    }
                }
                else if (des_dir == st->turn_dir_last)
                {
                    st->turn_dir_same_cnt = 0;
                }
                else
                {
                    st->turn_dir_last = des_dir;
                }

                if (des_dir < 0)
                {
                    make_cmd(&cmd, "edge-ac-turn-L", -turn_spd - 100, turn_spd + 100, 0, 0);
                }
                else
                {
                    make_cmd(&cmd, "edge-ac-turn-R", turn_spd + 100, -turn_spd - 100, 0, 0);
                }
                st->last_action_ms = now;
                return cmd;
            }

            uint8_t is_rear = (best_i == 3 || best_i == 4 || best_i == 5);

            /* 首次进入或方向变了 → 重新开始 */
            if (st->anti_circle_phase == 0)
            {
                st->anti_circle_target = best_i;
                st->anti_circle_start = now;
                if (is_rear)
                {
                    st->anti_circle_phase = 3; /* 后方: 先旋转到侧面 */
                }
                else
                {
                    st->anti_circle_phase = 1; /* 侧面: 直接后退 */
                }
            }

            uint32_t ac_elapsed = now - st->anti_circle_start;

            /* ===== 阶段3: 后方旋转到侧面 ===== */
            if (st->anti_circle_phase == 3)
            {
                if (ac_elapsed >= ANTI_CIRCLE_TURN_MS)
                {
                    st->anti_circle_phase = 0;
                    make_cmd(&cmd, "ac-timeout", SPD_PATROL, SPD_PATROL, 0, 1);
                }
                else
                {
                    /* 最短路径转到侧面: 索引3(右后)→右转到2, 索引5(左后)→左转到6, 索引4(正后)→右转 */
                    int16_t turn_spd = SPD_ROTATE_L_large;
                    if (st->anti_circle_target == 5)
                    {
                        make_cmd(&cmd, "ac-rear-to-side-L",
                                 -turn_spd, turn_spd, 0, 0);
                    }
                    else
                    {
                        make_cmd(&cmd, "ac-rear-to-side-R",
                                 turn_spd, -turn_spd, 0, 0);
                    }

                    /* 转到侧面(2或6) → 进入后退阶段，用归一化比值 */
                    uint32_t r2 = (sensor->ir_enemy_mask & (1 << 2))
                                      ? (uint32_t)sensor->ir_fast[2] * 100 / g_thresh.enemy_threshold[2]
                                      : 0;
                    uint32_t r6 = (sensor->ir_enemy_mask & (1 << 6))
                                      ? (uint32_t)sensor->ir_fast[6] * 100 / g_thresh.enemy_threshold[6]
                                      : 0;
                    if (r2 > 0 || r6 > 0)
                    {
                        st->anti_circle_phase = 1;
                        st->anti_circle_start = now;
                        st->anti_circle_target = (r2 >= r6) ? 2 : 6;
                    }
                    /* 敌人已进入前侧(1/7) → 跳到旋转对准阶段 */
                    else if (sensor->ir_enemy_mask & 0x82)
                    {
                        st->anti_circle_phase = 2;
                        st->anti_circle_start = now;
                    }
                    /* 敌人进入正前(0) → 退出，下帧正前攻击 */
                    else if (sensor->ir_enemy_mask & 0x01)
                    {
                        st->anti_circle_phase = 0;
                    }
                }
                st->last_action_ms = now;
                return cmd;
            }

            /* ===== 阶段1: 直线后退 ===== */
            if (st->anti_circle_phase == 1)
            {
                if (ac_elapsed >= ANTI_CIRCLE_BACK_MS)
                {
                    st->anti_circle_phase = 2;
                    st->anti_circle_start = now;
                }
                else
                {
                    make_cmd(&cmd, "anti-circle-bk",
                             -ANTI_CIRCLE_BACK_SPD, -ANTI_CIRCLE_BACK_SPD, 0, 0);
                    st->last_action_ms = now;
                    return cmd;
                }
            }

            /* ===== 阶段2: 旋转面向敌人 ===== */
            if (st->anti_circle_phase == 2)
            {
                /* 动态扫描当前敌人方向 */
                uint8_t cur_target = st->anti_circle_target;
                uint16_t cur_best = 0;
                for (int i = 0; i < 8; i++)
                {
                    if ((sensor->ir_enemy_mask & (1 << i)) && sensor->ir_fast[i] > cur_best)
                    {
                        cur_best = sensor->ir_fast[i];
                        cur_target = i;
                    }
                }

                if (!sensor->ir_enemy_mask)
                {
                    st->anti_circle_phase = 0;
                    make_cmd(&cmd, "ac-no-enemy", SPD_PATROL, SPD_PATROL, 0, 1);
                }
                else if (ac_elapsed >= ANTI_CIRCLE_TURN_MS)
                {
                    st->anti_circle_phase = 0;
                    make_cmd(&cmd, "ac-timeout", SPD_PATROL, SPD_PATROL, 0, 1);
                }
                else
                {
                    int16_t ang = wall_angle(cur_target);
                    int16_t turn_spd = (cur_target == 3 || cur_target == 4 || cur_target == 5)
                                           ? SPD_ROTATE_L_large
                                           : SPD_ROTATE_L;
                    if (ang < 0)
                    {
                        make_cmd(&cmd, "ac-turn-L", -turn_spd - 100, turn_spd + 100, 0, 0);
                    }
                    else
                    {
                        make_cmd(&cmd, "ac-turn-R", turn_spd + 100, -turn_spd - 100, 0, 0);
                    }

                    if (cur_target == 0)
                    {
                        st->anti_circle_phase = 0;
                    }
                }
                st->last_action_ms = now;
                return cmd;
            }
        }
        st->last_action_ms = now;
        return cmd;
    }
    /* --- 优先级 3.5: 增益能量块追踪(空闲加分任务) ---
     * 比赛规则: 把增益块撞下擂台己方加分。只在"没有其他任务"时执行 —
     * 排在攻敌(P3)之后、巡台(P4)之前: 有敌人打敌人, 没敌人才去撞块。
     * 无记忆电平逻辑: 本帧视觉报 BOOST 且帧新鲜(≤PI_TIMEOUT_MS)就追踪,
     * 下一帧看不到立即落回巡台, 不保存任何追踪状态(P1 防掉台天然兜底:
     * 追踪撞块冲到台边时, 四角光电悬空会立即接管). */
    // if (tag != NULL && tag->valid && tag->type == PI_TAG_BOOST &&
    //     (now - tag->last_update_ms) <= PI_TIMEOUT_MS)
    // {
    //     if (tag->rel_x > BOOST_DEADZONE)
    //     {
    //         /* 目标偏右 → 原地右转对准(左>右=右转) */
    //         make_cmd(&cmd, "boost-turn-R", BOOST_TURN_SPD, -BOOST_TURN_SPD, 0, 0);
    //     }
    //     else if (tag->rel_x < -BOOST_DEADZONE)
    //     {
    //         /* 目标偏左 → 原地左转对准(右>左=左转) */
    //         make_cmd(&cmd, "boost-turn-L", -BOOST_TURN_SPD, BOOST_TURN_SPD, 0, 0);
    //     }
    //     else
    //     {
    //         /* 已对准 → 直进撞击 */
    //         make_cmd(&cmd, "boost-charge", BOOST_CHASE_SPD, BOOST_CHASE_SPD, 0, 0);
    //     }
    //     st->last_action_ms = now;
    //     return cmd;
    // }
    /* --- 优先级 4: 直线巡台(默认行为) ---
     * 简单直线前进, 直到被更高优先级动作打断(P1光电防掉台 / 炸弹躲避 / 攻敌
     * / 增益块追踪)。
     * 巡台中若任一角落悬空, 下一帧 P1 光电防掉台立即接管, 无需巡台自己避边。
     * (走到这里说明无任务、无敌情、无增益块目标、全轮安全) */
    make_cmd(&cmd, "patrol", SPD_PATROL, SPD_PATROL, 0, 1);
    st->last_action_ms = now;
    return cmd;
}
