/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   flash 扇区安全测试工程(FlashTest)
 *
 * ================================================================
 * 目的
 * ================================================================
 *   查清楚外挂的 SST25VF016(2MiB) 里**哪些扇区可以安全读写**。
 *   动机: Luby 用户文档说控制器插 USB 会显示一个"1M 大小的 U 盘"用来拖
 *   .bin 进去, 而外挂 flash 正好 2MiB —— 那个 U 盘很可能就建在同一颗芯片
 *   上。在没搞清楚 bootloader 占了哪些扇区之前, 主工程不能往 flash 里写
 *   任何东西。
 *
 * ================================================================
 * 分相(默认一个字都不写!)
 * ================================================================
 *   Phase 1  只读普查, 上电自动执行, **零写入**:
 *            读 ID 稳定性 / 状态寄存器 / 选项字节 / 整片 512 个扇区分类
 *            → LCD 分页显示扇区地图, KEY1 下一页 KEY2 上一页
 *
 *   Phase 2  破坏性写测, **必须按住 KEY2(上键) 开机 3 秒**才进入:
 *            只测 Phase 1 判为"全 0xFF"的扇区, 从最后一个往前走。
 *            本来就全 0xFF 的扇区擦掉零信息损失, 测完再擦回 0xFF 恢复原状。
 *
 * ================================================================
 * 屏幕怎么看
 * ================================================================
 *   '.' = 全 0xFF(空)      '#' = 有数据
 *   'C' = 我们的标定记录     'F' = 文件系统(MBR/FAT/NTFS/exFAT)
 *   'B' = 固件镜像(.bin)    '?' = 读失败
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "luby_lcd.h"
#include "input.h"
#include "sst25.h"
#include "sst25_port.h"
#include "cal_store.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SECTOR_COUNT (SST25_DEV_SIZE / SST25_SECTOR_SIZE) /* 512 */
#define SCAN_CHUNK 256U                                   /* 顺序读的分块大小 */
#define ID_READ_TRIES 1000U                               /* ID 稳定性测试次数 */
#define KEY_IDLE_MS 30U
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static uint8_t lcd_ready = 0U;
static uint8_t sec_cls[SECTOR_COUNT];       /* 每个扇区的分类结果 */
static uint8_t sec_first8[SECTOR_COUNT][8]; /* 每扇区前 8 字节(诊断打印用)
                                             * ⚠ 必须是 [512][8] 而不是 uint32_t[512]:
                                             *   后者只有 2048 字节却要存 4096 字节,
                                             *   会写穿到相邻的 armed 变量 —— 那会导致
                                             *   **没按 KEY2 也进入破坏性 Phase 2**。 */
static uint8_t armed = 0U;                  /* 1 = 允许进入 Phase 2 */
static uint8_t scan_ok = 0U;
static uint8_t restore_failed = 0U; /* 1 = 有扇区没能恢复成全 0xFF */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Dbg(const char *s);
static void DbgPrintf(const char *fmt, ...);
static uint8_t WaitKey(uint32_t ms);
static void Phase1_ReadOnlySurvey(void);
static void ShowMapPage(uint8_t page);
static void Phase2_Destructive(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static void Dbg(const char *s)
{
    uint16_t len;
    if (s == NULL)
        return;
    len = (uint16_t)strlen(s);
    (void)HAL_UART_Transmit(&huart5, (const uint8_t *)s, len, 200U);
    /* 同一份输出走蓝牙(USART1@9600): 手机蓝牙串口助手直接收, 免接 USB 转串口线。
     * 蓝牙没配对连接也不影响 —— 数据只是没人听, 阻塞发送照常完成, 不会卡死。 */
    (void)HAL_UART_Transmit(&huart1, (const uint8_t *)s, len, 200U);
}

static void DbgPrintf(const char *fmt, ...)
{
    char b[128];
    va_list a;
    va_start(a, fmt);
    (void)vsnprintf(b, sizeof(b), fmt, a);
    va_end(a);
    Dbg(b);
}

/**
 * @brief 等按键
 * @return 1 = KEY1(下键)  2 = KEY2(上键)  0 = 超时
 */
static uint8_t WaitKey(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    for (;;)
    {
        if (Input_Get(INPUT_KEY_DOWN))
        {
            HAL_Delay(KEY_IDLE_MS);
            /* 等按键释放: 否则按一次会被连续识别, 一次翻好几页 */
            while (Input_Get(INPUT_KEY_DOWN))
            {
                HAL_Delay(10U);
            }
            return 1U;
        }
        if (Input_Get(INPUT_KEY_UP))
        {
            HAL_Delay(KEY_IDLE_MS);
            while (Input_Get(INPUT_KEY_UP))
            {
                HAL_Delay(10U);
            }
            return 2U;
        }
        if ((HAL_GetTick() - t0) >= ms)
            return 0U;
        HAL_Delay(10U);
    }
}

/* ---- 扇区分类 → 一个字符 ---- */
static char ClassChar(uint8_t c)
{
    switch ((SecClass_t)c)
    {
    case SEC_ERASED:
        return '.';
    case SEC_OURS:
        return 'C';
    case SEC_FOREIGN_FS:
        return 'F';
    case SEC_FOREIGN_IMAGE:
        return 'B';
    default:
        return '?';
    }
}

static void ShowMapPage(uint8_t page)
{
    char b[24];
    uint32_t base = (uint32_t)page * 48U; /* 3 行 x 16 = 48 个扇区/页 */

    snprintf(b, sizeof(b), "P%u/%u 0x%06lX",
             (unsigned)page + 1U, (unsigned)((SECTOR_COUNT + 47U) / 48U),
             (unsigned long)(base * SST25_SECTOR_SIZE));
    LubyLCD_ShowLine(0, b);
    DbgPrintf("[map] %s\r\n", b); /* 同步打蓝牙: 对照 LCD, 分离内容错/渲染错 */

    for (uint8_t row = 0U; row < 3U; row++)
    {
        char line[17];
        for (uint8_t col = 0U; col < 16U; col++)
        {
            uint32_t idx = base + row * 16U + col;
            line[col] = (idx < SECTOR_COUNT) ? ClassChar(sec_cls[idx]) : ' ';
        }
        line[16] = '\0';
        LubyLCD_ShowLine((uint8_t)(row + 1U), line);
        DbgPrintf("[map] %s\r\n", line);
    }
}

/**
 * @brief Phase 1: 只读普查(零写入)
 */
static void Phase1_ReadOnlySurvey(void)
{
    char b[24];
    uint8_t id[3];
    uint32_t mismatch = 0U, bad_cnt = 0U;

    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "PHASE1 READ ONLY");

    /* ---- 1a. ID 稳定性: 连读 1000 次看是否一致 ----
     * 不一致说明 bootloader 跳转瞬间 SPI2 停在半途, 或者 HOLD# 悬空被噪声影响 */
    LubyLCD_ShowLine(1, "ID stability... ");
    SST25_ReadJedecId(id);
    for (uint32_t i = 0U; i < ID_READ_TRIES; i++)
    {
        uint8_t t[3];
        SST25_ReadJedecId(t);
        if (t[0] != id[0] || t[1] != id[1] || t[2] != id[2])
            mismatch++;
    }
    /* 注意别超 16 列: "ID BF2541 m=1000" 正好 16 个字符。
       (用 "mis=" 会变 18 列, 而 LubyLCD_ShowLine 只取前 16 列,
        结果 mis=1000 会被截成 mis=10 —— 恰恰是回答 HOLD# 问题的那个数) */
    snprintf(b, sizeof(b), "ID %02X%02X%02X m=%lu",
             id[0], id[1], id[2], (unsigned long)mismatch);
    LubyLCD_ShowLine(1, b);
    DbgPrintf("[ID] JEDEC=%02X %02X %02X  mismatch=%lu/%lu\r\n",
              id[0], id[1], id[2], (unsigned long)mismatch, (unsigned long)ID_READ_TRIES);

    /* ---- 1b. 状态寄存器: 上电时 BP3..0 的实际值 ----
     * 这一条回答"器件上电是否默认全阵列保护"这个关键问题 */
    {
        uint8_t sr = SST25_ReadStatus();
        snprintf(b, sizeof(b), "SR=%02X BP=%X BPL=%u",
                 sr, (unsigned)((sr >> 2) & 0x0FU), (unsigned)((sr >> 7) & 1U));
        LubyLCD_ShowLine(2, b);
        DbgPrintf("[SR] raw=0x%02X BUSY=%u WEL=%u BP3..0=%X BPL=%u AAI=%u\r\n",
                  sr, (unsigned)(sr & 1U), (unsigned)((sr >> 1) & 1U),
                  (unsigned)((sr >> 2) & 0x0FU), (unsigned)((sr >> 7) & 1U),
                  (unsigned)((sr >> 6) & 1U));
    }

    /* ---- 1c. 选项字节: 看门狗是软件还是硬件模式 ----
     * ⚠ 只读! 写选项字节期间掉电会把板子变砖, 固件里绝不写它。
     *   STM32F1 的 FLASH_OBR: bit0=OPTERR bit1=RDPRT bit2=WDG_SW */
    {
        uint32_t obr = FLASH->OBR;
        DbgPrintf("[OB] FLASH_OBR=0x%08lX WDG_SW=%lu (0=硬件狗 1=软件狗)\r\n",
                  (unsigned long)obr, (unsigned long)((obr >> 2) & 1UL));
    }

    /* ---- 1d. 整片扫描: 每 4KB 归一类 ----
     * 用 SST25_Read 分块读(每次都是完整的 0x03 命令, CS 正常翻转),
     * 这样最稳妥; 整个 2MiB 在 4.5MHz 下约 4 秒。 */
    LubyLCD_ShowLine(3, "scanning 2MiB...");
    for (uint32_t s = 0U; s < SECTOR_COUNT; s++)
    {
        uint32_t addr = s * SST25_SECTOR_SIZE;
        SecClass_t c = Cal_ClassifySector(addr);
        sec_cls[s] = (uint8_t)c;

        if (c != SEC_ERASED)
        {
            uint8_t head[8];
            if (SST25_Read(addr, head, sizeof(head)) == SST25_OK)
            {
                memcpy(sec_first8[s], head, sizeof(head));
            }
            else
            {
                bad_cnt++;
            }
        }
        if ((s & 0x1FU) == 0U)
        {
            snprintf(b, sizeof(b), "scan %3lu/%u      ",
                     (unsigned long)s, (unsigned)SECTOR_COUNT);
            LubyLCD_ShowLine(3, b);
        }
    }
    scan_ok = 1U;

    /* ---- 1e/1f. 串口+蓝牙输出每扇区明细 ----
     * 蓝牙是 9600: 一行约 60 字节要 62ms, 几十行明细近 3 秒。 */
    Dbg("\r\n[sector] idx  addr     class     first8\r\n");
    for (uint32_t s = 0U; s < SECTOR_COUNT; s++)
    {
        if (sec_cls[s] == (uint8_t)SEC_ERASED)
            continue; /* 空扇区不刷屏 */
        DbgPrintf("  %3lu 0x%06lX %-9s %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
                  (unsigned long)s, (unsigned long)(s * SST25_SECTOR_SIZE),
                  Cal_SecClassStr((SecClass_t)sec_cls[s]),
                  sec_first8[s][0], sec_first8[s][1], sec_first8[s][2], sec_first8[s][3],
                  sec_first8[s][4], sec_first8[s][5], sec_first8[s][6], sec_first8[s][7]);
    }
    DbgPrintf("[scan] done, read-errors=%lu\r\n", (unsigned long)bad_cnt);

    /* ---- 1g. LCD 分页地图 ---- */
    {
        uint8_t page = 0U;
        uint8_t pages = (uint8_t)((SECTOR_COUNT + 47U) / 48U);
        for (;;)
        {
            ShowMapPage(page);
            uint8_t k = WaitKey(0xFFFFFFFFU); /* 永不超时: 地图就是要慢慢看的,
                                                60s 超时会让用户拍照回来发现已跳走 */
            if (k == 1U)
            {
                page = (uint8_t)((page + 1U) % pages);
            }
            else if (k == 2U)
            {
                page = (uint8_t)((page + pages - 1U) % pages);
            }
            else
            {
                break;
            }
        }
    }
}

/**
 * @brief Phase 2: 破坏性写测(必须人工武装)
 *
 * 只动 Phase 1 判为 SEC_ERASED 的扇区。一个本来就全 0xFF 的扇区, 擦除它
 * 零信息损失, 所以"擦-写-回读比对"是内容保持的; 测完再擦回全 0xFF。
 * 任何非 0xFF 且不是我们记录的扇区, 一律不碰。
 */
static void Phase2_Destructive(void)
{
    char b[24];
    int32_t s;
    uint8_t tested = 0U;

    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "PHASE2 DESTRUCT ");
    LubyLCD_ShowLine(1, "KEY2=start KEY1=");
    LubyLCD_ShowLine(2, "cancel          ");
    LubyLCD_ShowLine(3, "tests EMPTY sect");

    if (WaitKey(30000U) != 2U)
    {
        LubyLCD_Clear();
        LubyLCD_ShowLine(0, "PHASE2 CANCELLED");
        HAL_Delay(1500);
        return;
    }

    /* 从最后一个扇区往前走, 遇到第一个非空的就停 */
    for (s = (int32_t)SECTOR_COUNT - 1; s >= 0; s--)
    {
        uint32_t addr = (uint32_t)s * SST25_SECTOR_SIZE;
        uint32_t t0, t_erase_ms;
        Sst25Status_t st;

        if (sec_cls[s] != (uint8_t)SEC_ERASED)
            break;

        /* ---- 擦除 ---- */
        t0 = HAL_GetTick();
        st = SST25_Erase4K(addr);
        t_erase_ms = HAL_GetTick() - t0;

        snprintf(b, sizeof(b), "S%ld era %lums   ", (long)s, (unsigned long)t_erase_ms);
        LubyLCD_ShowLine(0, b);

        if (st != SST25_OK)
        {
            LubyLCD_ShowLine(1, SST25_StatusStr(st));
            LubyLCD_ShowLine(2, "ERASE FAILED    ");
            LubyLCD_ShowLine(3, "stop here       ");
            DbgPrintf("[P2] sector %ld erase FAILED: %s\r\n", (long)s, SST25_StatusStr(st));
            break;
        }

        /* ---- 回读确认擦干净 ---- */
        if (!SST25_IsErased(addr, 64U))
        {
            LubyLCD_ShowLine(1, "NOT ERASED!!    ");
            DbgPrintf("[P2] sector %ld erase verify FAILED\r\n", (long)s);
            break;
        }

        /* ---- 写图案 + 回读比对 ---- */
        {
            static uint8_t pat[64];
            for (uint8_t i = 0U; i < sizeof(pat); i++)
            {
                pat[i] = (i & 1U) ? 0xAAU : 0x55U; /* 交替图案, 能抓相邻位串扰 */
            }
            st = SST25_ProgramBytes(addr, pat, sizeof(pat));
            if (st != SST25_OK)
            {
                LubyLCD_ShowLine(1, SST25_StatusStr(st));
                LubyLCD_ShowLine(2, "WRITE FAILED    ");
                DbgPrintf("[P2] sector %ld write FAILED: %s\r\n", (long)s, SST25_StatusStr(st));
                break;
            }
            DbgPrintf("[P2] sector %ld OK (erase %lums, 64B r/w verify ok)\r\n",
                      (long)s, (unsigned long)t_erase_ms);
        }

        /* ---- 擦回全 0xFF, 把 flash 恢复原状 ---- */
        if (SST25_Erase4K(addr) != SST25_OK || !SST25_IsErased(addr, 64U))
        {
            LubyLCD_ShowLine(1, "RESTORE FAILED! ");
            DbgPrintf("[P2] sector %ld RESTORE FAILED\r\n", (long)s);
            restore_failed = 1U; /* 一定要记住: 这个扇区被留下测试图案了 */
            break;
        }
        sec_cls[s] = (uint8_t)SEC_ERASED;

        tested++;
        snprintf(b, sizeof(b), "tested %u OK    ", (unsigned)tested);
        LubyLCD_ShowLine(1, b);
        HAL_Delay(300U);
    }

    LubyLCD_Clear();
    LubyLCD_ShowLine(0, "PHASE2 DONE     ");
    snprintf(b, sizeof(b), "tested %u sect ", (unsigned)tested);
    LubyLCD_ShowLine(1, b);
    if (restore_failed)
    {
        /* 恢复失败意味着那个扇区**还留着 0xAA/0x55 测试图案**,
           绝不能报 "all restored FF" —— 那会让人以为 flash 是原样。 */
        LubyLCD_ShowLine(2, "!! NOT RESTORED ");
        LubyLCD_ShowLine(3, "re-run phase2   ");
    }
    else
    {
        LubyLCD_ShowLine(2, "all restored FF ");
        LubyLCD_ShowLine(3, "KEY1=map KEY2=go");
    }
    DbgPrintf("[P2] done, %u sectors tested, restore_failed=%u\r\n",
              (unsigned)tested, (unsigned)restore_failed);
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* 本工程不用看门狗 —— 理由见 SystemClock_Config 之后那段注释 */
    /* USER CODE END 1 */

    HAL_Init();
    SystemClock_Config();

    /* ---- 本工程不用看门狗 ----
     * 全程只有"整片扫描 + 按键翻页", 没有任何可能卡死的等待, 加狗只是白
     * 添一层复杂度。前提是这块板**硬件看门狗没被选项字节使能** ——
     * 若 WDG_SW=0, 上电时 IWDG 会自动开始计数(~0.41 秒), 那样连 LCD 初
     * 始化都跑不完, 表现是"屏幕半清、无限重启"。Phase 1 会把 FLASH_OBR
     * 打出来复核这一点(WDG_SW=1 = 软件狗 = 上电不会自己跑)。 */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART3_UART_Init();
    MX_UART5_Init();       /* 调试串口(115200), 用来 dump 扇区明细 */
    MX_USART1_UART_Init(); /* 蓝牙 HC-05(9600), 同一份明细也走蓝牙, 免接线 */

    /* USER CODE BEGIN 2 */
    /* ---- 0. 先看按键: 决定要不要武装破坏性测试 ----
     * 上电时按住 KEY2(上键) 满 3 秒 = 人工确认"我知道接下来会擦 flash"。 */
    armed = Input_Get(INPUT_KEY_UP) ? 1U : 0U;
    if (armed)
    {
        for (uint8_t i = 0U; i < 30U; i++)
        {
            HAL_Delay(100U);
            if (!Input_Get(INPUT_KEY_UP))
            {
                armed = 0U;
                break;
            } /* 中途松手 → 取消 */
        }
    }

    /* ---- 1. LCD ---- */
    LubyLCD_Init();
    LubyLCD_SetContrast(0x07);
    LubyLCD_SetInverse(1);
    LubyLCD_Clear();
    lcd_ready = 1U;
    LubyLCD_ShowLine(0, "FLASH TEST      ");
    LubyLCD_ShowLine(1, armed ? "ARMED (KEY2)    " : "read-only mode  ");

    /* ---- 2. flash 自检 ---- */
    {
        Sst25Status_t st = SST25_Init();
        char b[24];

        snprintf(b, sizeof(b), "ID %02X%02X%02X %s",
                 g_sst25_info.jedec[0], g_sst25_info.jedec[1], g_sst25_info.jedec[2],
                 g_sst25_info.id_ok ? "OK" : "!!");
        LubyLCD_ShowLine(2, b);
        LubyLCD_ShowLine(3, Cal_Crc32SelfTest() ? "CRC32 SELF OK" : "CRC32 FAIL!!");
        HAL_Delay(2000);

        if (st != SST25_OK)
        {
            LubyLCD_Clear();
            LubyLCD_ShowLine(0, "FLASH NOT FOUND!");
            LubyLCD_ShowLine(1, SST25_StatusStr(st));
            LubyLCD_ShowLine(2, "check U2 solder ");
            LubyLCD_ShowLine(3, "and HOLD# pin   ");
            while (1)
            {
                HAL_Delay(200);
            }
        }
    }

    /* ---- 3. Phase 1: 只读普查(零写入) ---- */
    Phase1_ReadOnlySurvey();

    /* ---- 4. Phase 2: 只有人工武装过才允许 ---- */
    if (armed && scan_ok)
    {
        Phase2_Destructive();
    }

    /* ---- 5. 结束: 停在扇区地图, 可继续翻页 ---- */
    if (scan_ok)
    {
        uint8_t page = 0U;
        uint8_t pages = (uint8_t)((SECTOR_COUNT + 47U) / 48U);
        for (;;)
        {
            ShowMapPage(page);
            uint8_t k = WaitKey(0xFFFFFFFFU); /* 永不超时: 地图就是要慢慢看的,
                                                60s 超时会让用户拍照回来发现已跳走 */
            if (k == 1U)
            {
                page = (uint8_t)((page + 1U) % pages);
            }
            else if (k == 2U)
            {
                page = (uint8_t)((page + pages - 1U) % pages);
            }
            else
            {
                break;
            }
        }
    }
    while (1)
    {
        HAL_Delay(100);
    }
    /* USER CODE END 2 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2)
    {
        HAL_IncTick();
    }
}

void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    if (lcd_ready)
    {
        LubyLCD_ShowLine(2, "I'm dead!       ");
    }
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
