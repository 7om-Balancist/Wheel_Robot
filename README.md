# 轮式格斗机器人 · 控制器端固件

中国机器人大赛轮式格斗赛项的控制器侧固件。机器人需要**自主冲上擂台、找到对手、把对手顶下擂台，同时保证自己不掉台**。

整机由两块算力组成：

- **控制器（本仓库）**：STM32F103 上跑 200Hz 实时控制回路，负责传感器采集、决策、双轮驱动、故障兜底与参数存储；
- **视觉（另一分支 `Raspberry_Pi-5_Vision`）**：树莓派 5 上跑 YOLO11，只负责"看到了什么能量块"，通过串口把 12 字节结果旁路注入控制器。

本分支 `Car_Controller` 包含控制器侧的三套固件 + 两套 PC 侧辅助工具。

---

## 硬件平台

| 项 | 型号 / 参数 |
|---|---|
| 主控 | STM32F103VCT6（Cortex-M3 @ 72MHz，256KB Flash / 48KB RAM） |
| 底板 | LUBY 控制器（厂商官方板，本仓库目录名 `luby端` 沿用此叫法） |
| 执行器 | 2 × CDS55xx 总线舵机，工作在**轮模式**，半双工总线 1 Mbps |
| 传感器 | 2 路灰度（台上/边缘判定）、8 路红外测距（敌人 + 围栏，每 45° 一路）、4 路防掉台光电（四角）、2 路爬台对准红外 |
| 配置存储 | 外挂 SST25VF016（2 MiB SPI NOR），A/B 双槽 + CRC32 保存标定参数 |
| 显示 | RA8816 类 64×16 点阵 LCD（6 根 GPIO 软件 SPI） |
| 调试 | HC-05 蓝牙 USART1 115200（命令下行 + 遥测上行） |
| 视觉链路 | UART5 115200，接收树莓派 5 的 12 字节定长帧 |
| 看门狗 | IWDG（÷64、RLR=3124 → 标称 5s，按 LSI 最坏值实际 3.33~6.67s） |

---

## 目录结构

```
luby端/            主固件（FreeRTOS，飞行固件）
  Core/            应用与驱动
    Src/           main.c / freertos.c / decision.c / sensors.c / luby_adc.c
                   drive.c / cds55xx.c / pi_comm.c / bluetooth.c / zigbee.c
                   luby_lcd.c / input.c / key_event.c / uart_debug.c
                   sst25.c / sst25_port.c / cal_store.c / calibrate.c / iwdg.c
                   adc.c / dma.c / usart.c / gpio.c / callbacks.c
    Inc/           对应头文件（接口契约都在这里）
  Drivers/         STM32F1 HAL + CMSIS（ST 原版）
  Middlewares/     FreeRTOS 10.3.1 + CMSIS-RTOS2 + heap_4
  hosttest/        PC 侧单元测试（mock flash + 按字节注入断电）
  tools/           syntaxcheck：无 ARM 工具链时的宿主机静态检查（含 canary 砝码）
  .cproject/.project/.mxproject   STM32CubeIDE 工程文件
  CMakeLists.txt   CLion / 命令行 CMake 构建
  STM32F103VCTX_FLASH.ld          链接脚本

CalibTool/         灰度标定固件（裸机）—— 全项目**唯一**能写 flash 的固件
FlashTest/         外部 flash 扇区普查固件（裸机）—— 回答"哪些扇区可以安全擦写"
```

三个工程共用同一批模块（`sst25` / `sst25_port` / `cal_store`），但**副本内容不同**，这是刻意的：

| 模块 | `luby端`（主固件） | `CalibTool` / `FlashTest`（工具固件） |
|---|---|---|
| `sst25.c/h` | **只读子集**：只定义 WRDI/RDSR/READ/JEDEC_ID 四个操作码，没有擦除与编程函数 | 完整：含 WREN / BYTE_PROGRAM / SECTOR_ERASE / 解块保护 |
| `cal_store.c` | 只有 `CalStore_Load` + CRC + 校验 | 含 `CalStore_Save` + 扇区内容分类 `Cal_ClassifySector` |
| `iwdg.c` | 有（IWDG 位图喂狗） | 无（工具固件全程顺序执行，不需要） |
| 擦写地址窗口 | 无写路径 | 主窗口 `0x1FE000..0x200000`；FlashTest 放宽到整片 |

**主固件里不存在任何能改动 flash 的代码，这让"比赛中把标定写坏"在编译期就不可能发生。**

---

## 主固件关键设计

### 1. 传感器数据链：16 路 ADC → DMA 双缓冲 → 多窗口滤波

16 路信号全部接 ADC1，扫描 + 连续转换 + 右对齐，采样时间取最慢档 239.5 周期：

- ADC 时钟 12 MHz（PCLK2÷6），单通道 21 µs，**一轮 16 通道 336 µs**，每通道 2.98 kS/s；
- `DMA1_Channel1` 循环模式，缓冲 `uint16_t adc_dma_buffer[2][16]`，**长度 32 个半字**——2 轮扫描，
  使得半满/全满中断的边界正好落在扫描边界上，**任一 bank 的 16 个值必然来自同一轮扫描**；
- 读取侧零锁：*"已完成的那一半，就是 DMA 当前没在写的那一半"*，任务只做 32 字节 memcpy 拿私有副本。

> ⚠ **注意一个隐蔽约定**：原理图的网络名 `ADC0..ADC15` **不等于** STM32 通道号（例如网络名 ADC0 实际是
> PB1 = ADC_IN9）。`Core/Src/adc.c` 里的 Rank 顺序是**故意置换**过的，目的就是让 DMA 缓冲下标 `i`
> 恰好等于网络名 `ADCi`，`sensors.c` 才敢用直线映射。**若用 CubeMX 重新生成代码并按引脚顺序重排 Rank，
> 所有传感器语义会整体错位，而编译零错误、运行不崩溃。**

滤波在 `Core/Src/sensors.c`，7 组滑动窗口（环形缓冲 + 32 位滚动求和，O(1)/帧、无浮点）：

| 数据 | 窗口 | 用途 |
|---|---|---|
| 灰度 slow | 16 帧 / 80 ms | 台上/台下判定（强调平滑） |
| 灰度 normal | 4 帧 / 20 ms | 边缘区判定（要响应快） |
| IR fast | 2 帧 / 10 ms | 敌人快检 |
| IR normal | 4 帧 / 20 ms | 围栏 + 敌人确认 |
| 正前 IR、爬台侧向 IR | 5 帧 / 25 ms | 爬台对准（求钝，防抖动误判） |

### 2. 实时任务与同步

三个任务，按**时间尺度**而不是功能划分：

| 任务 | 优先级（CMSIS 数值） | 栈 | 节拍 | 职责 |
|---|---|---|---|---|
| `SenseDecideTask` | High（40） | 2048 B | 5 ms → 200 Hz | ADC → 滤波 → 消费视觉帧 → 决策 → 电机输出 → 快照 |
| `defaultTask` | Normal（24） | 512 B | 1 ms | 只做看门狗"报到" |
| `DisplayTask` | Low（8） | 2048 B | 10 ms，每 750 ms 上报 | LCD 4 行 + 蓝牙遥测（只读快照） |

**整个应用层没有使用任何信号量 / 互斥量 / 队列**，因为三个共享点各自都有不需要锁的结构：

- **中断 → 任务**：单写者单读者环缓冲（深度 3，实际可用 2 帧）。写侧先算下一个位置、判断"不追尾读者"、
  填完字段**最后**才推进写指针（发布指针），所以读者永远看不到半成品；满了直接丢新帧，绝不阻塞中断。
- **控制 → 显示**：快照结构 + 临界区整块拷贝。把 LCD（位带写 24~37 ms）与蓝牙（数十 ms）的阻塞
  **彻底隔离在控制路径之外**，控制周期因此恒为 5 ms。
- **跨上下文标量**：`volatile` + Cortex-M3 对齐访问的原子性（手动模式速度、看门狗位图、复位原因）。

临界区用 `taskENTER_CRITICAL()`（Cortex-M3 移植上是 `BASEPRI = configMAX_SYSCALL_INTERRUPT_PRIORITY`
的分级屏蔽 + 嵌套计数，**不是** `__disable_irq`）。最长临界区约 5~10 µs，而 115200 下一字节 87 µs，
余量 8 倍以上。

### 3. 决策：两层状态机，全程非阻塞

`Core/Src/decision.c` 每 5 ms 调一次 `Decision_Run()`，纪律是：不调 `HAL_Delay`、不阻塞、所有计时用
`HAL_GetTick` 差值的无符号运算。

- **台上（`on_stage=1`）四级优先级仲裁**，命中即返回：
  `P1 四角光电防掉台` > `P2 躲避减益块` > `P3 攻敌` > `P3.5 增益块追踪` > `P4 直线巡台`
- **台下（`on_stage=0`）9 态爬台状态机**：
  `IDLE → CHECK_CORNER → ROUTE_DECISION → PRE_RUSH → APPROACH → PAUSE → RUSH → SLOW_ADVANCE → SETTLE`，
  每个有时长的状态都有超时出口，未对墙的状态每帧扫围栏可随时打断。
- `Decision_Init/Decision_Run` **只吃 `SensorState + TagInfo + now`，不读硬件、不读全局**，
  因此可以在 PC 上脱离硬件验证。

### 4. 参数存储：A/B 双槽 + 三道闸门

外挂 NOR 的最后两个 4 KB 扇区交替使用：

```
0x1FF000  扇区 511  槽 A
0x1FE000  扇区 510  槽 B
0x1F0000  硬地板 —— 低于此地址一律拒绝擦写
```

24 字节记录：`magic('CAL1') | version | length | seq | payload(4×u16) | CRC32`（覆盖前 20 字节）。
**永远写"非最新"的那一槽**，单调序号取大者，所以任意时刻掉电都还剩一份完整记录。

三道独立防线：

1. **器件物理边界**：`addr + len ≤ 2 MiB`；
2. **工程擦写窗口**：`SST25_Port_AddrWritable()`，主工程只有最后 8 KB；
3. **扇区内容分类**：`Cal_ClassifySector()` 只允许 `ERASED` / `OURS`，
   `FILESYSTEM` 与 `FW IMAGE` **硬拒绝**（背景：控制器插 USB 会显示 1 MB U 盘，那颗 flash 上可能存在 bootloader）。

读取侧除了 CRC 还查**数值合理性**：四个基准值必须在 `[1, 4094]`，且按其重算的派生阈值也在区间内。
这一条拦的是"CRC 正确但数值退化"——ADC 通道悬空会读到 0 或 4095，会让"台上判定"永久恒真，
车以为自己永远在台上、直接冲下擂台，比读不到记录危险得多。

### 5. 可靠性

- **三任务存活位图看门狗**：三个任务各自报到，位齐才喂一次狗。覆盖"单任务卡死"与
  "高优先级任务空转饿死低优先级"两类故障（idle hook 与单独的喂狗任务都测不出来）。
- **复位原因锁存 + 续跑**：`Wdg_LatchResetCause()` 在 `HAL_Init()` 之前锁存 `RCC->CSR`；
  若本次是 IWDG 咬的，跳过软启动闸门与硬编码冲台，直接回到战斗状态。
- **栈**：`configCHECK_FOR_STACK_OVERFLOW = 2`（哨兵法，能抓"冲过头又收回"的瞬时越界）
  + `uxTaskGetStackHighWaterMark` 每 750 ms 上报余量。
- **致命路径**：`Error_Handler` 与栈溢出钩子都是"先喂狗 10 秒让 LCD 提示被看见，再停手让 IWDG 复位"
  —— `__disable_irq()` 只关 CPU 中断，挡不住由独立 LSI 驱动的 IWDG，这是该套路成立的前提。
- **在线测量**：用 DWT 的 `CYCCNT` 逐拍测"ADC→电机全链耗时"与"纯决策耗时"（min/max/avg），
  外加实际周期抖动，经蓝牙每 750 ms 上报。全链 < 1 ms。

---

## 构建与烧写

### STM32CubeIDE（推荐）

分别打开三个工程目录下的 `.project` 即可（工程文件已入库）。工具链 `arm-none-eabi-gcc`。

### CLion / CMake

```bash
cd luby端
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
```

### 没有 ARM 工具链时（宿主机语法检查）

```bash
cd luby端
bash tools/syntaxcheck/run_check.sh              # 砝码自检 + 检查 Core/Src 全部 .c
bash tools/syntaxcheck/run_check.sh --canary     # 只跑砝码（验证装置本身可信）
bash tools/syntaxcheck/run_check.sh Core/Src/decision.c
```

用本机 gcc 的 `-fsyntax-only` + 工程里**真实的** HAL/CMSIS/FreeRTOS 头文件做检查，并配了
"canary 砝码"自证有效：`canary_good.c` 必须通过、三个 `canary_bad_*.c` 必须报错。
砝码一旦全绿，说明 include 顺序错了或头文件被顶替，此时正式结果一律不可信（脚本以退出码 1 报警）。
详见 `luby端/tools/syntaxcheck/README.md`。

### 烧写

三个工程的 `.bin` 通过 LUBY 控制器的 U 盘模式烧写：按住 BACK 开机进入 U 盘模式 → 把 bin 拷到根目录
→ 退出 → 在菜单里选择执行。

---

## 使用流程

```
① 烧 FlashTest → 只读普查 512 个扇区（约 4s），确认最后两个扇区为空、可安全使用
   需要做写测试时：按住 KEY2 上电 3 秒武装 Phase 2（只测已知为空的扇区，测完擦回 0xFF）
② 烧 CalibTool → 三步灰度标定（车头压前边缘 / 车尾压后边缘 / 整车放台下地面，各约 3s 引导）
   屏幕显示四个数 → KEY1 存盘 → 自动回读验证，出现 VERIFY OK! 才可断电
③ 烧主固件 → 上电自检屏应显示 CAL: FLASH OK + 四个基准值 + 复位原因
   若显示 CAL MISSING! 会在下一行给出具体原因，并用编译期兜底值继续运行（不停机）
```

标定值读不到时的降级策略：**绝不停机、绝不进 `Error_Handler`、绝不退回 18 秒阻塞采集**，
而是用编译期兜底值继续跑，故障由 LCD 与蓝牙遥测暴露给操作者。

---

## 调试接口（蓝牙，115200）

每 750 ms 一屏：

| 行 | 内容 |
|---|---|
| `[AI]/[MANUAL] ACT=…` | 当前动作名、爬台状态、左右轮速、台上标志、减益块告警 |
| `[SENSOR]` | 前后灰度值 / 边缘阈值 / 台上阈值 / 台上原始判定 |
| `[IR] N:` / `[IR] F:` | 8 路红外（4 帧 / 2 帧窗口）+ 通道角度与引脚对照 |
| `[IR] DIG= ENEMY=` | 二值掩码与敌人掩码（按位读，`0x01`=ch0） |
| `[IR] WALL: idx/max` | 8 路中最强的那一路 |
| `[TIMING]` | 全链耗时 / 决策耗时 min·max·avg + 周期抖动最大值 |
| `[STACK]` | 两个任务的栈余量（字节），低于 ~200 B 应加大栈 |

手动模式（排障用）：先发 `manual`，再用 `forward` / `back` / `left` / `right` / `stop`；发 `auto` 回到自动模式。
未发送 `manual` 之前所有运动指令都会被拒绝。

---

## PC 侧单元测试

`luby端/hosttest/` 用一份 mock flash 把"断电安全"这类只能在现场用命撞的性质搬到 PC 上验证：

- `mock.c` 语义与真驱动对齐（NOR 只能 1→0、未擦除拒写、支持"擦一半就断电"）；
- 给写/擦操作加"字节预算"，预算耗尽即模拟掉电，**把 24 个写字节断点全部跑一遍**，
  每个断点后都必须还能读回一份完整数据；
- 另覆盖 CRC 自检、空片、A/B 交替序号递增、数值退化被拒（读取端与保存端都测）、
  扇区分类的各类判定、位翻转残留仍可擦写。

```bash
cd luby端/hosttest
gcc -O2 -o hosttest test.c cal_store.c mock.c -I.
./hosttest
```

（目录内 `stm32f1xx_hal.h` / `sst25.h` / `calibrate.h` 是为主机编译准备的最小桩。）

---

## 已知限制

按影响排序，这些都是明确知道、写下来而不是藏起来的问题：

1. **蓝牙接收回调在中断上下文里做字符串比较与阻塞发送**（最长可达 500 ms）。功能可用，
   但属于"ISR 里做阻塞操作"的隐患；正确做法是中断里只投递消息，解析与回包放到低优先级任务。
2. **电机 `SyncWrite` 是阻塞发送**，正常 14 字节 @1Mbps ≈ 140 µs，但超时上限 20 ms，且在控制任务里。
   改进方向是改 DMA + TC 中断，或把超时收紧到 2 ms。
3. **`HardFault_Handler` 只有 `while(1)`**：靠"没人喂狗 → IWDG 复位"实现自动恢复，但拿不到现场
   （未解栈帧、未读 `CFSR/HFSR`）。
4. **ADC 缺少错误回调**：DMA 若停摆，`adc_completed_bank` 不再变化，任务会持续读越来越旧的静态数据，
   而系统感知不到。`ADC_GetSequence()` 这个现成探针目前未被调用。
5. **视觉链路的三个计数器（收字节 / 校验通过 / 校验失败）只在累加，未上报**，
   现场分不清"一帧都没收到"和"收到了全是垃圾"。
6. `configUSE_TIMERS = 1` 但没有任何软定时器，白占约 1.2 KB 堆。
7. 控制周期用 `osDelay(5)`（相对延时），周期 = 5 ms + 本拍执行时间，存在微漂移；
   已用周期抖动指标监控，若要严格 200 Hz 应改 `vTaskDelayUntil`。
8. **运动控制是开环的**（无编码器 / IMU）。直线与转角精度靠"两轮同帧下发 + 起步延迟补偿 + 短动作时长"
   控制误差，不是闭环控制。这是本方案明确的性能上限。
9. `zigbee.c` / `usb.c` / `peripheral_test.c` 属于编译进工程但主流程未接线的模块。

---

## 致谢与许可

- `Drivers/` 为 ST 官方 STM32F1 HAL 与 CMSIS，遵循 ST 的 SLA0044 许可；
- `Middlewares/` 为 FreeRTOS 10.3.1 与 CMSIS-RTOS2，遵循 MIT / Apache-2.0；
- `Core/` 与三个工程的业务代码为本项目原创；
- 控制器原理图、厂商用户文档与视觉模型权重**不在本仓库**。

对应视觉节点：见分支 [`Raspberry_Pi-5_Vision`](https://github.com/7om-Balancist/Wheel_Robot/tree/Raspberry_Pi-5_Vision)。
