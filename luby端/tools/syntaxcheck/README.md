# car7.15 宿主机语法检查装置 (host syntax check)

用本机 x86_64 的 `gcc -fsyntax-only` 机械地查 `Core/Src/*.c` 的 **C 编译错误**。

**为什么需要它**: 本工程原本用 `arm-none-eabi-gcc` 编译, 但工具链已从这台机器上
消失(全盘没有 `arm-none-eabi-gcc.exe`, 也没有 Keil/IAR/CubeIDE)。结果就是没人
编译过, 于是"引用了头文件里不存在的字段"这种错误可以一路潜伏到赛场。
本装置不需要 ARM 工具链, 只要本机的 mingw gcc 就能把这类错误当场抓出来。

---

## 怎么跑

```bash
cd /c/Users/AcerJ/Desktop/car7.15

bash tools/syntaxcheck/run_check.sh                 # 砝码自检 + 检查 Core/Src 全部 .c
bash tools/syntaxcheck/run_check.sh --canary        # 只跑砝码自检(验证装置本身可信)
bash tools/syntaxcheck/run_check.sh --no-canary     # 只检查, 不跑砝码
bash tools/syntaxcheck/run_check.sh Core/Src/decision.c   # 只检查指定文件(可多个)
```

退出码: `0` = 全部通过; `1` = 有真错误, **或**砝码表现异常(装置不可信)。

GCC 路径写死在脚本顶部, 可用环境变量覆盖:

```bash
GCC=gcc bash tools/syntaxcheck/run_check.sh          # 用 PATH 里的 gcc
```

---

## 设计要点: 为什么**几乎没有桩**

装置默认使用工程里**真实的** HAL / CMSIS / FreeRTOS 头文件:

| 依赖 | 来源 | 是真头文件还是桩 |
|---|---|---|
| STM32 HAL | `Drivers/STM32F1xx_HAL_Driver/Inc` | **真** |
| CMSIS core / device | `Drivers/CMSIS/...` | **真** |
| FreeRTOS kernel | `Middlewares/Third_Party/FreeRTOS/Source/include` | **真** |
| CMSIS-RTOS v2 | `Middlewares/.../CMSIS_RTOS_V2` | **真** |
| 项目自己的头 | `Core/Inc/*.h` | **真** |
| newlib `<reent.h>` | `tools/syntaxcheck/stubs/reent.h` | 桩 |

任务原本建议给 HAL/FreeRTOS 造桩。实际做下来发现工程里这些头全都在, 直接用真的
**更可信**: 结构体布局、宏、函数 prototype 与真机完全一致, 因此不存在"桩造窄了
导致漏报"的风险。所以最终只有 **1 个**桩, 而且是真正的外部依赖(见下)。

`-I tools/syntaxcheck/stubs` 放在 include 列表**最前**, 但该目录里只有 `reent.h`,
不会顶替任何项目头。已用 `gcc -H` 验证: `decision.h` / `sensors.h` / `pi_comm.h` /
`cds55xx.h` / `bluetooth.h` 全部来自 `Core/Inc` 的真实版本。

### 唯一的桩: `stubs/reent.h`

`Core/Inc/FreeRTOSConfig.h:93` 定义了 `configUSE_NEWLIB_REENTRANT 1`,
于是 `FreeRTOS.h:71` 会 `#include <reent.h>`。`<reent.h>` 是 **newlib** 的头,
真机 `arm-none-eabi-gcc` 自带, 但本机 mingw 用 msvcrt、全盘没有这个文件 →
`fatal error: reent.h: No such file or directory`。

桩里只给 `struct _reent` 一个空壳(FreeRTOS 把它按值嵌进 TCB, 只需要它是完整
类型)和两个指针声明。**没有补任何行为**, 也不含任何项目语义。

---

## 砝码 (canary): 证明装置有牙

一个"永远通过"的检查装置比没有装置更危险, 所以 `canary/` 下放了一组校准砝码,
每次运行默认都会跑。它们与受检代码用**同一批**头文件和**同一套**编译参数:

| 文件 | 期望 | 复现的事故 |
|---|---|---|
| `canary_good.c` | **通过** | 反证: 用真实字段若也报错, 说明装置本身坏了 |
| `canary_bad_field.c` | 报错 | 引用 `DecisionState` 里不存在的成员 |
| `canary_bad_subscript.c` | 报错 | 把标量 `on_stage` 当数组用 / 把数组 `wall_threshold` 当标量用 |
| `canary_bad_argc.c` | 报错 | `Decision_Run` 少传参数 |

实测输出:

```
  PASS  canary_good.c        (应通过 → 通过) ✓
  PASS  canary_bad_field.c         (应报错 → 报错) ✓  'DecisionState' {aka 'struct <anonymous>'} has no member named 'no_such_field_xyz'
  PASS  canary_bad_subscript.c     (应报错 → 报错) ✓  subscripted value is neither array nor pointer nor vector
  PASS  canary_bad_argc.c          (应报错 → 报错) ✓  too few arguments to function 'Decision_Run'
```

> 如果哪天 `canary_bad_*.c` 变成了"通过", 说明 `-I` 顺序错了或头文件被顶替了,
> **此时正式检查的结果一律不可信**, 脚本会以退出码 1 报警。
> 砝码是装置的校准砝码, **不要为了"让检查全绿"去改它们**。

---

## 被排除的文件

| 文件 | 原因 |
|---|---|
| `Core/Src/syscalls.c` | STM32CubeIDE 生成的 **newlib 系统调用垫片** (`_read`/`_write`/`_open`/`_fstat64i32`…)。mingw 的 CRT 自己也声明了这些符号且签名不同, 于是每个函数都报 `conflicting types for '_read'` —— 纯宿主 libc 与目标 libc 的冲突, 不是项目 bug, 在 x86 上无法有意义地检查。真机 newlib 编译它是没问题的。 |

除它之外, **`Core/Src` 下 31 个 .c 全部纳入检查**(含 `freertos.c` —— 真实
FreeRTOS 头文件齐全, 不需要跳过)。

---

## 目录结构

```
tools/syntaxcheck/
├── run_check.sh              一键入口(唯一需要会用的命令)
├── README.md                 本文件
├── stubs/
│   └── reent.h               唯一的外部头替身 (newlib)
└── canary/                   校准砝码, 别删别改
    ├── canary_good.c         必须通过
    ├── canary_bad_field.c    必须报错
    ├── canary_bad_subscript.c 必须报错
    └── canary_bad_argc.c     必须报错
```

---

## 局限 (重要)

`-fsyntax-only` **只做语法 + 语义检查**, 不生成代码, 因此它查不出:

- **链接错误** —— 例如声明了函数却没有实现、重复定义。这些在真机链接阶段才会暴露。
- **优化器/后端才能发现的问题**, 以及代码体积/栈深度。
- **HAL 的行为正确性** —— 只查你调用的 API 是否存在、参数类型对不对。
- **`syscalls.c`** (见上)。
- 硬件相关的东西一概查不到。

它擅长且可靠的是: **结构体字段是否存在、数组/标量下标是否合法、类型是否匹配、
函数参数个数与类型是否正确、声明与定义是否一致、宏是否未定义** —— 也正是
"改了代码但没编译过"最容易出的那一类错。

---

## 也可以手动跑(不用脚本)

```bash
gcc -fsyntax-only -std=gnu11 -DSTM32F103xE -DUSE_HAL_DRIVER \
    -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast \
    -I tools/syntaxcheck/stubs \
    -I Core/Inc \
    -I Drivers/STM32F1xx_HAL_Driver/Inc \
    -I Drivers/STM32F1xx_HAL_Driver/Inc/Legacy \
    -I Drivers/CMSIS/Device/ST/STM32F1xx/Include \
    -I Drivers/CMSIS/Include \
    -I Middlewares/Third_Party/FreeRTOS/Source/include \
    -I Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2 \
    -I Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3 \
    Core/Src/decision.c
```

想看警告(默认只看错误): 把 `-fsyntax-only` 后面加上 `-Wall -Wextra`。
注意宿主机是 64 位、CMSIS 是 32 位 ARM 代码, 会产生 `-Wint-to-pointer-cast`
之类的噪声(脚本已静音), 与项目代码无关。
