#!/usr/bin/env bash
# ============================================================================
#  car7.15 宿主机语法检查 (host syntax check)
# ----------------------------------------------------------------------------
#  用 x86 gcc 的 -fsyntax-only 机械地找出 Core/Src/*.c 的编译错误。
#  不需要 ARM 工具链 —— 本机 arm-none-eabi-gcc 已丢失。
#
#  用法(Git Bash):
#      bash tools/syntaxcheck/run_check.sh              # 检查全部
#      bash tools/syntaxcheck/run_check.sh --canary     # 只跑砝码(自检)
#      bash tools/syntaxcheck/run_check.sh Core/Src/decision.c   # 只查指定文件
#
#  退出码: 0 = 全部通过且砝码表现正常; 1 = 有真错误或砝码异常
# ============================================================================
set -u

# ---- gcc 路径(本机唯一可用的 gcc; 若 PATH 里已有 gcc, 改回 "gcc" 即可) ----
GCC="${GCC:-/c/mingw64/mingw64/bin/gcc}"

# ---- 定位项目根(本脚本在 <root>/tools/syntaxcheck/ 下) ----
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
cd "$ROOT" || exit 1

# ============================================================================
#  包含路径 —— 顺序很重要
# ============================================================================
#  ★ 全部使用**真实头文件**。工程里 Drivers/ 与 Middlewares/ 都在, 所以
#    STM32 HAL / CMSIS / FreeRTOS 的头都是真的, 不需要造桩。
#    这比造桩更可信: gcc 看到的结构体布局、宏、prototype 与真机完全一致。
#
#  ★ tools/syntaxcheck/stubs 放在最前, 只用来顶替**本机确实不存在的外部头**
#    (目前只有 newlib 的 <reent.h>)。项目自己的头(Core/Inc/*.h)一律取真实版本,
#    绝不顶替 —— 否则就查不出"字段不存在"这类真 bug 了。
INC=(
    -I "$HERE/stubs"                                                # 唯一的外部头替身
    -I "Core/Inc"                                                   # 项目头(真实)
    -I "Drivers/STM32F1xx_HAL_Driver/Inc"
    -I "Drivers/STM32F1xx_HAL_Driver/Inc/Legacy"
    -I "Drivers/CMSIS/Device/ST/STM32F1xx/Include"
    -I "Drivers/CMSIS/Include"
    -I "Middlewares/Third_Party/FreeRTOS/Source/include"
    -I "Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
    -I "Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3"
)

# 与真机 CubeIDE 工程一致的宏
DEFS=(-D STM32F103xE -D USE_HAL_DRIVER)

# 宿主机 x86_64(8 字节指针) 跑 CMSIS 的 ARM 代码会产生"指针截断"警告,
# 与项目代码无关, 静音掉以免淹没真错误。
FLAGS=(-fsyntax-only -std=gnu11 -Wno-int-to-pointer-cast -Wno-pointer-to-int-cast)

# ============================================================================
#  被排除的文件(附理由) —— 只有真正无法在宿主机上检查的才排除
# ============================================================================
#  syscalls.c: STM32CubeIDE 生成的 **newlib 系统调用垫片**(_read/_write/
#    _open/_fstat64i32...)。mingw 的 CRT 自己也声明了这些符号且签名不同,
#    于是每个函数都报 "conflicting types for '_read'" —— 纯粹是宿主 libc
#    与目标 libc 的冲突, 不是项目 bug, 在 x86 上无法有意义地检查。
#    真机 arm-none-eabi(newlib) 编译它是没问题的。
EXCLUDE=("Core/Src/syscalls.c")

err=0

hr() { printf '%s\n' "----------------------------------------------------------------"; }

# ============================================================================
#  1) 砝码自检 —— 证明装置有牙
# ============================================================================
run_canary() {
    hr; echo "[CANARY] 砝码自检: 装置该报错时必须报错, 该通过时必须通过"; hr

    local local_err=0

    # 1a. 必须通过的那个
    if out=$("$GCC" "${FLAGS[@]}" "${DEFS[@]}" "${INC[@]}" "$HERE/canary/canary_good.c" 2>&1); then
        echo "  PASS  canary_good.c        (应通过 → 通过) ✓"
    else
        echo "  FAIL  canary_good.c        (应通过, 却报错了!) ✗ —— 装置本身有问题"
        printf '%s\n' "$out" | grep -E 'error' | head -5
        local_err=1
    fi

    # 1b. 必须失败的三个
    local f
    for f in canary_bad_field canary_bad_subscript canary_bad_argc; do
        if out=$("$GCC" "${FLAGS[@]}" "${DEFS[@]}" "${INC[@]}" "$HERE/canary/$f.c" 2>&1); then
            echo "  FAIL  $f.c  (应报错, 却通过了!) ✗ —— 装置没牙, 结果不可信"
            local_err=1
        else
            printf '  PASS  %-26s (应报错 → 报错) ✓  %s\n' "$f.c" \
                "$(printf '%s\n' "$out" | grep -m1 -E 'error' | sed 's/^.*error: //')"
        fi
    done

    hr
    [ $local_err -eq 0 ] \
        && echo "[CANARY] 全部符合预期 —— 装置可信" \
        || echo "[CANARY] 砝码表现异常 —— 上面的检查结果不可信!"
    return $local_err
}

# ============================================================================
#  2) 真正检查
# ============================================================================
run_check() {
    local files=("$@")
    local f n rc out

    hr; echo "[CHECK] 语法检查 $((${#files[@]})) 个文件 (真实项目头文件, x86 gcc -fsyntax-only)"; hr

    for f in "${files[@]}"; do
        out=$("$GCC" "${FLAGS[@]}" "${DEFS[@]}" "${INC[@]}" "$f" 2>&1)
        rc=$?
        if [ $rc -eq 0 ]; then
            printf '  OK    %s\n' "$f"
        else
            n=$(printf '%s\n' "$out" | grep -c 'error:')
            printf '  ERROR %s   (%s 个 error)\n' "$f" "$n"
            printf '%s\n' "$out" | grep -E 'error:' | sed 's/^/          /'
            err=1
        fi
    done
    hr
}

# ============================================================================
#  入口
# ============================================================================
MODE=all
FILES=()
for a in "$@"; do
    case "$a" in
        --canary) MODE=canary ;;
        --no-canary) MODE=check ;;
        *) FILES+=("$a") ;;
    esac
done

if [ "$MODE" != "check" ]; then
    run_canary || err=1
    echo
fi

if [ "$MODE" = "canary" ]; then exit $err; fi

if [ ${#FILES[@]} -eq 0 ]; then
    # 默认: Core/Src 下所有 .c, 去掉排除表
    while IFS= read -r f; do
        skip=0
        for x in "${EXCLUDE[@]}"; do [ "$f" = "$x" ] && skip=1; done
        [ $skip -eq 0 ] && FILES+=("$f")
    done < <(ls Core/Src/*.c | sed 's#\\#/#g' | sort)
fi

run_check "${FILES[@]}"

echo
if [ $err -eq 0 ]; then
    echo "==== 结果: 全部通过 ===="
    echo "(被排除: ${EXCLUDE[*]} —— 理由见本脚本顶部注释)"
else
    echo "==== 结果: 发现编译错误, 见上 ===="
fi
exit $err
