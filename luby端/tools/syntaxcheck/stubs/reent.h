/**
 * @file    reent.h   (syntax-check STUB)
 * @brief   最小 newlib <reent.h> 替身, 仅用于宿主机 gcc 语法检查
 *
 * ── 为什么需要这个桩 ──────────────────────────────────────────────
 * 它是**真正的外部依赖**, 不是项目代码:
 *   Core/Inc/FreeRTOSConfig.h:93  #define configUSE_NEWLIB_REENTRANT 1
 *   → Middlewares/.../FreeRTOS.h:71  #include <reent.h>
 *
 * <reent.h> 是 newlib 的头文件, 真实工具链 arm-none-eabi-gcc 自带,
 * 所以真机编译没问题。本机只有 x86_64-w64-mingw32-gcc (用 msvcrt, 不带
 * newlib), 全盘没有 reent.h, 于是 FreeRTOS.h 直接 fatal error。
 *
 * 语法检查只需要 struct _reent **是个完整类型**(FreeRTOS.h 把它按值嵌进 TCB),
 * 不需要任何字段 —— 宿主机上永远不会链接/运行这份代码。
 * 这里**故意不补任何行为**, 只补类型存在性。
 *
 * ⚠ 如果哪天发现必须往这里加真字段才能过, 那说明项目代码在直接访问
 *   _reent 的成员 —— 那才是要报告的真问题。
 */
#ifndef __SYNTAXCHECK_STUB_REENT_H
#define __SYNTAXCHECK_STUB_REENT_H

/* newlib 的 struct _reent 在宿主机上无对应实现, 给一个空壳即可。
 * 大小不重要: -fsyntax-only 不生成代码, 也不做 sizeof 的语义检查。 */
struct _reent {
    int _syntaxcheck_stub_unused;
};

/* newlib 用这两个符号做 per-thread 重入状态; FreeRTOS.h/portmacro 会引用
 * _impure_ptr。声明成指针即可(只做类型检查, 不链接)。 */
extern struct _reent *_impure_ptr;
extern struct _reent *_global_impure_ptr;

#endif /* __SYNTAXCHECK_STUB_REENT_H */
