/**
 * @file    canary_bad_subscript.c
 * @brief   CANARY — **必须编译失败**。
 *
 * 复现任务里点名的另一类事故: **把标量当数组用 / 把数组当标量用**。
 *   - on_stage 是 uint8_t 标量 → on_stage[0] 必须报 "subscripted value is
 *     neither array nor pointer"
 *   - wall_threshold 是 uint16_t[8] → 直接赋值给 uint16_t 必须报类型不匹配
 *
 * gcc 预期原话(之一):
 *   error: subscripted value is neither array nor pointer nor vector
 *   error: assignment to expression with array type
 */
#include "decision.h"
#include "sensors.h"

static DecisionState canary_st;

void canary_bad_subscript(void)
{
    canary_st.on_stage[0] = 1;              /* on_stage 是标量, 本行必须报错 */
    g_thresh.wall_threshold = 500;          /* wall_threshold 是数组, 本行必须报错 */
}
