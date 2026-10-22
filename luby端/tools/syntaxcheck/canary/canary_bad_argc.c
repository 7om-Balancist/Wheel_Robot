/**
 * @file    canary_bad_argc.c
 * @brief   CANARY — **必须编译失败**。
 *
 * 验证装置能查出**函数签名/参数个数**错误: Decision_Run 在 decision.h 里是
 * 4 参数 (state, sensor, tag, now_ms), 这里故意只传 2 个。
 *
 * gcc 预期原话:
 *   error: too few arguments to function 'Decision_Run'
 */
#include "decision.h"

void canary_bad_argc(void)
{
    DecisionState st;
    SensorState   sn;
    Decision_Run(&st, &sn);   /* 少传 tag / now_ms, 本行必须报错 */
}
