/**
 * @file    canary_bad_field.c
 * @brief   CANARY — **必须编译失败**。
 *
 * 故意访问 DecisionState 里不存在的成员。若装置有牙, gcc 必须报:
 *   error: 'DecisionState' has no member named 'no_such_field_xyz'
 *
 * 这正是任务里描述的那类事故(有人引用了头文件里没有的字段却没人发现)
 * 的机械化复现。这个文件**永远不该被"修好"** —— 它是装置的校准砝码。
 */
#include "decision.h"

static DecisionState canary_st;

void canary_bad_field(void)
{
    canary_st.no_such_field_xyz = 1;   /* 本行必须报错 */
}
