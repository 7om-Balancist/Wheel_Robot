#ifndef __ZIGBEE_H
#define __ZIGBEE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* ================================================================
 * 硬件接口配置 — LUBY 控制器 CC2530 Zigbee 模块
 * 接口: UART4 @ 19200-8N1  (PC10=TX, PC11=RX)
 * ================================================================ */
extern UART_HandleTypeDef huart4;
#define ZB_UART         &huart4

/* ================================================================
 * 协议帧格式常量 (CC2530 Z-Stack 透传协议)
 * ================================================================ */
#define ZB_SOP1             0xFE
#define ZB_SOP2_DATA        0xDA    /* 数据透传帧 */
#define ZB_SOP2_CMD         0xCC    /* 配置命令帧 */
#define ZB_SOP2_SENSOR      0xED    /* 传感器广播帧(接收) */

#define ZB_MAX_DATA_LEN     253     /* 单帧最大有效载荷 */
#define ZB_RX_BUF_SIZE      260     /* 接收缓冲(SOP2 + 最长帧) */

/* 设备类型 */
#define ZB_DEV_COORDINATOR  0x00
#define ZB_DEV_ROUTER       0x01
#define ZB_DEV_END_DEVICE   0x02

/* 广播地址 */
#define ZB_BROADCAST_ID     0xFF

/* ================================================================
 * 命令码
 * ================================================================ */
#define ZB_CMD_PING             0x41, 0x00
#define ZB_CMD_GET_NODE_ID      0x21, 0x08
#define ZB_CMD_SET_NODE_ID      0x21, 0x09
#define ZB_CMD_SET_PAN_ID       0x27, 0x02
#define ZB_CMD_SET_CHANNEL      0x27, 0x03
#define ZB_CMD_SET_DEV_TYPE     0x27, 0x04
#define ZB_CMD_SCAN_NETWORK     0x27, 0x00
#define ZB_CMD_READ_NV          0x27, 0x01
#define ZB_CMD_SET_TIMEOUT      0x23, 0x03

/* ================================================================
 * 帧解析状态机状态
 * ================================================================ */
typedef enum {
    ZB_PARSE_WAIT_SOP1 = 0,
    ZB_PARSE_WAIT_SOP2,
    ZB_PARSE_WAIT_LEN,
    ZB_PARSE_WAIT_BODY,
    ZB_PARSE_DONE
} ZB_ParseState;

/* ================================================================
 * 帧类型
 * ================================================================ */
typedef enum {
    ZB_FRAME_NONE = 0,
    ZB_FRAME_DATA,          /* 收到数据透传帧 */
    ZB_FRAME_CMD_RESP,      /* 收到命令响应帧 */
    ZB_FRAME_SENSOR,        /* 收到传感器广播帧 */
    ZB_FRAME_ERROR          /* 帧校验错误 */
} ZB_FrameType;

/* ================================================================
 * 解析后的帧数据
 * ================================================================ */
typedef struct {
    ZB_FrameType type;
    uint8_t      len;           /* 帧体长度(不含SOP) */
    uint8_t      src_id;        /* 源节点ID(仅DATA帧) */
    uint8_t      dst_id;        /* 目标节点ID(仅DATA帧, 0xFF=广播) */
    uint8_t      cmd0;          /* 命令码高位(仅CMD帧) */
    uint8_t      cmd1;          /* 命令码低位(仅CMD帧) */
    uint8_t      data[ZB_MAX_DATA_LEN]; /* 有效载荷 */
    uint8_t      data_len;      /* 有效载荷字节数 */
    uint8_t      fcs;           /* 收到的校验值(仅CMD帧) */
    uint8_t      fcs_ok;        /* 校验是否通过 */
} ZB_Frame;

/* ================================================================
 * 接收回调函数类型 — 收到帧时调用
 * ================================================================ */
typedef void (*ZB_RxCallback)(ZB_Frame *frame);

/* ================================================================
 * API
 * ================================================================ */
void ZB_Init(void);
void ZB_SetRxCallback(ZB_RxCallback cb);
void ZB_Process(void);
void ZB_RxByteHandler(UART_HandleTypeDef *huart);  /* callbacks.c 调用 */

/* 数据收发 */
void ZB_SendData(uint8_t dst_id, uint8_t *data, uint8_t len);
void ZB_BroadcastData(uint8_t *data, uint8_t len);

/* 配置命令 */
void ZB_Ping(void);
void ZB_SetNodeID(uint8_t node_id);
void ZB_GetNodeID(void);
void ZB_SetPANID(uint16_t pan_id);
void ZB_SetChannel(uint8_t channel);      /* 11-26 */
void ZB_SetDeviceType(uint8_t dev_type);  /* ZB_DEV_COORDINATOR/ROUTER/END_DEVICE */
void ZB_ScanNetwork(void);
void ZB_SetTimeout(uint16_t timeout_ms);

/* 工具函数 */
uint8_t  ZB_GetLocalNodeID(void);
uint16_t ZB_GetLocalPANID(void);
uint8_t  ZB_IsConnected(void);
void     ZB_PrintFrame(ZB_Frame *frame);  /* 串口打印调试信息 */

#endif /* __ZIGBEE_H */
