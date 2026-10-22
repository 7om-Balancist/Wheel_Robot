#include "zigbee.h"
#include <string.h>
#include <stdio.h>

/* ================================================================
 * 模块级变量
 * ================================================================ */
static ZB_RxCallback  zb_rx_cb = NULL;       /* 用户注册的帧回调 */

static uint8_t         zb_local_node_id = 0;  /* 本机节点ID */
static uint16_t        zb_local_pan_id  = 0;  /* 本机PAN ID */
static volatile uint8_t zb_connected = 0;     /* 已成功加入网络标志 */

/* ---- 帧接收状态机 ---- */
static ZB_ParseState   zb_state = ZB_PARSE_WAIT_SOP1;
static uint8_t         zb_rx_buf[ZB_RX_BUF_SIZE];
static uint8_t         zb_rx_idx = 0;
static uint8_t         zb_rx_len = 0;         /* 期望帧体长度 */
static uint8_t         zb_rx_sop2 = 0;        /* 当前帧类型标记 */

/* ---- UART 中断逐字节接收 ---- */
static uint8_t         zb_rx_byte;            /* 单字节接收缓冲 */

/* ================================================================
 * 内部辅助函数
 * ================================================================ */

/** 计算命令帧 FCS = XOR(LEN, CMD0, CMD1, DATA[0..N]) */
static uint8_t ZB_CalcFCS(uint8_t len, uint8_t *body)
{
    uint8_t fcs = len;
    for (uint8_t i = 0; i < len; i++) {
        fcs ^= body[i];
    }
    return fcs;
}

/** 通过 UART4 发送原始字节（无 DMA，阻塞发送） */
static void ZB_SendRaw(uint8_t *buf, uint8_t len)
{
    HAL_UART_Transmit(ZB_UART, buf, len, 50);
}

/** 帧解析状态机 — 每收到一个字节调用一次 */
static void ZB_ParseByte(uint8_t byte)
{
    switch (zb_state) {

    case ZB_PARSE_WAIT_SOP1:
        if (byte == ZB_SOP1) {
            zb_state = ZB_PARSE_WAIT_SOP2;
        }
        break;

    case ZB_PARSE_WAIT_SOP2:
        if (byte == ZB_SOP2_DATA || byte == ZB_SOP2_CMD || byte == ZB_SOP2_SENSOR) {
            zb_rx_sop2 = byte;
            zb_rx_idx  = 0;
            zb_state   = ZB_PARSE_WAIT_LEN;
        } else {
            zb_state = ZB_PARSE_WAIT_SOP1;  /* 非法SOP2, 重新同步 */
        }
        break;

    case ZB_PARSE_WAIT_LEN:
        zb_rx_len  = byte;
        zb_rx_idx  = 0;
        if (zb_rx_len == 0 || zb_rx_len > ZB_MAX_DATA_LEN) {
            zb_state = ZB_PARSE_WAIT_SOP1;  /* 非法长度, 丢弃 */
        } else {
            zb_state = ZB_PARSE_WAIT_BODY;
        }
        break;

    case ZB_PARSE_WAIT_BODY:
        zb_rx_buf[zb_rx_idx++] = byte;
        if (zb_rx_idx >= zb_rx_len) {
            zb_state = ZB_PARSE_DONE;
        }
        break;

    default:
        zb_state = ZB_PARSE_WAIT_SOP1;
        break;
    }
}

/** 重新启动单字节中断接收 */
static void ZB_StartRx(void)
{
    HAL_UART_Receive_IT(ZB_UART, &zb_rx_byte, 1);
}

/* ================================================================
 * HAL UART 接收完成回调 — 被中断服务例程调用
 * ================================================================ */
/**
 * 公开接口 — 被 callbacks.c 的 HAL_UART_RxCpltCallback 统一分派调用
 * zigbee.c 不再自行定义 HAL_UART_RxCpltCallback，避免与 callbacks.c 链接冲突
 */
void ZB_RxByteHandler(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        ZB_ParseByte(zb_rx_byte);
        ZB_StartRx();
    }
}

/* ================================================================
 * 公开 API
 * ================================================================ */

/**
 * @brief 初始化 Zigbee 模块。
 *        注册中断回调，启动逐字节接收。
 *        调用前确保 MX_UART4_Init() 已完成。
 */
void ZB_Init(void)
{
    zb_state     = ZB_PARSE_WAIT_SOP1;
    zb_connected = 0;
    ZB_StartRx();   /* 启动中断驱动的逐字节接收 */
}

/**
 * @brief 注册帧接收回调函数。
 * @param cb 回调函数指针，NULL 取消注册
 */
void ZB_SetRxCallback(ZB_RxCallback cb)
{
    zb_rx_cb = cb;
}

/**
 * @brief 主循环处理 — 检查是否收到完整帧并分发给回调。
 *        必须在主循环或 FreeRTOS 任务中以 ≥10Hz 的频率调用。
 */
void ZB_Process(void)
{
    if (zb_state != ZB_PARSE_DONE) return;

    /* ---- 帧接收完毕，开始解析 ---- */
    ZB_Frame frame;
    memset(&frame, 0, sizeof(frame));
    frame.len      = zb_rx_len;
    frame.data_len = 0;
    frame.fcs_ok   = 1;

    switch (zb_rx_sop2) {

    case ZB_SOP2_DATA:
        /* 格式: LEN | SrcID | DstID | DATA[0..N] */
        if (zb_rx_len < 2) { frame.type = ZB_FRAME_ERROR; break; }
        frame.type  = ZB_FRAME_DATA;
        frame.src_id = zb_rx_buf[0];
        frame.dst_id = zb_rx_buf[1];
        frame.data_len = zb_rx_len - 2;
        if (frame.data_len > 0) {
            memcpy(frame.data, &zb_rx_buf[2], frame.data_len);
        }
        break;

    case ZB_SOP2_CMD:
        /* 格式: LEN | CMD0 | CMD1 | DATA[0..N] | FCS */
        if (zb_rx_len < 3) { frame.type = ZB_FRAME_ERROR; break; }
        frame.type  = ZB_FRAME_CMD_RESP;
        frame.cmd0  = zb_rx_buf[0];
        frame.cmd1  = zb_rx_buf[1];
        frame.data_len = zb_rx_len - 3;
        if (frame.data_len > 0) {
            memcpy(frame.data, &zb_rx_buf[2], frame.data_len);
        }
        frame.fcs     = zb_rx_buf[zb_rx_len - 1];
        frame.fcs_ok  = (ZB_CalcFCS(zb_rx_len - 1, zb_rx_buf) == frame.fcs);

        /* 解析常见命令响应 */
        if (frame.fcs_ok) {
            if (frame.cmd0 == 0x21 && frame.cmd1 == 0x08 && frame.data_len >= 1) {
                zb_local_node_id = frame.data[0];
            }
            if (frame.cmd0 == 0x27 && frame.cmd1 == 0x01 && frame.data_len >= 6) {
                zb_local_pan_id = (frame.data[4] << 8) | frame.data[5];
            }
            if (frame.cmd0 == 0x27 && frame.cmd1 == 0x00) {
                zb_connected = 1;  /* 扫描成功 → 已连接 */
            }
        }
        break;

    case ZB_SOP2_SENSOR:
        /* 格式: LEN | ADC0_H | ADC0_L | ADC1_H | ADC1_L | ADC2_H | ADC2_L | ADC3_H | ADC3_L */
        frame.type     = ZB_FRAME_SENSOR;
        frame.data_len = zb_rx_len;
        if (zb_rx_len > 0) {
            memcpy(frame.data, zb_rx_buf, zb_rx_len);
        }
        break;

    default:
        frame.type = ZB_FRAME_ERROR;
        break;
    }

    /* ---- 通知用户回调 ---- */
    if (zb_rx_cb && frame.type != ZB_FRAME_ERROR) {
        zb_rx_cb(&frame);
    }

    /* ---- 重置状态机，准备下一帧 ---- */
    zb_state = ZB_PARSE_WAIT_SOP1;
}

/* ================================================================
 * 数据发送
 * ================================================================ */

/** 构建并发送透传数据帧: SOP + LEN + SrcID + DstID + DATA */
static void ZB_SendDataFrame(uint8_t dst_id, uint8_t *data, uint8_t len)
{
    uint8_t buf[ZB_RX_BUF_SIZE];
    uint8_t idx = 0;
    buf[idx++] = ZB_SOP1;
    buf[idx++] = ZB_SOP2_DATA;
    buf[idx++] = (uint8_t)(len + 2);    /* LEN = SrcID + DstID + data */
    buf[idx++] = zb_local_node_id;      /* SrcID = 本机ID */
    buf[idx++] = dst_id;
    if (len > 0) {
        memcpy(&buf[idx], data, len);
        idx += len;
    }
    ZB_SendRaw(buf, idx);
}

void ZB_SendData(uint8_t dst_id, uint8_t *data, uint8_t len)
{
    if (len > ZB_MAX_DATA_LEN - 2) len = ZB_MAX_DATA_LEN - 2;
    ZB_SendDataFrame(dst_id, data, len);
}

void ZB_BroadcastData(uint8_t *data, uint8_t len)
{
    ZB_SendDataFrame(ZB_BROADCAST_ID, data, len);
}

/* ================================================================
 * 命令发送
 * ================================================================ */

/** 构建并发送命令帧: SOP + LEN + CMD0 + CMD1 + DATA + FCS */
static void ZB_SendCmd(uint8_t cmd0, uint8_t cmd1, uint8_t *data, uint8_t data_len)
{
    uint8_t buf[ZB_RX_BUF_SIZE];
    uint8_t body_len = 2 + data_len;    /* CMD0 + CMD1 + DATA */

    buf[0] = ZB_SOP1;
    buf[1] = ZB_SOP2_CMD;
    buf[2] = body_len;                  /* LEN = CMD0+CMD1+DATA */
    buf[3] = cmd0;
    buf[4] = cmd1;
    if (data_len > 0) {
        memcpy(&buf[5], data, data_len);
    }
    buf[5 + data_len] = ZB_CalcFCS(body_len, &buf[3]);

    ZB_SendRaw(buf, 6 + data_len);
}

void ZB_Ping(void)
{
    ZB_SendCmd(ZB_CMD_PING, NULL, 0);
}

void ZB_SetNodeID(uint8_t node_id)
{
    ZB_SendCmd(ZB_CMD_SET_NODE_ID, &node_id, 1);
}

void ZB_GetNodeID(void)
{
    ZB_SendCmd(ZB_CMD_GET_NODE_ID, NULL, 0);
}

void ZB_SetPANID(uint16_t pan_id)
{
    uint8_t data[2];
    data[0] = (uint8_t)(pan_id & 0xFF);
    data[1] = (uint8_t)((pan_id >> 8) & 0xFF);
    ZB_SendCmd(ZB_CMD_SET_PAN_ID, data, 2);
}

void ZB_SetChannel(uint8_t channel)
{
    if (channel < 11) channel = 11;
    if (channel > 26) channel = 26;
    ZB_SendCmd(ZB_CMD_SET_CHANNEL, &channel, 1);
}

void ZB_SetDeviceType(uint8_t dev_type)
{
    ZB_SendCmd(ZB_CMD_SET_DEV_TYPE, &dev_type, 1);
}

void ZB_ScanNetwork(void)
{
    ZB_SendCmd(ZB_CMD_SCAN_NETWORK, NULL, 0);
}

void ZB_SetTimeout(uint16_t timeout_ms)
{
    uint8_t data[2];
    data[0] = (uint8_t)(timeout_ms & 0xFF);
    data[1] = (uint8_t)((timeout_ms >> 8) & 0xFF);
    ZB_SendCmd(ZB_CMD_SET_TIMEOUT, data, 2);
}

/* ================================================================
 * 状态查询
 * ================================================================ */

uint8_t ZB_GetLocalNodeID(void)
{
    return zb_local_node_id;
}

uint16_t ZB_GetLocalPANID(void)
{
    return zb_local_pan_id;
}

uint8_t ZB_IsConnected(void)
{
    return zb_connected;
}

/* ================================================================
 * 调试打印
 * ================================================================ */

void ZB_PrintFrame(ZB_Frame *frame)
{
    char type_str[16];
    switch (frame->type) {
    case ZB_FRAME_DATA:      snprintf(type_str, sizeof(type_str), "DATA");      break;
    case ZB_FRAME_CMD_RESP:  snprintf(type_str, sizeof(type_str), "CMD_RESP");  break;
    case ZB_FRAME_SENSOR:    snprintf(type_str, sizeof(type_str), "SENSOR");    break;
    case ZB_FRAME_ERROR:     snprintf(type_str, sizeof(type_str), "ERROR");     break;
    default:                 snprintf(type_str, sizeof(type_str), "UNKNOWN");   break;
    }

    printf("[ZB] RX frame: type=%s len=%d", type_str, frame->len);

    if (frame->type == ZB_FRAME_DATA) {
        printf(" src=%d dst=%d data_len=%d\r\n", frame->src_id, frame->dst_id, frame->data_len);
    } else if (frame->type == ZB_FRAME_CMD_RESP) {
        printf(" cmd=0x%02X%02X fcs=%s\r\n",
               frame->cmd0, frame->cmd1, frame->fcs_ok ? "OK" : "FAIL");
    } else if (frame->type == ZB_FRAME_SENSOR) {
        if (frame->data_len >= 8) {
            uint16_t adc[4];
            for (int i = 0; i < 4; i++) {
                adc[i] = (frame->data[2*i] << 8) | frame->data[2*i+1];
            }
            printf(" ADC=%d,%d,%d,%d\r\n", adc[0], adc[1], adc[2], adc[3]);
        } else {
            printf("\r\n");
        }
    } else {
        printf("\r\n");
    }
}
