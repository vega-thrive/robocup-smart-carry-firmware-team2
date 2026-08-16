#ifndef __MAXICAM_H__
#define __MAXICAM_H__

#include "main.h"

/* 发送给摄像头的单字节工作模式命令。 */
typedef enum
{
    MAXICAM_MODE_CIRCLE = 1,       /* 模式1：圆心识别，返回目标中心坐标。 */
    MAXICAM_MODE_ABC = 2,          /* 模式2：保留的兼容名称。 */
    MAXICAM_MODE_COLOR = 3,        /* 模式3：任务1色块颜色识别。 */
    MAXICAM_MODE_STOP = 4,         /* 模式4：退出摄像头当前识别循环。 */
    MAXICAM_MODE_TROPHY_WHITE = 2  /* 模式2：任务2白色奖杯检测，命中码26。 */
} MaxiCam_Mode;

/* 一帧已经通过帧头、帧尾校验的摄像头数据。
 * data[0..5]的含义由当前模式决定：
 * - CIRCLE：data[0]为模式码，data[2]/data[3]为X/Y；
 * - COLOR：任务1使用data[0]作为颜色码；
 * - TROPHY_WHITE：任务2使用data[0]判断奖杯检测码26。
 * received_at_ms用于判断数据是否过期；sequence每收到一帧递增一次。 */
typedef struct
{
    uint8_t data[6];
    uint32_t received_at_ms;
    uint32_t sequence;
} MaxiCam_Frame;

/* 新视觉协议：data[0]=d1, data[1]=d2，X/Y为16位小端坐标。 */
static inline uint16_t MaxiCam_FrameX(const MaxiCam_Frame *frame)
{
    return (uint16_t)((uint16_t)frame->data[2]
                    | ((uint16_t)frame->data[3] << 8));
}

static inline uint16_t MaxiCam_FrameY(const MaxiCam_Frame *frame)
{
    return (uint16_t)((uint16_t)frame->data[4]
                    | ((uint16_t)frame->data[5] << 8));
}

/* 保存摄像头UART句柄并清空解析状态；不会主动启动接收。 */
void MaxiCam_Init(UART_HandleTypeDef *uart);

/* 启动单字节串口中断接收；接收回调会自动续接下一字节。 */
HAL_StatusTypeDef MaxiCam_StartReceiveIT(void);

/* 请求模式切换；STOP立即发送，紧随STOP的目标模式会延迟发送。 */
HAL_StatusTypeDef MaxiCam_SetMode(MaxiCam_Mode mode);

/* 在主循环中发送延迟的目标模式命令。 */
void MaxiCam_Process(void);

/* UART接收完成回调入口：解析本字节并重新挂起下一字节接收。 */
void MaxiCam_OnRxComplete(UART_HandleTypeDef *uart);

/* UART错误回调入口：丢弃未完成帧并重新启动接收。 */
void MaxiCam_OnUartError(void);

/* 读取最近一帧有效数据。
 * max_age_ms为允许的最大帧龄，防止使用旧坐标或旧识别结果。
 * 返回1表示成功；返回0表示参数无效、无帧、帧过期或复制时帧被更新。 */
uint8_t MaxiCam_GetLatest(MaxiCam_Frame *frame, uint32_t max_age_ms);

/* 串口诊断计数器，用于定位“NO FRAME”：
 * rx_bytes=总字节数，headers=FF AA帧头数，packets=完整有效帧数。 */
extern volatile uint32_t maxicam_diag_rx_bytes;
extern volatile uint32_t maxicam_diag_headers;
extern volatile uint32_t maxicam_diag_packets;
extern volatile uint32_t maxicam_diag_mode_tx;
extern volatile uint32_t maxicam_diag_mode_tx_error;

#endif
