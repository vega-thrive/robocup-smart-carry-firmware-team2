#ifndef __MAXICAM_H__
#define __MAXICAM_H__

#include "main.h"

typedef enum
{
    MAXICAM_MODE_CIRCLE = 1,  /* 模式1: 同心圆识别 */
    MAXICAM_MODE_ABC = 2,     /* 模式2: 奖杯识别 (jiang_bei, 0x02) */
    MAXICAM_MODE_COLOR = 3,   /* 模式3: 色块识别 */
    MAXICAM_MODE_STOP = 4
} MaxiCam_Mode;

typedef struct
{
    uint8_t data[6];
    uint32_t received_at_ms;
    uint32_t sequence;
} MaxiCam_Frame;

void MaxiCam_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef MaxiCam_StartReceiveIT(void);
HAL_StatusTypeDef MaxiCam_SetMode(MaxiCam_Mode mode);
void MaxiCam_OnRxComplete(UART_HandleTypeDef *uart);
void MaxiCam_OnUartError(void);
uint8_t MaxiCam_GetLatest(MaxiCam_Frame *frame, uint32_t max_age_ms);

/* 串口诊断计数器: 定位 NO FRAME 根因 */
extern volatile uint32_t maxicam_diag_rx_bytes;
extern volatile uint32_t maxicam_diag_headers;
extern volatile uint32_t maxicam_diag_packets;

#endif
