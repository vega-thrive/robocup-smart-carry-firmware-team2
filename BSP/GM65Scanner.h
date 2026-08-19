#ifndef __GM65_SCANNER_H__
#define __GM65_SCANNER_H__

#include "main.h"

#define GM65_TEXT_CAPACITY 64U

typedef enum
{
    GM65_IDLE = 0,
    GM65_WAITING,
    GM65_READY,
    GM65_TIMEOUT,
    GM65_OVERFLOW
} GM65_Status;

void GM65_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef GM65_StartReceiveIT(void);
HAL_StatusTypeDef GM65_StartScan(uint32_t timeout_ms);
void GM65_Process(void);
void GM65_OnRxComplete(UART_HandleTypeDef *uart);
void GM65_OnRxByte(uint8_t byte);
void GM65_OnUartError(void);
GM65_Status GM65_GetStatus(void);
uint8_t GM65_TakeText(char *destination, uint16_t capacity);

/* 诊断 (原地扫码专项测试): 原始字节计数 + 最近8字节环形缓冲 */
extern volatile uint32_t gm65_diag_rx_bytes;
extern volatile uint8_t  gm65_diag_last_bytes[8];
extern volatile uint8_t  gm65_diag_last_index;

#endif
