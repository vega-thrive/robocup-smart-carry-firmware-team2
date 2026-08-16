#ifndef __VOFA_TUNE_H__
#define __VOFA_TUNE_H__

#include "main.h"

/* VOFA 全程在线监控与调参模块总开关.
 * =1: 使用 USART2 (PA2/PA3, 115200 8N1) 挂 VOFA, 主循环 20Hz 上传监控数据;
 * =0: 模块全部空转, 不影响比赛行为/时序/内存/原有串口. */
#define VOFA_TUNE_ENABLE 1

/* 由 HWT101.c 的全局 UART 回调转调 (接收一个字节, 内部自动续接). */
void VofaTune_OnRxByte(void);
void VofaTune_OnUartError(void);

#if VOFA_TUNE_ENABLE
void VofaTune_Init(void);
void VofaTune_Process(void);
#else
static inline void VofaTune_Init(void)    { }
static inline void VofaTune_Process(void) { }
#endif

#endif
