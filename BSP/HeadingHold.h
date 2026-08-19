#ifndef __HEADING_HOLD_H__
#define __HEADING_HOLD_H__

#include "main.h"

#define HEADING_HOLD_KP    3.0f
#define HEADING_HOLD_KI    3.1f
#define HEADING_HOLD_KD    0.12f
#define HEADING_HOLD_LIMIT 125
/* 回家倒车专用低积分: 长距离倒车(200cm)中默认 KI=3.1 积分累积会
 * 过冲振荡形成 S 型轨迹, 倒车阶段降到 1.0, 完成后再恢复默认. */
#define HEADING_HOLD_DRIVE_KI  1.0f
#define HEADING_HOLD_I_LIMIT 44.0f
#define HEADING_HOLD_DEADBAND 2.0f
#define HEADING_HOLD_SIGN -1

extern volatile int heading_correction;

void HeadingHold_Init(void);
void HeadingHold_Reset(void);
void HeadingHold_SetGains(float kp, float ki, float kd, int limit);
void HeadingHold_GetGains(float *kp, float *ki, float *kd, int *limit);
int  HeadingHold_GetFreshCorrection(void);
float HeadingHold_WrapAngle180(float angle);

#endif
