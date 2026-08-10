#ifndef __HEADING_HOLD_H__
#define __HEADING_HOLD_H__

#include "main.h"

#define HEADING_HOLD_KP    3.0f
#define HEADING_HOLD_KI    3.1f
#define HEADING_HOLD_KD    0.12f
#define HEADING_HOLD_LIMIT 125
#define HEADING_HOLD_I_LIMIT 44.0f
#define HEADING_HOLD_DEADBAND 2.0f
#define HEADING_HOLD_SIGN -1

extern volatile int heading_correction;

void HeadingHold_Init(void);
void HeadingHold_Reset(void);
int  HeadingHold_GetFreshCorrection(void);
float HeadingHold_WrapAngle180(float angle);

#endif
