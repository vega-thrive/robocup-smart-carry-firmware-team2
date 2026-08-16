#ifndef __MOVE_H__
#define __MOVE_H__

#include "main.h"

void Move_Init(void);
void Move_Control(int motoa, int motob, int motoc, int motod);
void Move_Velocity(void);
void Move_Encoder(void);

extern volatile int velocity[5];
extern volatile int encoder[5];
extern volatile int motor_pwm[5];

#endif
