#ifndef __CHASSIS_MOTOR_H__
#define __CHASSIS_MOTOR_H__

#include "main.h"

/* Closed-loop chassis motor boundary.
 * Route code supplies four wheel-speed targets; this module owns encoder
 * sampling, speed PID reset/update and the final PWM polarity conversion. */

void ChassisMotor_Init(float kp, float ki, float kd);
void ChassisMotor_ResetSpeedPid(void);
void ChassisMotor_SetGains(float kp, float ki, float kd);
void ChassisMotor_GetGains(float *kp, float *ki, float *kd);
void ChassisMotor_SetInhibit(uint8_t inhibit);   /* 1=锁存急停, 0=解除 */
uint8_t ChassisMotor_GetInhibit(void);
void ChassisMotor_ResetEncoders(void);
void ChassisMotor_SampleEncoders(void);
void ChassisMotor_IntegrateEncoders(void);
void ChassisMotor_ControlTick(const volatile int wheel_target[5]);

#endif
