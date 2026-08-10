#ifndef __CHASSIS_POS_H__
#define __CHASSIS_POS_H__

#include "main.h"

/* 梯形加减速位置环参数 */
#define POS_ERROR_THRESHOLD       600
#define POS_VEL_THRESHOLD          25
#define POS_DEBOUNCE_TICKS         15
#define POS_MIN_SPEED              35
#define POS_HEADING_CORR_LIMIT      120
#define POS_HEADING_DONE_LIMIT      25
#define POS_ENCODER_TICK_LIMIT    1600
#define POS_BAD_SAMPLE_LIMIT         3
#define POS_STALL_DELTA_LIMIT        4
#define POS_STALL_TICKS            150

extern volatile uint8_t pos_control_active;
extern uint8_t pos_fault;
extern volatile int debug_trap_speed;
extern volatile int debug_trap_position;
extern volatile int debug_trap_error;
extern volatile int debug_trap_left_position;
extern volatile int debug_trap_left_error;

void     ChassisPos_Init(void);
void     ChassisPos_Start(int t1, int t2, int t3, int t4);
uint8_t  ChassisPos_Update(int max_fwd, int max_left, int hc);
int      ChassisPos_GetLeftPosition(void);
void     ChassisPos_ResetEncoder(void);
void     ChassisPos_ResetVelocityPid(void);

void ChassisPos_SetRotate90Target(void);
void ChassisPos_SetCurveTarget(int heading_correction);

void ChassisPos_DrawStatus(uint16_t y, const char *text);
void ChassisPos_FlushDisplay(void);

#endif
