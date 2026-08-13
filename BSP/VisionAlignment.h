#ifndef __VISION_ALIGNMENT_H__
#define __VISION_ALIGNMENT_H__

#include "main.h"

typedef enum
{
    VISION_ALIGN_IDLE = 0,
    VISION_ALIGN_RUNNING,
    VISION_ALIGN_DONE,
    VISION_ALIGN_TIMEOUT
} VisionAlignment_Status;

typedef enum
{
    VISION_PROFILE_TASK1 = 0,
    VISION_PROFILE_TROPHY
} VisionProfile;

/* 视觉诊断码 */
#define VISION_DIAG_IDLE      0
#define VISION_DIAG_NO_FRAME  1
#define VISION_DIAG_BAD_MODE  2
#define VISION_DIAG_BAD_XY    3
#define VISION_DIAG_CENTERED  4
#define VISION_DIAG_TIMEOUT   5
#define VISION_DIAG_MOVING    6
#define VISION_DIAG_COARSE    7

#define ALIGN_COARSE_SPEED    150

/* 视觉配置参数: 任务1/任务2 独立标定, 编译期常量 */
typedef struct
{
    uint16_t target_x;
    uint16_t target_y;
    uint16_t enter_band;
    uint16_t hysteresis;
    uint16_t coarse_error;
    uint8_t  min_speed;
    uint8_t  max_speed;
    uint8_t  fine_speed;
    int8_t   x_move_sign;
    int8_t   y_move_sign;
    uint32_t timeout_ms;
    uint32_t fine_pulse_ms;
    uint32_t fine_settle_ms;
    uint32_t finish_hold_ms;
    uint8_t  max_pulses;        /* 精调脉冲上限, 0=不限 */
    uint8_t  use_hysteresis;    /* 滞环: Task2=1, Task1=0 */
    uint8_t  use_coarse;        /* 编码器粗调: Task2=1, Task1=0 */
    uint8_t  use_kp_speed;      /* KP比例速度: Task1=1, Task2=0 */
    uint16_t coarse_px_to_counts;
    int16_t  coarse_min;
    int16_t  coarse_max;
} VisionProfileCfg;

extern const VisionProfileCfg vision_cfg_task1;
extern const VisionProfileCfg vision_cfg_trophy;

extern volatile uint8_t  vision_diag_code;
extern volatile uint8_t  vision_diag_mode;
extern volatile uint16_t vision_diag_x;
extern volatile uint16_t vision_diag_y;
extern volatile uint32_t vision_diag_seq;
extern volatile uint32_t vision_diag_age_ms;

void VisionAlignment_Init(UART_HandleTypeDef *uart);
void VisionAlignment_Start(VisionProfile profile);
void VisionAlignment_Restart(void);
void VisionAlignment_ResumeAfterCoarse(void);
void VisionAlignment_Process(void);
void VisionAlignment_Cancel(void);
void VisionAlignment_Finish(void);
VisionAlignment_Status VisionAlignment_GetStatus(void);
void VisionAlignment_GetWheelTargets(int target[4]);

uint8_t VisionAlignment_NeedCoarseMove(void);
int16_t VisionAlignment_GetCoarseDistance(void);
uint8_t VisionAlignment_GetCoarseAxis(void);
void    VisionAlignment_ClearCoarseMove(void);

#endif
