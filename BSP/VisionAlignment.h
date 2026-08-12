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

/* 视觉诊断码: 标识轮速清零原因 */
#define VISION_DIAG_IDLE      0
#define VISION_DIAG_NO_FRAME  1
#define VISION_DIAG_BAD_MODE  2
#define VISION_DIAG_BAD_XY    3
#define VISION_DIAG_CENTERED  4
#define VISION_DIAG_TIMEOUT   5
#define VISION_DIAG_MOVING    6
#define VISION_DIAG_COARSE    7

#define ALIGN_COARSE_SPEED    150    /* 粗调编码器闭环移动速度 */

extern volatile uint8_t  vision_diag_code;
extern volatile uint8_t  vision_diag_mode;
extern volatile uint8_t  vision_diag_x;
extern volatile uint8_t  vision_diag_y;
extern volatile uint32_t vision_diag_seq;
extern volatile uint32_t vision_diag_age_ms;

void VisionAlignment_Init(UART_HandleTypeDef *uart);
void VisionAlignment_Start(void);
void VisionAlignment_Restart(void);
void VisionAlignment_ResumeAfterCoarse(void);
void VisionAlignment_Process(void);
void VisionAlignment_Cancel(void);
void VisionAlignment_Finish(void);  /* 仅清零轮速+复位状态, 不改变摄像头模式 */
VisionAlignment_Status VisionAlignment_GetStatus(void);
void VisionAlignment_GetWheelTargets(int target[4]);

/* 粗调编码器闭环: Process() 判定需要粗调后, 状态机调用以下接口执行 ChassisPos */
uint8_t VisionAlignment_NeedCoarseMove(void);
int16_t VisionAlignment_GetCoarseDistance(void);
uint8_t VisionAlignment_GetCoarseAxis(void);    /* 0=Y轴(前后), 1=X轴(左右) */
void    VisionAlignment_ClearCoarseMove(void);

#endif
