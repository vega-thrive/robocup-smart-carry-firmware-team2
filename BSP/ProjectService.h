#ifndef __PROJECT_SERVICE_H__
#define __PROJECT_SERVICE_H__

#include "main.h"

/* 视觉任务类型: 色块放置 / 奖杯 */
typedef enum
{
    VISION_JOB_COLOR_PLACE = 0,
    VISION_JOB_TROPHY
} VisionJob;

/* 比赛路线状态机 */
typedef enum
{
    /* ---- 任务1: 已验证基线 (f75c177) ---- */
    ROUTE_HOME_LAUNCH = 0,
    ROUTE_HOME_FIXED_FORWARD,
    ROUTE_HOME_FIXED_PAUSE,
    ROUTE_HOME_FIXED_LEFT,
    ROUTE_HOME_FIXED_SETTLE,
    ROUTE_TRACK_FIRST,
    ROUTE_TRACK_RETURN,
    ROUTE_TROPHY,
    ROUTE_ROTATE_90,
    ROUTE_CURVE,
    ROUTE_PLACE_PREPARE,
    ROUTE_REAR_ALIGN,
    ROUTE_PAUSE,
    ROUTE_REAR_ALIGN_2,
    ROUTE_PAUSE_2,
    ROUTE_FINAL_LEFT_FORWARD,
    ROUTE_FINAL_ROTATE_30,
    ROUTE_FINAL_FORWARD_2,
    ROUTE_FINAL_ROTATE_ALIGN,
    ROUTE_FINAL_LEFT_40,
    ROUTE_FINAL_ROTATE_180,
    ROUTE_PAUSE_GENERIC,
    ROUTE_VISION_ALIGN,
    ROUTE_VISION_FORWARD,
    ROUTE_VISION_BACK,
    ROUTE_PLACE,
    ROUTE_PLACE_ERROR,

    /* ---- 过渡 ---- */
    ROUTE_TASK1_TO_TASK2,

    /* ---- 任务2：收集、放置与回家 ---- */
    ROUTE_TASK2_PREPARE,

    /* ---- 回家 ---- */
    ROUTE_HOME_RIGHT_SHIFT,
    ROUTE_HOME_RETURN_ALIGN,
    ROUTE_HOME_RETURN_DRIVE,

    ROUTE_MISSION_DONE
} RouteState;

void ProjectService_Init(void);
void ProjectService_Run(void);
void ProjectService_OnTimerElapsed(TIM_HandleTypeDef *htim);

void ProjectService_SetRouteState(RouteState state);
void ProjectService_SetVisionSkip(uint8_t on_off);
void ProjectService_SetInitialHeading(void);

#endif
