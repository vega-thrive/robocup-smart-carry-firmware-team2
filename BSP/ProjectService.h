#ifndef __PROJECT_SERVICE_H__
#define __PROJECT_SERVICE_H__

#include "main.h"

/* 比赛路线状态机 */
typedef enum
{
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
    ROUTE_PLACE
} RouteState;

void ProjectService_Init(void);
void ProjectService_Run(void);
void ProjectService_OnTimerElapsed(TIM_HandleTypeDef *htim);

void ProjectService_SetRouteState(RouteState state);
void ProjectService_SetVisionSkip(uint8_t on_off);
void ProjectService_SetInitialHeading(void);

#endif
