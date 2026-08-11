#ifndef __TROPHY_MECHANISM_H__
#define __TROPHY_MECHANISM_H__

#include <stdint.h>

/* Task2 奖杯台三组位姿: 抬升(LIFTED) → 冠军台(CHAMPION) → 亚军台(PODIUM) */
typedef enum
{
    TROPHY_POSE_LIFTED = 0,
    TROPHY_POSE_CHAMPION,
    TROPHY_POSE_PODIUM,
    TROPHY_POSE_COUNT
} TrophyPoseType;

typedef struct
{
    uint16_t big_arm_us;
    uint16_t small_arm_us;
} TrophyArmPose;

static inline const TrophyArmPose *TrophyMechanism_GetPose(TrophyPoseType type)
{
    static const TrophyArmPose poses[TROPHY_POSE_COUNT] =
    {
        { 900U, 1000U },  /* LIFTED:   大臂900, 小臂1000 */
        {1000U, 1100U },  /* CHAMPION: 大臂1000, 小臂1100 */
        {1100U, 1200U }   /* PODIUM:   大臂1100, 小臂1200 */
    };
    if ((unsigned)type >= TROPHY_POSE_COUNT) type = TROPHY_POSE_LIFTED;
    return &poses[type];
}

#endif
