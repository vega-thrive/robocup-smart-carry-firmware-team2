#ifndef __TROPHY_MECHANISM_H__
#define __TROPHY_MECHANISM_H__

#include <stdint.h>

/* 视觉微调期间小臂临时抬高量, 防止转盘前端磕碰领奖台.
 * 若实车方向相反(抬小臂反而下压), 改成 -200U (减 200us). */
#define TROPHY_ALIGN_SMALL_ARM_LIFT_US  200U

/* 杯间运输姿态: 冠军抬头 = 冠军基础 + 200us 抬头量.
 * 所有领奖台之间统一使用此姿态, 提高机构避免撞台.
 * 大臂=1000 (冠军基础), 小臂=1300 (冠军基础1100 + 200). */
#define TROPHY_TRANSPORT_BIG_US   1000U
#define TROPHY_TRANSPORT_SMALL_US 1300U

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

/*
 * 放置流程三阶段小臂参数:
 *   align:   视觉微调抬头 (formal + 200us)
 *   release: 奖杯接近台面 (formal + 50~100us)
 *   clear:   开爪后脱离姿态 (formal + 100~150us)
 *
 * 大臂在三个阶段保持一致, 仅小臂升降.
 * 具体值需实车标定, 增减方向取决于机械结构.
 */

static inline uint16_t TrophyMechanism_AlignSmallUs(TrophyPoseType type)
{
    uint16_t base = TrophyMechanism_GetPose(type)->small_arm_us;
    return (uint16_t)(base + TROPHY_ALIGN_SMALL_ARM_LIFT_US);
}

static inline uint16_t TrophyMechanism_ReleaseSmallUs(TrophyPoseType type)
{
    /* 释放: 比正式姿态略高, 让奖杯接近台面但不完全落下 */
    if (type == TROPHY_POSE_PODIUM)   return 1275U;
    if (type == TROPHY_POSE_CHAMPION) return 1175U;
    return 1075U; /* LIFTED */
}

static inline uint16_t TrophyMechanism_ClearSmallUs(TrophyPoseType type)
{
    /* 脱离: 比释放姿态抬高约 100us, 夹爪退出奖杯 */
    if (type == TROPHY_POSE_PODIUM)   return 1350U;
    if (type == TROPHY_POSE_CHAMPION) return 1250U;
    return 1150U; /* LIFTED */
}

#endif
