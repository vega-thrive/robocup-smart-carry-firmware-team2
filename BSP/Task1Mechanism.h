#ifndef __TASK1_MECHANISM_H__
#define __TASK1_MECHANISM_H__

#include <stdint.h>

typedef struct
{
    uint16_t big_arm_us;
    uint16_t small_arm_us;
} Task1ArmPose;

/* FLAT 平地工作姿态 */
static inline const Task1ArmPose *Task1Mechanism_GetWorkingPose(void)
{
    static const Task1ArmPose working_pose =
    {
        1250U, /* big arm: CH0 / PC2 */
        1500U  /* small arm: CH1 / PC3 */
    };
    return &working_pose;
}

/* 竖直向上安全起始姿态: 上电从此位缓慢降至工作姿态, 避免猛砸 */
static inline const Task1ArmPose *Task1Mechanism_GetSafeStartPose(void)
{
    static const Task1ArmPose safe_start =
    {
         500U, /* big arm: CH0 / PC2 */
        1500U  /* small arm: CH1 / PC3 */
    };
    return &safe_start;
}

#endif
