#ifndef __TASK1_MECHANISM_H__
#define __TASK1_MECHANISM_H__

#include <stdint.h>

typedef struct
{
    uint16_t big_arm_us;
    uint16_t small_arm_us;
} Task1ArmPose;

/* Task0 竖直向上收拢姿态: 运动中防碰撞已放置物块 */
static inline const Task1ArmPose *Task1Mechanism_GetTask0Pose(void)
{
    static const Task1ArmPose task0 =
    {
         500U, /* big arm: CH0 / PC2 */
        1500U  /* small arm: CH1 / PC3 */
    };
    return &task0;
}

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

#endif
