#ifndef __SERVO4_H__
#define __SERVO4_H__

#include "main.h"
#include "TrophyMechanism.h"

/* ---- 通道定义 ----
 * CH0 / PC2: 大臂舵机 (Task1 平地 | Trophy 三组位姿)
 * CH1 / PC3: 小臂舵机 (同上)
 * CH2 / PA4: 旋转舵机 360° 位置舵机 (转盘, RotationHome/RotationSetStep)
 * CH3 / PA5: 小爪子 1 (LittleAllOpen/LittleAllClose 统控)
 * CH4 / PA6: 小爪子 2
 * CH5 / PA7: 小爪子 3
 * CH6 / PC4: 小爪子 4
 * CH7 / PC5: 小爪子 5
 */

#define SERVO4_CHANNEL_COUNT 8U
#define SERVO4_MIN_US       500U
#define SERVO4_MAX_US      2500U

/* 小舵机 CH3~CH7 开合角度 (反向: 890us=开, 2000us=合) */
#define LITTLE_SERVO_OPEN_US  1600U
#define LITTLE_SERVO_CLOSE_US 2000U

/* ---- 底层驱动 ---- */
void Servo4_Init(void);
void Servo4_SetPulse(uint8_t channel, uint16_t pulse_us);
uint16_t Servo4_GetPulse(uint8_t channel);
void Servo4_TIM9_IRQHandler(void);

/* ---- 高层舵机控制 ---- */

/* 大臂+小臂：任务1 平地 / 任务2 奖杯台位姿 */
void Servo4_UseTask1Parameters(void);
void Servo4_UseTrophyPose(TrophyPoseType pose);
void Servo4_ArmRampTo(uint16_t tgt0_us, uint16_t tgt1_us);
uint8_t Servo4_ArmRampTick(uint16_t tgt0_us, uint16_t tgt1_us);

/* 旋转舵机 CH2 */
void Servo4_RotationHome(void);
void Servo4_RotationSetStep(uint8_t step);
void Servo4_RotationLockNow(void);
void Servo4_RotationUnlock(void);
void Servo4_RotationUnlockTo(uint16_t target_us);

/* 小舵机 CH3~CH7 全开/全合 */
void Servo4_LittleAllOpen(void);
void Servo4_LittleAllClose(void);

#endif
