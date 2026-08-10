#include "Servo4.h"
#include "Task1Mechanism.h"
#include "TrophyMechanism.h"

/* =================================================================== *
 *  底层：8 路舵机 TIM9 硬件 PWM (20ms 帧周期)
 * =================================================================== */

#define SERVO4_FRAME_US 20000U

/* CH2 旋转舵机: 840us=零点, 每步 +400us (约 72°), 共 0~4 五个档位 */
#define ROTATION_SERVO_CHANNEL       2U
#define ROTATION_SERVO_HOME_US     840U
#define ROTATION_SERVO_STEP_US     400U
#define ROTATION_SERVO_MAX_STEP      4U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} Servo4Pin;

/* 通道-引脚-功能对照 (与 Servo4.h 头注释一致):
 * CH0/PC2 大臂 | CH1/PC3 小臂 | CH2/PA4 旋转(360°转盘) | CH3~CH7 小爪子 */
static const Servo4Pin servo4_pins[SERVO4_CHANNEL_COUNT] =
{
    {GPIOC, GPIO_PIN_2},   /* CH0: 大臂 */
    {GPIOC, GPIO_PIN_3},   /* CH1: 小臂 */
    {GPIOA, GPIO_PIN_4},   /* CH2: 旋转 360° 位置舵机 */
    {GPIOA, GPIO_PIN_5},   /* CH3: 小爪子 1 */
    {GPIOA, GPIO_PIN_6},   /* CH4: 小爪子 2 */
    {GPIOA, GPIO_PIN_7},   /* CH5: 小爪子 3 */
    {GPIOC, GPIO_PIN_4},   /* CH6: 小爪子 4 */
    {GPIOC, GPIO_PIN_5}    /* CH7: 小爪子 5 */
};

static volatile uint16_t servo4_pulse[SERVO4_CHANNEL_COUNT];
static uint16_t servo4_frame_pulse[SERVO4_CHANNEL_COUNT];
static uint16_t servo4_elapsed_us;
static uint8_t servo4_frame_start;

/* CH2 锁死状态 (灰度识别到十字后、90°旋转前 PWM -200, 旋转完解锁) */
static uint16_t rotation_pre_lock_us;

static void Servo4_PinWrite(uint8_t channel, GPIO_PinState state)
{
    HAL_GPIO_WritePin(servo4_pins[channel].port,
                      servo4_pins[channel].pin, state);
}

static void Servo4_ScheduleUs(uint16_t delay_us)
{
    if (delay_us == 0U) delay_us = 1U;
    TIM9->ARR = (uint32_t)delay_us - 1U;
    TIM9->CNT = 0U;
}

void Servo4_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    uint32_t timer_clock;
    uint8_t channel;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    HAL_GPIO_Init(GPIOA, &gpio);

    for (channel = 0U; channel < SERVO4_CHANNEL_COUNT; channel++)
    {
        servo4_pulse[channel] = 0U;
        servo4_frame_pulse[channel] = 0U;
        Servo4_PinWrite(channel, GPIO_PIN_RESET);
    }

    timer_clock = HAL_RCC_GetPCLK2Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1)
    {
        timer_clock *= 2U;
    }

    TIM9->CR1 = 0U;
    TIM9->PSC = (timer_clock / 1000000U) - 1U;
    TIM9->ARR = 99U;
    TIM9->CNT = 0U;
    TIM9->SR = 0U;
    TIM9->DIER = TIM_DIER_UIE;
    servo4_elapsed_us = 0U;
    servo4_frame_start = 1U;
    /* 优先级 0 (最高): 软件 PWM 边沿绝不能被抢占, 否则脉宽漂移导致舵机
     * 异常转动(巡线时小臂抬升/砸下). TIM9 ISR 仅 ~30us, 抢占 TIM7 电机
     * 控制每 5ms 最多 30us, 不影响. */
    HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    TIM9->CR1 = TIM_CR1_CEN;
}

void Servo4_SetPulse(uint8_t channel, uint16_t pulse_us)
{
    if (channel >= SERVO4_CHANNEL_COUNT) return;
    if (pulse_us != 0U)
    {
        if (pulse_us < SERVO4_MIN_US) pulse_us = SERVO4_MIN_US;
        if (pulse_us > SERVO4_MAX_US) pulse_us = SERVO4_MAX_US;
    }
    servo4_pulse[channel] = pulse_us;
}

void Servo4_TIM9_IRQHandler(void)
{
    uint16_t next_edge = SERVO4_FRAME_US;
    uint8_t channel;

    TIM9->SR &= ~TIM_SR_UIF;

    if (servo4_frame_start)
    {
        for (channel = 0U; channel < SERVO4_CHANNEL_COUNT; channel++)
        {
            servo4_frame_pulse[channel] = servo4_pulse[channel];
            if (servo4_frame_pulse[channel] != 0U)
            {
                Servo4_PinWrite(channel, GPIO_PIN_SET);
                if (servo4_frame_pulse[channel] < next_edge)
                    next_edge = servo4_frame_pulse[channel];
            }
            else
            {
                Servo4_PinWrite(channel, GPIO_PIN_RESET);
            }
        }

        servo4_elapsed_us = 0U;
        if (next_edge < SERVO4_FRAME_US)
        {
            servo4_elapsed_us = next_edge;
            servo4_frame_start = 0U;
            Servo4_ScheduleUs(next_edge);
        }
        else
        {
            Servo4_ScheduleUs(SERVO4_FRAME_US);
        }
        return;
    }

    for (channel = 0U; channel < SERVO4_CHANNEL_COUNT; channel++)
    {
        if ((servo4_frame_pulse[channel] != 0U) &&
            (servo4_frame_pulse[channel] <= servo4_elapsed_us))
        {
            Servo4_PinWrite(channel, GPIO_PIN_RESET);
        }
        else if ((servo4_frame_pulse[channel] > servo4_elapsed_us) &&
                 (servo4_frame_pulse[channel] < next_edge))
        {
            next_edge = servo4_frame_pulse[channel];
        }
    }

    if (next_edge < SERVO4_FRAME_US)
    {
        uint16_t delay_us = next_edge - servo4_elapsed_us;
        servo4_elapsed_us = next_edge;
        Servo4_ScheduleUs(delay_us);
    }
    else
    {
        Servo4_ScheduleUs(SERVO4_FRAME_US - servo4_elapsed_us);
        servo4_frame_start = 1U;
    }
}

/* =================================================================== *
 *  高层：比赛舵机姿态控制
 * =================================================================== */

void Servo4_UseTask1Parameters(void)
{
    const Task1ArmPose *pose = Task1Mechanism_GetWorkingPose();
    Servo4_SetPulse(0U, pose->big_arm_us);
    Servo4_SetPulse(1U, pose->small_arm_us);
}

void Servo4_UseTask2Parameters(void)
{
    const TrophyArmPose *pose = TrophyMechanism_GetReadyPose();
    Servo4_SetPulse(0U, pose->big_arm_us);
    Servo4_SetPulse(1U, pose->small_arm_us);
}

void Servo4_RotationHome(void)
{
    rotation_pre_lock_us = 0U;  /* 回零清锁死状态 */
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, ROTATION_SERVO_HOME_US);
}

void Servo4_RotationSetStep(uint8_t step)
{
    if (step > ROTATION_SERVO_MAX_STEP) step = ROTATION_SERVO_MAX_STEP;
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL,
                    (uint16_t)(ROTATION_SERVO_HOME_US
                               + ROTATION_SERVO_STEP_US * step));
}

/* CH2 锁死: 灰度识别到十字后、90°旋转前 PWM 减 200us 卡住物块防掉落;
 * 旋转完成后 Unlock 恢复锁死前的 PWM. */
#define ROTATION_LOCK_DELTA_US  200U

void Servo4_RotationLockNow(void)
{
    uint16_t cur;

    if (rotation_pre_lock_us != 0U) return;  /* 已锁死, 不重复 */
    cur = servo4_pulse[ROTATION_SERVO_CHANNEL];
    if (cur > (SERVO4_MIN_US + ROTATION_LOCK_DELTA_US))
    {
        rotation_pre_lock_us = cur;
        Servo4_SetPulse(ROTATION_SERVO_CHANNEL,
                        (uint16_t)(cur - ROTATION_LOCK_DELTA_US));
    }
}

void Servo4_RotationUnlock(void)
{
    if (rotation_pre_lock_us != 0U)
    {
        Servo4_SetPulse(ROTATION_SERVO_CHANNEL, rotation_pre_lock_us);
        rotation_pre_lock_us = 0U;
    }
}

/* 小爪子通道: CH3~CH7 (PA5/PA6/PA7/PC4/PC5) */
static const uint8_t little_channels[] = {3U, 4U, 5U, 6U, 7U};

void Servo4_LittleAllOpen(void)
{
    uint8_t i;
    for (i = 0U; i < sizeof(little_channels); i++)
        Servo4_SetPulse(little_channels[i], LITTLE_SERVO_OPEN_US);
}

void Servo4_LittleAllClose(void)
{
    uint8_t i;
    for (i = 0U; i < sizeof(little_channels); i++)
        Servo4_SetPulse(little_channels[i], LITTLE_SERVO_CLOSE_US);
}
