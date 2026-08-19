#include "Servo4.h"
#include "Task1Mechanism.h"
#include "TurntableInventory.h"   /* TURRET_SLOT_PWM_OFFSET_* 槽位脉宽偏移 */

/* =================================================================== *
 *  底层：8 路舵机 TIM9 硬件 PWM (20ms 帧周期)
 * =================================================================== */

#define SERVO4_FRAME_US 20000U

/* CH2 旋转舵机: 840us=零点, 每步 +400us (约 72°), 共 0~4 五个档位 */
#define ROTATION_SERVO_CHANNEL       2U
#define ROTATION_SERVO_HOME_US     500U
#define ROTATION_SERVO_STEP_US     400U
#define ROTATION_SERVO_MAX_STEP      4U
#define TROPHY_ROTATION_HOME_US   1260U
#define TROPHY_ROTATION_STEP_US    400U
#define TROPHY_ROTATION_MAX_STEP     2U
#define TROPHY_ROTATION_SAFE_US   1260U

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

uint16_t Servo4_GetPulse(uint8_t channel)
{
    if (channel >= SERVO4_CHANNEL_COUNT) return 0U;
    return servo4_pulse[channel];
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

/* 从当前脉宽缓降至目标, 5us/15ms, 阻塞. */
void Servo4_ArmRampTo(uint16_t tgt0_us, uint16_t tgt1_us)
{
    uint16_t cur0 = servo4_pulse[0U], cur1 = servo4_pulse[1U];
    uint8_t done = 0U;
    while (!done)
    {
        done = 1U;
        if (cur0 < tgt0_us) { cur0 += 5U; if (cur0 > tgt0_us) cur0 = tgt0_us; done = 0U; }
        else if (cur0 > tgt0_us) { cur0 -= 5U; if (cur0 < tgt0_us) cur0 = tgt0_us; done = 0U; }
        if (cur1 < tgt1_us) { cur1 += 5U; if (cur1 > tgt1_us) cur1 = tgt1_us; done = 0U; }
        else if (cur1 > tgt1_us) { cur1 -= 5U; if (cur1 < tgt1_us) cur1 = tgt1_us; done = 0U; }
        Servo4_SetPulse(0U, cur0);
        Servo4_SetPulse(1U, cur1);
        HAL_Delay(15U);
    }
}

/* 非阻塞单步缓降: 每次调用走 5us, 返回 1 表示已到目标. */
uint8_t Servo4_ArmRampTick(uint16_t tgt0_us, uint16_t tgt1_us)
{
    uint16_t cur0 = servo4_pulse[0U], cur1 = servo4_pulse[1U];
    uint8_t done = 1U;

    if (cur0 < tgt0_us) { cur0 += 5U; if (cur0 > tgt0_us) cur0 = tgt0_us; done = 0U; }
    else if (cur0 > tgt0_us) { cur0 -= 5U; if (cur0 < tgt0_us) cur0 = tgt0_us; done = 0U; }
    if (cur1 < tgt1_us) { cur1 += 5U; if (cur1 > tgt1_us) cur1 = tgt1_us; done = 0U; }
    else if (cur1 > tgt1_us) { cur1 -= 5U; if (cur1 < tgt1_us) cur1 = tgt1_us; done = 0U; }

    Servo4_SetPulse(0U, cur0);
    Servo4_SetPulse(1U, cur1);
    return done;
}

/* 慢速缓降/缓升, 2us/15ms, 非阻塞, 用于奖杯放置防止动作过快.
 * 与 ArmRampTick 逻辑相同, 仅步长减半. */
uint8_t Servo4_ArmRampTickSlow(uint16_t tgt0_us, uint16_t tgt1_us)
{
    static uint8_t slow_divider = 0U;
    uint16_t cur0 = servo4_pulse[0U], cur1 = servo4_pulse[1U];
    uint8_t done = 1U;

    /* This function is called by the 200 Hz route state machine.  Advance
     * only once every three calls so the physical rate is really 2 us/15 ms,
     * rather than the previous accidental 2 us/5 ms. */
    if ((cur0 == tgt0_us) && (cur1 == tgt1_us))
    {
        slow_divider = 0U;
        return 1U;
    }
    if (++slow_divider < 3U)
        return 0U;
    slow_divider = 0U;

    if (cur0 < tgt0_us) { cur0 += 2U; if (cur0 > tgt0_us) cur0 = tgt0_us; done = 0U; }
    else if (cur0 > tgt0_us) { cur0 -= 2U; if (cur0 < tgt0_us) cur0 = tgt0_us; done = 0U; }
    if (cur1 < tgt1_us) { cur1 += 2U; if (cur1 > tgt1_us) cur1 = tgt1_us; done = 0U; }
    else if (cur1 > tgt1_us) { cur1 -= 2U; if (cur1 < tgt1_us) cur1 = tgt1_us; done = 0U; }

    Servo4_SetPulse(0U, cur0);
    Servo4_SetPulse(1U, cur1);
    return done;
}

/* 转盘缓变: 每次 5us, 非阻塞, 返回 1 表示已到目标.
 * 避免 Servo4_TrophyRotationSafe() 直接跳 PWM 导致转盘横扫范围过大. */
uint8_t Servo4_RotationRampTick(uint16_t target_us)
{
    uint16_t cur = servo4_pulse[ROTATION_SERVO_CHANNEL];
    uint8_t done = 1U;

    if (target_us > SERVO4_MAX_US) target_us = SERVO4_MAX_US;
    if (target_us != 0U && target_us < SERVO4_MIN_US) target_us = SERVO4_MIN_US;

    if (cur < target_us)      { cur += 5U; if (cur > target_us) cur = target_us; done = 0U; }
    else if (cur > target_us) { cur -= 5U; if (cur < target_us) cur = target_us; done = 0U; }

    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, cur);
    return done;
}

void Servo4_RotationHome(void)
{
    rotation_pre_lock_us = 0U;  /* 回零清锁死状态 */
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, ROTATION_SERVO_HOME_US);
}

void Servo4_RotationSetStep(uint8_t step)
{
    static const int16_t pwm_offset[ROTATION_SERVO_MAX_STEP + 1] =
    {
        TURRET_SLOT_PWM_OFFSET_0,
        TURRET_SLOT_PWM_OFFSET_1,
        TURRET_SLOT_PWM_OFFSET_2,
        TURRET_SLOT_PWM_OFFSET_3,
        TURRET_SLOT_PWM_OFFSET_4,
    };

    if (step > ROTATION_SERVO_MAX_STEP) step = ROTATION_SERVO_MAX_STEP;
    /* 与任务1色块收取保持一致：直接并持续输出档位 PWM。 */
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL,
                    (uint16_t)(ROTATION_SERVO_HOME_US
                               + ROTATION_SERVO_STEP_US * step
                               + pwm_offset[step]));
}

/* 解锁并直接跳到目标 PWM, 跳过中间旧值, 避免转盘二次抖动. */
void Servo4_RotationUnlockTo(uint16_t target_us)
{
    rotation_pre_lock_us = 0U;
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, target_us);
}

/* =================================================================== *
 *  任务1 转盘锁死 (f75c177 基线: PWM-200 物理锁死, Unlock 恢复)
 * =================================================================== */

#define ROTATION_LOCK_DELTA_US  200U

void Servo4_Task1RotationLockNow(void)
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

/* =================================================================== *
 *  任务2 转盘锁死 (07409af 基线: 仅记录状态, 不改变 PWM)
 * =================================================================== */

void Servo4_TrophyRotationHome(void)
{
    rotation_pre_lock_us = 0U;
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, TROPHY_ROTATION_HOME_US);
}

void Servo4_TrophyRotationSetStep(uint8_t step)
{
    if (step > TROPHY_ROTATION_MAX_STEP) step = TROPHY_ROTATION_MAX_STEP;
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL,
                    (uint16_t)(TROPHY_ROTATION_HOME_US
                               + TROPHY_ROTATION_STEP_US * step));
}

void Servo4_TrophyRotationLock(void)
{
    if (rotation_pre_lock_us != 0U) return;  /* 已锁死, 不重复 */
    rotation_pre_lock_us = servo4_pulse[ROTATION_SERVO_CHANNEL];
}

void Servo4_TrophyRotationSafe(void)
{
    rotation_pre_lock_us = 0U;
    Servo4_SetPulse(ROTATION_SERVO_CHANNEL, TROPHY_ROTATION_SAFE_US);
}

/* 扩展板通道从 CH1 编号，代码内部从 0 编号，因此板上编号=内部编号+1。
 * 逻辑槽位按实车抓取顺序排列：
 * slot0 -> 板上 CH8 (内部 7)，slot1 -> CH6 (内部 5)，
 * slot2 -> CH4 (内部 3)。剩余 CH5/CH7 作为未使用槽位。
 * 任务1 色块夹爪用此数组 (5 个). */
static const uint8_t little_channels[] = {7U, 5U, 3U, 4U, 6U};

/* 任务2 奖杯夹爪单独映射 (2026-08-19: 物理接线更换, CH4 松动改插 CH5):
 * slot0 -> 板上 CH8 (内部 7), slot1 -> CH6 (内部 5), slot2 -> CH5 (内部 4).
 * 不影响任务1 的 little_channels. */
static const uint8_t trophy_little_channels[] = {7U, 5U, 4U};

/* 任务2各逻辑槽位的独立夹紧 PWM。slot0 是板上 CH8，
 * 实车需要比其他通道多 50us 夹紧量。 */
static const uint16_t little_close_us[5] =
{
    2200U, 2150U, 2150U, 2150U, 2150U
};

/* 每槽位脱钩 PWM, 独立标定.
 * CH3~CH7 虽同型号舵机, 但安装角度和连杆误差不同,
 * 统一值可能导致某些槽位已完全脱开而另一个仍挂杯.
 * 初始值 1350, 实车逐槽标定: 找到刚好脱钩值, 再额外 -30~50us. */
static const uint16_t little_release_us[5] =
{
    1350U, 1320U, 1350U, 1300U, 1350U
};

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

/* =================================================================== *
 *  任务2 小舵机统控 (07409af 基线: OPEN=1450, CLOSE=2060)
 * =================================================================== */

void Servo4_TrophyLittleAllOpen(void)
{
    uint8_t i;
    for (i = 0U; i < sizeof(trophy_little_channels); i++)
        Servo4_SetPulse(trophy_little_channels[i], TROPHY_LITTLE_OPEN_US);
}

void Servo4_TrophyLittleAllClose(void)
{
    uint8_t i;
    for (i = 0U; i < sizeof(trophy_little_channels); i++)
        Servo4_SetPulse(trophy_little_channels[i], little_close_us[i]);
}

void Servo4_TrophyLittleHalfOpenSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(trophy_little_channels)) return;
    Servo4_SetPulse(trophy_little_channels[slot_index], TROPHY_LITTLE_HALF_OPEN_US);
}

void Servo4_TrophyLittleOpenSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(trophy_little_channels)) return;
    Servo4_SetPulse(trophy_little_channels[slot_index], TROPHY_LITTLE_OPEN_US);
}

void Servo4_TrophyLittleReleaseSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(trophy_little_channels)) return;
    Servo4_SetPulse(trophy_little_channels[slot_index], little_release_us[slot_index]);
}

void Servo4_TrophyLittleCloseSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(trophy_little_channels)) return;
    Servo4_SetPulse(trophy_little_channels[slot_index], little_close_us[slot_index]);
}

/* 任务1 按槽位操作 (LITTLE_GRIPPER_SEQ_TEST 等调试用) */
void Servo4_LittleOpenSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(little_channels)) return;
    Servo4_SetPulse(little_channels[slot_index], LITTLE_SERVO_OPEN_US);
}

void Servo4_LittleCloseSlot(uint8_t slot_index)
{
    if (slot_index >= sizeof(little_channels)) return;
    Servo4_SetPulse(little_channels[slot_index], LITTLE_SERVO_CLOSE_US);
}
