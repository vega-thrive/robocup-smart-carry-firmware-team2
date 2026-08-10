#include "ChassisPos.h"
#include "TrapMove.h"
#include "ChassisMotor.h"
#include "Moto.h"
#include "LCD.h"
#include "LineFollow.h"
#include <stdio.h>

static TrapMove_Planner trap_fwd;
static TrapMove_Planner trap_left;
volatile uint8_t pos_control_active;
uint8_t pos_fault;
static uint8_t pos_reached_ticks;
static uint8_t pos_skip_first_sample;
static uint8_t pos_bad_sample_ticks;
static uint16_t pos_stall_ticks;
static uint16_t pos_stall_grace;
volatile int debug_trap_speed;
volatile int debug_trap_position;
volatile int debug_trap_error;
volatile int debug_trap_left_position;
volatile int debug_trap_left_error;

static int AbsInt(int value)
{
    return (value >= 0) ? value : -value;
}

static void PositionFault(const char *message)
{
    pos_fault = 1U;
    pos_control_active = 0U;
    pos_reached_ticks = 0U;
    wheel_target[1] = 0;
    wheel_target[2] = 0;
    wheel_target[3] = 0;
    wheel_target[4] = 0;
}

void ChassisPos_Start(int t1, int t2, int t3, int t4)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    TrapMove_Reset(&trap_fwd, (t1 + t2 + t3 + t4) / 4);
    TrapMove_Reset(&trap_left, (-t1 + t2 - t3 + t4) / 4);
    pos_reached_ticks = 0U;
    pos_skip_first_sample = 1U;
    pos_bad_sample_ticks = 0U;
    pos_stall_ticks = 0U;
    pos_stall_grace = 50U;
    pos_fault = 0U;
    ChassisMotor_ResetEncoders();
    pos_control_active = 1U;
    if (primask == 0U) __enable_irq();
}

uint8_t ChassisPos_Update(int max_fwd, int max_left, int hc)
{
    int i;
    int delta_forward;
    int delta_left;
    int error_forward;
    int error_left;
    int speed_forward;
    int speed_left;
    int wheel[4];
    int wheel_peak = 0;
    uint8_t sample_valid = 1U;
    uint8_t position_ok;
    uint8_t velocity_ok;

    if (pos_fault)
    {
        LineFollow_Stop();
        return 0U;
    }

    for (i = 1; i <= 4; i++)
    {
        if (AbsInt(velocity[i]) > POS_ENCODER_TICK_LIMIT)
            sample_valid = 0U;
    }

    if (!sample_valid)
    {
        if (++pos_bad_sample_ticks >= POS_BAD_SAMPLE_LIMIT)
            PositionFault("ENCODER ERROR");
        else
            LineFollow_Stop();
        return 0U;
    }
    pos_bad_sample_ticks = 0U;

    delta_forward = (velocity[1] + velocity[2]
                   + velocity[3] + velocity[4]) / 4;
    delta_left = (-velocity[1] + velocity[2]
                - velocity[3] + velocity[4]) / 4;

    if (pos_skip_first_sample)
    {
        pos_skip_first_sample = 0U;
        delta_forward = 0;
        delta_left = 0;
    }

    speed_forward = TrapMove_Update(&trap_fwd, delta_forward, max_fwd);
    speed_left   = TrapMove_Update(&trap_left, delta_left, max_left);
    debug_trap_speed   = speed_forward;
    debug_trap_position = trap_fwd.position;
    debug_trap_error    = TrapMove_GetError(&trap_fwd);
    error_forward = debug_trap_error;
    error_left    = TrapMove_GetError(&trap_left);
    debug_trap_left_position = trap_left.position;
    debug_trap_left_error    = error_left;

    if (hc > POS_HEADING_CORR_LIMIT) hc = POS_HEADING_CORR_LIMIT;
    if (hc < -POS_HEADING_CORR_LIMIT) hc = -POS_HEADING_CORR_LIMIT;

    wheel[0] = speed_forward - speed_left - hc;
    wheel[1] = speed_forward + speed_left - hc;
    wheel[2] = speed_forward - speed_left + hc;
    wheel[3] = speed_forward + speed_left + hc;

    for (i = 0; i < 4; i++)
    {
        int magnitude = AbsInt(wheel[i]);
        if (magnitude > wheel_peak) wheel_peak = magnitude;
    }
    {
        int wheel_limit = max_fwd + max_left;
        if (wheel_peak > wheel_limit)
        {
            for (i = 0; i < 4; i++)
                wheel[i] = wheel[i] * wheel_limit / wheel_peak;
        }
    }
    LineFollow_SetWheelTargets(wheel[0], wheel[1], wheel[2], wheel[3]);

    position_ok = (AbsInt(error_forward) <= POS_ERROR_THRESHOLD)
               && (AbsInt(error_left) <= POS_ERROR_THRESHOLD);
    velocity_ok = (AbsInt(delta_forward) <= POS_VEL_THRESHOLD)
               && (AbsInt(delta_left) <= POS_VEL_THRESHOLD);

    if (position_ok && velocity_ok
        && (AbsInt(hc) <= POS_HEADING_DONE_LIMIT))
    {
        if (++pos_reached_ticks >= POS_DEBOUNCE_TICKS)
        {
            pos_reached_ticks = 0U;
            pos_control_active = 0U;
            LineFollow_Stop();
            return 1U;
        }
    }
    else
    {
        pos_reached_ticks = 0U;
    }

    if (pos_stall_grace > 0U)
    {
        pos_stall_grace--;
        pos_stall_ticks = 0U;
    }
    else if ((AbsInt(speed_forward) >= POS_MIN_SPEED
              || AbsInt(speed_left) >= POS_MIN_SPEED)
             && (AbsInt(delta_forward) <= POS_STALL_DELTA_LIMIT)
             && (AbsInt(delta_left) <= POS_STALL_DELTA_LIMIT))
    {
        if (++pos_stall_ticks >= POS_STALL_TICKS)
            PositionFault("POSITION STALL");
    }
    else
    {
        pos_stall_ticks = 0U;
    }
    return 0U;
}

void ChassisPos_ResetEncoder(void)
{
    ChassisMotor_ResetEncoders();
    pos_control_active = 0;
}

int ChassisPos_GetLeftPosition(void)
{
    return trap_left.position;
}

void ChassisPos_ResetVelocityPid(void)
{
    ChassisMotor_ResetSpeedPid();
}

void ChassisPos_Init(void)
{
    TrapMove_Init(&trap_fwd, TRAP_ACCEL, TRAP_DECEL,
                    TRAP_MIN_SPEED, TRAP_ERROR_THRESHOLD);
    TrapMove_Init(&trap_left, TRAP_ACCEL, TRAP_DECEL,
                    TRAP_MIN_SPEED, TRAP_ERROR_THRESHOLD);
}

void ChassisPos_SetRotate90Target(void)
{
    wheel_target[1] =  ROTATE_90_SPEED;
    wheel_target[2] =  ROTATE_90_SPEED;
    wheel_target[3] = -ROTATE_90_SPEED;
    wheel_target[4] = -ROTATE_90_SPEED;
}

void ChassisPos_SetCurveTarget(int heading_correction)
{
    wheel_target[1] =  CURVE_FORWARD_SPEED + CURVE_RIGHT_SPEED - heading_correction;
    wheel_target[2] =  CURVE_FORWARD_SPEED - CURVE_RIGHT_SPEED - heading_correction;
    wheel_target[3] =  CURVE_FORWARD_SPEED + CURVE_RIGHT_SPEED + heading_correction;
    wheel_target[4] =  CURVE_FORWARD_SPEED - CURVE_RIGHT_SPEED + heading_correction;
}

/* LCD 显示请求先存入缓冲区, 由主循环 ChassisPos_FlushDisplay 统一绘制.
 * TIM7 控制中断里只存不画, 避免 FSMC 写入阻塞 200Hz 底盘控制. */
#define POS_LCD_TEXT_MAX 24U

static volatile uint16_t pos_lcd_y;
static volatile char pos_lcd_text[POS_LCD_TEXT_MAX];
static volatile uint8_t pos_lcd_pending;

void ChassisPos_DrawStatus(uint16_t y, const char *text)
{
    uint8_t i;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    pos_lcd_y = y;
    for (i = 0U; i < (POS_LCD_TEXT_MAX - 1U); i++)
    {
        if (text[i] == '\0') break;
        pos_lcd_text[i] = text[i];
    }
    pos_lcd_text[i] = '\0';
    pos_lcd_pending = 1U;
    if (primask == 0U) __enable_irq();
}

void ChassisPos_FlushDisplay(void)
{
    uint8_t i;
    uint16_t y;
    char text[POS_LCD_TEXT_MAX];
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (pos_lcd_pending == 0U)
    {
        if (primask == 0U) __enable_irq();
        return;
    }
    y = pos_lcd_y;
    for (i = 0U; i < POS_LCD_TEXT_MAX; i++)
        text[i] = pos_lcd_text[i];
    pos_lcd_pending = 0U;
    if (primask == 0U) __enable_irq();

    LCD_DrawString_NoDMA(1, y, "                    ", LCD_WHITE, LCD_BLACK);
    LCD_DrawString_NoDMA(1, y, text, LCD_WHITE, LCD_BLACK);
}
