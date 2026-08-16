#include "HeadingHold.h"
#include "HWT101.h"

volatile int heading_correction;
static float heading_integral;
static uint32_t last_heading_tick;
static float last_heading_error;
static uint8_t heading_error_valid;
static uint32_t last_heading_frame;

/* 运行时增益 (初值=宏默认, VofaTune 在线可调) */
static float heading_kp = HEADING_HOLD_KP;
static float heading_ki = HEADING_HOLD_KI;
static float heading_kd = HEADING_HOLD_KD;
static int   heading_limit = HEADING_HOLD_LIMIT;

void HeadingHold_SetGains(float kp, float ki, float kd, int limit)
{
    heading_kp = kp;
    heading_ki = ki;
    heading_kd = kd;
    if (limit < 0) limit = 0;
    if (limit > 400) limit = 400;
    heading_limit = limit;
    HeadingHold_Reset();   /* 清积分/历史误差, 防止输出突变 */
}

void HeadingHold_GetGains(float *kp, float *ki, float *kd, int *limit)
{
    if (kp != NULL)    *kp = heading_kp;
    if (ki != NULL)    *ki = heading_ki;
    if (kd != NULL)    *kd = heading_kd;
    if (limit != NULL) *limit = heading_limit;
}

float HeadingHold_WrapAngle180(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int HeadingHold_GetCorrection(void)
{
    float error;
    float dt;
    float derivative = 0.0f;
    uint32_t now;
    int correction;

    if (!HWT101_IsReady())
    {
        return 0;
    }

    error = HEADING_HOLD_SIGN * Angle;
    if ((error > -HEADING_HOLD_DEADBAND) && (error < HEADING_HOLD_DEADBAND))
    {
        error = 0.0f;
    }

    now = HAL_GetTick();
    dt = (float)(now - last_heading_tick) * 0.001f;
    if (dt < 0.005f) dt = 0.005f;
    if (dt > 0.2f) dt = 0.2f;
    last_heading_tick = now;

    heading_integral += error * dt;
    if (heading_integral > HEADING_HOLD_I_LIMIT) heading_integral = HEADING_HOLD_I_LIMIT;
    if (heading_integral < -HEADING_HOLD_I_LIMIT) heading_integral = -HEADING_HOLD_I_LIMIT;

    if (heading_error_valid)
    {
        derivative = (error - last_heading_error) / dt;
    }
    last_heading_error = error;
    heading_error_valid = 1;

    correction = (int)(heading_kp * error +
                       heading_ki * heading_integral +
                       heading_kd * derivative);
    if (correction > heading_limit) correction = heading_limit;
    if (correction < -heading_limit) correction = -heading_limit;
    return correction;
}

void HeadingHold_Reset(void)
{
    heading_correction = 0;
    heading_integral = 0.0f;
    last_heading_tick = HAL_GetTick();
    last_heading_frame = HWT101_GetUpdateCount();
    last_heading_error = 0.0f;
    heading_error_valid = 0;
}

int HeadingHold_GetFreshCorrection(void)
{
    uint32_t frame = HWT101_GetUpdateCount();

    if ((frame != last_heading_frame) && HWT101_IsReady())
    {
        last_heading_frame = frame;
        heading_correction = HeadingHold_GetCorrection();
    }

    return heading_correction;
}

void HeadingHold_Init(void)
{
    HeadingHold_Reset();
}
