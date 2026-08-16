#include "TrapMove.h"

/* 整数开方（牛顿法），避免浮点库依赖 */
static int isqrt(int x)
{
    int r;
    if (x <= 0) return 0;
    r = x;
    while (r > x / r)
        r = (r + x / r) / 2;
    return r;
}

void TrapMove_Init(TrapMove_Planner *tp, int accel, int decel,
                   int min_speed, int error_threshold)
{
    tp->target = 0;
    tp->position = 0;
    tp->cmd_speed = 0;
    tp->accel = accel;
    tp->decel = decel;
    tp->min_speed = min_speed;
    tp->error_threshold = error_threshold;
}

void TrapMove_Reset(TrapMove_Planner *tp, int32_t target)
{
    tp->target = target;
    tp->position = 0;
    tp->cmd_speed = tp->min_speed;
}

int TrapMove_Update(TrapMove_Planner *tp, int32_t delta, int max_speed)
{
    int32_t error;
    int abs_err, sign, abs_spd, desired;

    tp->position += delta;
    error = tp->target - tp->position;

    abs_err = (error >= 0) ? error : -error;
    sign = (error > 0) ? 1 : ((error < 0) ? -1 : 0);
    abs_spd = (tp->cmd_speed >= 0) ? tp->cmd_speed : -tp->cmd_speed;

    if (abs_err <= tp->error_threshold) {
        tp->cmd_speed = 0;
        return 0;
    }

    /* v = sqrt(2 * decel * d)，整数运算，无浮点依赖 */
    desired = isqrt(2 * tp->decel * abs_err);
    if (desired < tp->min_speed) desired = tp->min_speed;
    /* The configured minimum must never override the caller's speed cap.
     * This is important for a low-speed zero-position hold axis. */
    if (desired > max_speed) desired = max_speed;

    if (abs_spd < desired) {
        abs_spd += tp->accel;
        if (abs_spd > desired) abs_spd = desired;
    } else if (abs_spd > desired) {
        abs_spd -= tp->decel;
        if (abs_spd < desired) abs_spd = desired;
    }

    tp->cmd_speed = sign * abs_spd;
    return tp->cmd_speed;
}

int TrapMove_GetError(TrapMove_Planner *tp)
{
    return (int)(tp->target - tp->position);
}
