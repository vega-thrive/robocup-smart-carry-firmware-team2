#include "VisionAlignment.h"
#include "MaxiCam.h"

/* 共享常量 (两套配置不变) */
#define ALIGN_KP              2
#define ALIGN_FRAME_MAX_AGE 800U

/* =================================================================== *
 *  配置实例: 任务1 (f75c177 基线) / 任务2 (07409af 基线)
 * =================================================================== */

const VisionProfileCfg vision_cfg_task1 =
{
    /* 76实测会进一步右偏，因此改到原中心反方向+4px。 */
    .target_x         = 218,
    .target_y         = 160,
    .enter_band       = 18,
    .hysteresis       = 0,
    .coarse_error     = 26,   /* Task1: 速度切换阈值, 非粗调 */
    .min_speed        = 80,
    .max_speed        = 110,
    .fine_speed       = 100,
    .x_move_sign      = -1,
    .y_move_sign      = -1,
    .timeout_ms       = 12000U,
    .fine_pulse_ms    = 50U,
    .fine_settle_ms   = 120U,
    .finish_hold_ms   = 180U,
    .max_pulses       = 0,    /* 不限 */
    .use_hysteresis   = 0,
    .use_coarse       = 0,
    .use_kp_speed     = 1,
    .coarse_px_to_counts = 0,
    .coarse_min       = 0,
    .coarse_max       = 0,
};

const VisionProfileCfg vision_cfg_trophy =
{
    .target_x         = 208,
    .target_y         = 160,
    .enter_band       = 26,
    .hysteresis       = 36,
    .coarse_error     = 47,
    .min_speed        = 80,
    .max_speed        = 90,
    .fine_speed       = 80,
    .x_move_sign      = -1,
    .y_move_sign      = -1,
    .timeout_ms       = 2500U,
    .fine_pulse_ms    = 35U,
    .fine_settle_ms   = 100U,
    .finish_hold_ms   = 60U,
    .max_pulses       = 4U,
    .use_hysteresis   = 1,
    .use_coarse       = 1,
    .use_kp_speed     = 0,
    .coarse_px_to_counts = 27,
    .coarse_min       = 1110,
    .coarse_max       = 2775,
};

/* =================================================================== *
 *  运行时状态
 * =================================================================== */

static const VisionProfileCfg *active_cfg;
static VisionProfile active_profile_type;
static volatile VisionAlignment_Status align_status;
static volatile int align_target[4];
static uint32_t align_started_ms;
static uint32_t align_next_command_ms;
static uint32_t align_last_sequence;
static uint8_t  align_filter_count;
static uint8_t  align_filter_index;
static uint16_t align_x_history[3];
static uint16_t align_y_history[3];
static uint32_t align_pulse_until_ms;
static uint32_t align_settle_until_ms;
static uint32_t align_finish_at_ms;
static uint8_t  align_pulse_count;
static uint8_t  align_in_band;
static uint8_t  align_coarse_pending;
static uint8_t  align_coarse_axis;
static int16_t  align_coarse_dist;

volatile uint8_t  vision_diag_code;
volatile uint8_t  vision_diag_mode;
volatile uint16_t vision_diag_x;
volatile uint16_t vision_diag_y;
volatile uint32_t vision_diag_seq;
volatile uint32_t vision_diag_age_ms;

/* =================================================================== *
 *  辅助
 * =================================================================== */

static uint16_t VisionAlignment_Median3(const uint16_t value[3])
{
    uint16_t a = value[0];
    uint16_t b = value[1];
    uint16_t c = value[2];
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { uint16_t t = b; b = c; c = t; }
    if (a > b) b = a;
    return b;
}

/* Task1 KP比例速度 */
static int VisionAlignment_Speed(int error, const VisionProfileCfg *cfg)
{
    int magnitude = (error >= 0) ? error : -error;
    int speed = magnitude * ALIGN_KP;
    if (speed < (int)cfg->min_speed) speed = (int)cfg->min_speed;
    if (speed > (int)cfg->max_speed) speed = (int)cfg->max_speed;
    return (error >= 0) ? speed : -speed;
}

static void VisionAlignment_SetTarget(int forward, int left)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    align_target[0] = forward - left;
    align_target[1] = forward + left;
    align_target[2] = forward - left;
    align_target[3] = forward + left;
    if (primask == 0U) __enable_irq();
}

/* =================================================================== *
 *  公共接口
 * =================================================================== */

void VisionAlignment_Init(UART_HandleTypeDef *uart)
{
    MaxiCam_Init(uart);
    active_cfg = &vision_cfg_task1;
    active_profile_type = VISION_PROFILE_TASK1;
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_IDLE;
    align_started_ms = 0U;
    align_next_command_ms = 0U;
    align_last_sequence = 0U;
    align_filter_count = 0U;
    align_filter_index = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    (void)MaxiCam_StartReceiveIT();
}

void VisionAlignment_Start(VisionProfile profile)
{
    if (profile == VISION_PROFILE_TROPHY)
    {
        active_cfg = &vision_cfg_trophy;
        active_profile_type = VISION_PROFILE_TROPHY;
    }
    else
    {
        active_cfg = &vision_cfg_task1;
        active_profile_type = VISION_PROFILE_TASK1;
    }

    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_RUNNING;
    align_started_ms = HAL_GetTick();
    align_next_command_ms = 0U;
    align_last_sequence = 0U;
    align_filter_count = 0U;
    align_filter_index = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
}

void VisionAlignment_Restart(void)
{
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_RUNNING;
    align_started_ms = HAL_GetTick();
    align_next_command_ms = 0U;
    align_last_sequence = 0U;
    align_filter_count = 0U;
    align_filter_index = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
}

void VisionAlignment_ResumeAfterCoarse(void)
{
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_RUNNING;
    align_last_sequence = 0U;
    align_filter_count = 0U;
    align_filter_index = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
}

/* =================================================================== *
 *  主处理: 根据 active_cfg 分叉 Task1 / Task2 算法
 * =================================================================== */

void VisionAlignment_Process(void)
{
    MaxiCam_Frame frame;
    uint16_t frame_x;
    uint16_t frame_y;
    uint32_t now;
    const VisionProfileCfg *cfg = active_cfg;

    if (align_status != VISION_ALIGN_RUNNING) { vision_diag_code = VISION_DIAG_IDLE; return; }

    if (align_coarse_pending) { vision_diag_code = VISION_DIAG_COARSE; return; }

    now = HAL_GetTick();

    if ((now - align_started_ms) >= cfg->timeout_ms)
    {
        VisionAlignment_SetTarget(0, 0);
        align_status = VISION_ALIGN_TIMEOUT;
        vision_diag_code = VISION_DIAG_TIMEOUT;
        return;
    }

    if (align_finish_at_ms != 0U)
    {
        VisionAlignment_SetTarget(0, 0);
        vision_diag_code = VISION_DIAG_CENTERED;
        if ((int32_t)(now - align_finish_at_ms) >= 0)
            align_status = VISION_ALIGN_DONE;
        return;
    }

    if (align_pulse_until_ms != 0U)
    {
        if ((int32_t)(now - align_pulse_until_ms) < 0) { vision_diag_code = VISION_DIAG_MOVING; return; }
        VisionAlignment_SetTarget(0, 0);
        align_pulse_until_ms = 0U;
        align_settle_until_ms = now + cfg->fine_settle_ms;
    }

    if (align_settle_until_ms != 0U)
    {
        VisionAlignment_SetTarget(0, 0);
        vision_diag_code = VISION_DIAG_MOVING;
        if ((int32_t)(now - align_settle_until_ms) < 0) return;
        align_settle_until_ms = 0U;
    }

    if (!MaxiCam_GetLatest(&frame, ALIGN_FRAME_MAX_AGE))
    {
        VisionAlignment_SetTarget(0, 0);
        vision_diag_code = VISION_DIAG_NO_FRAME;
        if ((int32_t)(now - align_next_command_ms) >= 0)
        {
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
            align_next_command_ms = now + 1000U;
        }
        return;
    }

    frame_x = MaxiCam_FrameX(&frame);
    frame_y = MaxiCam_FrameY(&frame);
    vision_diag_mode = frame.data[0];
    vision_diag_x    = frame_x;
    vision_diag_y    = frame_y;
    vision_diag_seq  = frame.sequence;
    vision_diag_age_ms = now - frame.received_at_ms;

    if ((frame.data[0] != 1U) ||
        (frame_x == 0xFFFFU) || (frame_y == 0xFFFFU) ||
        (frame_x >= 416U) || (frame_y >= 320U))
    {
        VisionAlignment_SetTarget(0, 0);
        vision_diag_code = (frame.data[0] != 1U) ? VISION_DIAG_BAD_MODE : VISION_DIAG_BAD_XY;
        return;
    }

    if (frame.sequence == align_last_sequence) return;
    align_last_sequence = frame.sequence;

    {
        uint16_t filtered_x;
        uint16_t filtered_y;
        int error_x;
        int error_y;
        int abs_x;
        int abs_y;

        align_x_history[align_filter_index] = frame_x;
        align_y_history[align_filter_index] = frame_y;
        align_filter_index = (uint8_t)((align_filter_index + 1U) % 3U);
        if (align_filter_count < 3U) align_filter_count++;

        if (align_filter_count < 3U)
        {
            filtered_x = frame_x;
            filtered_y = frame_y;
        }
        else
        {
            filtered_x = VisionAlignment_Median3(align_x_history);
            filtered_y = VisionAlignment_Median3(align_y_history);
        }

        error_x = (int)filtered_x - (int)cfg->target_x;
        error_y = (int)filtered_y - (int)cfg->target_y;
        abs_x = (error_x >= 0) ? error_x : -error_x;
        abs_y = (error_y >= 0) ? error_y : -error_y;

        /* ---- 对准判定: 进入允许范围 ---- */
        if ((abs_x <= (int)cfg->enter_band) &&
            (abs_y <= (int)cfg->enter_band))
        {
            if (cfg->use_hysteresis)
                align_in_band = 1U;
            VisionAlignment_SetTarget(0, 0);
            vision_diag_code = VISION_DIAG_CENTERED;
            align_pulse_until_ms = 0U;
            align_settle_until_ms = 0U;
            align_finish_at_ms = now + cfg->finish_hold_ms;
            return;
        }

        /* ---- 滞环 (Task2 only) ---- */
        if (cfg->use_hysteresis)
        {
            if (align_in_band &&
                (abs_x <= (int)cfg->hysteresis) &&
                (abs_y <= (int)cfg->hysteresis))
            {
                VisionAlignment_SetTarget(0, 0);
                vision_diag_code = VISION_DIAG_CENTERED;
                align_pulse_until_ms = 0U;
                align_settle_until_ms = 0U;
                align_finish_at_ms = now + cfg->finish_hold_ms;
                return;
            }
            if ((abs_x > (int)cfg->hysteresis) ||
                (abs_y > (int)cfg->hysteresis))
                align_in_band = 0U;
        }

        vision_diag_code = VISION_DIAG_MOVING;

        /* ================================================================ *
         *  运动策略分叉
         * ================================================================ */

        if (abs_y > (int)cfg->enter_band)
        {
            int move_sign = (error_y >= 0) ? (int)cfg->y_move_sign
                                           : -(int)cfg->y_move_sign;

            if (cfg->use_coarse && abs_y > (int)cfg->coarse_error)
            {
                /* ---- Task2: 编码器粗调 (Y轴) ---- */
                int32_t dist = (int32_t)abs_y * cfg->coarse_px_to_counts;
                if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                if (dist < (int32_t)cfg->coarse_min) dist = (int32_t)cfg->coarse_min;
                if (dist > (int32_t)cfg->coarse_max) dist = (int32_t)cfg->coarse_max;
                align_pulse_count++;
                align_coarse_dist  = (int16_t)(move_sign * (int)dist);
                align_coarse_axis  = 0U;
                align_coarse_pending = 1U;
                VisionAlignment_SetTarget(0, 0);
            }
            else
            {
                /* ---- 精调 (Y轴) ---- */
                if (cfg->use_kp_speed)
                {
                    /* Task1: KP比例速度, 小误差切换固定速度 */
                    int spd;
                    if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                    {
                        VisionAlignment_SetTarget(0, 0);
                        align_status = VISION_ALIGN_TIMEOUT;
                        vision_diag_code = VISION_DIAG_TIMEOUT;
                        return;
                    }
                    if (abs_y <= (int)cfg->coarse_error)
                    {
                        spd = (error_y >= 0) ? (int)cfg->fine_speed
                                             : -(int)cfg->fine_speed;
                    }
                    else
                    {
                        spd = VisionAlignment_Speed(error_y, cfg);
                    }
                    VisionAlignment_SetTarget(cfg->y_move_sign * spd, 0);
                    if (abs_y <= (int)cfg->coarse_error)
                    {
                        if (cfg->max_pulses) align_pulse_count++;
                        align_pulse_until_ms = now + cfg->fine_pulse_ms;
                    }
                }
                else
                {
                    /* Task2: 固定精调速度, 脉冲限制 */
                    if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                    {
                        VisionAlignment_SetTarget(0, 0);
                        align_status = VISION_ALIGN_TIMEOUT;
                        vision_diag_code = VISION_DIAG_TIMEOUT;
                        return;
                    }
                    VisionAlignment_SetTarget(move_sign * (int)cfg->fine_speed, 0);
                    align_pulse_count++;
                    align_pulse_until_ms = now + cfg->fine_pulse_ms;
                }
            }
        }
        else if (abs_x > (int)cfg->enter_band)
        {
            int move_sign = (error_x >= 0) ? (int)cfg->x_move_sign
                                           : -(int)cfg->x_move_sign;

            if (cfg->use_coarse && abs_x > (int)cfg->coarse_error)
            {
                /* ---- Task2: 编码器粗调 (X轴) ---- */
                int32_t dist = (int32_t)abs_x * cfg->coarse_px_to_counts;
                if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                if (dist < (int32_t)cfg->coarse_min) dist = (int32_t)cfg->coarse_min;
                if (dist > (int32_t)cfg->coarse_max) dist = (int32_t)cfg->coarse_max;
                align_pulse_count++;
                align_coarse_dist  = (int16_t)(move_sign * (int)dist);
                align_coarse_axis  = 1U;
                align_coarse_pending = 1U;
                VisionAlignment_SetTarget(0, 0);
            }
            else
            {
                /* ---- 精调 (X轴) ---- */
                if (cfg->use_kp_speed)
                {
                    /* Task1: KP比例速度 */
                    int spd;
                    if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                    {
                        VisionAlignment_SetTarget(0, 0);
                        align_status = VISION_ALIGN_TIMEOUT;
                        vision_diag_code = VISION_DIAG_TIMEOUT;
                        return;
                    }
                    if (abs_x <= (int)cfg->coarse_error)
                    {
                        spd = (error_x >= 0) ? (int)cfg->fine_speed
                                             : -(int)cfg->fine_speed;
                    }
                    else
                    {
                        spd = VisionAlignment_Speed(error_x, cfg);
                    }
                    VisionAlignment_SetTarget(0, cfg->x_move_sign * spd);
                    if (abs_x <= (int)cfg->coarse_error)
                    {
                        if (cfg->max_pulses) align_pulse_count++;
                        align_pulse_until_ms = now + cfg->fine_pulse_ms;
                    }
                }
                else
                {
                    /* Task2: 固定精调速度 */
                    if (cfg->max_pulses && align_pulse_count >= cfg->max_pulses)
                    {
                        VisionAlignment_SetTarget(0, 0);
                        align_status = VISION_ALIGN_TIMEOUT;
                        vision_diag_code = VISION_DIAG_TIMEOUT;
                        return;
                    }
                    VisionAlignment_SetTarget(0, move_sign * (int)cfg->fine_speed);
                    align_pulse_count++;
                    align_pulse_until_ms = now + cfg->fine_pulse_ms;
                }
            }
        }
    }
}

void VisionAlignment_Cancel(void)
{
    VisionAlignment_SetTarget(0, 0);
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    align_status = VISION_ALIGN_IDLE;
}

void VisionAlignment_Finish(void)
{
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_IDLE;
}

VisionAlignment_Status VisionAlignment_GetStatus(void)
{
    return align_status;
}

void VisionAlignment_GetWheelTargets(int target[4])
{
    uint32_t primask;
    if (target == NULL) return;
    primask = __get_PRIMASK();
    __disable_irq();
    target[0] = align_target[0];
    target[1] = align_target[1];
    target[2] = align_target[2];
    target[3] = align_target[3];
    if (primask == 0U) __enable_irq();
}

uint8_t VisionAlignment_NeedCoarseMove(void)
{
    return align_coarse_pending;
}

int16_t VisionAlignment_GetCoarseDistance(void)
{
    return align_coarse_dist;
}

uint8_t VisionAlignment_GetCoarseAxis(void)
{
    return align_coarse_axis;
}

void VisionAlignment_ClearCoarseMove(void)
{
    align_coarse_pending = 0U;
}
