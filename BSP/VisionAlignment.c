#include "VisionAlignment.h"
#include "MaxiCam.h"

#define ALIGN_TARGET_X       80
#define ALIGN_TARGET_Y       60
#define ALIGN_ENTER_BAND      10   /* ±10px: 判定完成 */
#define ALIGN_HYSTERESIS      14   /* 进入后误差需超过±14px才允许再调 */
#define ALIGN_COARSE_ERROR    18   /* >18px: 编码器闭环粗调 */
#define ALIGN_MIN_SPEED       80
#define ALIGN_MAX_SPEED       90
#define ALIGN_FINE_SPEED      80
#define ALIGN_KP              5
#define ALIGN_X_MOVE_SIGN    -1
#define ALIGN_Y_MOVE_SIGN    -1
#define ALIGN_FRAME_MAX_AGE 800U
#define ALIGN_TIMEOUT_MS    2500U
#define ALIGN_COARSE_PX_TO_COUNTS 70   /* 1px ≈ 70 counts (0.38mm) */
#define ALIGN_COARSE_MIN     1110       /* 6mm 下限 */
#define ALIGN_COARSE_MAX     2775       /* 15mm 上限 */
#define ALIGN_FINE_PULSE_MS   35U
#define ALIGN_FINE_SETTLE_MS 100U
#define ALIGN_FINISH_HOLD_MS  60U
#define ALIGN_MAX_PULSES       4U

static volatile VisionAlignment_Status align_status;
static volatile int align_target[4];
static uint32_t align_started_ms;
static uint32_t align_next_command_ms;
static uint32_t align_last_sequence;
static uint8_t  align_filter_count;
static uint8_t  align_filter_index;
static uint8_t  align_x_history[3];
static uint8_t  align_y_history[3];
static uint32_t align_pulse_until_ms;
static uint32_t align_settle_until_ms;
static uint32_t align_finish_at_ms;
static uint8_t  align_pulse_count;
static uint8_t  align_in_band;          /* 滞环: 是否曾进入允许范围 */
static uint8_t  align_coarse_pending;   /* 粗调待执行 */
static uint8_t  align_coarse_axis;      /* 0=Y(前后), 1=X(左右) */
static int16_t  align_coarse_dist;      /* 编码器 counts, 带符号 */

volatile uint8_t  vision_diag_code;
volatile uint8_t  vision_diag_mode;
volatile uint8_t  vision_diag_x;
volatile uint8_t  vision_diag_y;
volatile uint32_t vision_diag_seq;
volatile uint32_t vision_diag_age_ms;

static uint8_t VisionAlignment_Median3(const uint8_t value[3])
{
    uint8_t a = value[0];
    uint8_t b = value[1];
    uint8_t c = value[2];
    if (a > b) { uint8_t t = a; a = b; b = t; }
    if (b > c) { uint8_t t = b; b = c; c = t; }
    if (a > b) b = a;
    return b;
}

static int VisionAlignment_Speed(int error)
{
    int magnitude = (error >= 0) ? error : -error;
    int speed = magnitude * ALIGN_KP;
    if (speed < ALIGN_MIN_SPEED) speed = ALIGN_MIN_SPEED;
    if (speed > ALIGN_MAX_SPEED) speed = ALIGN_MAX_SPEED;
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

void VisionAlignment_Init(UART_HandleTypeDef *uart)
{
    MaxiCam_Init(uart);
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

void VisionAlignment_Start(void)
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
    /* 铁律: 摄像头可能还在其它模式循环里(如奖杯左移后的 0x02),
     * 0x01 会被吞掉, 必须先发 0x04 退回外层循环. */
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
}

/* 仅复位视觉状态和计数器, 不改变摄像头模式(用于连续放置间复用 CIRCLE). */
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

/* Resume the same placement after an encoder coarse correction.  Keep the
 * original start time and correction count so a sequence of coarse moves
 * cannot bypass ALIGN_TIMEOUT_MS or ALIGN_MAX_PULSES. */
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

void VisionAlignment_Process(void)
{
    MaxiCam_Frame frame;
    uint32_t now;

    if (align_status != VISION_ALIGN_RUNNING) { vision_diag_code = VISION_DIAG_IDLE; return; }

    /* 粗调 ChassisPos 待执行: 状态机执行期间暂停视觉处理 */
    if (align_coarse_pending) { vision_diag_code = VISION_DIAG_COARSE; return; }

    now = HAL_GetTick();

    if ((now - align_started_ms) >= ALIGN_TIMEOUT_MS)
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
        align_settle_until_ms = now + ALIGN_FINE_SETTLE_MS;
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

    vision_diag_mode = frame.data[0];
    vision_diag_x    = frame.data[2];
    vision_diag_y    = frame.data[3];
    vision_diag_seq  = frame.sequence;
    vision_diag_age_ms = now - frame.received_at_ms;

    if ((frame.data[0] != 1U) ||
        (frame.data[2] >= 160U) || (frame.data[3] >= 120U))
    {
        VisionAlignment_SetTarget(0, 0);
        vision_diag_code = (frame.data[0] != 1U) ? VISION_DIAG_BAD_MODE : VISION_DIAG_BAD_XY;
        return;
    }

    if (frame.sequence == align_last_sequence) return;
    align_last_sequence = frame.sequence;

    {
        uint8_t filtered_x;
        uint8_t filtered_y;
        int error_x;
        int error_y;
        int abs_x;
        int abs_y;

        align_x_history[align_filter_index] = frame.data[2];
        align_y_history[align_filter_index] = frame.data[3];
        align_filter_index = (uint8_t)((align_filter_index + 1U) % 3U);
        if (align_filter_count < 3U) align_filter_count++;

        if (align_filter_count < 3U)
        {
            filtered_x = frame.data[2];
            filtered_y = frame.data[3];
        }
        else
        {
            filtered_x = VisionAlignment_Median3(align_x_history);
            filtered_y = VisionAlignment_Median3(align_y_history);
        }

        error_x = (int)filtered_x - ALIGN_TARGET_X;
        error_y = (int)filtered_y - ALIGN_TARGET_Y;
        abs_x = (error_x >= 0) ? error_x : -error_x;
        abs_y = (error_y >= 0) ? error_y : -error_y;

        /* 进入允许范围: 立即停止, 设置滞环 */
        if ((abs_x <= ALIGN_ENTER_BAND) &&
            (abs_y <= ALIGN_ENTER_BAND))
        {
            align_in_band = 1U;
            VisionAlignment_SetTarget(0, 0);
            vision_diag_code = VISION_DIAG_CENTERED;
            align_pulse_until_ms = 0U;
            align_settle_until_ms = 0U;
            align_finish_at_ms = now + ALIGN_FINISH_HOLD_MS;
            return;
        }

        /* 滞环: 已进入过允许范围且误差未超过回差, 保持完成 */
        if (align_in_band &&
            (abs_x <= ALIGN_HYSTERESIS) &&
            (abs_y <= ALIGN_HYSTERESIS))
        {
            VisionAlignment_SetTarget(0, 0);
            vision_diag_code = VISION_DIAG_CENTERED;
            align_pulse_until_ms = 0U;
            align_settle_until_ms = 0U;
            align_finish_at_ms = now + ALIGN_FINISH_HOLD_MS;
            return;
        }

        /* 误差超过滞环阈值: 清除 in_band, 允许重新调整 */
        if ((abs_x > ALIGN_HYSTERESIS) || (abs_y > ALIGN_HYSTERESIS))
            align_in_band = 0U;

        vision_diag_code = VISION_DIAG_MOVING;

        /*
         * Y 轴优先: 先修正前后方向, Y 合格后才修左右.
         * 禁止同时修正两个方向.
         */
        if (abs_y > ALIGN_ENTER_BAND)
        {
            int move_sign = (error_y >= 0) ? ALIGN_Y_MOVE_SIGN
                                           : -ALIGN_Y_MOVE_SIGN;

            if (abs_y > ALIGN_COARSE_ERROR)
            {
                /* 粗调: 编码器位置闭环 */
                int32_t dist = (int32_t)abs_y * ALIGN_COARSE_PX_TO_COUNTS;
                if (align_pulse_count >= ALIGN_MAX_PULSES)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                if (dist < ALIGN_COARSE_MIN) dist = ALIGN_COARSE_MIN;
                if (dist > ALIGN_COARSE_MAX) dist = ALIGN_COARSE_MAX;
                align_pulse_count++;
                align_coarse_dist  = (int16_t)(move_sign * dist);
                align_coarse_axis  = 0U;  /* Y 轴 */
                align_coarse_pending = 1U;
                VisionAlignment_SetTarget(0, 0);
            }
            else
            {
                /* 精调: 短时间轮速脉冲 */
                /* move_sign already contains the error direction.  Applying
                 * the error sign to movement again would cancel it and make
                 * both positive and negative errors drive the same way. */
                VisionAlignment_SetTarget(move_sign * ALIGN_FINE_SPEED, 0);
                if (align_pulse_count >= ALIGN_MAX_PULSES)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                align_pulse_count++;
                align_pulse_until_ms = now + ALIGN_FINE_PULSE_MS;
            }
        }
        else if (abs_x > ALIGN_ENTER_BAND)
        {
            int move_sign = (error_x >= 0) ? ALIGN_X_MOVE_SIGN
                                           : -ALIGN_X_MOVE_SIGN;

            if (abs_x > ALIGN_COARSE_ERROR)
            {
                int32_t dist = (int32_t)abs_x * ALIGN_COARSE_PX_TO_COUNTS;
                if (align_pulse_count >= ALIGN_MAX_PULSES)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                if (dist < ALIGN_COARSE_MIN) dist = ALIGN_COARSE_MIN;
                if (dist > ALIGN_COARSE_MAX) dist = ALIGN_COARSE_MAX;
                align_pulse_count++;
                align_coarse_dist  = (int16_t)(move_sign * dist);
                align_coarse_axis  = 1U;  /* X 轴 */
                align_coarse_pending = 1U;
                VisionAlignment_SetTarget(0, 0);
            }
            else
            {
                VisionAlignment_SetTarget(0, move_sign * ALIGN_FINE_SPEED);
                if (align_pulse_count >= ALIGN_MAX_PULSES)
                {
                    VisionAlignment_SetTarget(0, 0);
                    align_status = VISION_ALIGN_TIMEOUT;
                    vision_diag_code = VISION_DIAG_TIMEOUT;
                    return;
                }
                align_pulse_count++;
                align_pulse_until_ms = now + ALIGN_FINE_PULSE_MS;
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

/* 仅清零轮速和状态, 不发送摄像头 STOP.
 * 用于连续放置间切换: 保持 CIRCLE 模式不中断. */
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
