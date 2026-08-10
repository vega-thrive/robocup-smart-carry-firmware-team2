#include "VisionAlignment.h"
#include "MaxiCam.h"

#define ALIGN_TARGET_X       80
#define ALIGN_TARGET_Y       60
#define ALIGN_ENTER_BAND       5
#define ALIGN_COARSE_ERROR    10
#define ALIGN_MIN_SPEED     120
#define ALIGN_MAX_SPEED     150
#define ALIGN_FINE_SPEED    100
#define ALIGN_KP              8
#define ALIGN_X_MOVE_SIGN    -1
#define ALIGN_Y_MOVE_SIGN    -1
#define ALIGN_FRAME_MAX_AGE 800U
#define ALIGN_TIMEOUT_MS   12000U
#define ALIGN_FINE_PULSE_MS   45U
#define ALIGN_FINE_SETTLE_MS 120U
#define ALIGN_FINISH_HOLD_MS 180U

static volatile VisionAlignment_Status align_status;
static volatile int align_target[4];
static uint32_t align_started_ms;
static uint32_t align_next_command_ms;
static uint32_t align_last_sequence;
static uint8_t align_filter_count;
static uint8_t align_filter_index;
static uint8_t align_x_history[3];
static uint8_t align_y_history[3];
static uint32_t align_pulse_until_ms;
static uint32_t align_settle_until_ms;
static uint32_t align_finish_at_ms;

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
    /* 铁律: 摄像头可能还在其它模式循环里(如奖杯左移后的 0x02),
     * 0x01 会被吞掉, 必须先发 0x04 退回外层循环. */
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
}

void VisionAlignment_Process(void)
{
    MaxiCam_Frame frame;
    uint32_t now;

    if (align_status != VISION_ALIGN_RUNNING) { vision_diag_code = VISION_DIAG_IDLE; return; }
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

        /*
         * Three-frame median filtering has already rejected isolated coordinate
         * spikes.  Once the filtered point enters the acceptable window, capture
         * it: stop immediately and ignore later camera jitter during the hold.
         * This prevents alternating corrections around the optical centre.
         */
        if ((abs_x <= ALIGN_ENTER_BAND) &&
            (abs_y <= ALIGN_ENTER_BAND))
        {
            VisionAlignment_SetTarget(0, 0);
            vision_diag_code = VISION_DIAG_CENTERED;
            align_pulse_until_ms = 0U;
            align_settle_until_ms = 0U;
            align_finish_at_ms = now + ALIGN_FINISH_HOLD_MS;
        }
        else
        {
            vision_diag_code = VISION_DIAG_MOVING;
            int movement;
            int movement_sign;

            if (abs_y > ALIGN_ENTER_BAND)
            {
                movement = VisionAlignment_Speed(error_y);
                movement_sign = ALIGN_Y_MOVE_SIGN;
                if (abs_y <= ALIGN_COARSE_ERROR)
                    movement = (movement >= 0) ? ALIGN_FINE_SPEED
                                               : -ALIGN_FINE_SPEED;
                VisionAlignment_SetTarget(movement_sign * movement, 0);
            }
            else
            {
                movement = VisionAlignment_Speed(error_x);
                movement_sign = ALIGN_X_MOVE_SIGN;
                if (abs_x <= ALIGN_COARSE_ERROR)
                    movement = (movement >= 0) ? ALIGN_FINE_SPEED
                                               : -ALIGN_FINE_SPEED;
                VisionAlignment_SetTarget(0, movement_sign * movement);
            }

            if (((abs_y > ALIGN_ENTER_BAND) &&
                 (abs_y <= ALIGN_COARSE_ERROR)) ||
                ((abs_y <= ALIGN_ENTER_BAND) &&
                 (abs_x <= ALIGN_COARSE_ERROR)))
                align_pulse_until_ms = now + ALIGN_FINE_PULSE_MS;
        }
    }
}

void VisionAlignment_Cancel(void)
{
    VisionAlignment_SetTarget(0, 0);
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
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
