#include "VisionAlignment.h"
#include "MaxiCam.h"
#include "Moto.h"

#define VISION_ENTRY_SETTLE_MS  250U
#define VISION_CENTER_SETTLE_MS 200U
#define VISION_STOP_VEL_MAX      20

/* 共享常量 (两套配置不变) */
#define ALIGN_FRAME_MAX_AGE 800U

/* ---- Task1 视觉对准参数 (第一版: 纯P外环) ----
 * MaixCAM 已做 3 帧滤波, F407 不再二次滤波.
 * 帧差 D 项在 15FPS 下不稳定, 第一版禁用 (KD=0). */
#define VISION_KP_X             1.0f
#define VISION_KP_Y             0.65f
#define VISION_KD_X             0.0f
#define VISION_KD_Y             0.0f
#define VISION_MAX_SPEED        70
#define VISION_X_START_SPEED    60    /* 静止启动速度, 实车可调 65/70/80 */
#define VISION_Y_START_SPEED    60
#define VISION_X_HOLD_MIN_SPEED 38    /* 运动中的最低维持速度 */
#define VISION_Y_HOLD_MIN_SPEED 38
#define VISION_ACCEL_STEP       8     /* 仅同方向斜坡, 反向直接停车 */
#define VISION_X_ENTER_BAND     8
#define VISION_Y_ENTER_BAND     8
#define VISION_CENTER_FRAMES    4U
#define TROPHY_CENTER_FRAMES    4U
#define TROPHY_HYST_TIMEOUT_MS   500U  /* 滞环停车超时: 超时后退出滞环继续微调 */
#define TASK1_SEARCH_DELAY_MS    500U
#define TASK1_SEARCH_MOVE_MS     600U
#define TASK1_SEARCH_SETTLE_MS   200U
#define TASK1_SEARCH_SPEED       120
#define VISION_AXIS_HYST        4     /* 轴选择滞环, 防止 X/Y 控制权抖动 */
#define VISION_Y_SETTLE_MS      150U  /* 反向停车后的等待窗口 */
#define VISION_FRAME_STOP_MS    250U  /* 无新序号超过此时长立即停车 (15FPS≈3~4帧, 容忍单帧漏检) */

/* =================================================================== *
 *  配置实例: 任务1 (f75c177 基线) / 任务2 (07409af 基线)
 * =================================================================== */

const VisionProfileCfg vision_cfg_task1 =
{
    /* 与MaixCAM画面红色参考点/CAL_TARGET保持一致。机械末端固定偏移另行标定。 */
    .target_x         = 208,
    .target_y         = 160,
    .enter_band       = 18,
    .hysteresis       = 0,
    .coarse_error     = 26,
    .min_speed        = 80,
    .max_speed        = 80,
    .fine_speed       = 80,
    .x_move_sign      = -1,
    .y_move_sign      = -1,
    .timeout_ms       = 30000U,  /* 丢圆时留出纵向搜索时间 */
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
    .enter_band       = 10,
    .hysteresis       = 28,
    .coarse_error     = 47,
    .min_speed        = 80,
    .max_speed        = 90,
    .fine_speed       = 80,
    .x_move_sign      = -1,
    .y_move_sign      = -1,
    .timeout_ms       = 10000U,
    .fine_pulse_ms    = 15U,
    .fine_settle_ms   = 150U,
    .finish_hold_ms   = 300U,
    .max_pulses       = 12U,
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
static uint32_t align_last_sequence;
static uint32_t align_pulse_until_ms;
static uint32_t align_settle_until_ms;
static uint32_t align_finish_at_ms;
static uint8_t  align_pulse_count;
static uint8_t  align_in_band;
static uint32_t align_hyst_until_ms;   /* 滞环停车超时时刻, 0=未计时 */
static uint8_t  align_coarse_pending;
static uint8_t  align_coarse_axis;
static int16_t  align_coarse_dist;

/* Task1 视觉对准状态 (每次 Start/Restart/Resume 全部清零) */
typedef enum
{
    Y_STATE_IDLE = 0,
    Y_STATE_MOVING,
    Y_STATE_SETTLING
} VisionYState;

static float    vision_error_x_last;
static float    vision_error_y_last;
static int      vision_speed_x;
static int      vision_speed_y;
static uint8_t  vision_y_state;          /* VisionYState */
static uint32_t vision_y_settle_until;
static uint32_t vision_y_stop_sequence;
static uint8_t  vision_center_count;
static uint8_t  vision_center_candidate;
static uint32_t vision_entry_settle_until;
static uint8_t  vision_entry_wait_frame;
static uint32_t vision_center_settle_until;
static uint32_t vision_center_stop_sequence;
static uint8_t  vision_active_axis;      /* 0=X 1=Y 2=未选择 */
static uint32_t vision_last_new_frame_ms;  /* 最后一个新序号帧的时间戳 */
/* 连续无有效圆帧的起始时刻, 0=当前有有效圆. 无效帧(丢帧/BAD_XY)不会刷新它,
 * 保证 Task1 丢圆搜索的 2.5s 延迟不会被摄像头持续发送的无效帧无限重置. */
static uint32_t vision_search_start_ms;

volatile uint8_t  vision_diag_code;
volatile uint8_t  vision_diag_mode;
volatile uint16_t vision_diag_x;
volatile uint16_t vision_diag_y;
volatile uint32_t vision_diag_seq;
volatile uint32_t vision_diag_age_ms;
volatile int16_t  vision_diag_err_x;
volatile int16_t  vision_diag_err_y;
volatile int16_t  vision_diag_cmd_x;
volatile int16_t  vision_diag_cmd_y;
volatile uint8_t  vision_diag_y_state;

/* =================================================================== *
 *  辅助
 * =================================================================== */

/* 同方向目标轮速斜坡: 每帧最大变化 ±ACCEL_STEP.
 * 静止启动和反向不走此函数. */
static int VisionSlewLimit(int target, int previous)
{
    if (target > previous + VISION_ACCEL_STEP)
        return previous + VISION_ACCEL_STEP;
    if (target < previous - VISION_ACCEL_STEP)
        return previous - VISION_ACCEL_STEP;
    return target;
}

static uint8_t VisionAlignment_WheelsStopped(void)
{
    uint8_t i;
    for (i = 1U; i <= 4U; ++i)
    {
        int v = velocity[i];
        if (v < 0) v = -v;
        if (v > VISION_STOP_VEL_MAX)
            return 0U;
    }
    return 1U;
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

/* Task1 丢圆后沿 Y 轴作对称搜索：前、停、后、停、前。
 * 每个完整周期理论净位移为零，避免搜圆动作持续把固定距离误差放大。 */
static void VisionAlignment_Task1SearchY(uint32_t now)
{
    uint32_t base_ms;
    uint32_t phase_ms;
    uint32_t cycle_ms;
    int forward;

    base_ms = (vision_search_start_ms != 0U) ?
              vision_search_start_ms : align_started_ms;
    if ((now - base_ms) < TASK1_SEARCH_DELAY_MS)
    {
        VisionAlignment_SetTarget(0, 0);
        return;
    }

    cycle_ms = 4U * TASK1_SEARCH_MOVE_MS +
               2U * TASK1_SEARCH_SETTLE_MS;
    phase_ms = (now - base_ms - TASK1_SEARCH_DELAY_MS) % cycle_ms;

    if (phase_ms < TASK1_SEARCH_MOVE_MS)
        forward = TASK1_SEARCH_SPEED;
    else if (phase_ms < (TASK1_SEARCH_MOVE_MS + TASK1_SEARCH_SETTLE_MS))
        forward = 0;
    else if (phase_ms < (3U * TASK1_SEARCH_MOVE_MS + TASK1_SEARCH_SETTLE_MS))
        forward = -TASK1_SEARCH_SPEED;
    else if (phase_ms < (3U * TASK1_SEARCH_MOVE_MS +
                         2U * TASK1_SEARCH_SETTLE_MS))
        forward = 0;
    else
        forward = TASK1_SEARCH_SPEED;

    VisionAlignment_SetTarget(forward, 0);
    vision_speed_x = 0;
    vision_speed_y = forward;
    vision_y_state = (forward == 0) ? Y_STATE_IDLE : Y_STATE_MOVING;
    vision_diag_cmd_x = 0;
    vision_diag_cmd_y = (int16_t)forward;
    vision_diag_code = (forward == 0) ? VISION_DIAG_NO_FRAME
                                      : VISION_DIAG_MOVING;
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
    align_last_sequence = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
    vision_entry_settle_until = 0U;
    vision_entry_wait_frame = 0U;
    vision_center_settle_until = 0U;
    vision_center_stop_sequence = 0U;
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
    align_last_sequence = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
    vision_entry_settle_until = align_started_ms + VISION_ENTRY_SETTLE_MS;
    vision_entry_wait_frame = (active_profile_type == VISION_PROFILE_TASK1) ? 1U : 0U;
    vision_center_settle_until = 0U;
    vision_center_stop_sequence = 0U;
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
}

void VisionAlignment_Restart(void)
{
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_RUNNING;
    align_started_ms = HAL_GetTick();
    align_last_sequence = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_pulse_count = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
    vision_entry_settle_until = align_started_ms + VISION_ENTRY_SETTLE_MS;
    vision_entry_wait_frame = (active_profile_type == VISION_PROFILE_TASK1) ? 1U : 0U;
    vision_center_settle_until = 0U;
    vision_center_stop_sequence = 0U;
}

void VisionAlignment_ResumeAfterCoarse(void)
{
    VisionAlignment_SetTarget(0, 0);
    align_status = VISION_ALIGN_RUNNING;
    align_last_sequence = 0U;
    align_pulse_until_ms = 0U;
    align_settle_until_ms = 0U;
    align_finish_at_ms = 0U;
    align_in_band = 0U;
    align_coarse_pending = 0U;
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
    vision_entry_settle_until = HAL_GetTick() + VISION_ENTRY_SETTLE_MS;
    vision_entry_wait_frame = (active_profile_type == VISION_PROFILE_TASK1) ? 1U : 0U;
    vision_center_settle_until = 0U;
    vision_center_stop_sequence = 0U;
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
        vision_speed_x = 0;
        vision_speed_y = 0;
        vision_y_state = Y_STATE_IDLE;
        vision_y_settle_until = 0U;
        vision_center_candidate = 0U;
        vision_center_count = 0U;
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

    /* Task1 entry: wait for chassis inertia to disappear, then discard the
     * latest buffered frame. Only the following new sequence may drive. */
    if ((active_profile_type == VISION_PROFILE_TASK1) &&
        vision_entry_wait_frame)
    {
        VisionAlignment_SetTarget(0, 0);
        vision_speed_x = 0;
        vision_speed_y = 0;
        vision_diag_cmd_x = 0;
        vision_diag_cmd_y = 0;
        vision_diag_code = VISION_DIAG_MOVING;
        if ((int32_t)(now - vision_entry_settle_until) < 0)
            return;
        if (!VisionAlignment_WheelsStopped())
            return;
        if (!MaxiCam_GetLatest(&frame, ALIGN_FRAME_MAX_AGE))
            return;
        align_last_sequence = frame.sequence;
        vision_last_new_frame_ms = now;
        vision_entry_wait_frame = 0U;
        return;
    }

    if (!MaxiCam_GetLatest(&frame, ALIGN_FRAME_MAX_AGE))
    {
        VisionAlignment_SetTarget(0, 0);
        vision_speed_x = 0;  /* 丢帧直接停, 不经过斜坡 */
        vision_speed_y = 0;
        vision_center_count = 0U;
        vision_diag_code = VISION_DIAG_NO_FRAME;
        /* 模式已在Start时可靠切换。丢圆期间不能反复STOP+CIRCLE，
         * 否则摄像头识别循环会不断重启，屏幕已有圆也来不及上报。 */
        if (vision_search_start_ms == 0U)
            vision_search_start_ms = now;
        if (active_profile_type == VISION_PROFILE_TASK1)
            VisionAlignment_Task1SearchY(now);
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
        vision_speed_x = 0;
        vision_speed_y = 0;
        vision_center_count = 0U;
        vision_diag_code = (frame.data[0] != 1U) ? VISION_DIAG_BAD_MODE : VISION_DIAG_BAD_XY;
        if (vision_search_start_ms == 0U)
            vision_search_start_ms = now;
        if (active_profile_type == VISION_PROFILE_TASK1)
            VisionAlignment_Task1SearchY(now);
        return;
    }

    if (frame.sequence == align_last_sequence)
    {
        /* 相机跳变重确认期间会暂停发包: 旧序号不能维持旧轮速.
         * 运动状态超过 FRAME_STOP_MS 没有新序号 -> 立即停车. */
        if ((vision_speed_x != 0) || (vision_speed_y != 0))
        {
            if ((int32_t)(now - vision_last_new_frame_ms) >
                (int32_t)VISION_FRAME_STOP_MS)
            {
                VisionAlignment_SetTarget(0, 0);
                vision_speed_x = 0;
                vision_speed_y = 0;
                vision_y_state = Y_STATE_IDLE;
                vision_diag_cmd_x = 0;
                vision_diag_cmd_y = 0;
            }
        }
        return;
    }
    align_last_sequence = frame.sequence;
    vision_last_new_frame_ms = now;
    /* 已通过 BAD_XY 检查, 确认是新序号的圆帧: 清零丢圆计时 */
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;

    {
        int error_x;
        int error_y;
        int abs_x;
        int abs_y;

        error_x = (int)frame_x - (int)cfg->target_x;
        error_y = (int)frame_y - (int)cfg->target_y;
        abs_x = (error_x >= 0) ? error_x : -error_x;
        abs_y = (error_y >= 0) ? error_y : -error_y;

        if (cfg->use_kp_speed)
        {
            /* ============================================================ *
             *  Task1: 视觉位置纯P外环 (内环为 200Hz 速度 PID)
             *  MaixCAM 已做 3 帧滤波, 此处直接用最新坐标, 不再二次滤波.
             * ============================================================ */

            float ex_f = (float)error_x;
            float ey_f = (float)error_y;

            vision_diag_err_x = (int16_t)error_x;
            vision_diag_err_y = (int16_t)error_y;
            vision_diag_y_state = vision_y_state;

            /* ---- 反向停车后的 SETTLING: 输出 0, 等 150ms + 新视觉帧 ---- */
            if (vision_y_state == Y_STATE_SETTLING)
            {
                VisionAlignment_SetTarget(0, 0);
                vision_diag_cmd_x = 0;
                vision_diag_cmd_y = 0;
                vision_diag_code = VISION_DIAG_MOVING;
                if ((int32_t)(now - vision_y_settle_until) < 0)
                    return;
                if ((int32_t)(frame.sequence - vision_y_stop_sequence) <= 0)
                    return;   /* 丢弃停止前/停止期间的旧帧 */
                vision_y_state = Y_STATE_IDLE;
                /* 本帧是新帧, 继续用本帧重新计算, 不用旧误差反向启动 */
            }

            /* ---- 中心候选滞环: 进带停车, 4 帧确认; 超过退出带才重启 ---- */
            if (!vision_center_candidate)
            {
                if ((abs_x <= VISION_X_ENTER_BAND) &&
                    (abs_y <= VISION_Y_ENTER_BAND))
                {
                    VisionAlignment_SetTarget(0, 0);
                    vision_speed_x = 0;
                    vision_speed_y = 0;
                    vision_y_state = Y_STATE_IDLE;
                    vision_center_candidate = 1U;
                    vision_center_count = 0U;
                    vision_center_settle_until = now + VISION_CENTER_SETTLE_MS;
                    vision_center_stop_sequence = frame.sequence;
                    vision_diag_cmd_x = 0;
                    vision_diag_cmd_y = 0;
                    vision_diag_code = VISION_DIAG_CENTERED;
                    return;
                }
            }
            else
            {
                VisionAlignment_SetTarget(0, 0);
                vision_speed_x = 0;
                vision_speed_y = 0;
                vision_y_state = Y_STATE_IDLE;
                vision_diag_cmd_x = 0;
                vision_diag_cmd_y = 0;
                vision_diag_code = VISION_DIAG_CENTERED;

                /* Frames captured while the chassis is still coasting do not
                 * count. Require both a settled chassis and a post-stop frame. */
                if ((int32_t)(now - vision_center_settle_until) < 0)
                    return;
                if (!VisionAlignment_WheelsStopped())
                {
                    vision_center_count = 0U;
                    return;
                }
                if ((int32_t)(frame.sequence - vision_center_stop_sequence) <= 0)
                    return;

                if ((abs_x <= VISION_X_ENTER_BAND) &&
                    (abs_y <= VISION_Y_ENTER_BAND))
                {
                    if (++vision_center_count >= VISION_CENTER_FRAMES)
                    {
                        vision_center_count = 0U;
                        align_status = VISION_ALIGN_DONE;
                    }
                    return;
                }

                /* A settled fresh frame outside the 8 px acceptance band must
                 * resume correction. Do not park forever in the old 8..12 px
                 * hysteresis zone. */
                vision_center_candidate = 0U;
                vision_center_count = 0U;
                vision_center_settle_until = 0U;
                vision_center_stop_sequence = 0U;
            }

            vision_diag_code = VISION_DIAG_MOVING;
            {
                float dx = ex_f - vision_error_x_last;
                float dy = ey_f - vision_error_y_last;
                float left_cmd;
                float forward_cmd;

                vision_error_x_last = ex_f;
                vision_error_y_last = ey_f;

                /* 纯P: KD 暂为 0 (帧差 D 在 15FPS 下不稳定) */
                left_cmd    = VISION_KP_X * ex_f + VISION_KD_X * dx;
                forward_cmd = VISION_KP_Y * ey_f + VISION_KD_Y * dy;

                left_cmd    *= (float)cfg->x_move_sign;
                forward_cmd *= (float)cfg->y_move_sign;

                /* ---- 单轴选择 + 滞环: 防止 X/Y 控制权频繁切换 ---- */
                {
                    uint8_t sel = (abs_y >= abs_x) ? 1U : 0U;
                    if ((vision_active_axis == 0U) ||
                        (vision_active_axis == 1U))
                    {
                        int cur_err   = (vision_active_axis == 1U) ? abs_y : abs_x;
                        int other_err = (vision_active_axis == 1U) ? abs_x : abs_y;
                        if ((other_err - cur_err) <= VISION_AXIS_HYST)
                            sel = vision_active_axis;
                    }
                    vision_active_axis = sel;
                }

                if (vision_active_axis == 1U)
                {
                    /* ---- Y 轴 ---- */
                    int raw = (int)forward_cmd;
                    int speed;

                    if (raw >  VISION_MAX_SPEED) raw =  VISION_MAX_SPEED;
                    if (raw < -VISION_MAX_SPEED) raw = -VISION_MAX_SPEED;

                    if (vision_speed_y != 0)
                    {
                        if ((raw >= 0) != (vision_speed_y > 0))
                        {
                            /* 方向反转: 立即停车进 SETTLING, 不做反向斜坡 */
                            VisionAlignment_SetTarget(0, 0);
                            vision_speed_x = 0;
                            vision_speed_y = 0;
                            vision_y_state = Y_STATE_SETTLING;
                            vision_y_settle_until = now + VISION_Y_SETTLE_MS;
                            vision_y_stop_sequence = frame.sequence;
                            vision_diag_cmd_x = 0;
                            vision_diag_cmd_y = 0;
                            return;
                        }
                        speed = VisionSlewLimit(raw, vision_speed_y);
                        if ((speed > 0) && (speed < VISION_Y_HOLD_MIN_SPEED))
                            speed = VISION_Y_HOLD_MIN_SPEED;
                        if ((speed < 0) && (speed > -VISION_Y_HOLD_MIN_SPEED))
                            speed = -VISION_Y_HOLD_MIN_SPEED;
                    }
                    else
                    {
                        /* 静止启动: 直接给 START_SPEED, 不做低速爬升 */
                        speed = (raw >= 0) ? VISION_Y_START_SPEED
                                           : -VISION_Y_START_SPEED;
                    }

                    vision_speed_y = speed;
                    vision_speed_x = 0;
                    vision_y_state = Y_STATE_MOVING;
                    VisionAlignment_SetTarget(speed, 0);
                    vision_diag_cmd_x = 0;
                    vision_diag_cmd_y = (int16_t)speed;
                }
                else
                {
                    /* ---- X 轴 ---- */
                    int raw = (int)left_cmd;
                    int speed;

                    if (raw >  VISION_MAX_SPEED) raw =  VISION_MAX_SPEED;
                    if (raw < -VISION_MAX_SPEED) raw = -VISION_MAX_SPEED;

                    if (vision_speed_x != 0)
                    {
                        if ((raw >= 0) != (vision_speed_x > 0))
                        {
                            /* 方向反转: 同样停车进 SETTLING */
                            VisionAlignment_SetTarget(0, 0);
                            vision_speed_x = 0;
                            vision_speed_y = 0;
                            vision_y_state = Y_STATE_SETTLING;
                            vision_y_settle_until = now + VISION_Y_SETTLE_MS;
                            vision_y_stop_sequence = frame.sequence;
                            vision_diag_cmd_x = 0;
                            vision_diag_cmd_y = 0;
                            return;
                        }
                        speed = VisionSlewLimit(raw, vision_speed_x);
                        if ((speed > 0) && (speed < VISION_X_HOLD_MIN_SPEED))
                            speed = VISION_X_HOLD_MIN_SPEED;
                        if ((speed < 0) && (speed > -VISION_X_HOLD_MIN_SPEED))
                            speed = -VISION_X_HOLD_MIN_SPEED;
                    }
                    else
                    {
                        speed = (raw >= 0) ? VISION_X_START_SPEED
                                           : -VISION_X_START_SPEED;
                    }

                    vision_speed_x = speed;
                    vision_speed_y = 0;
                    vision_y_state = Y_STATE_MOVING;
                    VisionAlignment_SetTarget(0, speed);
                    vision_diag_cmd_x = (int16_t)speed;
                    vision_diag_cmd_y = 0;
                }
            }
        }
        else
        {
            /* ============================================================ *
             *  Task2: 原逻辑不变 (滞环+编码器粗调+脉冲精调)
             * ============================================================ */

            /* ---- 对准判定: 进入允许范围 ---- */
            if ((abs_x <= (int)cfg->enter_band) &&
                (abs_y <= (int)cfg->enter_band))
            {
                if (cfg->use_hysteresis)
                    align_in_band = 1U;
                align_hyst_until_ms = 0U;
                VisionAlignment_SetTarget(0, 0);
                vision_diag_code = VISION_DIAG_CENTERED;
                align_pulse_until_ms = 0U;
                align_settle_until_ms = 0U;
                if (++vision_center_count >= TROPHY_CENTER_FRAMES)
                {
                    vision_center_count = 0U;
                    align_finish_at_ms = now + cfg->finish_hold_ms;
                }
                return;
            }

            vision_center_count = 0U;

            /* ---- 滞环 (Task2 only) ----
             * 滞环内停车等待, 但 500ms 未回到完成带则退出滞环重新微调,
             * 防止误差停在 18~28px 区永久停车直到超时放行(落点偏后). */
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
                    if (align_hyst_until_ms == 0U)
                        align_hyst_until_ms = now + TROPHY_HYST_TIMEOUT_MS;
                    if ((int32_t)(now - align_hyst_until_ms) >= 0)
                    {
                        align_hyst_until_ms = 0U;
                        align_in_band = 0U;
                    }
                    return;
                }
                if ((abs_x > (int)cfg->hysteresis) ||
                    (abs_y > (int)cfg->hysteresis))
                {
                    align_in_band = 0U;
                    align_hyst_until_ms = 0U;
                }
            }

            vision_diag_code = VISION_DIAG_MOVING;

            /* 每帧只调整误差更大的一轴, 避免 Y 轴始终抢占控制权. */
            if ((abs_y > (int)cfg->enter_band) && (abs_y >= abs_x))
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
                    /* ---- Task2: 固定精调速度, 脉冲限制 ---- */
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
                    /* ---- Task2: 固定精调速度 ---- */
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
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    align_status = VISION_ALIGN_IDLE;
}

void VisionAlignment_Finish(void)
{
    VisionAlignment_SetTarget(0, 0);
    vision_error_x_last = 0.0f;
    vision_error_y_last = 0.0f;
    vision_speed_x = 0;
    vision_speed_y = 0;
    vision_y_state = Y_STATE_IDLE;
    vision_y_settle_until = 0U;
    vision_y_stop_sequence = 0U;
    vision_center_count = 0U;
    vision_center_candidate = 0U;
    vision_active_axis = 2U;
    vision_last_new_frame_ms = 0U;
    vision_search_start_ms = 0U;
    align_hyst_until_ms = 0U;
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
