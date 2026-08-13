#include "ProjectService.h"
#include "LineFollow.h"
#include "HWT101.h"
#include "Sensor.h"
#include "ChassisMotor.h"
#include "CompetitionQr.h"
#include "Servo4.h"
#include "Task1Mechanism.h"
#include "TrophyMechanism.h"
#include "VisionAlignment.h"
#include "TurntableInventory.h"
#include "MaxiCam.h"
#include "Task1Color.h"
#include <stdio.h>

/* ---- 状态变量 ---- */
static volatile uint8_t vision_skip;
static volatile RouteState route_state = ROUTE_HOME_LAUNCH;
static volatile RouteState pause_next_mode;
static volatile RouteState vision_next_mode;
static volatile uint8_t vision_started;
static volatile int marker_suppress_ticks;
static volatile int home_launch_tick;
static volatile int mode_entry_tick;
static volatile uint16_t home_fixed_ticks;
static uint8_t trophy_phase;
static uint16_t trophy_ticks;
static uint8_t trophy_cam_mode_on;
static uint8_t trophy_det_confirm;
static uint32_t trophy_det_last_seq;
static uint8_t trophy_det_count;
static uint32_t trophy_det_gap_until_ms;
static VisionJob vision_job;
static uint8_t place_step;
static uint8_t task1_back_phase;
static int32_t task1_back_total;
static uint8_t final_left_stage;
static uint8_t arm_to_task0_done;
static uint8_t  final_left_heading_stable_ticks;
static uint32_t final_align_last_frame;
static uint16_t final_align_tick;

/* 放置流程状态机 */
typedef enum {
    PLACE_MOVE_LEFT            = 0,
    PLACE_PREPARE_TARGET_POSE  = 27,
    PLACE_TARGET_POSE_SETTLE   = 28,
    PLACE_ALIGN                = 1,
    PLACE_COARSE_MOVE          = 10,
    PLACE_COARSE_SETTLE        = 11,
    PLACE_FORWARD_INSERT       = 30,
    PLACE_INSERT_SETTLE        = 31,
    PLACE_ROTATE_SLOT          = 2,
    PLACE_ROTATE_SETTLE        = 3,
    PLACE_LOWER_TO_RELEASE     = 12,
    PLACE_RELEASE_SETTLE       = 13,
    PLACE_HALF_OPEN            = 14,
    PLACE_FULL_OPEN            = 15,
    PLACE_RELEASE_DETACH       = 21,
    PLACE_DROP_SETTLE          = 16,
    PLACE_LIFT_CLEAR           = 17,
    PLACE_LIFT_CLEAR_SETTLE    = 29,
    PLACE_BACK_CLEAR           = 18,
    PLACE_BACK_SETTLE          = 32,
    PLACE_ROTATE_SAFE          = 23,
    PLACE_ROTATE_SAFE_SETTLE   = 24,
    PLACE_CLOSE_GRIPPER        = 25,
    PLACE_TRANSPORT_POSE       = 26,
    PLACE_FINISH               = 20
} TrophyPlaceStep;

static uint8_t  place_slot_idx;
static uint16_t place_pwm_us;
static uint8_t  place_seq_index;
static uint8_t  task2_capture_index;
static uint8_t  task2_place_index;
static uint8_t  task2_prep_step;
static volatile uint8_t task2_transition_step;
static volatile uint16_t task2_transition_qr_ticks;
static uint8_t  task2_collect_complete;
static uint8_t  task2_collect_done;
static float home_forward_heading;
static uint8_t home_forward_heading_valid;
static uint16_t home_return_align_ticks;
static uint8_t home_return_stable_frames;
static uint32_t home_return_last_frame;
static TrophyPlaceStep trophy_place_step;
static uint8_t  trophy_place_settle_ticks;
static uint8_t  trophy_place_back_active;
static uint8_t  trophy_detect_armed;
static uint8_t  trophy_clear_frames;
static uint8_t  trophy_last_rank;
static uint8_t  trophy_last_slot;
static float left_heading_absolute;
static uint8_t left_heading_valid;
static int8_t yaw_ccw_sign = 1;
static float home_turn_start_absolute;
static int last_steer;
static int cross_marker_ticks;
static uint8_t lcd_div;
static int debug_line_state;
static int debug_turn_target;

typedef struct
{
    uint16_t align_big;
    uint16_t align_small;
    uint16_t release_big;
    uint16_t release_small;
    uint16_t clear_big;
    uint16_t clear_small;
    int32_t  insert_distance;
    int32_t  back_distance;
} TrophyPlaceProfile;

static const TrophyPlaceProfile *TrophyPlace_GetProfile(uint8_t place_index)
{
    static const TrophyPlaceProfile profiles[3] =
    {
        /* 亚军 PODIUM */
        { 1100U, 1400U, 1100U, 1200U, 1100U, 1350U,
          (int32_t)TROPHY_INSERT_RUNNER_COUNTS,
          (int32_t)TROPHY_BACK_RUNNER_COUNTS },
        /* 冠军 CHAMPION */
        { 1000U, 1300U, 1000U, 1100U, 1000U, 1250U,
          (int32_t)TROPHY_INSERT_CHAMPION_COUNTS,
          (int32_t)TROPHY_BACK_CHAMPION_COUNTS },
        /* 季军 */
        { 1250U, 1500U, 1250U, 1500U, 1300U, 1600U,
          (int32_t)TROPHY_INSERT_THIRD_COUNTS,
          (int32_t)TROPHY_BACK_THIRD_COUNTS },
    };
    if (place_index >= 3U) place_index = 0U;
    return &profiles[place_index];
}

static uint8_t TrophyPlace_Process(uint8_t place_index);

/* ---- 对外状态查询 ---- */
int ProjectService_GetRouteState(void)
{
    return (int)route_state;
}

void ProjectService_SetRouteState(RouteState state)
{
    __disable_irq();
    route_state = state;
    __enable_irq();
}

void ProjectService_SetVisionSkip(uint8_t on_off)
{
    vision_skip = on_off;
}

void ProjectService_SetInitialHeading(void)
{
    home_forward_heading = HWT101_GetAbsoluteYaw();
    home_forward_heading_valid = HWT101_IsReady();
    left_heading_absolute = home_forward_heading;
    left_heading_valid = 1;
    yaw_ccw_sign = -1;
}

static uint8_t TrophyDetect_Seen(void)
{
    MaxiCam_Frame frame;

    if (ChassisPos_GetLeftPosition() < (int)TROPHY_DETECT_ARM_COUNTS)
        return 0U;

    if (!MaxiCam_GetLatest(&frame, 250U))
        return 0U;
    if (frame.sequence == trophy_det_last_seq)
        return 0U;
    trophy_det_last_seq = frame.sequence;

    if ((int32_t)(HAL_GetTick() - trophy_det_gap_until_ms) < 0)
        return 0U;

    if (frame.data[0] != TROPHY_DETECT_CODE)
    {
        trophy_det_confirm = 0U;
        return 0U;
    }
    if (++trophy_det_confirm >= TROPHY_DETECT_CONFIRM)
    {
        trophy_det_confirm = 0U;
        trophy_det_gap_until_ms = HAL_GetTick() + TROPHY_DETECT_CLEAR_MS;
        return 1U;
    }
    return 0U;
}

static uint8_t TrophyDetect_CollectSeen(void)
{
    MaxiCam_Frame frame;

    if (!MaxiCam_GetLatest(&frame, 250U))
        return 0U;
    if (frame.sequence == trophy_det_last_seq)
        return 0U;
    trophy_det_last_seq = frame.sequence;

    if ((int32_t)(HAL_GetTick() - trophy_det_gap_until_ms) < 0)
        return 0U;

    if (frame.data[0] != TROPHY_DETECT_CODE)
    {
        trophy_det_confirm = 0U;
        return 0U;
    }
    if (++trophy_det_confirm >= TROPHY_DETECT_CONFIRM)
    {
        trophy_det_confirm = 0U;
        trophy_det_gap_until_ms = HAL_GetTick() + TROPHY_DETECT_CLEAR_MS;
        return 1U;
    }
    return 0U;
}

static uint8_t TrophyDetect_CheckClear(void)
{
    MaxiCam_Frame frame;

    if (!MaxiCam_GetLatest(&frame, 250U))
        return 0U;
    if (frame.sequence == trophy_det_last_seq)
        return 0U;
    trophy_det_last_seq = frame.sequence;

    if (frame.data[0] != TROPHY_DETECT_CODE)
    {
        if (++trophy_clear_frames >= 2U)
        {
            trophy_detect_armed = 1U;
            trophy_clear_frames = 0U;
            return 1U;
        }
    }
    else
    {
        trophy_clear_frames = 0U;
    }
    return 0U;
}

/* ---- 初始化 ---- */
void ProjectService_Init(void)
{
    LineFollow_Init();
    HWT101_getAngle_Reset();
    LineFollow_ResetLinePID();
    last_steer = 0;
    place_seq_index = 0;
    task1_back_phase = 0U;
    task1_back_total = 0;
    task2_collect_complete = 0U;
    task2_collect_done = 0U;
    task2_transition_step = 0U;
    home_forward_heading = HWT101_GetAbsoluteYaw();
    home_forward_heading_valid = HWT101_IsReady();
    home_return_align_ticks = 0U;
    home_return_stable_frames = 0U;
    home_return_last_frame = 0U;
    TurntableInventory_Init(840U, 400U);
    /* Keep CH3..CH7 open from power-up through Task1.  The Task2 transition
     * switches them to the trophy-specific open pulse; they close only after
     * all three trophies have actually been collected. */
    Servo4_LittleAllOpen();
    {
        const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
        Servo4_SetPulse(0U, t0->big_arm_us);
        Servo4_SetPulse(1U, t0->small_arm_us);
    }
    {
        const Task1ArmPose *tgt = Task1Mechanism_GetWorkingPose();
        Servo4_ArmRampTo(tgt->big_arm_us, tgt->small_arm_us);
    }
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    ChassisPos_DrawStatus(1, "Home Launch");
    HAL_TIM_Base_Start_IT(&htim7);
}

/* ---- 放置流程共享函数 ---- */
static uint8_t TrophyPlace_Process(uint8_t place_index)
{
    const TrophyPlaceProfile *profile = TrophyPlace_GetProfile(place_index);
    int hc;

    switch (trophy_place_step)
    {
    case PLACE_MOVE_LEFT: {
        int dist = (place_index == 0U) ? (int)TROPHY_LEFT1_DISTANCE
                 : (place_index == 1U) ? (int)TROPHY_LEFT2_DISTANCE
                 :                       (int)TROPHY_LEFT3_DISTANCE;

        if (!pos_control_active)
            ChassisPos_Start(-dist, dist, -dist, dist);

        hc = HeadingHold_GetFreshCorrection();
        if (ChassisPos_Update(0, TROPHY_LEFT_SPEED, hc))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_PREPARE_TARGET_POSE;
        }
        break;
    }

    case PLACE_PREPARE_TARGET_POSE:
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(profile->align_big, profile->align_small))
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_TARGET_POSE_SETTLE;
        }
        break;

    case PLACE_TARGET_POSE_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_TARGET_POSE_SETTLE_TICKS)
        {
            if (place_index == 0U)
                VisionAlignment_Start(VISION_PROFILE_TROPHY);
            else
                VisionAlignment_Restart();

            trophy_place_step = PLACE_ALIGN;
        }
        break;

    case PLACE_ALIGN: {
        VisionAlignment_Status vs;

        if (VisionAlignment_NeedCoarseMove())
        {
            int16_t dist  = VisionAlignment_GetCoarseDistance();
            uint8_t axis  = VisionAlignment_GetCoarseAxis();
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            if (axis == 0U)
                ChassisPos_Start(dist, dist, dist, dist);
            else
                ChassisPos_Start(-dist, dist, -dist, dist);
            VisionAlignment_ClearCoarseMove();
            trophy_place_step = PLACE_COARSE_MOVE;
        }
        else
        {
            vs = VisionAlignment_GetStatus();
            if (vs == VISION_ALIGN_RUNNING)
            {
                int vt[4];
                VisionAlignment_GetWheelTargets(vt);
                wheel_target[1] = vt[0];
                wheel_target[2] = vt[1];
                wheel_target[3] = vt[2];
                wheel_target[4] = vt[3];
            }
            else
            {
                VisionAlignment_Finish();
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                Move_Control(0, 0, 0, 0);
                trophy_place_step = PLACE_FORWARD_INSERT;
            }
        }
        break;
    }

    case PLACE_COARSE_MOVE: {
        uint8_t coarse_axis = VisionAlignment_GetCoarseAxis();
        hc = HeadingHold_GetFreshCorrection();
        if (pos_fault)
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            VisionAlignment_Finish();
            trophy_place_step = PLACE_ROTATE_SLOT;
        }
        else if (ChassisPos_Update(
                     (coarse_axis == 0U) ? ALIGN_COARSE_SPEED : 0,
                     (coarse_axis == 1U) ? ALIGN_COARSE_SPEED : 0,
                     hc))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_COARSE_SETTLE;
        }
        break;
    }

    case PLACE_COARSE_SETTLE:
        LineFollow_Stop();
        ChassisPos_ResetVelocityPid();
        Move_Control(0, 0, 0, 0);
        if (++trophy_place_settle_ticks >= 10U)
        {
            VisionAlignment_ResumeAfterCoarse();
            trophy_place_step = PLACE_ALIGN;
        }
        break;

    case PLACE_FORWARD_INSERT:
        if (!pos_control_active)
        {
            int32_t fwd = profile->insert_distance;
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            ChassisPos_Start(fwd, fwd, fwd, fwd);
        }
        hc = HeadingHold_GetFreshCorrection();
        if (ChassisPos_Update(TROPHY_FORWARD_INSERT_SPEED, 0, hc))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_INSERT_SETTLE;
        }
        break;

    case PLACE_INSERT_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_INSERT_SETTLE_TICKS)
            trophy_place_step = PLACE_ROTATE_SLOT;
        break;

    case PLACE_ROTATE_SLOT: {
        QrTask2Trophy wanted;
        uint8_t slot_idx;
        uint16_t pwm;

        LineFollow_Stop();
        ChassisPos_ResetVelocityPid();
        Move_Control(0, 0, 0, 0);

        wanted = CompetitionQr_GetTask2PlaceTrophy(place_index);
        if (TurntableInventory_FindTrophyRank((uint8_t)wanted, &slot_idx, &pwm))
        {
            place_slot_idx = slot_idx;
            place_pwm_us   = pwm;
            Servo4_RotationUnlockTo(pwm);
        }
        else
        {
            place_slot_idx = place_index;
            place_pwm_us   = TurntableInventory_GetPwmForSlot(place_index);
            Servo4_RotationUnlockTo(place_pwm_us);
        }
        trophy_place_settle_ticks = 0U;
        trophy_place_step = PLACE_ROTATE_SETTLE;
        break;
    }

    case PLACE_ROTATE_SETTLE:
        LineFollow_Stop();
        ChassisPos_ResetVelocityPid();
        Move_Control(0, 0, 0, 0);
        if (++trophy_place_settle_ticks >= TROPHY_PLACE_SETTLE_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_LOWER_TO_RELEASE;
        }
        break;

    case PLACE_LOWER_TO_RELEASE:
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(profile->release_big, profile->release_small))
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_RELEASE_SETTLE;
        }
        break;

    case PLACE_RELEASE_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_RELEASE_SETTLE_TICKS)
        {
            Servo4_TrophyLittleHalfOpenSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_HALF_OPEN;
        }
        break;

    case PLACE_HALF_OPEN:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_HALF_OPEN_WAIT_TICKS)
        {
            Servo4_TrophyLittleOpenSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_FULL_OPEN;
        }
        break;

    case PLACE_FULL_OPEN:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= GRIPPER_OPEN_WAIT_TICKS)
        {
            Servo4_TrophyLittleReleaseSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_RELEASE_DETACH;
        }
        break;

    case PLACE_RELEASE_DETACH:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_DETACH_WAIT_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_DROP_SETTLE;
        }
        break;

    case PLACE_DROP_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_DROP_SETTLE_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_LIFT_CLEAR;
        }
        break;

    case PLACE_LIFT_CLEAR:
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(profile->clear_big, profile->clear_small))
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_LIFT_CLEAR_SETTLE;
        }
        break;

    case PLACE_LIFT_CLEAR_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_LIFT_CLEAR_SETTLE_TICKS)
        {
            trophy_place_back_active = 0U;
            trophy_place_step = PLACE_BACK_CLEAR;
        }
        break;

    case PLACE_BACK_CLEAR:
        if (!trophy_place_back_active)
        {
            int32_t back = -profile->back_distance;
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            ChassisPos_Start(back, back, back, back);
            trophy_place_back_active = 1U;
        }
        hc = HeadingHold_GetFreshCorrection();
        if (ChassisPos_Update(TROPHY_BACK_CLEAR_SPEED, 0, hc))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            trophy_place_back_active = 0U;
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_BACK_SETTLE;
        }
        break;

    case PLACE_BACK_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_BACK_SETTLE_TICKS)
            trophy_place_step = PLACE_ROTATE_SAFE;
        break;

    case PLACE_ROTATE_SAFE:
        LineFollow_Stop();
        Servo4_TrophyRotationSafe();
        trophy_place_settle_ticks = 0U;
        trophy_place_step = PLACE_ROTATE_SAFE_SETTLE;
        break;

    case PLACE_ROTATE_SAFE_SETTLE:
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_ROTATE_SAFE_SETTLE_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_CLOSE_GRIPPER;
        }
        break;

    case PLACE_CLOSE_GRIPPER:
        LineFollow_Stop();
        Servo4_TrophyLittleCloseSlot(place_slot_idx);
        trophy_place_settle_ticks = 0U;
        trophy_place_step = PLACE_TRANSPORT_POSE;
        break;

    case PLACE_TRANSPORT_POSE:
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(TROPHY_TRANSPORT_BIG_US,
                                   TROPHY_TRANSPORT_SMALL_US))
        {
            trophy_place_step = PLACE_FINISH;
        }
        break;

    case PLACE_FINISH:
        LineFollow_Stop();
        TurntableInventory_MarkEmpty(place_slot_idx);
        task2_place_index++;
        return 1U;

    default:
        trophy_place_step = PLACE_MOVE_LEFT;
        break;
    }

    return 0U;
}

/* ---- 主循环 ---- */
void ProjectService_Run(void)
{
    char buf[24];

    ChassisPos_FlushDisplay();
    Task1Color_Process();

    if (route_state == ROUTE_MISSION_DONE)
    {
        LineFollow_Stop();
        if (++lcd_div >= 10)
        {
            ChassisPos_DrawStatus(1, "MISSION DONE");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PLACE_PREPARE)
    {
        uint8_t ok = 0U;

        LineFollow_Stop();

        if (vision_job == VISION_JOB_TROPHY)
        {
            QrTask2Trophy wanted;
            uint8_t slot_idx;
            uint16_t pwm;

            if (task2_place_index >= QR_TASK2_PLACE_COUNT)
                goto place_prep_done;

            wanted = CompetitionQr_GetTask2PlaceTrophy(task2_place_index);
            if (wanted == QR_TASK2_TROPHY_INVALID)
                goto place_prep_error;

            if (TurntableInventory_FindTrophyRank((uint8_t)wanted,
                                                   &slot_idx, &pwm))
            {
                place_slot_idx = slot_idx;
                place_pwm_us = pwm;
                ok = 1U;
            }
        }
        else
        {
            uint8_t target_color;

            if (place_seq_index >= TURNTABLE_SLOT_COUNT)
                goto place_prep_done;

            target_color = CompetitionQr_GetTask1PlaceColor(
                CompetitionQr_GetTaskCode(COMP_QR_TASK1), place_seq_index);

            if ((target_color >= TURNTABLE_COLOR_CODE_MIN) &&
                (target_color <= TURNTABLE_COLOR_CODE_MAX) &&
                TurntableInventory_FindColor(target_color,
                                             &place_slot_idx,
                                             &place_pwm_us))
            {
                ok = 1U;
            }
        }

        if (ok)
        {
            vision_job = (vision_job == VISION_JOB_TROPHY)
                         ? VISION_JOB_TROPHY : VISION_JOB_COLOR_PLACE;
            vision_started = 0U;
            __disable_irq();
            route_state = ROUTE_VISION_ALIGN;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Vision Align");
            return;
        }

place_prep_error:
        __disable_irq();
        route_state = ROUTE_PLACE_ERROR;
        __enable_irq();
        mode_entry_tick = 0;
        lcd_div = 0;
        ChassisPos_DrawStatus(1, "Place ERR");
        return;

place_prep_done:
        __disable_irq();
        route_state = vision_next_mode;
        __enable_irq();
        mode_entry_tick = 0;
        lcd_div = 0;
        ChassisPos_DrawStatus(1, "Route Resume");
        return;
    }

    if (route_state == ROUTE_VISION_ALIGN)
    {
        VisionAlignment_Status vision_status;

        if (vision_skip)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            LineFollow_Stop();
            if (vision_next_mode == ROUTE_REAR_ALIGN)
            {
                ChassisPos_Start(-46250, 46250, -46250, 46250);
                ChassisPos_DrawStatus(1, "Left 2.5cm");
            }
            else if (vision_next_mode == ROUTE_REAR_ALIGN_2)
            {
                ChassisPos_Start(-74000, -74000, -74000, -74000);
                ChassisPos_DrawStatus(1, "Back 40cm");
            }
            else if (vision_next_mode == ROUTE_FINAL_LEFT_FORWARD)
            {
                ChassisPos_Start(-37000, -37000, -37000, -37000);
                ChassisPos_DrawStatus(1, "Back 20cm");
            }
            else if (vision_next_mode == ROUTE_FINAL_ROTATE_30)
            {
                ChassisPos_DrawStatus(1, "Rotate 30 CW");
            }
            else if (vision_next_mode == ROUTE_FINAL_ROTATE_ALIGN)
            {
                ChassisPos_DrawStatus(1, "Rotate 48 CW");
            }
            __disable_irq();
            route_state = vision_next_mode;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            return;
        }

        if (!vision_started)
        {
            VisionAlignment_Start(VISION_PROFILE_TASK1);
            vision_started = 1U;
            ChassisPos_DrawStatus(1, "Vision Align");
        }

        vision_status = VisionAlignment_GetStatus();

        if (++lcd_div >= 25)
        {
            lcd_div = 0;
            if (vision_status == VISION_ALIGN_RUNNING)
            {
                int wt[4];
                const char *diag_msg;
                VisionAlignment_GetWheelTargets(wt);
                switch (vision_diag_code)
                {
                    case VISION_DIAG_NO_FRAME: diag_msg = "NO FRAME";   break;
                    case VISION_DIAG_BAD_MODE: diag_msg = "BAD MODE";   break;
                    case VISION_DIAG_BAD_XY:   diag_msg = "BAD XY";     break;
                    case VISION_DIAG_CENTERED: diag_msg = "CENTERED";   break;
                    case VISION_DIAG_TIMEOUT:  diag_msg = "TIMEOUT";    break;
                    case VISION_DIAG_MOVING:   diag_msg = "MOVING";     break;
                    default:                   diag_msg = "IDLE";       break;
                }
                sprintf(buf, "M:%u X:%u Y:%u",
                        (unsigned)vision_diag_mode,
                        (unsigned)vision_diag_x,
                        (unsigned)vision_diag_y);
                LCD_DrawString_NoDMA(1, 60, buf, LCD_WHITE, LCD_BLACK);
                sprintf(buf, "SEQ:%lu AGE:%lu",
                        (unsigned long)vision_diag_seq,
                        (unsigned long)vision_diag_age_ms);
                LCD_DrawString_NoDMA(1, 80, buf, LCD_WHITE, LCD_BLACK);
                LCD_DrawString_NoDMA(1, 100, (char *)diag_msg,
                                     LCD_GREEN, LCD_BLACK);
                sprintf(buf, "W:%d %d %d %d",
                        wt[0], wt[1], wt[2], wt[3]);
                LCD_DrawString_NoDMA(1, 120, buf, LCD_WHITE, LCD_BLACK);
                sprintf(buf, "RX:%lu H:%lu P:%lu",
                        (unsigned long)maxicam_diag_rx_bytes,
                        (unsigned long)maxicam_diag_headers,
                        (unsigned long)maxicam_diag_packets);
                LCD_DrawString_NoDMA(1, 140, buf, LCD_GREEN, LCD_BLACK);
            }
        }

        if (vision_status == VISION_ALIGN_TIMEOUT)
        {
            VisionAlignment_Cancel();
            if (vision_job == VISION_JOB_COLOR_PLACE)
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                vision_started = 0U;
                __disable_irq();
                route_state = ROUTE_PAUSE_GENERIC;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Vision TO Skip");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
        else if (vision_status == VISION_ALIGN_DONE)
        {
        }
        else
        {
            return;
        }

        VisionAlignment_Cancel();
        ChassisPos_ResetEncoder();
        ChassisPos_ResetVelocityPid();
        HWT101_getAngle_Reset();
        HeadingHold_Reset();
        LineFollow_Stop();
        ChassisPos_Start(VISION_FORWARD_DISTANCE, VISION_FORWARD_DISTANCE,
                         VISION_FORWARD_DISTANCE, VISION_FORWARD_DISTANCE);
        __disable_irq();
        route_state = ROUTE_VISION_FORWARD;
        __enable_irq();
        mode_entry_tick = 0;
        lcd_div = 0;
        vision_started = 0U;
        ChassisPos_DrawStatus(1, "Fwd 2.2cm");
        return;
    }

    if (route_state == ROUTE_VISION_FORWARD)
    {
        if (ChassisPos_Update(VISION_FORWARD_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            LineFollow_Stop();

            {
                place_step = 0U;
                __disable_irq();
                route_state = ROUTE_PLACE;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Place Release");
            }
        }
        return;
    }

    if (route_state == ROUTE_PLACE)
    {
        if (place_step == 0U)
        {
            uint16_t pre_pwm;
            uint16_t delta;
            char buf[24];

            LineFollow_Stop();
            pre_pwm = Servo4_GetPulse(2U);
            Servo4_RotationUnlockTo(place_pwm_us);

            delta = (pre_pwm > place_pwm_us)
                    ? (pre_pwm - place_pwm_us)
                    : (place_pwm_us - pre_pwm);
            mode_entry_tick = (int)(40 + (int)delta / 20);

            sprintf(buf, "PL:%d S:%d P:%u",
                    place_seq_index + 1, place_slot_idx,
                    (unsigned)place_pwm_us);
            ChassisPos_DrawStatus(1, buf);

            place_step = 1U;
            lcd_div = 0;
        }
        else if (place_step == 1U)
        {
            lcd_div++;
            if (lcd_div >= (unsigned)mode_entry_tick)
            {
                if ((vision_job == VISION_JOB_COLOR_PLACE) &&
                    (place_seq_index == 4U))
                {
                    arm_to_task0_done = 0U;
                    place_step = 2U;
                }
                else if (vision_job == VISION_JOB_COLOR_PLACE)
                {
                    /* CH2 has released the block.  Freeze it at this slot and
                     * give the block time to land before the chassis moves. */
                    lcd_div = 0U;
                    place_step = 4U;
                    ChassisPos_DrawStatus(1, "Block Settle");
                }
                else
                {
                    int32_t back_dist = -VISION_BACK_DISTANCE;
                    if (vision_next_mode == ROUTE_FINAL_ROTATE_30)
                        back_dist -= 5550;
                    ChassisPos_ResetEncoder();
                    ChassisPos_ResetVelocityPid();
                    ChassisPos_Start(back_dist, back_dist, back_dist, back_dist);
                    __disable_irq();
                    route_state = ROUTE_VISION_BACK;
                    __enable_irq();
                    mode_entry_tick = 0;
                    lcd_div = 0;
                    ChassisPos_DrawStatus(1,
                        (vision_next_mode == ROUTE_FINAL_ROTATE_30)
                            ? "Back 5.7cm" : "Back 2.7cm");
                }
            }
        }
        else if (place_step == 2U)
        {
            const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
            if (Servo4_ArmRampTick(t0->big_arm_us, t0->small_arm_us))
            {
                arm_to_task0_done = 1U;
                lcd_div = 0U;
                place_step = 4U;
            }
        }
        else if (place_step == 4U)
        {
            /* Do not rotate or lock CH2 while the block is settling. */
            LineFollow_Stop();
            if (++lcd_div >= TASK1_BLOCK_DROP_SETTLE_TICKS)
            {
                task1_back_total = (int32_t)TASK1_PLACE_BACK_DISTANCE;
                if (vision_next_mode == ROUTE_FINAL_ROTATE_30)
                    task1_back_total += 5550;

                ChassisPos_ResetEncoder();
                ChassisPos_ResetVelocityPid();
                ChassisPos_Start(-(int32_t)TASK1_BACK_CLEAR_DISTANCE,
                                 -(int32_t)TASK1_BACK_CLEAR_DISTANCE,
                                 -(int32_t)TASK1_BACK_CLEAR_DISTANCE,
                                 -(int32_t)TASK1_BACK_CLEAR_DISTANCE);
                task1_back_phase = 1U;
                __disable_irq();
                route_state = ROUTE_VISION_BACK;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Back Clear 1cm");
            }
        }
        return;
    }

    if (route_state == ROUTE_PLACE_ERROR)
    {
        LineFollow_Stop();
        if (++lcd_div >= 50)
        {
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Place ERR");
        }
        return;
    }

    if (route_state == ROUTE_VISION_BACK)
    {
        int back_speed = ((vision_job == VISION_JOB_COLOR_PLACE) &&
                          (task1_back_phase == 1U))
                       ? TASK1_BACK_CLEAR_SPEED : VISION_FORWARD_SPEED;

        if (ChassisPos_Update(back_speed, 0,
              HeadingHold_GetFreshCorrection()))
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            LineFollow_Stop();

            /* Task1 withdrawal is deliberately two-stage: creep through the
             * first centimetre, then finish the remaining clearance normally.
             * CH2 remains untouched throughout both stages. */
            if ((vision_job == VISION_JOB_COLOR_PLACE) &&
                (task1_back_phase == 1U) &&
                (task1_back_total > TASK1_BACK_CLEAR_DISTANCE))
            {
                int32_t remaining = task1_back_total -
                                    (int32_t)TASK1_BACK_CLEAR_DISTANCE;
                ChassisPos_Start(-remaining, -remaining,
                                 -remaining, -remaining);
                task1_back_phase = 2U;
                ChassisPos_DrawStatus(1, "Back Fast");
                return;
            }

            if (vision_job == VISION_JOB_COLOR_PLACE)
            {
                task1_back_phase = 0U;
                task1_back_total = 0;
                TurntableInventory_MarkEmpty(place_slot_idx);
                place_seq_index++;
                Servo4_Task1RotationLockNow();
            }
            else if (vision_job == VISION_JOB_TROPHY)
            {
                TurntableInventory_MarkEmpty(place_slot_idx);
                task2_place_index++;
                Servo4_TrophyRotationLock();

                if (task2_place_index >= QR_TASK2_PLACE_COUNT)
                {
                    ChassisPos_ResetEncoder();
                    HeadingHold_Reset();
                    home_return_align_ticks = 0U;
                    home_return_stable_frames = 0U;
                    home_return_last_frame = HWT101_GetUpdateCount();
                    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                    __disable_irq();
                    route_state = ROUTE_HOME_RETURN_ALIGN;
                    __enable_irq();
                    mode_entry_tick = 0;
                    lcd_div = 0;
                    ChassisPos_DrawStatus(1, "Home Align");
                    return;
                }
            }

            if (vision_job == VISION_JOB_TROPHY)
            {
                __disable_irq();
                route_state = ROUTE_PLACE_PREPARE;
                __enable_irq();
            }
            else if (vision_next_mode == ROUTE_REAR_ALIGN)
            {
                ChassisPos_Start(-46250, 46250, -46250, 46250);
                ChassisPos_DrawStatus(1, "Left 2.5cm");
                __disable_irq();
                route_state = vision_next_mode;
                __enable_irq();
            }
            else if (vision_next_mode == ROUTE_REAR_ALIGN_2)
            {
                ChassisPos_Start(-74000, -74000, -74000, -74000);
                ChassisPos_DrawStatus(1, "Back 40cm");
                __disable_irq();
                route_state = vision_next_mode;
                __enable_irq();
            }
            else if (vision_next_mode == ROUTE_FINAL_LEFT_FORWARD)
            {
                ChassisPos_Start(-37000, -37000, -37000, -37000);
                ChassisPos_DrawStatus(1, "Back 20cm");
                __disable_irq();
                route_state = vision_next_mode;
                __enable_irq();
            }
            else
            {
                __disable_irq();
                route_state = vision_next_mode;
                __enable_irq();
            }
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Route Resume");
        }
        return;
    }

    if (route_state == ROUTE_TASK1_TO_TASK2)
    {
        if (++lcd_div >= 5)
        {
            switch (task2_transition_step)
            {
            case 1U:
                snprintf(buf, sizeof(buf), "QR S:%u A:%u T:%u",
                         (unsigned)CompetitionQr_GetStatus(),
                         (unsigned)CompetitionQr_GetAttemptCount(),
                         (unsigned)(task2_transition_qr_ticks / 200U));
                break;
            case 4U: snprintf(buf, sizeof(buf), "T1->T2 S:4 LEFT"); break;
            case 5U: snprintf(buf, sizeof(buf), "T1->T2 S:5 LEFT"); break;
            case 6U: snprintf(buf, sizeof(buf), "T1->T2 S:6 ROT");  break;
            case 7U: snprintf(buf, sizeof(buf), "T1->T2 S:7 SETTLE"); break;
            default:
                snprintf(buf, sizeof(buf), "T1->T2 S:%u",
                         task2_transition_step);
                break;
            }
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_TASK2_PREPARE)
    {
        const Task1ArmPose *tgt;

        if (task2_prep_step == 0U)
        {
            LineFollow_Stop();
            TurntableInventory_Init(840U, 400U);
            Servo4_RotationHome();
            Servo4_TrophyLittleAllOpen();
            task2_capture_index = 0U;
            task2_place_index = 0U;
            task2_collect_complete = 0U;
            task2_collect_done = 0U;
            trophy_det_count = 0U;
            trophy_det_confirm = 0U;
            trophy_det_last_seq = 0U;
            trophy_det_gap_until_ms = 0U;
            trophy_detect_armed = 1U;
            trophy_clear_frames = 0U;
            trophy_last_rank = 0U;
            trophy_last_slot = 0U;
            task2_prep_step = 2U;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "T2 Prepare");
        }
        else
        {
            tgt = Task1Mechanism_GetWorkingPose();
            if (Servo4_ArmRampTickSlow(tgt->big_arm_us, tgt->small_arm_us))
            {
                ChassisPos_ResetEncoder();
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                marker_suppress_ticks = 0;
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                (void)MaxiCam_SetMode(MAXICAM_MODE_TROPHY_WHITE);
                __disable_irq();
                route_state = ROUTE_TRACK_RETURN;
                __enable_irq();
                task2_prep_step = 0U;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "T2 Collect");
            }
        }
        return;
    }

    if (route_state == ROUTE_HOME_LAUNCH)
    {
        if (++lcd_div >= 5)
        {
            if (home_launch_tick < HOME_LAUNCH_FWD_TICKS)
                sprintf(buf, "Launch T:%3d", home_launch_tick);
            else
                sprintf(buf, "Turn A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_FORWARD)
    {
        if (++lcd_div >= 5)
        {
            char buf[24];
            sprintf(buf, "T:%3d S:%3d P:%5d",
                    wheel_target[1], debug_trap_speed, debug_trap_position);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "E:%5d F:%1u A:%1u",
                    debug_trap_error, pos_fault, pos_control_active);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_PAUSE)
    {
        if (++lcd_div >= 5)
        {
            CompetitionQrStatus qr = CompetitionQr_GetStatus();
            if (qr == COMP_QR_READY)
                sprintf(buf, "T1 QR:%u", CompetitionQr_GetTaskCode(COMP_QR_TASK1));
            else if (qr == COMP_QR_DEFAULTED)
                sprintf(buf, "T1 DEFAULT:1");
            else
                sprintf(buf, "T1 SCAN:%u", CompetitionQr_GetAttemptCount());
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_LEFT)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "Home Left");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_FIXED_SETTLE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Settle:%2u", home_fixed_ticks);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_ROTATE_90)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot90 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_CURVE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Ph:%1d D:%5ld",
                    mode_entry_tick,
                    (long)((abs(encoder[1]) + abs(encoder[2])
                          + abs(encoder[3]) + abs(encoder[4])) / 4));
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_REAR_ALIGN)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Ph:%1d D:%5ld", mode_entry_tick,
                    (long)((abs(encoder[1]) + abs(encoder[2])
                          + abs(encoder[3]) + abs(encoder[4])) / 4));
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PAUSE)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Pause:%3d", mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_REAR_ALIGN_2)
    {
        if (++lcd_div >= 5)
        {
            int total_distance = (abs(encoder[1]) + abs(encoder[2])
                               + abs(encoder[3]) + abs(encoder[4])) / 4;
            sprintf(buf, "Rear2 D:%5d X:%1d", total_distance, mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PAUSE_2)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Pause2:%3d", mode_entry_tick);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_LEFT_FORWARD)
    {
        if (++lcd_div >= 5)
        {
            int dist = (abs(encoder[1]) + abs(encoder[2])
                     + abs(encoder[3]) + abs(encoder[4])) / 4;
            sprintf(buf, "LF Ph:%1d D:%5d", mode_entry_tick, dist);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_30)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot30 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_FORWARD_2)
    {
        if (++lcd_div >= 5)
        {
            int fwd = (encoder[1] + encoder[2] + encoder[3] + encoder[4]) / 4;
            sprintf(buf, "Fwd2:%5ld", (long)fwd);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_ALIGN)
    {
        if (++lcd_div >= 5)
        {
            float e = HeadingHold_WrapAngle180(
                left_heading_absolute - HWT101_GetAbsoluteYaw()) * yaw_ccw_sign;
            sprintf(buf, "Align E:%4.1f", (double)e);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_LEFT_40)
    {
        if (++lcd_div >= 5)
        {
            if (final_left_stage == 0)
                ChassisPos_DrawStatus(20, "Left 11cm");
            else
                ChassisPos_DrawStatus(20, "Left 2cm");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_FINAL_ROTATE_180)
    {
        if (++lcd_div >= 5)
        {
            sprintf(buf, "Rot180 A:%3d", (int)Angle);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PAUSE_GENERIC)
    {
        if (++lcd_div >= 5)
        {
            if ((pause_next_mode == ROUTE_FINAL_LEFT_40) &&
                (final_left_stage == 1U))
            {
                CompetitionQrStatus qr = CompetitionQr_GetStatus();
                if (qr == COMP_QR_READY)
                    sprintf(buf, "T2 QR:%u",
                            CompetitionQr_GetTaskCode(COMP_QR_TASK2));
                else if (qr == COMP_QR_DEFAULTED)
                    sprintf(buf, "T2 DEFAULT:1");
                else
                    sprintf(buf, "T2 SCAN:%u",
                            CompetitionQr_GetAttemptCount());
            }
            else
            {
                sprintf(buf, "Pause:%3d", mode_entry_tick);
            }
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_RIGHT_SHIFT)
    {
        if (++lcd_div >= 10)
        {
            ChassisPos_DrawStatus(20, "Home Right 12cm");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_RETURN_ALIGN)
    {
        if (++lcd_div >= 10)
        {
            sprintf(buf, "Home Align T:%u", home_return_align_ticks);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_HOME_RETURN_DRIVE)
    {
        if (++lcd_div >= 10)
        {
            int d = (int)(-ChassisPos_GetLeftPosition());
            ChassisPos_DrawStatus(20, "Home Drive");
            sprintf(buf, "D:%d", d);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_TRACK_RETURN)
    {
        if (++lcd_div >= 10)
        {
            sprintf(buf, "R S:%2d T:%3d", debug_line_state, debug_turn_target);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "W:%3d V:%3d", wheel_target[1], velocity[1]);
            ChassisPos_DrawStatus(40, buf);
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_TRACK_FIRST)
    {
        if (++lcd_div >= 10)
        {
            sprintf(buf, "S:%2d T:%3d", debug_line_state, debug_turn_target);
            ChassisPos_DrawStatus(20, buf);
            sprintf(buf, "W:%3d V:%3d", wheel_target[1], velocity[1]);
            ChassisPos_DrawStatus(40, buf);
            /* 摄像头收帧诊断: RX=总字节 P=完整帧 R=色块识别结果 */
            sprintf(buf, "RX:%u P:%u R:%u",
                    (unsigned)maxicam_diag_rx_bytes,
                    (unsigned)maxicam_diag_packets,
                    (unsigned)Task1Color_GetLastResult());
            ChassisPos_DrawStatus(60, buf);
            lcd_div = 0;
        }
        return;
    }
}

/* ---- 200Hz ISR ---- */
void ProjectService_OnTimerElapsed(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM7)
    {
        return;
    }

    ChassisMotor_SampleEncoders();
    ChassisMotor_IntegrateEncoders();

    if (route_state == ROUTE_HOME_LAUNCH)
    {
        if (home_launch_tick == 0)
        {
            ChassisPos_Start(120000, 120000, 120000, 120000);
            home_launch_tick = 1;
        }

        if (home_launch_tick == 1)
        {
            if (ChassisPos_Update(HOME_LAUNCH_SPEED, 0, 0))
            {
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                home_turn_start_absolute = HWT101_GetAbsoluteYaw();
                home_launch_tick = 2;
            }
        }

        if (home_launch_tick >= 2)
        {
            float a;
            int spd = HOME_TURN_SPEED;
            a = Angle;
            if (a < 0.0f) a = -a;
            if (a > 70.0f)
            {
                spd = HOME_TURN_SPEED - (int)((a - 70.0f) * 13.2f);
                if (spd < 120) spd = 120;
            }
            wheel_target[1] = -spd;
            wheel_target[2] = -spd;
            wheel_target[3] =  spd;
            wheel_target[4] =  spd;
            if (a >= HOME_TURN_ANGLE)
            {
                float ccw_delta = HeadingHold_WrapAngle180(
                    HWT101_GetAbsoluteYaw() - home_turn_start_absolute);
                yaw_ccw_sign = (ccw_delta >= 0.0f) ? 1 : -1;
                left_heading_absolute = HWT101_GetAbsoluteYaw();
                left_heading_valid = 1;
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                home_fixed_ticks = 0;
                route_state = ROUTE_HOME_FIXED_FORWARD;
                lcd_div = 0;
                mode_entry_tick = 0;
                ChassisPos_Start(155400, 155400, 155400, 155400);
                ChassisPos_DrawStatus(1, "Home Fwd 84cm");
            }
        }
    }

    if (route_state == ROUTE_HOME_FIXED_FORWARD)
    {
        if (ChassisPos_Update(HOME_FIXED_FORWARD_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            home_fixed_ticks = 0;
            route_state = ROUTE_HOME_FIXED_PAUSE;
            lcd_div = 0;
            CompetitionQr_RequestStart(COMP_QR_TASK1);
            ChassisPos_DrawStatus(1, "Scan Task 1");
        }
    }

    if (route_state == ROUTE_HOME_FIXED_PAUSE)
    {
        LineFollow_Stop();
        CompetitionQrStatus qr_status = CompetitionQr_GetStatus();
        home_fixed_ticks++;
        if ((home_fixed_ticks >= HOME_FIXED_PAUSE_TICKS) &&
            ((qr_status == COMP_QR_READY) ||
             (qr_status == COMP_QR_DEFAULTED)))
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            home_fixed_ticks = 0;
            lcd_div = 0;
            route_state = ROUTE_HOME_FIXED_LEFT;
            ChassisPos_Start(-(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST,
                             -(int)HOME_FIXED_LEFTBACK_BACK_DIST);
            ChassisPos_DrawStatus(1, "Home Back");
        }
    }

    if (route_state == ROUTE_HOME_FIXED_LEFT)
    {
        if (home_fixed_ticks == 0)
        {
            if (ChassisPos_Update(HOME_FIXED_LEFTBACK_FWD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                home_fixed_ticks = 1;
                ChassisPos_Start(-(int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                  (int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                 -(int)HOME_FIXED_LEFTBACK_LEFT_DIST,
                                  (int)HOME_FIXED_LEFTBACK_LEFT_DIST);
                ChassisPos_DrawStatus(1, "Home Left");
            }
        }
        else
        {
            uint8_t pos_done = ChassisPos_Update(0,
                  HOME_FIXED_LEFTBACK_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection());
            Sensor_Get();
            {
                int bc = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
                if (bc == 2 && GS_state[4] && GS_state[5])
                    mode_entry_tick++;
                else
                    mode_entry_tick = 0;
            }
            if (pos_done || mode_entry_tick >= 3)
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                home_fixed_ticks = 0;
                route_state = ROUTE_HOME_FIXED_SETTLE;
                lcd_div = 0;
                mode_entry_tick = 0;
                ChassisPos_DrawStatus(1, "Home Settle");
            }
        }
    }

    if (route_state == ROUTE_HOME_FIXED_SETTLE)
    {
        LineFollow_Stop();
        if (++home_fixed_ticks >= HOME_FIXED_SETTLE_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            last_steer = 0;
            LineFollow_ResetLinePID();
            marker_suppress_ticks = 0;
            mode_entry_tick = 0;
            lcd_div = 0;
            route_state = ROUTE_TRACK_FIRST;
            TurntableInventory_Reset();
            Task1Color_Start();
            ChassisPos_DrawStatus(1, "Line Follow");
            return; /* Do not execute ROUTE_TRACK_FIRST again in this ISR. */
        }
    }

    if (route_state == ROUTE_ROTATE_90)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= ROTATE_90_TARGET_ANGLE)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            {
                int curve_right = 22200;
                ChassisPos_Start(CURVE_ENCODER_COUNTS + curve_right,
                                 CURVE_ENCODER_COUNTS - curve_right,
                                 CURVE_ENCODER_COUNTS + curve_right,
                                 CURVE_ENCODER_COUNTS - curve_right);
            }
            route_state = ROUTE_CURVE;
            mode_entry_tick = 0;
            lcd_div = 0;
        }
        else
        {
            int spd = ROTATE_90_SPEED;
            if (a > 70.0f)
            {
                spd = ROTATE_90_SPEED - (int)((a - 70.0f) * 7.5f);
                if (spd < 80) spd = 80;
            }
            wheel_target[1] =  spd;
            wheel_target[2] =  spd;
            wheel_target[3] = -spd;
            wheel_target[4] = -spd;
        }
    }

    if (route_state == ROUTE_CURVE)
    {
        int cross_marker;
        Sensor_Get();
        {
            int bc = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                    + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
            cross_marker = (bc >= 3) && (bc <= 4)
                        && (GS_state[3] || GS_state[4]);
        }
        if (cross_marker)
            cross_marker_ticks++;
        else
            cross_marker_ticks = 0;

        if (ChassisPos_Update(CURVE_FORWARD_SPEED, CURVE_RIGHT_MOVE_SPEED,
              HeadingHold_GetFreshCorrection())
            /* || cross_marker_ticks >= 3 */)
        {
            ChassisPos_ResetVelocityPid();
            LineFollow_Stop();
            vision_next_mode = ROUTE_REAR_ALIGN;
            vision_started = 0U;
            pause_next_mode = ROUTE_PLACE_PREPARE;
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause Vision");
            Move_Control(0, 0, 0, 0);
            return;
        }

        if (++lcd_div >= 5)
        {
            char buf[24];
            int right_cnt = (int)(-debug_trap_left_position);
            int right_cm  = right_cnt / 18500;
            int right_mm  = (right_cnt % 18500) * 10 / 18500;
            if (right_cm > 9) right_cm = 9;
            sprintf(buf, "R:%d.%dcm E:%d", right_cm, right_mm,
                    debug_trap_left_error);
            ChassisPos_DrawStatus(20, buf);
            lcd_div = 0;
        }
    }

    if (route_state == ROUTE_REAR_ALIGN)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(0, REAR_ALIGN_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                ChassisPos_Start(55500, 55500, 55500, 55500);
                ChassisPos_DrawStatus(1, "Fwd 30cm");
            }
        }
        else
        {
            if (ChassisPos_Update(FINAL_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                route_state = ROUTE_PAUSE;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_PAUSE)
    {
        mode_entry_tick++;
        if (mode_entry_tick >= PAUSE_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            vision_next_mode = ROUTE_REAR_ALIGN_2;
            vision_started = 0U;
            route_state = ROUTE_PLACE_PREPARE;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Place Prepare");
        }
        else
        {
            LineFollow_Stop();
        }
    }

    if (route_state == ROUTE_REAR_ALIGN_2)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(REAR_ALIGN_2_BACK_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                ChassisPos_Start(-REAR_ALIGN_2_DISTANCE,  REAR_ALIGN_2_DISTANCE,
                                -REAR_ALIGN_2_DISTANCE,  REAR_ALIGN_2_DISTANCE);
                ChassisPos_DrawStatus(1, "Left 8.5cm");
            }
        }
        else
        {
            if (ChassisPos_Update(0, REAR_ALIGN_2_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection())
                /* || cross_marker_ticks >= 3 */)
            {
                ChassisPos_ResetVelocityPid();
                LineFollow_Stop();
                route_state = ROUTE_PAUSE_2;
                mode_entry_tick = 0;
                cross_marker_ticks = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause 2");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_PAUSE_2)
    {
        mode_entry_tick++;
        if (mode_entry_tick >= PAUSE_2_TICKS)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            vision_next_mode = ROUTE_FINAL_LEFT_FORWARD;
            vision_started = 0U;
            route_state = ROUTE_PLACE_PREPARE;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Vision Align");
        }
        else
        {
            LineFollow_Stop();
        }
    }

    if (route_state == ROUTE_FINAL_LEFT_FORWARD)
    {
        if (mode_entry_tick == 0)
        {
            if (ChassisPos_Update(FINAL_LF_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 1;
                {
                    int lat = (int)((long)FINAL_LF_DISTANCE
                                  * FINAL_LF_LEFT_SPEED / FINAL_LF_FORWARD_SPEED);
                    lat = lat * 2 - 74000;
                    ChassisPos_Start(-lat, lat, -lat, lat);
                }
                ChassisPos_DrawStatus(1, "Left 7cm");
            }
        }
        else if (mode_entry_tick == 1)
        {
            if (ChassisPos_Update(0, FINAL_LF_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection()))
            {
                ChassisPos_ResetVelocityPid();
                mode_entry_tick = 2;
                ChassisPos_Start(92500, 92500, 92500, 92500);
                ChassisPos_DrawStatus(1, "Fwd 50cm");
            }
        }
        else
        {
            if (ChassisPos_Update(FINAL_LF_FORWARD_SPEED, 0,
                  HeadingHold_GetFreshCorrection()))
            {
                LineFollow_Stop();
                vision_next_mode = ROUTE_FINAL_ROTATE_30;
                vision_started = 0U;
                pause_next_mode = ROUTE_PLACE_PREPARE;
                route_state = ROUTE_PAUSE_GENERIC;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Pause");
                Move_Control(0, 0, 0, 0);
                return;
            }
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_30)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= FINAL_ROTATE_30_ANGLE)
        {
            LineFollow_Stop();
            pause_next_mode = ROUTE_FINAL_FORWARD_2;
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
        }
        else
        {
            wheel_target[1] =  FINAL_ROTATE_30_SPEED;
            wheel_target[2] =  FINAL_ROTATE_30_SPEED;
            wheel_target[3] = -FINAL_ROTATE_30_SPEED;
            wheel_target[4] = -FINAL_ROTATE_30_SPEED;
        }
    }

    if (route_state == ROUTE_FINAL_FORWARD_2)
    {
        if (ChassisPos_Update(FINAL_FORWARD_2_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
            vision_next_mode = ROUTE_FINAL_ROTATE_ALIGN;
            vision_started = 0U;
            pause_next_mode = ROUTE_PLACE_PREPARE;
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
            Move_Control(0, 0, 0, 0);
            return;
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_ALIGN)
    {
        if (!arm_to_task0_done)
        {
            const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
            if (Servo4_ArmRampTick(t0->big_arm_us, t0->small_arm_us))
            {
                arm_to_task0_done = 1U;
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
            }
            else
            {
                LineFollow_Stop();
                return;
            }
        }

        float a = Angle;
        if (a < 0.0f) a = -a;

        if (a < FINAL_ROTATE_ALIGN_ANGLE * 0.8f)
        {
            int spd = FINAL_ROTATE_ALIGN_SPEED;
            if (a > FINAL_ROTATE_ALIGN_ANGLE * 0.6f)
            {
                spd = FINAL_ROTATE_ALIGN_SPEED - (int)((a - FINAL_ROTATE_ALIGN_ANGLE * 0.6f) * 10.0f);
                if (spd < 100) spd = 100;
            }
            wheel_target[1] =  spd;
            wheel_target[2] =  spd;
            wheel_target[3] = -spd;
            wheel_target[4] = -spd;
        }
        else
        {
            if (!left_heading_valid || !HWT101_IsReady())
            {
                LineFollow_Stop();
                return;
            }

            final_align_tick++;

            if (final_align_tick > 200)
            {
                LineFollow_Stop();
                pause_next_mode = ROUTE_FINAL_LEFT_40;
                route_state = ROUTE_PAUSE_GENERIC;
                mode_entry_tick = 0;
                lcd_div = 0;
                final_left_heading_stable_ticks = 0;
                final_align_tick = 0;
                final_left_stage = 0;
                ChassisPos_DrawStatus(1, "Align TO");
                Move_Control(0, 0, 0, 0);
                return;
            }

            uint32_t frame = HWT101_GetUpdateCount();
            if (frame == final_align_last_frame)
                return;
            final_align_last_frame = frame;

            float error_ccw = HeadingHold_WrapAngle180(
                left_heading_absolute - HWT101_GetAbsoluteYaw()) * yaw_ccw_sign;
            float error_abs = (error_ccw >= 0.0f) ? error_ccw : -error_ccw;

            if (error_abs <= 2.0f)
            {
                LineFollow_Stop();
                if (++final_left_heading_stable_ticks >= 3)
                {
                    LineFollow_Stop();
                    pause_next_mode = ROUTE_FINAL_LEFT_40;
                    route_state = ROUTE_PAUSE_GENERIC;
                    mode_entry_tick = 0;
                    lcd_div = 0;
                    final_left_heading_stable_ticks = 0;
                    final_align_tick = 0;
                    final_left_stage = 0;
                    ChassisPos_DrawStatus(1, "Pause");
                    Move_Control(0, 0, 0, 0);
                    return;
                }
            }
            else if (error_abs > 3.5f)
            {
                final_left_heading_stable_ticks = 0;
                int spd;
                if      (error_abs > 15.0f) spd = 280;
                else if (error_abs > 6.0f)  spd = 220;
                else                         spd = 140;

                if (error_ccw > 0.0f)
                {
                    wheel_target[1] = -spd;
                    wheel_target[2] = -spd;
                    wheel_target[3] =  spd;
                    wheel_target[4] =  spd;
                }
                else
                {
                    wheel_target[1] =  spd;
                    wheel_target[2] =  spd;
                    wheel_target[3] = -spd;
                    wheel_target[4] = -spd;
                }
            }
            else
            {
                LineFollow_Stop();
                final_left_heading_stable_ticks = 0;
            }
        }
    }

    if (route_state == ROUTE_FINAL_LEFT_40)
    {
        if (ChassisPos_Update(0, FINAL_LEFT_40_SPEED, 0))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            task2_transition_step = 0U;
            route_state = ROUTE_TASK1_TO_TASK2;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "T1->T2");
            return;
        }
    }

    if (route_state == ROUTE_PAUSE_GENERIC)
    {
        uint8_t waiting_for_task2 =
            ((pause_next_mode == ROUTE_FINAL_LEFT_40) &&
             (final_left_stage == 1U));
        CompetitionQrStatus qr_status = CompetitionQr_GetStatus();
        mode_entry_tick++;
        if ((mode_entry_tick >= PAUSE_GENERIC_TICKS) &&
            ((!waiting_for_task2) ||
             (qr_status == COMP_QR_READY) ||
             (qr_status == COMP_QR_DEFAULTED)))
        {
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            switch (pause_next_mode)
            {
                case ROUTE_FINAL_ROTATE_30:
                    ChassisPos_ResetEncoder();
                    ChassisPos_DrawStatus(1, "Rotate 30 CW"); break;
                case ROUTE_FINAL_FORWARD_2:
                    ChassisPos_Start(61050, 61050, 61050, 61050);
                    ChassisPos_DrawStatus(1, "Final Fwd 33cm"); break;
                case ROUTE_FINAL_ROTATE_ALIGN:
                    ChassisPos_ResetEncoder();
                    final_align_last_frame = 0;
                    final_align_tick = 0;
                    final_left_heading_stable_ticks = 0;
                    ChassisPos_DrawStatus(1, "Rotate 48 CW"); break;
                case ROUTE_FINAL_LEFT_40:
                    if (final_left_stage == 0)
                    {
                        ChassisPos_Start(
                            -203500,  203500, -203500,  203500);
                        ChassisPos_DrawStatus(1, "Left 11 cm");
                    }
                    else
                    {
                        ChassisPos_Start(
                            -37000,  37000, -37000,  37000);
                        ChassisPos_DrawStatus(1, "Left 2 cm");
                    }
                    break;
                case ROUTE_FINAL_ROTATE_180:
                    ChassisPos_ResetEncoder();
                    ChassisPos_DrawStatus(1, "Rotate 180 CW"); break;
                case ROUTE_PLACE_PREPARE:
                    mode_entry_tick = 0;
                    ChassisPos_DrawStatus(1, "Place Prepare"); break;
                case ROUTE_VISION_ALIGN:
                    ChassisPos_DrawStatus(1, "Vision Align"); break;
                default: break;
            }
            route_state = pause_next_mode;
            mode_entry_tick = 0;
            lcd_div = 0;
        }
        else
        {
            LineFollow_Stop();
        }
    }

    /* ---- ROUTE_TASK1_TO_TASK2: 任务1→任务2过渡 (QR→初始化→左移→179°旋转→收杯) ---- */
    if (route_state == ROUTE_TASK1_TO_TASK2)
    {
        switch (task2_transition_step)
        {
        case 0U:
            Task1Color_Stop();
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            LineFollow_Stop();
            task2_transition_qr_ticks = 0U;
            CompetitionQr_RequestStart(COMP_QR_TASK2);
            task2_transition_step = 1U;
            break;

        case 1U:
            {
                CompetitionQrStatus qr = CompetitionQr_GetStatus();

                if ((qr == COMP_QR_READY) ||
                    (qr == COMP_QR_DEFAULTED))
                {
                    task2_transition_qr_ticks = 0U;
                    task2_transition_step = 2U;
                }
                else
                {
                    if (task2_transition_qr_ticks <
                        TASK2_TRANSITION_QR_TIMEOUT_TICKS)
                    {
                        task2_transition_qr_ticks++;
                    }

                    if (task2_transition_qr_ticks >=
                        TASK2_TRANSITION_QR_TIMEOUT_TICKS)
                    {
                        CompetitionQr_ForceDefault(COMP_QR_TASK2);
                        task2_transition_qr_ticks = 0U;
                        task2_transition_step = 2U;
                    }
                }
            }
            break;

        case 2U:
            TurntableInventory_Init(840U, 400U);
            Servo4_RotationHome();
            Servo4_TrophyLittleAllOpen();
            task2_capture_index = 0U;
            task2_place_index = 0U;
            task2_collect_complete = 0U;
            task2_collect_done = 0U;
            trophy_det_count = 0U;
            trophy_det_confirm = 0U;
            trophy_det_last_seq = 0U;
            trophy_det_gap_until_ms = 0U;
            trophy_detect_armed = 1U;
            trophy_clear_frames = 0U;
            trophy_last_rank = 0U;
            trophy_last_slot = 0U;
            task2_transition_step = 3U;
            break;

        case 3U:
            {
                const Task1ArmPose *tgt = Task1Mechanism_GetWorkingPose();
                if (Servo4_ArmRampTickSlow(tgt->big_arm_us, tgt->small_arm_us))
                {
                    LineFollow_Stop();
                    ChassisPos_ResetVelocityPid();
                    task2_transition_step = 4U;
                }
            }
            break;

        case 4U:
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            ChassisPos_Start(
                -(int)TASK2_ENTRY_LEFT_DISTANCE,
                 (int)TASK2_ENTRY_LEFT_DISTANCE,
                -(int)TASK2_ENTRY_LEFT_DISTANCE,
                 (int)TASK2_ENTRY_LEFT_DISTANCE);
            task2_transition_step = 5U;
            break;

        case 5U:
            if (ChassisPos_Update(0, TASK2_ENTRY_LEFT_SPEED,
                  HeadingHold_GetFreshCorrection()))
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                Move_Control(0, 0, 0, 0);
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                task2_transition_step = 6U;
            }
            break;

        case 6U:
            {
                float a = Angle;
                if (a < 0.0f) a = -a;
                if (a >= FINAL_ROTATE_180_ANGLE)
                {
                    LineFollow_Stop();
                    ChassisPos_ResetVelocityPid();
                    Move_Control(0, 0, 0, 0);
                    mode_entry_tick = 0;
                    task2_transition_step = 7U;
                }
                else
                {
                    wheel_target[1] =  FINAL_ROTATE_180_SPEED;
                    wheel_target[2] =  FINAL_ROTATE_180_SPEED;
                    wheel_target[3] = -FINAL_ROTATE_180_SPEED;
                    wheel_target[4] = -FINAL_ROTATE_180_SPEED;
                }
            }
            break;

        case 7U:
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            if (++mode_entry_tick >= TASK2_ENTRY_SETTLE_TICKS)
            {
                task2_transition_step = 8U;
            }
            break;

        case 8U:
            trophy_det_confirm = 0U;
            trophy_det_last_seq = 0U;
            trophy_det_gap_until_ms = 0U;
            trophy_detect_armed = 1U;
            trophy_clear_frames = 0U;
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            LineFollow_ResetLinePID();
            last_steer = 0;
            marker_suppress_ticks = 0;
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            (void)MaxiCam_SetMode(MAXICAM_MODE_TROPHY_WHITE);
            task2_transition_step = 0U;
            route_state = ROUTE_TRACK_RETURN;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "T2 Collect");
            break;

        default:
            task2_transition_step = 0U;
            break;
        }
    }

    if (route_state == ROUTE_FINAL_ROTATE_180)
    {
        float a = Angle;
        if (a < 0.0f) a = -a;
        if (a >= FINAL_ROTATE_180_ANGLE)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            LineFollow_Stop();
            route_state = ROUTE_TRACK_RETURN;
            LineFollow_ResetLinePID();
            marker_suppress_ticks = 0;
            lcd_div = 0;
            mode_entry_tick = 0;
            ChassisPos_DrawStatus(1, "Line Follow");
        }
        else
        {
            wheel_target[1] =  FINAL_ROTATE_180_SPEED;
            wheel_target[2] =  FINAL_ROTATE_180_SPEED;
            wheel_target[3] = -FINAL_ROTATE_180_SPEED;
            wheel_target[4] = -FINAL_ROTATE_180_SPEED;
        }
    }

    if (route_state == ROUTE_PLACE_ERROR)
    {
        LineFollow_Stop();
    }

    if (route_state == ROUTE_TASK2_PREPARE)
    {
        LineFollow_Stop();
    }

    if (route_state == ROUTE_MISSION_DONE)
    {
        LineFollow_Stop();
    }

    /* ---- ROUTE_TROPHY: 奖杯放置 ---- */
    if (route_state == ROUTE_TROPHY)
    {
        int hc;

        if (trophy_phase == 0)
        {
            LineFollow_Stop();
            if (++trophy_ticks >= TROPHY_PAUSE_TICKS)
            {
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                Servo4_TrophyLittleAllClose();
                Servo4_TrophyRotationLock();
                trophy_ticks = 0U;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_back_active = 0U;
                trophy_phase = 1;
            }
        }
        else if (trophy_phase == 1)
        {
            float a = Angle;
            if (a < 0.0f) a = -a;
            if (a >= TROPHY_ROTATE_ANGLE)
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                trophy_phase = 2;
            }
            else
            {
                wheel_target[1] =  TROPHY_ROTATE_SPEED;
                wheel_target[2] =  TROPHY_ROTATE_SPEED;
                wheel_target[3] = -TROPHY_ROTATE_SPEED;
                wheel_target[4] = -TROPHY_ROTATE_SPEED;
            }
        }
        else if (trophy_phase == 2)
        {
            if (!pos_control_active)
            {
                ChassisPos_Start(-(int)TROPHY_BACK_DISTANCE,
                                 -(int)TROPHY_BACK_DISTANCE,
                                 -(int)TROPHY_BACK_DISTANCE,
                                 -(int)TROPHY_BACK_DISTANCE);
            }
            hc = HeadingHold_GetFreshCorrection();
            if (ChassisPos_Update(TROPHY_BACK_SPEED, 0, hc))
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                trophy_ticks = 0U;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_back_active = 0U;
                trophy_phase = 3;
            }
        }
        else if (trophy_phase >= 3 && trophy_phase <= 5)
        {
            if (TrophyPlace_Process(task2_place_index))
            {
                trophy_phase++;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_settle_ticks = 0U;
                trophy_place_back_active = 0U;
            }
        }
        /* Phase 6: 三个奖杯放置完成 → 回家 */
        else
        {
            LineFollow_Stop();
            trophy_cam_mode_on = 0U;
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            home_return_align_ticks = 0U;
            home_return_stable_frames = 0U;
            home_return_last_frame = HWT101_GetUpdateCount();
            __disable_irq();
            route_state = ROUTE_HOME_RETURN_ALIGN;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Home Align");
        }
    }

    if (route_state == ROUTE_VISION_ALIGN)
    {
        if (!vision_skip)
        {
            int vision_target[4];
            VisionAlignment_GetWheelTargets(vision_target);
            LineFollow_SetWheelTargets(vision_target[0], vision_target[1],
                                       vision_target[2], vision_target[3]);
        }
    }

    if (route_state == ROUTE_TRACK_FIRST)
    {
        int i, black_count, line_state = 0, turn_target;
        Sensor_Get();

        if (mode_entry_tick > 0)
        {
            LineFollow_Stop();
            mode_entry_tick++;
            if (mode_entry_tick >= 100)
            {
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                ChassisPos_SetRotate90Target();
                Task1Color_Stop();
                route_state = ROUTE_ROTATE_90;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Rotate 90");
            }
        }
        else if ((Task1Color_GetCount() >= TURNTABLE_SLOT_COUNT) &&
                 GS_state[1] && GS_state[2] &&
                 GS_state[3] && GS_state[4])
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Servo4_Task1RotationLockNow();
            mode_entry_tick = 1;
        }
        else
        {
            black_count = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                        + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
            if (black_count == 0)
                turn_target = last_steer;
            else {
                line_state = 0;
                for (i = 1; i <= 8; i++)
                    if (GS_state[i]) line_state += line_weights[i];
                turn_target = LineFollow_ComputeLinePID(line_state);
                last_steer = turn_target;
            }
            debug_line_state = line_state;
            debug_turn_target = turn_target;
            LineFollow_SetWheelTargets(TRACK_FIRST_SPEED_TARGET + turn_target,
                                       TRACK_FIRST_SPEED_TARGET + turn_target,
                                       TRACK_FIRST_SPEED_TARGET - turn_target,
                                       TRACK_FIRST_SPEED_TARGET - turn_target);
        }
    }

    if (route_state == ROUTE_TRACK_RETURN)
    {
        int i, black_count, line_state = 0, turn_target;
        int collect_speed;
        Sensor_Get();

        /* 唯一的十字标记抑制计数点 */
        if (marker_suppress_ticks < MARKER_SUPPRESS_TICKS)
            marker_suppress_ticks++;

        /* 快慢速策略: 第一杯前快速, 命中后降速 */
        collect_speed = (task2_capture_index == 0U)
                      ? (int)TROPHY_GRAB_FAST_SPEED
                      : (int)TROPHY_GRAB_SLOW_SPEED;

        /* 收集阶段: 摄像头检测奖杯, 记录等级→转盘槽位 */
        if (!task2_collect_complete)
        {
            if (TrophyDetect_CollectSeen())
            {
                QrTask2Trophy rank;
                uint8_t cap_slot;

                rank = CompetitionQr_GetTask2CaptureTrophy(
                    CompetitionQr_GetTaskCode(COMP_QR_TASK2),
                    task2_capture_index);

                if (TurntableInventory_RecordTrophyRank(
                        (uint8_t)rank, &cap_slot))
                {
                    task2_capture_index++;
                    trophy_det_count++;
                    trophy_last_rank = (uint8_t)rank;
                    trophy_last_slot = cap_slot;
                    Servo4_RotationSetStep(task2_capture_index);
                }

                if (task2_capture_index >= 3U)
                {
                    task2_collect_complete = 1U;
                    task2_collect_done = 0U;
                    trophy_ticks = 0U;
                    Servo4_TrophyLittleAllClose();
                }

                marker_suppress_ticks = -(int)MARKER_SUPPRESS_TICKS;
            }
        }

        if (task2_collect_complete && !task2_collect_done)
        {
            if (++trophy_ticks >= TROPHY_GRIP_CLOSE_TICKS)
            {
                if (Servo4_ArmRampTickSlow(1250U, 2000U))
                {
                    task2_collect_done = 1U;
                    marker_suppress_ticks = (int)MARKER_SUPPRESS_TICKS;
                }
            }
        }

        /* LCD 收集诊断 */
        if (++lcd_div >= 25)
        {
            char buf[24];
            uint8_t qr = CompetitionQr_GetTaskCode(COMP_QR_TASK2);
            char rc = '?';
            if (trophy_last_rank == QR_TASK2_TROPHY_CHAMPION)  rc = 'A';
            else if (trophy_last_rank == QR_TASK2_TROPHY_RUNNER_UP) rc = 'B';
            else if (trophy_last_rank == QR_TASK2_TROPHY_THIRD_PLACE) rc = 'C';

            ChassisPos_DrawStatus(1, "T2 COLLECT");
            sprintf(buf, "QR:%u C:%u/3", (unsigned)qr,(unsigned)trophy_det_count);
            LCD_DrawString_NoDMA(1, 20, buf, LCD_WHITE, LCD_BLACK);
            sprintf(buf, "LAST:%c SLOT:%u", rc, (unsigned)trophy_last_slot);
            LCD_DrawString_NoDMA(1, 40, buf, LCD_WHITE, LCD_BLACK);
            sprintf(buf, "SPD:%d", collect_speed);
            LCD_DrawString_NoDMA(1, 60, buf, LCD_WHITE, LCD_BLACK);
            lcd_div = 0;
        }


        /* 前灰度右四路(5~8)全黑 → ROUTE_TROPHY */
        if (GS_state[5] && GS_state[6] && GS_state[7] && GS_state[8]
            && marker_suppress_ticks >= MARKER_SUPPRESS_TICKS)
        {
            if (!task2_collect_complete)
            {
                marker_suppress_ticks = -(int)MARKER_SUPPRESS_TICKS;
            }
            else
            {
                ChassisPos_ResetVelocityPid();
                HeadingHold_Reset();
                LineFollow_Stop();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                trophy_phase = 0;
                trophy_ticks = 0;
                trophy_cam_mode_on = 0U;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_settle_ticks = 0U;
                trophy_place_back_active = 0U;
                trophy_det_confirm = 0U;
                trophy_det_last_seq = 0U;
                trophy_det_count = 0U;
                trophy_det_gap_until_ms = 0U;
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                route_state = ROUTE_TROPHY;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "T2 Place");
                Move_Control(0, 0, 0, 0);
                goto skip_track_return;
            }
        }

        black_count = GS_state[1] + GS_state[2] + GS_state[3] + GS_state[4]
                    + GS_state[5] + GS_state[6] + GS_state[7] + GS_state[8];
        if (black_count == 0)
            turn_target = last_steer;
        else {
            line_state = 0;
            for (i = 1; i <= 8; i++)
                if (GS_state[i]) line_state += line_weights[i];
            turn_target = LineFollow_ComputeLinePID(line_state);
            last_steer = turn_target;
        }
        debug_line_state = line_state;
        debug_turn_target = turn_target;
        LineFollow_SetWheelTargets(collect_speed + turn_target,
                                   collect_speed + turn_target,
                                   collect_speed - turn_target,
                                   collect_speed - turn_target);
        skip_track_return: ;
    }

    /* ---- ROUTE_HOME_RIGHT_SHIFT: 后退到位后纯右移12cm ---- */
    if (route_state == ROUTE_HOME_RIGHT_SHIFT)
    {
        static uint8_t home_right_shift_started = 0;
        if (!home_right_shift_started)
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            ChassisPos_Start(HOME_RIGHT_SHIFT_DISTANCE,
                            -(int)HOME_RIGHT_SHIFT_DISTANCE,
                             HOME_RIGHT_SHIFT_DISTANCE,
                            -(int)HOME_RIGHT_SHIFT_DISTANCE);
            home_right_shift_started = 1;
        }
        else if (ChassisPos_Update(HOME_RIGHT_SHIFT_SPEED,
                                    HOME_RIGHT_SHIFT_SPEED, 0))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            home_right_shift_started = 0;
            route_state = ROUTE_MISSION_DONE;
            ChassisPos_DrawStatus(1, "MISSION DONE");
        }
        ChassisMotor_ControlTick(wheel_target);
        return;
    }

    /* ---- ROUTE_HOME_RETURN_ALIGN: 陀螺仪对准回家方向 ---- */
    if (route_state == ROUTE_HOME_RETURN_ALIGN)
    {
        float error;
        float error_abs;
        uint32_t frame;

        if (!home_forward_heading_valid || !HWT101_IsReady())
        {
            LineFollow_Stop();
            return;
        }

        if (++home_return_align_ticks > HOME_RETURN_ALIGN_TIMEOUT)
        {
            LineFollow_Stop();
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            ChassisPos_Start(-(int)HOME_RETURN_DISTANCE,
                             -(int)HOME_RETURN_DISTANCE,
                             -(int)HOME_RETURN_DISTANCE,
                             -(int)HOME_RETURN_DISTANCE);
            route_state = ROUTE_HOME_RETURN_DRIVE;
            ChassisPos_DrawStatus(1, "Home Drive TO");
            return;
        }

        frame = HWT101_GetUpdateCount();
        if (frame == home_return_last_frame) return;
        home_return_last_frame = frame;
        error = HeadingHold_WrapAngle180(
                    home_forward_heading - HWT101_GetAbsoluteYaw());
        error_abs = (error >= 0.0f) ? error : -error;

        if (error_abs <= 2.0f)
        {
            LineFollow_Stop();
            if (++home_return_stable_frames >= HOME_RETURN_ALIGN_STABLE)
            {
                ChassisPos_ResetEncoder();
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                ChassisPos_Start(-(int)HOME_RETURN_DISTANCE,
                                 -(int)HOME_RETURN_DISTANCE,
                                 -(int)HOME_RETURN_DISTANCE,
                                 -(int)HOME_RETURN_DISTANCE);
                route_state = ROUTE_HOME_RETURN_DRIVE;
                ChassisPos_DrawStatus(1, "Home Drive");
            }
        }
        else
        {
            int spd = (error_abs > 15.0f) ? 280
                    : (error_abs > 6.0f)  ? 220 : 140;
            home_return_stable_frames = 0U;
            if (error > 0.0f)
            {
                wheel_target[1] = -spd; wheel_target[2] = -spd;
                wheel_target[3] =  spd; wheel_target[4] =  spd;
            }
            else
            {
                wheel_target[1] =  spd; wheel_target[2] =  spd;
                wheel_target[3] = -spd; wheel_target[4] = -spd;
            }
        }
    }

    /* ---- ROUTE_HOME_RETURN_DRIVE: 编码器后退回家 ---- */
    if (route_state == ROUTE_HOME_RETURN_DRIVE)
    {
        Sensor_Get();
        if (((ChassisPos_GetLeftPosition() <= -(int)HOME_RETURN_MARKER_ARM) &&
             GS_state[5] && GS_state[6] && GS_state[7] && GS_state[8]) ||
            ChassisPos_Update(HOME_RETURN_SPEED, 0,
                              HeadingHold_GetFreshCorrection()))
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            route_state = ROUTE_HOME_RIGHT_SHIFT;
            ChassisPos_DrawStatus(1, "Home Right");
        }
    }

    ChassisMotor_ControlTick(wheel_target);
}
