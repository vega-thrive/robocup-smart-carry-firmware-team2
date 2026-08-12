#include "ProjectService.h"
#include "LineFollow.h"
#include "HWT101.h"
#include "Sensor.h"
#include "ChassisMotor.h"
#include "CompetitionQr.h"
#include "Servo4.h"
#include "Task1Mechanism.h"
#include "VisionAlignment.h"
#include "TurntableInventory.h"
#include "MaxiCam.h"
#include "TestMode.h"
#include <stdio.h>

/* ---- 状态变量 ---- */
static volatile uint8_t task1_route_test_done;
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
static uint8_t final_left_stage;
static uint8_t arm_to_task0_done;  /* 最后放置后缓降至 Task0 收拢 */
static uint8_t  final_left_heading_stable_ticks;
static uint32_t final_align_last_frame;
static uint16_t final_align_tick;

/* 放置流程状态机: 横移→停车→切目标姿态→稳定→视觉对准→粗调→
 * 固定前插→前插稳定→转盘→缓慢降到释放姿态→低位稳定→三段开爪→落稳→
 * 缓慢抬离→抬离稳定→后退→后退稳定→转盘回安全位→关爪→杯间运输姿态→标记完成.
 * 杯间运输: 统一使用冠军抬头姿态 1000/1300, 横移完成后才切换下一杯姿态.
 * 每杯完整参数集中于 TrophyPlaceProfile, 不再散落各处 if(place_index==2). */
typedef enum {
    PLACE_MOVE_LEFT            = 0,   /* 定距左移(仅底盘, 不启动视觉) */
    PLACE_PREPARE_TARGET_POSE  = 27,  /* 缓慢切换到当前杯目标视觉姿态 */
    PLACE_TARGET_POSE_SETTLE   = 28,  /* 姿态稳定后启动视觉 */
    PLACE_ALIGN                = 1,   /* 视觉对准(手臂已就位, 不额外缓升) */
    PLACE_COARSE_MOVE          = 10,  /* 编码器闭环粗调 */
    PLACE_COARSE_SETTLE        = 11,  /* 粗调后稳定 */
    PLACE_FORWARD_INSERT       = 30,  /* 视觉完成后固定前插, 保证转盘对准 */
    PLACE_INSERT_SETTLE        = 31,  /* 前插到位后停车稳定, 消除俯仰 */
    PLACE_ROTATE_SLOT          = 2,   /* 转盘旋至目标槽位 */
    PLACE_ROTATE_SETTLE        = 3,   /* 等转盘稳定 */
    PLACE_LOWER_TO_RELEASE     = 12,  /* 缓慢降到基础释放姿态(2us/step) */
    PLACE_RELEASE_SETTLE       = 13,  /* 低位稳定 1000ms */
    PLACE_HALF_OPEN            = 14,  /* 夹爪1750: 半开解除夹紧力 */
    PLACE_FULL_OPEN            = 15,  /* 夹爪1450: 正常释放 */
    PLACE_RELEASE_DETACH       = 21,  /* 夹爪1350: 最终脱钩 */
    PLACE_DROP_SETTLE          = 16,  /* 低位保持奖杯落稳 1000ms */
    PLACE_LIFT_CLEAR           = 17,  /* 缓慢抬离奖杯(2us/step) */
    PLACE_LIFT_CLEAR_SETTLE    = 29,  /* 抬离后稳定 500ms */
    PLACE_BACK_CLEAR           = 18,  /* 直线后退到安全距离 */
    PLACE_BACK_SETTLE          = 32,  /* 后退到位后停车稳定, 消除滑行 */
    PLACE_ROTATE_SAFE          = 23,  /* 转盘回安全位置 */
    PLACE_ROTATE_SAFE_SETTLE   = 24,  /* 等转盘安全位稳定 */
    PLACE_CLOSE_GRIPPER        = 25,  /* 关闭当前夹爪 */
    PLACE_TRANSPORT_POSE       = 26,  /* 缓慢切到杯间运输姿态1000/1300 */
    PLACE_FINISH               = 20   /* 标记空位 + 递增索引 → 返回完成 */
} TrophyPlaceStep;

static uint8_t  place_slot_idx;
static uint16_t place_pwm_us;
static uint8_t  place_seq_index;   /* 顺序放置: 0→1→2→3→4 */
static uint8_t  task2_capture_index;
static uint8_t  task2_place_index;
static uint8_t  task2_prep_step;
static uint8_t  task2_collect_complete;
static uint8_t  task2_collect_done;
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

/* 每杯完整放置参数, 全部集中在一处.
 * 季军从此与 TROPHY_POSE_LIFTED (900/1000) 无任何关联, 只用任务1平地 1250/1500. */
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
        /* 亚军 PODIUM: 识别 1100/1400, 释放 1100/1200, 抬离 1100/1350 */
        { 1100U, 1400U, 1100U, 1200U, 1100U, 1350U,
          (int32_t)TROPHY_INSERT_RUNNER_COUNTS,
          (int32_t)TROPHY_BACK_RUNNER_COUNTS },
        /* 冠军 CHAMPION: 识别 1000/1300, 释放 1000/1100, 抬离 1000/1250 */
        { 1000U, 1300U, 1000U, 1100U, 1000U, 1250U,
          (int32_t)TROPHY_INSERT_CHAMPION_COUNTS,
          (int32_t)TROPHY_BACK_CHAMPION_COUNTS },
        /* 季军: 任务1 平地工作姿态 1250/1500, 释放不变, 抬离 1300/1600 */
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
    left_heading_absolute = HWT101_GetAbsoluteYaw();
    left_heading_valid = 1;
    yaw_ccw_sign = -1;
}

/* 模式2(0x02)奖杯检测: 连续 TROPHY_DETECT_CONFIRM 帧 code=26 算命中一次,
 * 每次命中计入一个奖杯, 然后进入 TROPHY_DETECT_CLEAR_MS 清屏期, 防止
 * 同一奖杯还在 ROI 内被连续重复计数.
 * 左移未越过 TROPHY_DETECT_ARM_COUNTS 时忽略, 防止刚夹住的奖杯仍在
 * 摄像头下半屏 ROI 内, 新的一轮刚启动就误触发. */
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

/* 巡线收集用奖杯检测: 与TrophyDetect_Seen相同但不检查左移阈值,
 * 因为TRACK_RETURN期间底盘是前向移动, 左移位置始终为0. */
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

/* 离框确认: 命中后必须连续2帧看不到奖杯才重新使能检测,
 * 防止同一奖杯滞留黄框内被重复计数. 原400ms静默期仍保留. */
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
    task1_route_test_done = 0U;
    place_seq_index = 0;
    task2_collect_complete = 0U;
    task2_collect_done = 0U;
    TurntableInventory_Init(840U, 400U);
    /* 先设 Task0 起始脉宽再缓降至 FLAT, 避免上电猛砸. */
    {
        const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
        Servo4_SetPulse(0U, t0->big_arm_us);
        Servo4_SetPulse(1U, t0->small_arm_us);
    }
    {
        const Task1ArmPose *tgt = Task1Mechanism_GetWorkingPose();
        Servo4_ArmRampTo(tgt->big_arm_us, tgt->small_arm_us);
    }
    /* 上电先退掉摄像头可能残留的模式循环, 不启动颜色收集 */
    (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
#if TASK2_ONLY_TEST_MODE
    __disable_irq();
    route_state = ROUTE_TASK2_PREPARE;
    __enable_irq();
    task2_prep_step = 0U;
    task2_capture_index = 0U;
    task2_place_index = 0U;
    task2_collect_complete = 0U;
    task2_collect_done = 0U;
    trophy_det_count = 0U;
    trophy_det_confirm = 0U;
    trophy_det_last_seq = 0U;
    trophy_det_gap_until_ms = 0U;
    CompetitionQr_RequestStart(COMP_QR_TASK2);
    ChassisPos_DrawStatus(1, "T2 Test Mode");
#else
    ChassisPos_DrawStatus(1, "Home Launch");
#endif
    HAL_TIM_Base_Start_IT(&htim7);
}

/* ---- 放置流程共享函数 ----
 * 一次完整放置: 横移→切目标姿态→稳定→视觉对准→粗调→固定前插→
 * 前插稳定→转盘→缓慢降到释放姿态→低位稳定→三段开爪→落稳→
 * 缓慢抬离→抬离稳定→后退→后退稳定→转盘回安全位→关爪→运输姿态→标记完成.
 * 杯间统一使用冠军抬头运输姿态 1000/1300.
 * 每杯完整参数(识别/释放/抬离/前插距离/后退距离)统一在 TrophyPlaceProfile 中,
 * 状态机从头到尾只读 profile, 不再散落 if(place_index==2) 分支.
 * 季军为最后一杯, 关爪后直接FINISH, 不切换运输姿态.
 * 返回 1 表示放置完成, 调用方负责推进 trophy_phase.
 * place_index: 0=第一杯(亚军), 1=第二杯(冠军), 2=第三杯(季军). */
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
        /* 缓慢切换到本领奖台专用视觉姿态 (2us/step). */
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(profile->align_big, profile->align_small))
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_TARGET_POSE_SETTLE;
        }
        break;

    case PLACE_TARGET_POSE_SETTLE:
        /* 姿态到位后稳定 800ms, 再启动视觉.
         * 确保机构完全静止, 摄像头目标稳定. */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_TARGET_POSE_SETTLE_TICKS)
        {
            if (place_index == 0U)
                VisionAlignment_Start();
            else
                VisionAlignment_Restart();

            trophy_place_step = PLACE_ALIGN;
        }
        break;

    case PLACE_ALIGN: {
        VisionAlignment_Status vs;

        /* 视觉对准 (手臂已就位, 不再缓升) */
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
        /* 视觉对准完成后按本领奖台距离前插, 编码器闭环. */
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
        /* 前插到位后停车稳定, 消除车身俯仰, 避免转盘旋转时扫偏 */
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
        /* 缓慢降到本领奖台释放姿态 (2us/step). */
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
            Servo4_LittleHalfOpenSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_HALF_OPEN;
        }
        break;

    case PLACE_HALF_OPEN:
        /* 半开等待后全开, 两段释放减少横向冲击 */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_HALF_OPEN_WAIT_TICKS)
        {
            Servo4_LittleOpenSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_FULL_OPEN;
        }
        break;

    case PLACE_FULL_OPEN:
        /* 夹爪全开 1450us → 等待 300ms */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= GRIPPER_OPEN_WAIT_TICKS)
        {
            Servo4_LittleReleaseSlot(place_slot_idx);
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_RELEASE_DETACH;
        }
        break;

    case PLACE_RELEASE_DETACH:
        /* 最终脱钩 1350us → 等待 300ms, 确保爪尖完全离开奖杯 */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_DETACH_WAIT_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_DROP_SETTLE;
        }
        break;

    case PLACE_DROP_SETTLE:
        /* 奖杯落稳, 避免未落稳就抬臂导致奖杯移位 */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_DROP_SETTLE_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_LIFT_CLEAR;
        }
        break;

    case PLACE_LIFT_CLEAR:
        /* 缓慢抬离奖杯 (2us/step), 避免快速抬升勾住奖杯. */
        LineFollow_Stop();
        if (Servo4_ArmRampTickSlow(profile->clear_big, profile->clear_small))
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_LIFT_CLEAR_SETTLE;
        }
        break;

    case PLACE_LIFT_CLEAR_SETTLE:
        /* 抬离完成后稳定 500ms, 确保爪尖完全脱离再后退 */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_LIFT_CLEAR_SETTLE_TICKS)
        {
            trophy_place_back_active = 0U;
            trophy_place_step = PLACE_BACK_CLEAR;
        }
        break;

    case PLACE_BACK_CLEAR:
        /* 按本领奖台距离后退, 编码器闭环 */
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
        /* 后退到位后停车稳定, 消除惯性滑行.
         * 车身滑动时旋转转盘, 边缘可能扫到奖杯. */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_BACK_SETTLE_TICKS)
            trophy_place_step = PLACE_ROTATE_SAFE;
        break;

    case PLACE_ROTATE_SAFE:
        /* 转盘回到安全位置(独立可标定), 避免横移时撞领奖台 */
        LineFollow_Stop();
        Servo4_RotationSafe();
        trophy_place_settle_ticks = 0U;
        trophy_place_step = PLACE_ROTATE_SAFE_SETTLE;
        break;

    case PLACE_ROTATE_SAFE_SETTLE:
        /* 等转盘安全位稳定 */
        LineFollow_Stop();
        if (++trophy_place_settle_ticks >= TROPHY_ROTATE_SAFE_SETTLE_TICKS)
        {
            trophy_place_settle_ticks = 0U;
            trophy_place_step = PLACE_CLOSE_GRIPPER;
        }
        break;

    case PLACE_CLOSE_GRIPPER:
        /* 关闭当前槽位夹爪, 奖杯已完全脱离.
         * 季军是最后一杯, 完成后直接结束, 无需切运输姿态. */
        LineFollow_Stop();
        Servo4_LittleCloseSlot(place_slot_idx);
        trophy_place_settle_ticks = 0U;
        if (place_index == 2U)
            trophy_place_step = PLACE_FINISH;
        else
            trophy_place_step = PLACE_TRANSPORT_POSE;
        break;

    case PLACE_TRANSPORT_POSE:
        /* 切换到冠军抬头运输姿态 1000/1300, 提高机构避免撞台.
         * 下一杯若是冠军则直接保持, 若是季军则横移到达后再切换. */
        LineFollow_Stop();
        if (Servo4_ArmRampTick(TROPHY_TRANSPORT_BIG_US,
                               TROPHY_TRANSPORT_SMALL_US))
        {
            trophy_place_step = PLACE_FINISH;
        }
        break;

    case PLACE_FINISH:
        /* 运输姿态就绪 → 标记空位 + 递增索引.
         * task2_place_index 在所有退出动作完成后才递增. */
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

#if LITTLE_GRIPPER_SEQ_TEST
    {
        static uint8_t  grip_test_state  = 0U;
        static uint16_t grip_test_ticks  = 0U;
        static uint8_t  grip_test_slot   = 0U;

        /* ~100Hz loop → 1 tick = 10ms */
        switch (grip_test_state)
        {
        case 0U: /* 初始化: 全部夹紧 */
            Servo4_LittleAllClose();
            grip_test_ticks = 0U;
            grip_test_state = 1U;
            break;

        case 1U: /* 稳定等待 500ms */
            if (++grip_test_ticks >= 50U)
            {
                grip_test_ticks = 0U;
                grip_test_state = 2U;
            }
            break;

        case 2U: /* 张开当前槽位 (1450us = 全开) */
            Servo4_LittleOpenSlot(grip_test_slot);
            {
                char _buf[16];
                sprintf(_buf, "GRIP %d OPEN", (int)grip_test_slot);
                ChassisPos_DrawStatus(1, _buf);
            }
            grip_test_ticks = 0U;
            grip_test_state = 3U;
            break;

        case 3U: /* 保持张开 2s, 肉眼确认是第几个爪子 */
            if (++grip_test_ticks >= 200U)
            {
                grip_test_ticks = 0U;
                grip_test_state = 4U;
            }
            break;

        case 4U: /* 夹紧当前槽位 */
            Servo4_LittleCloseSlot(grip_test_slot);
            {
                char _buf[16];
                sprintf(_buf, "GRIP %d CLOSE", (int)grip_test_slot);
                ChassisPos_DrawStatus(1, _buf);
            }
            grip_test_ticks = 0U;
            grip_test_state = 5U;
            break;

        case 5U: /* 槽位间等待 800ms */
            if (++grip_test_ticks >= 80U)
            {
                grip_test_ticks = 0U;
                grip_test_slot++;
                if (grip_test_slot >= 5U)
                    grip_test_state = 6U;
                else
                    grip_test_state = 2U;
            }
            break;

        case 6U: /* 测试完成, 全部夹紧 */
            Servo4_LittleAllClose();
            ChassisPos_DrawStatus(1, "GRIP SEQ DONE");
            grip_test_state = 7U;
            break;

        default: /* 完成, 不再动作 */
            break;
        }
        return;
    }
#endif

#if HARDWARE_DEBUG_MODE
    if (++lcd_div >= 20)
    {
        ChassisPos_DrawStatus(1, "HW DEBUG");
        lcd_div = 0;
    }
    return;
#endif

#if !TASK2_ONLY_TEST_MODE
    if (task1_route_test_done)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "T1 ROUTE DONE");
            lcd_div = 0;
        }
        return;
    }
#endif

    if (route_state == ROUTE_MISSION_DONE)
    {
        LineFollow_Stop();
        if (++lcd_div >= 10)
        {
#if TASK2_COLLECT_ONLY
            ChassisPos_DrawStatus(1, "T2 COLL DONE");
#else
            ChassisPos_DrawStatus(1, "T2 MISSION OK");
#endif
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
            /* 任务2 奖杯放置: 按固定顺序 B→A→C 查找槽位 */
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
            /* 任务1 色块放置: 按路线顺序查询目标颜色 → 库存查找槽位 */
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
            /* 编码器测试: 跳过视觉对准+直进3cm+后退3cm, 直接到 vision_next_mode */
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
            VisionAlignment_Start();
            vision_started = 1U;
            ChassisPos_DrawStatus(1, "Vision Align");
        }

        vision_status = VisionAlignment_GetStatus();

        /* LCD 视觉诊断: 每 250ms 刷新一次 (5 次主循环 × 10ms) */
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
                /* 色块放置超时: 停车, 保持锁死, 不标记槽位, 跳过 */
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
            /* 继续到前进阶段(保持锁死) */
        }
        else
        {
            return;
        }

        /* DONE 或 奖杯 TIMEOUT: 固定航向直进, 保持转盘锁死 */
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
        ChassisPos_DrawStatus(1, "Fwd 1.2cm");
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
                /* 色块和奖杯都走 ROUTE_PLACE 解锁转盘 */
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
            pre_pwm = Servo4_GetPulse(2U);  /* 锁死状态的 CH2 PWM */
            Servo4_RotationUnlockTo(place_pwm_us);

            /* 按 PWM 跨度动态计算等待: 40 + delta/20 ticks (10ms/tick) */
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
                /* 第5块(末块, 仅色块): 抬臂后再后退; 其余直接后退 */
                if ((vision_job == VISION_JOB_COLOR_PLACE) &&
                    (place_seq_index == 4U))
                {
                    arm_to_task0_done = 0U;
                    place_step = 2U;
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
            /* 第5块: 缓慢抬臂到 Task0 收拢, 护住已放置物块 */
            const Task1ArmPose *t0 = Task1Mechanism_GetTask0Pose();
            if (Servo4_ArmRampTick(t0->big_arm_us, t0->small_arm_us))
            {
                arm_to_task0_done = 1U;
                place_step = 3U;
            }
        }
        else
        {
            /* place_step == 3: 抬臂完成, 启动后退 */
            int32_t back_dist = -VISION_BACK_DISTANCE;
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            ChassisPos_Start(back_dist, back_dist, back_dist, back_dist);
            __disable_irq();
            route_state = ROUTE_VISION_BACK;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Back 2.7cm");
        }
        return;
    }

    if (route_state == ROUTE_PLACE_ERROR)
    {
        /* 颜色查找失败: 停车, 保持锁死, 不递增索引, 不标记空槽 */
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
        if (ChassisPos_Update(VISION_FORWARD_SPEED, 0,
              HeadingHold_GetFreshCorrection()))
        {
            ChassisPos_ResetEncoder();
            ChassisPos_ResetVelocityPid();
            LineFollow_Stop();

            if (vision_job == VISION_JOB_COLOR_PLACE)
            {
                TurntableInventory_MarkEmpty(place_slot_idx);
                place_seq_index++;
                Servo4_RotationLockNow();
            }
            else if (vision_job == VISION_JOB_TROPHY)
            {
                TurntableInventory_MarkEmpty(place_slot_idx);
                task2_place_index++;
                Servo4_RotationLockNow();

                if (task2_place_index >= QR_TASK2_PLACE_COUNT)
                {
                    /* 三个奖杯全部放完 → 180°回Home */
                    ChassisPos_ResetEncoder();
                    HWT101_getAngle_Reset();
                    HeadingHold_Reset();
                    __disable_irq();
                    route_state = ROUTE_FINAL_ROTATE_180;
                    __enable_irq();
                    mode_entry_tick = 0;
                    lcd_div = 0;
                    ChassisPos_DrawStatus(1, "Rotate 180 CW");
                    return;
                }
            }

            if (vision_job == VISION_JOB_TROPHY)
            {
                /* 还有奖杯未放: 继续 PLACE_PREPARE */
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

    if (route_state == ROUTE_TASK2_PREPARE)
    {
        const Task1ArmPose *tgt;

        if (task2_prep_step == 0U)
        {
            LineFollow_Stop();
            TurntableInventory_Reset();
            /* 复用任务1已验证的转盘归零路径。 */
            Servo4_RotationHome();
            Servo4_LittleAllOpen();
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
#if TASK2_ONLY_TEST_MODE
            task2_prep_step = 1U;  /* 等QR扫描完成 */
#else
            task2_prep_step = 2U;  /* 跳过QR等待, 直接缓降 */
#endif
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "T2 Prepare");
        }
#if TASK2_ONLY_TEST_MODE
        else if (task2_prep_step == 1U)
        {
            CompetitionQrStatus qr;

            CompetitionQr_Process();
            qr = CompetitionQr_GetStatus();
            if ((qr == COMP_QR_READY) || (qr == COMP_QR_DEFAULTED))
            {
                task2_prep_step = 2U;
                ChassisPos_DrawStatus(1, "T2 QR OK");
            }
        }
#endif
        else
        {
            tgt = Task1Mechanism_GetWorkingPose();
            if (Servo4_ArmRampTick(tgt->big_arm_us, tgt->small_arm_us))
            {
                ChassisPos_ResetEncoder();
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
#if TASK2_ONLY_TEST_MODE
                /* 测试模式: 先走 TRACK_RETURN 收集奖杯 */
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
#else
                trophy_phase = 0;
                trophy_ticks = 0;
                trophy_cam_mode_on = 0U;
                __disable_irq();
                route_state = ROUTE_TROPHY;
                __enable_irq();
                task2_prep_step = 0U;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "T2 Capture");
#endif
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

    /* 最终179°旋转后的返程循迹模式（控制已移至ISR，此处仅显示） */
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

    if (task1_route_test_done)
    {
        LineFollow_Stop();
        ChassisMotor_ControlTick(wheel_target);
        return;
    }

    if (route_state == ROUTE_HOME_LAUNCH)
    {
        if (home_launch_tick == 0)
        {
            /* First tick: start TrapMove position control for forward burst. */
            ChassisPos_Start(120000, 120000, 120000, 120000);
            home_launch_tick = 1;
        }

        if (home_launch_tick == 1)
        {
            if (ChassisPos_Update(HOME_LAUNCH_SPEED, 0, 0))
            {
                /* Forward burst complete, begin rotation. */
                ChassisPos_ResetVelocityPid();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                home_turn_start_absolute = HWT101_GetAbsoluteYaw();
                home_launch_tick = 2;
            }
            /* else: still moving forward, UpdatePositionControl has set
             * wheel_target. Fall through to ChassisMotor_ControlTick below. */
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
        }
    }

    if (route_state == ROUTE_TRACK_RETURN)
    {
        if (marker_suppress_ticks < MARKER_SUPPRESS_TICKS)
        {
            marker_suppress_ticks++;
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
            /* 斜向前进: 前向+右移同时, 右移目标 12cm = 22200 counts
             * 转盘保持锁死, 直到第一个放置点 PLACE_PREPARE 才解锁 */
            {
                int curve_right = 22200;  /* 12 cm × 1850 */
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
            /* 接近目标时线性减速: 转过70°后每多1°降7.5, 90°时降到80 */
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

        /* 同时前向+右移(斜向). 十字提前出口已注释(调参期间). */
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

        /* LCD: 右移进度, 横向标定 18500 counts/cm (每 25ms 刷新) */
        if (++lcd_div >= 5)
        {
            char buf[24];
            int right_cnt = (int)(-debug_trap_left_position);
            int right_cm  = right_cnt / 18500;
            int right_mm  = (right_cnt % 18500) * 10 / 18500;
            if (right_cm > 9) right_cm = 9;      /* 防溢出单字符 */
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
            /* 十字标线提前出口已注释(调参期间). */
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
                    /* 先按速比折算对角位移, 再乘2减74000为实测修正公式,
                     * 等效横向位移约 7.3cm (横向标定 18500 counts/cm). */
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
        /* Phase 0: 最后一个色块放置后缓降至 Task0 收拢, 再旋转 */
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

        /* 阶段1: 开环快速转 ~80% 路程, 不碰绝对偏航 */
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
        /* 阶段2: 闭环精修, 恢复旧版验证逻辑 + 新帧检测 + 滞回 + 超时 */
        else
        {
            if (!left_heading_valid || !HWT101_IsReady())
            {
                LineFollow_Stop();
                return;
            }

            final_align_tick++;

            /* 超时保护: 2秒 (200 tick × 10ms) */
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

            /* 只在收到新陀螺仪数据帧时才计算误差 */
            uint32_t frame = HWT101_GetUpdateCount();
            if (frame == final_align_last_frame)
                return;
            final_align_last_frame = frame;

            float error_ccw = HeadingHold_WrapAngle180(
                left_heading_absolute - HWT101_GetAbsoluteYaw()) * yaw_ccw_sign;
            float error_abs = (error_ccw >= 0.0f) ? error_ccw : -error_ccw;

            if (error_abs <= 2.0f)
            {
                /* 死区内: 停车, 累积稳定帧 */
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
                /* 滞回上界外: 重置, 分段速度修正 */
                final_left_heading_stable_ticks = 0;
                int spd;
                if      (error_abs > 15.0f) spd = 280;
                else if (error_abs > 6.0f)  spd = 220;
                else                         spd = 140;

                if (error_ccw > 0.0f)
                {
                    /* error_ccw > 0: CCW 旋转 */
                    wheel_target[1] = -spd;
                    wheel_target[2] = -spd;
                    wheel_target[3] =  spd;
                    wheel_target[4] =  spd;
                }
                else
                {
                    /* error_ccw < 0: CW 旋转 */
                    wheel_target[1] =  spd;
                    wheel_target[2] =  spd;
                    wheel_target[3] = -spd;
                    wheel_target[4] = -spd;
                }
            }
            else
            {
                /* 2° < error ≤ 3.5°: 滞回区, 停车, 重置计数 */
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
#if TASK1_ROUTE_TEST_MODE || ENCODER_ROUTE_TEST_MODE
            /* 任务1路段测试 / 编码器测试: 到达任务2扫码点停车. */
            ChassisPos_ResetVelocityPid();
            Move_Control(0, 0, 0, 0);
            task1_route_test_done = 1U;
            ChassisPos_DrawStatus(1, "T1 ROUTE DONE");
            return;
#else
            if (final_left_stage == 0)
            {
                final_left_stage = 1;
                pause_next_mode = ROUTE_FINAL_LEFT_40;
                CompetitionQr_RequestStart(COMP_QR_TASK2);
            }
            else
            {
                /* 任务2: 不再走原ROUTE_FINAL_ROTATE_180→TRACK_RETURN,
                 * 直接进入 TASK2_PREPARE → 收集 → 放置 → 回Home */
                task2_prep_step = 0U;
                route_state = ROUTE_TASK2_PREPARE;
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "T2 Prepare");
                Move_Control(0, 0, 0, 0);
                return;
            }
            route_state = ROUTE_PAUSE_GENERIC;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Pause");
            Move_Control(0, 0, 0, 0);
            return;
#endif
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

    if (route_state == ROUTE_TROPHY)
    {
        int hc;

#if TASK2_ONLY_TEST_MODE
        /* ================================================================
         * 测试模式放置流程
         * Phase 0: 停车暂停
         * Phase 1: 向右旋转90°
         * Phase 2: 后退24cm
         * Phase 3: 放置 B (亚军) — PODIUM 姿态
         * Phase 4: CHAMPION 姿态 → 放置 A (冠军)
         * Phase 5: CHAMPION 姿态 → 放置 C (季军)
         * Phase 6: MISSION_DONE
         * ================================================================ */
        if (trophy_phase == 0)
        {
            LineFollow_Stop();
            if (++trophy_ticks >= TROPHY_PAUSE_TICKS)
            {
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                Servo4_LittleAllClose();
                Servo4_RotationLockNow();
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
        /* Phase 6: 三个奖杯放置完成 */
        else
        {
            LineFollow_Stop();
            trophy_cam_mode_on = 0U;
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            __disable_irq();
            route_state = ROUTE_MISSION_DONE;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "T2 MISSION OK");
        }
#else
        /* ---- 原有流程: 暂停 → 旋转90° → 后退 → 放置×3 → ROTATE_180 ---- */
        /* 相位 0/4/6: 暂停 */
        if (trophy_phase == 0 || trophy_phase == 4 || trophy_phase == 6)
        {
            LineFollow_Stop();
            if (++trophy_ticks >= TROPHY_PAUSE_TICKS)
            {
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                trophy_ticks = 0;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_back_active = 0U;
                trophy_phase++;
            }
        }
        /* 相位 1: 顺时针旋转 90° */
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
        /* 相位 2: 后退 24cm */
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
                trophy_ticks = 0;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_back_active = 0U;
                trophy_phase++;
            }
        }
        else if ((trophy_phase == 3U) || (trophy_phase == 5U) || (trophy_phase == 7U))
        {
            if (TrophyPlace_Process(task2_place_index))
            {
                trophy_phase++;
                trophy_place_step = PLACE_MOVE_LEFT;
                trophy_place_settle_ticks = 0U;
                trophy_place_back_active = 0U;
            }
        }
        /* 相位 8: 三个奖杯放置完成 → ROTATE_180 */
        else
        {
            LineFollow_Stop();
            trophy_cam_mode_on = 0U;
            (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
            ChassisPos_ResetEncoder();
            HWT101_getAngle_Reset();
            HeadingHold_Reset();
            __disable_irq();
            route_state = ROUTE_FINAL_ROTATE_180;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Rotate 180 CW");
        }
#endif
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
        else if (GS_state[1] && GS_state[2] && GS_state[3] && GS_state[4])
        {
            LineFollow_Stop();
            ChassisPos_ResetVelocityPid();
            Servo4_RotationLockNow();  /* 灰度识别到十字: 90°旋转前锁死转盘防物块掉落 */
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

#if TASK2_ONLY_TEST_MODE
        /* 快慢速策略: 第一杯前580, 命中后降至200 */
        collect_speed = (task2_capture_index == 0U)
                      ? (int)TROPHY_GRAB_FAST_SPEED
                      : (int)TROPHY_GRAB_SLOW_SPEED;

        /* 收集阶段: 摄像头0x02检测奖杯, 记录等级→转盘槽位 */
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

                /* 第三个奖杯: 立即停车, 等转盘稳定后安全收尾 */
                if (task2_capture_index >= 3U)
                {
                    task2_collect_complete = 1U;
                    task2_collect_done = 0U;
                    trophy_ticks = 0U;
                    /* 先闭合 CH3~CH7，Task0 在0.4s后再启动。 */
                    Servo4_LittleAllClose();
#if TASK2_COLLECT_ONLY
                    LineFollow_Stop();
#endif
                }

                /* 前两杯后抑制黑线误触发；第三杯后已收集完成，
                 * 立即允许前灰度黑线切入放置，避免4秒窗口内错过横线。 */
                marker_suppress_ticks = -(int)MARKER_SUPPRESS_TICKS;
            }
        }

#if !TASK2_COLLECT_ONLY
        /* 夹爪完成闭合后抬臂, 然后允许黑线切入放置。 */
        if (task2_collect_complete && !task2_collect_done)
        {
            if (++trophy_ticks >= TROPHY_GRIP_CLOSE_TICKS)
            {
                if (Servo4_ArmRampTick(1250U, 2000U))
                {
                    task2_collect_done = 1U;
                    marker_suppress_ticks = (int)MARKER_SUPPRESS_TICKS;
                }
            }
        }
#endif

        /* LCD 收集诊断: 每250ms刷新 */
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

#if TASK2_COLLECT_ONLY
        /* 仅收集模式: 停稳 → 等转盘80tick → 夹紧 → 锁死 → 关摄像头 → MISSION_DONE */
        if (task2_collect_complete && !task2_collect_done)
        {
            if (++trophy_ticks >= TROPHY_PLACE_SETTLE_TICKS)
            {
                Servo4_LittleAllClose();
                Servo4_RotationLockNow();
                task2_collect_done = 1U;
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                __disable_irq();
                route_state = ROUTE_MISSION_DONE;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "T2 COLL DONE");
                Move_Control(0, 0, 0, 0);
                goto skip_track_return;
            }
            goto skip_track_return;
        }
#endif
#else
        collect_speed = (int)LINE_FOLLOW_SPEED_TARGET;
#endif /* TASK2_ONLY_TEST_MODE */

        /* 前灰度右四路(5~8)全黑：先停车，再进入右转90°放置流程。 */
        if (GS_state[5] && GS_state[6] && GS_state[7] && GS_state[8]
            && marker_suppress_ticks >= MARKER_SUPPRESS_TICKS)
        {
#if TASK2_ONLY_TEST_MODE
            if (!task2_collect_complete)
            {
                /* 未收集完, 忽略此标记继续巡线 */
                marker_suppress_ticks = -(int)MARKER_SUPPRESS_TICKS;
            }
            else
#endif
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
        /* 左右列差速转向：W1/W2同侧，W3/W4同侧 */
        LineFollow_SetWheelTargets(collect_speed + turn_target,
                                   collect_speed + turn_target,
                                   collect_speed - turn_target,
                                   collect_speed - turn_target);
        skip_track_return: ;
    }

#if TASK2_ONLY_TEST_MODE && TASK2_MOTOR_DISABLE
    /* 原地联调仍保留 TIM7 内的摄像头、计数和转盘状态机，
     * 只在最终电机输出口将四轮目标强制清零。 */
    wheel_target[1] = 0;
    wheel_target[2] = 0;
    wheel_target[3] = 0;
    wheel_target[4] = 0;
#endif
    ChassisMotor_ControlTick(wheel_target);
}
