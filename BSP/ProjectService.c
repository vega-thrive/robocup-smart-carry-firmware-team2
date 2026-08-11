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
static uint8_t  place_slot_idx;
static uint16_t place_pwm_us;
static uint8_t  place_seq_index;   /* 顺序放置: 0→1→2→3→4 */
static float left_heading_absolute;
static uint8_t left_heading_valid;
static int8_t yaw_ccw_sign = 1;
static float home_turn_start_absolute;
static int last_steer;
static int cross_marker_ticks;
static uint8_t lcd_div;
static int debug_line_state;
static int debug_turn_target;

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

/* ---- 初始化 ---- */
void ProjectService_Init(void)
{
    LineFollow_Init();
    HWT101_getAngle_Reset();
    LineFollow_ResetLinePID();
    last_steer = 0;
    task1_route_test_done = 0U;
    place_seq_index = 0;
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
    ChassisPos_DrawStatus(1, "Home Launch");
    HAL_TIM_Base_Start_IT(&htim7);
}

/* ---- 主循环 ---- */
void ProjectService_Run(void)
{
    char buf[24];

    ChassisPos_FlushDisplay();
    Task1Color_Process();

#if HARDWARE_DEBUG_MODE
    if (++lcd_div >= 20)
    {
        ChassisPos_DrawStatus(1, "HW DEBUG");
        lcd_div = 0;
    }
    return;
#endif

    if (task1_route_test_done)
    {
        if (++lcd_div >= 5)
        {
            ChassisPos_DrawStatus(20, "T1 ROUTE DONE");
            lcd_div = 0;
        }
        return;
    }

    if (route_state == ROUTE_PLACE_PREPARE)
    {
        uint8_t target_color;

        LineFollow_Stop();

        if (place_seq_index >= TURNTABLE_SLOT_COUNT)
        {
            __disable_irq();
            route_state = vision_next_mode;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Route Resume");
            return;
        }

        /* 根据当前放置点目标颜色查找实际槽位 */
        target_color = CompetitionQr_GetTask1PlaceColor(
            CompetitionQr_GetTaskCode(COMP_QR_TASK1), place_seq_index);

        if ((target_color >= TURNTABLE_COLOR_CODE_MIN) &&
            (target_color <= TURNTABLE_COLOR_CODE_MAX) &&
            TurntableInventory_FindColor(target_color,
                                         &place_slot_idx,
                                         &place_pwm_us))
        {
            /* 按颜色查找到槽位 */
        }
        else
        {
            /* 暂无比赛颜色映射, 按抓取顺序放置 (不等于按颜色放置) */
            place_slot_idx = place_seq_index;
            place_pwm_us = TurntableInventory_GetPwmForSlot(place_slot_idx);
            if (place_pwm_us == 0U)
            {
                ChassisPos_DrawStatus(1, "Slot ERR");
                return;
            }
        }

        vision_job = VISION_JOB_COLOR_PLACE;
        vision_started = 0U;
        __disable_irq();
        route_state = ROUTE_VISION_ALIGN;
        __enable_irq();
        mode_entry_tick = 0;
        lcd_div = 0;
        ChassisPos_DrawStatus(1, "Vision Align");
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

            if (vision_job == VISION_JOB_COLOR_PLACE)
            {
                place_step = 0U;
                __disable_irq();
                route_state = ROUTE_PLACE;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Place Release");
            }
            else
            {
                int32_t back_dist = -VISION_BACK_DISTANCE;
                ChassisPos_Start(back_dist, back_dist, back_dist, back_dist);
                __disable_irq();
                route_state = ROUTE_VISION_BACK;
                __enable_irq();
                mode_entry_tick = 0;
                lcd_div = 0;
                ChassisPos_DrawStatus(1, "Back 2.7cm");
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
                /* 第5块(末块): 抬臂后再后退; 其余块直接后退 */
                if (place_seq_index == 4U)
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
                /* 色块放置: 后退完成 → 标记空槽 → 锁死转盘 */
                TurntableInventory_MarkEmpty(place_slot_idx);
                place_seq_index++;
                Servo4_RotationLockNow();
            }

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
            __disable_irq();
            route_state = vision_next_mode;
            __enable_irq();
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "Route Resume");
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
                pause_next_mode = ROUTE_FINAL_ROTATE_180;
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

    if (route_state == ROUTE_TROPHY)
    {
        int hc;

        if (trophy_phase == 0 || trophy_phase == 4
            || trophy_phase == 6 || trophy_phase == 8 || trophy_phase == 10)
        {
            LineFollow_Stop();
            if (trophy_phase != 10 && ++trophy_ticks >= TROPHY_PAUSE_TICKS)
            {
                ChassisPos_ResetVelocityPid();
                ChassisPos_ResetEncoder();
                HWT101_getAngle_Reset();
                HeadingHold_Reset();
                trophy_ticks = 0;
                trophy_phase++;
                if ((trophy_phase == 5U) ||
                    (trophy_phase == 7U) ||
                    (trophy_phase == 9U))
                {
                    vision_next_mode = ROUTE_TROPHY;
                    vision_started = 0U;
                    vision_job = VISION_JOB_TROPHY;
                    route_state = ROUTE_VISION_ALIGN;
                    ChassisPos_DrawStatus(1, "Vision Align");
                }
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
                Servo4_UseTrophyPose(TROPHY_POSE_PODIUM);
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
        else if ((trophy_phase == 3U) || (trophy_phase == 5U) || (trophy_phase == 7U))
        {
            /* 左移寻杯: 模式2(0x02)视觉找奖杯, 命中即停车夹取;
             * 走满距离仍未见则兜底按原固定距离进入下一阶段. */
            int hc;

            if (!trophy_cam_mode_on)
            {
                /* 铁律: 摄像头可能仍在模式循环内, 先 0x04 退出再切 0x02 */
                (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
                (void)MaxiCam_SetMode(MAXICAM_MODE_ABC);
                trophy_cam_mode_on = 1U;
                trophy_det_confirm = 0U;
            }

            if (!pos_control_active)
            {
                int dist = (trophy_phase == 3U) ? (int)TROPHY_LEFT1_DISTANCE
                         : (trophy_phase == 5U) ? (int)TROPHY_LEFT2_DISTANCE
                         :                        (int)TROPHY_LEFT3_DISTANCE;
                ChassisPos_Start(-dist, dist, -dist, dist);
            }

            if (TrophyDetect_Seen() && (trophy_det_count < TROPHY_DETECT_COUNT))
                trophy_det_count++;

            if (trophy_det_count >= TROPHY_DETECT_COUNT)
            {
                /* 识别满3次: 停车, 小爪子全部夹住 */
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                Servo4_LittleAllClose();
                trophy_cam_mode_on = 0U;
                trophy_ticks = 0U;
                trophy_phase++;
            }
            else
            {
                hc = HeadingHold_GetFreshCorrection();
                if (ChassisPos_Update(0, TROPHY_LEFT_SPEED, hc))
                {
                    LineFollow_Stop();
                    ChassisPos_ResetVelocityPid();
                    trophy_cam_mode_on = 0U;
                    trophy_ticks = 0U;
                    trophy_phase++;
                }
            }
        }
        else
        {
            /* 阶段 2(后退) 与 9(右后撤): 盲走 */
            int max_fwd, max_left;
            int pos_t1, pos_t2, pos_t3, pos_t4;

            if (!pos_control_active)
            {
                if (trophy_phase == 2U)
                {
                    pos_t1 = pos_t2 = pos_t3 = pos_t4 = -(int)TROPHY_BACK_DISTANCE;
                }
                else
                {
                    int back = (int)TROPHY_FINAL_BACK_DISTANCE;
                    int right = (int)TROPHY_FINAL_RIGHT_DISTANCE;
                    pos_t1 =  right - back;
                    pos_t2 = -right - back;
                    pos_t3 =  right - back;
                    pos_t4 = -right - back;
                }
                ChassisPos_Start(pos_t1, pos_t2, pos_t3, pos_t4);
            }

            if (trophy_phase == 2U)
                { max_fwd = TROPHY_BACK_SPEED; max_left = 0; }
            else
                { max_fwd = TROPHY_RIGHT_BACK_SPEED; max_left = TROPHY_RIGHT_SPEED; }
            hc = HeadingHold_GetFreshCorrection();
            if (ChassisPos_Update(max_fwd, max_left, hc))
            {
                LineFollow_Stop();
                ChassisPos_ResetVelocityPid();
                trophy_ticks   = 0;
                trophy_phase++;
            }
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
        Sensor_Get();

        /* Rear 4 sensors (5-8) all on black + suppression expired → TROPHY */
        if (GS_state[5] && GS_state[6] && GS_state[7] && GS_state[8]
            && marker_suppress_ticks >= MARKER_SUPPRESS_TICKS)
        {
            ChassisPos_ResetVelocityPid();
            HeadingHold_Reset();
            LineFollow_Stop();
            ChassisPos_ResetEncoder();
            HWT101_getAngle_Reset();
            trophy_phase = 0;
            trophy_ticks = 0;
            trophy_cam_mode_on = 0U;
            trophy_det_confirm = 0U;
            trophy_det_last_seq = 0U;
            trophy_det_count = 0U;
            trophy_det_gap_until_ms = 0U;
            route_state = ROUTE_TROPHY;
            mode_entry_tick = 0;
            lcd_div = 0;
            ChassisPos_DrawStatus(1, "TROPHY - STOP");
            Move_Control(0, 0, 0, 0);
            goto skip_track_return;
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
        LineFollow_SetWheelTargets(LINE_FOLLOW_SPEED_TARGET + turn_target,
                                   LINE_FOLLOW_SPEED_TARGET + turn_target,
                                   LINE_FOLLOW_SPEED_TARGET - turn_target,
                                   LINE_FOLLOW_SPEED_TARGET - turn_target);
        skip_track_return: ;
    }

    ChassisMotor_ControlTick(wheel_target);
}
