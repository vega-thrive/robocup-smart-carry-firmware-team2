#ifndef __LINE_FOLLOW_H__
#define __LINE_FOLLOW_H__

#include "main.h"
#include "HeadingHold.h"
#include "ChassisPos.h"
#include "Task1Color.h"

/*
 * ============================================================================
 * 警告 / WARNING
 * ============================================================================
 * 本文件中所有距离常量（_DISTANCE、_DIST、_TICKS）均基于实测标定值
 * 1850 counts/cm，非理论计算值（理论约 2315 counts/cm @ 77mm 轮径 +
 * 500PPR × 4 倍频 × 1:28 减速比）。
 *
 * 标定值与理论值的差异来源于：麦轮打滑、轮子形变、地面摩擦、编码器
 * 安装误差等实际工况。改动这些常量前，必须实车重标定，切勿按理论值
 * 直接换算替换，否则所有定点运动距离将偏离。
 * ============================================================================
 */

/* Encoder counts per 5 ms control period. */
#define LINE_FOLLOW_SPEED_TARGET 580
#define TRACK_FIRST_SPEED_TARGET 276  /* 286 略降 */
#define LINE_FOLLOW_TURN_LIMIT   450

/* 收集完成(任务1第5个色块/任务2第3个奖杯)后:
 * 继续巡线固定编码器距离, 然后立刻旋转进入放置阶段.
 * 8.21 改为编码器定距离 (8.20 曾用定时, 速度变化会导致旋转点漂移).
 * 8.21 先用理论值实车校准: 任务1 = 260×700, 任务2 = 280×1200 (速度×原定时).
 * TIMEOUT 为原定时的 2 倍, 防黑线丢失/打滑时距离达不到而卡死. */
#define TASK1_COLLECT_DONE_DISTANCE   163800U  /* 理论 182000 减10% */
#define TASK2_COLLECT_DONE_DISTANCE   250280U  /* 287280 减20cm (37000 counts) */
#define TASK1_COLLECT_DONE_TIMEOUT_TICKS 1400U /* 7s @200Hz */
#define TASK2_COLLECT_DONE_TIMEOUT_TICKS 1400U /* 7s @200Hz */




/* Home route: timed forward move, 90-degree CCW turn, encoder-based
 * forward/lateral moves, then enter TRACK_FIRST with a speed ramp. */
#define HOME_LAUNCH_SPEED      300
#define HOME_LAUNCH_FWD_TICKS  400   /* 2000 ms forward burst */
#define HOME_TURN_SPEED         384
#define HOME_TURN_ANGLE        90.0f /* CCW to face left      */
#define HOME_FIXED_FORWARD_SPEED     400
#define HOME_FIXED_PAUSE_TICKS       100    /* 500 ms */
#define HOME_FIXED_LEFTBACK_FWD_SPEED  240
#define HOME_FIXED_LEFTBACK_LEFT_SPEED 290
#define HOME_FIXED_LEFTBACK_BACK_DIST  70300 /* about 3.8 cm backward */
#define HOME_FIXED_LEFTBACK_LEFT_DIST   9250 /* 5.0 cm × 1850 (8.21 修正标定) */
#define HOME_FIXED_SETTLE_TICKS       30    /* 150 ms */

/* Fast return after Task2: align to the absolute heading saved in Home,
 * then drive straight by encoder with gyro heading hold. */
#define HOME_RETURN_SPEED             400
#define HOME_RETURN_DISTANCE       370000  /* reverse 200 cm (保留作参考) */
#define HOME_RETURN_RIGHT_DISTANCE  46250  /* 25 cm right component (保留作参考) */
#define HOME_RETURN_RIGHT_SPEED       180  /* lateral component speed (保留作参考) */
#define HOME_RETURN_MARKER_ARM      37000  /* ignore podium line for first 20 cm */
#define HOME_RETURN_ALIGN_TIMEOUT     400  /* 2 s at 200 Hz */
#define HOME_RETURN_ALIGN_STABLE        3  /* fresh gyro frames within 2 deg */

/* 回家灰度检测停车 (8.19, 替代编码器定位):
 * 固定速度倒车, 前灰度 8 路全黑事件计数 (上升沿), 每条线计数后 200ms 屏蔽
 * (粗线防重复计数), 第 4 次全黑 → 立即停车 → 后退 10cm. */
#define HOME_RETURN_GS_STOP_COUNT      4U  /* 全黑事件次数 */
#define HOME_RETURN_GS_BLANK_MS      200U  /* 每条黑线计数后屏蔽时长 */
#define HOME_RETURN_DRIVE_SPEED      400   /* 倒车固定速度 */
#define HOME_RETURN_BACK_DISTANCE   27750  /* 停车后后退 15 cm */
#define HOME_RETURN_BACK_SPEED       100
#define HOME_RETURN_GS_MAX_BACK    555000  /* 灰度漏检兜底: 后退300cm强制停车 */

/* ---- 8.19 新回家流程 (放置完第3杯后) ----
 * 左移10cm → 左转90°(车头向左) → 右移5cm → 巡线(左四路GS1~4全黑停)
 * → 前进10cm → 右转90°(车头向前) → 后退10cm → 机械臂Task0 → 完赛 */
#define HOME_LEFT_10_DISTANCE     37000  /* 左移 20cm (8.19 实测改) */
#define HOME_LEFT_10_SPEED          300
#define HOME_RIGHT_5_DISTANCE     55500  /* 右移 30cm (20+10) */
#define HOME_RIGHT_5_SPEED          300
#define HOME_FWD_10_DISTANCE      55500  /* 触发后前进 30cm */
#define HOME_FWD_10_SPEED           300
#define HOME_RIGHT_25_DISTANCE    46250  /* 右移 25cm (触发后) */
#define HOME_RIGHT_25_SPEED         300
#define HOME_TRACK_STOP_TICKS       100  /* 停靠线触发后停车 0.5s @200Hz */
/* 回家巡线速度 = 第一次巡线速度(linefollow_task1_speed_active)的 150% */

#define MARKER_SUPPRESS_TICKS   400  /* TRACK_RETURN only: 2 s */

/* Task2 entry: left move before 179 deg rotation to face return track */
#define TASK2_ENTRY_LEFT_DISTANCE   37000  /* ~2.0 cm lateral, tune on-car */
#define TASK2_ENTRY_LEFT_SPEED        320
#define TASK2_ENTRY_SETTLE_TICKS       40  /* 200 ms at 200 Hz */
#define TASK2_TRANSITION_QR_TIMEOUT_TICKS  800U  /* ~4 s at 200 Hz */

/* Trophy route: pause, rotate CW, reverse, then three stepped left moves. */
#define TROPHY_PAUSE_TICKS          100  /* 0.5 s at 200 Hz */
#define TROPHY_PRE_ROTATE_BACK_DISTANCE 3700 /* 2 cm x 1850 counts/cm */
#define TROPHY_PRE_ROTATE_BACK_SPEED  180
#define TROPHY_ROTATE_SPEED         372
#define TROPHY_ROTATE_ANGLE        90.0f
#define TROPHY_BACK_SPEED           300
#define TROPHY_BACK_DISTANCE      70300  /* 38 cm x 1850 counts/cm (34+4) */
#define TROPHY_LEFT_SPEED           300
#define TROPHY_LEFT1_DISTANCE     11100  /* 6cm × 1850 (8.22) */
#define TROPHY_LEFT2_DISTANCE     64750  /* calibrated 3.5 cm lateral travel */
#define TROPHY_LEFT3_DISTANCE     64750  /* calibrated 3.5 cm lateral travel */
#define TROPHY_GRAB_FAST_SPEED       280  /* 298 额外降低 */
#define TROPHY_GRAB_SLOW_SPEED       250  /* 当前收集阶段保持统一速度 */
#define TROPHY_PLACE_SETTLE_TICKS         80  /* 转盘旋转稳定等待 */
#define TROPHY_ROTATE_SAFE_SETTLE_TICKS   80  /* 转盘回安全位稳定等待 */
#define TROPHY_TARGET_POSE_SETTLE_TICKS   80  /* 目标姿态到位后稳定 800ms */
#define GRIPPER_OPEN_WAIT_TICKS           30  /* 全开后等待 300ms */
#define TROPHY_RELEASE_SETTLE_TICKS      100  /* 释放姿态稳定 1000ms */
#define TROPHY_HALF_OPEN_WAIT_TICKS       10  /* 半开等待 100ms */
#define TROPHY_DETACH_WAIT_TICKS          50  /* 脱钩开度等待 500ms */
#define TROPHY_DROP_SETTLE_TICKS         100  /* 奖杯落稳 1000ms */
#define TROPHY_LIFT_CLEAR_SETTLE_TICKS    50  /* 抬离后稳定 500ms */
#define TROPHY_GRIP_CLOSE_TICKS       80  /* 0.4s at 200Hz: close claws before Task0 */
/* Trophy visual search (camera mode 2, 0x02): jiang_bei = code 26.
 * Camera-side green-box confirmation already filters detections, so the MCU
 * accepts the first fresh code 26 frame.  The little grippers close only
 * after TROPHY_DETECT_COUNT hits (3 trophies), like the bench test.
 * TROPHY_DETECT_CLEAR_MS = quiet gap after a hit so the same trophy still
 * inside the camera ROI cannot be counted repeatedly.
 * Detection arms only after the left move passes TROPHY_DETECT_ARM_COUNTS
 * so a just-grabbed trophy cannot re-trigger on the next phase. */
#define TROPHY_DETECT_CODE        26U
#define TROPHY_DETECT_CONFIRM      1U  /* 摄像头绿框已确认，STM32收到首帧26即响应 */
#define TROPHY_DETECT_COUNT        3U
#define TROPHY_DETECT_CLEAR_MS   400U
#define TROPHY_DETECT_ARM_COUNTS 22000U

/* Rotate 90° CW in place before the diagonal move.  Done in the simple
 * line-follow area to avoid false cross triggers later.
 * 速度取第一次巡线水平(TRACK_FIRST=230)放慢. */
#define ROTATE_90_SPEED        180
#define ROTATE_90_TARGET_ANGLE  90.0f

/* 斜向前进: 前向与右移同时执行(真斜向), 右移 12cm = 22200 counts 硬编码.
 * 横向速度 150, 低于此值易打滑; 调距离改 ProjectService.c curve_right 常量. */
#define CURVE_FORWARD_SPEED        500
#define CURVE_RIGHT_SPEED           47   /* 遗留: ChassisPos_SetCurveTarget 使用 */
#define CURVE_RIGHT_MOVE_SPEED     230  /* faster while preserving diagonal ratio */
#define CURVE_ENCODER_COUNTS      70300  /* 前向 38 cm × 1850 */

/* FINAL_ROTATE_ALIGN 回正完成后、FINAL_LEFT_40 左移11cm 前: 前进 4cm (8.21) */
#define TASK1_ALIGN_FWD_DISTANCE  7400   /* 4cm × 1850 */
#define TASK1_ALIGN_FWD_SPEED       200

/* 任务1视觉对准后：左移1.2cm，再固定航向直进2.5cm放置。
 * 5个放置点(A~E)机械补偿独立标定: {左移counts, 直进counts},
 * 视觉把圆心对准后机械末端到落点的固定偏移各点不同. */
#define TASK1_PLACE_LEFT_OFFSET_SPEED    100
/* 8.19实测: A 4255; B 白色进槽0测偏右1.5cm → -185+2775=2590; C/E 3330; D 4255 */
#define TASK1_PLACE_COMP_A  { 4255, 4625 }
#define TASK1_PLACE_COMP_B  { 2590, 4625 }
#define TASK1_PLACE_COMP_C  { 3330, 4625 }
#define TASK1_PLACE_COMP_D  { 4255, 4625 }
#define TASK1_PLACE_COMP_E  { 3330, 4625 }

/* 5个转盘槽位(500/900/1300/1700/2100us)的转盘口横向偏差补偿 (counts, 8.19半修正).
 * 算法: 带补偿测残余(中位数) x 0.5 半修正防噪声过拟合.
 * 8.19残余中位数: 槽0 0.3 / 槽1 0.6 / 槽2 2.2cm → -1265/-247/-1110.
 * 槽3/4 已验证基本十环, 保持 -185/-1542.
 * 放置时叠加到 TASK1_PLACE_COMP 的左移补偿上: left = comp.left + slot_comp[idx]. */
#define TURRET_SLOT_COMP_0  (-1265)
#define TURRET_SLOT_COMP_1  (-247)
#define TURRET_SLOT_COMP_2  (-1110)
#define TURRET_SLOT_COMP_3  (-185)
#define TURRET_SLOT_COMP_4  (-1542)
#define VISION_FORWARD_SPEED     250
#define VISION_FORWARD_DISTANCE 4625   /* 2.5 cm × 1850 counts/cm (1.4cm+1.1cm) */
#define VISION_BACK_DISTANCE    4995   /* legacy/shared path: 2.7 cm */
#define TASK1_PLACE_BACK_DISTANCE 8695 /* 4.7 cm total Task1 clearance */
#define TASK1_BLOCK_DROP_SETTLE_TICKS 40U  /* 400 ms at 100 Hz main loop */
#define TASK1_BACK_CLEAR_DISTANCE     1850  /* first 1 cm after release */
#define TASK1_BACK_CLEAR_SPEED         100
#define TASK1_PLACE_BACK_FAST_SPEED    300  /* remaining withdrawal after first 1 cm */

/* 放置物块: 视觉对准+直进3cm后, 转盘解锁→旋转到目标槽位→释放→锁定→后退3cm */
#define PLACE_DEFAULT_COLOR      21U   /* 默认放置颜色码, 后续按需修改 */
#define PLACE_TURRET_WAIT_TICKS  40    /* 转盘旋转等待 200ms */
#define PLACE_ARM_WAIT_TICKS     20    /* 释放动作等待 100ms */

/* 视觉对准完成后: 固定前进插入距离, 按领奖台分别标定.
 * 亚军/冠军/季军手臂高度不同, 相同前进距离落点不同, 必须独立设置. */
#define TROPHY_INSERT_RUNNER_COUNTS    6013  /* 亚军 3.25cm × 1850 (原6.5cm减半) */
#define TROPHY_INSERT_CHAMPION_COUNTS  6013  /* 冠军 3.25cm × 1850 (原6.5cm减半) */
#define TROPHY_INSERT_THIRD_COUNTS     6013  /* 季军 3.25cm × 1850 (原6.5cm减半) */
#define TROPHY_FORWARD_INSERT_SPEED     100
#define TROPHY_INSERT_SETTLE_TICKS       30  /* 前插到位后稳定 300ms */

/* 放置完成后后退距离, 按领奖台分别标定.
 * 不同姿态下机构伸出水平长度不同, 亚军最低可能需更远. */
#define TROPHY_BACK_RUNNER_COUNTS     12950  /* 亚军 7.0cm × 1850 (6+1) */
#define TROPHY_BACK_CHAMPION_COUNTS   11100  /* 冠军 6.0cm × 1850 (5+1) */
#define TROPHY_BACK_THIRD_COUNTS      20350  /* 季军 11.0cm × 1850 (10+1) */
#define TROPHY_BACK_CLEAR_SPEED          80  /* 后退脱离速度 */
#define TROPHY_BACK_SETTLE_TICKS         50  /* 后退到位后稳定 500ms */

/* 后对准一: 视觉对准+直进3cm完成后左移2.5cm(46250 counts, 由直进完成分支预置),
 * 到位后前进40cm(74000 counts). 纯编码器定位, 不再用后八路灰度. */
#define REAR_ALIGN_LEFT_SPEED   340

/* 后对准二: 视觉对准+直进3cm完成后后退30cm(54964 counts, 预置), 再左移8.5cm
 * (157250 counts); 左移期间前八路灰度检测到十字交叉线(GS3/GS4)可提前退出. */
#define REAR_ALIGN_2_LEFT_SPEED  480
#define REAR_ALIGN_2_BACK_SPEED  380  /* 后退提速幅度大: 原220太慢 */
#define REAR_ALIGN_2_DISTANCE   157250  /* 8.5 cm lateral */

/* Final forward: 80 cm with encoder + gyro heading hold. */
#define FINAL_FORWARD_SPEED     510
#define PAUSE_TICKS              50    /* 250 ms pause */

/* Final left-forward: 30 cm at ~62° after second pause. */
#define PAUSE_2_TICKS           100    /* 0.5 s */
#define FINAL_LF_BACK_SPEED     270
#define FINAL_LF_FORWARD_SPEED  270
#define FINAL_LF_LEFT_SPEED     500
#define FINAL_LF_DISTANCE     55500   /* 30 cm × 1850 */


/* Final rotate 30° CW, then forward 30 cm. */
#define FINAL_ROTATE_30_SPEED   280
#define FINAL_ROTATE_30_ANGLE    35.0f
#define FINAL_FORWARD_2_SPEED    480

/* Align to the saved left heading, move left 14.5 cm, then rotate 179° CW. */
#define PAUSE_GENERIC_TICKS      100   /* 0.5 s */
#define FINAL_ROTATE_ALIGN_SPEED 280
#define FINAL_ROTATE_ALIGN_ANGLE  48.0f
#define FINAL_LEFT_40_SPEED     480
#define FINAL_LEFT_FWD_HOLD_SPEED 200 /* minimum effective speed for fore/aft drift correction */
#define FINAL_ROTATE_180_SPEED  372
#define FINAL_ROTATE_180_ANGLE  179.0f

extern volatile int wheel_target[5];
extern const int8_t line_weights[9];

/* 运行时参数 (VofaTune 在线调, 初值=宏默认) */
extern volatile float linefollow_kp;
extern volatile float linefollow_ki;
extern volatile float linefollow_kd;
extern volatile int   linefollow_turn_limit;
extern volatile int   linefollow_task1_speed;
extern volatile int   linefollow_task2_speed;
extern volatile int   linefollow_home_return_speed;
/* 动作中实际使用的速度: 状态进入时由 LineFollow_LatchSpeeds 锁存 */
extern volatile int linefollow_task1_speed_active;
extern volatile int linefollow_task2_speed_active;
extern volatile int linefollow_home_return_speed_active;
void LineFollow_LatchSpeeds(void);

/* 最近一次 PID 各项与转向输出 (监控用) */
extern volatile float linefollow_debug_p;
extern volatile float linefollow_debug_i;
extern volatile float linefollow_debug_d;
extern volatile int   linefollow_debug_steer;

void LineFollow_Init(void);
void LineFollow_ResetLinePID(void);
void LineFollow_SetGains(float kp, float ki, float kd);
void LineFollow_SetTurnLimit(int limit);
int  LineFollow_ComputeLinePID(int error);
void LineFollow_SetWheelTargets(int m1, int m2, int m3, int m4);
void LineFollow_Stop(void);

#endif
