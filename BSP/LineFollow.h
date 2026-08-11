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
#define TRACK_FIRST_SPEED_TARGET 230
#define LINE_FOLLOW_TURN_LIMIT   450




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
#define HOME_FIXED_LEFTBACK_LEFT_DIST  77700 /* about 4.2 cm left */
#define HOME_FIXED_SETTLE_TICKS       30    /* 150 ms */

#define MARKER_SUPPRESS_TICKS   400  /* TRACK_RETURN only: 2 s */

/* Trophy route: pause, rotate CW, reverse, then three stepped left moves. */
#define TROPHY_PAUSE_TICKS          100  /* 0.5 s at 200 Hz */
#define TROPHY_ROTATE_SPEED         372
#define TROPHY_ROTATE_ANGLE        90.0f
#define TROPHY_BACK_SPEED           300
#define TROPHY_BACK_DISTANCE      44400  /* 24 cm x 1850 counts/cm */
#define TROPHY_LEFT_SPEED           300
#define TROPHY_LEFT1_DISTANCE     55500  /* calibrated 3.0 cm lateral travel */
#define TROPHY_LEFT2_DISTANCE     64750  /* calibrated 3.5 cm lateral travel */
#define TROPHY_LEFT3_DISTANCE     64750  /* calibrated 3.5 cm lateral travel */
#define TROPHY_RIGHT_BACK_SPEED     420
#define TROPHY_RIGHT_SPEED           46
#define TROPHY_FINAL_BACK_DISTANCE 410700 /* 222 cm x 1850 counts/cm */
#define TROPHY_FINAL_RIGHT_DISTANCE 46250  /* calibrated 2.5 cm lateral travel */

/* Trophy visual search (camera mode 2, 0x02): jiang_bei = code 26.
 * TROPHY_DETECT_CONFIRM = 3-frame confirm matches main.py CONFIRM;
 * every confirmed hit counts one trophy.  The little grippers close only
 * after TROPHY_DETECT_COUNT hits (3 trophies), like the bench test.
 * TROPHY_DETECT_CLEAR_MS = quiet gap after a hit so the same trophy still
 * inside the camera ROI cannot be counted repeatedly.
 * Detection arms only after the left move passes TROPHY_DETECT_ARM_COUNTS
 * so a just-grabbed trophy cannot re-trigger on the next phase. */
#define TROPHY_DETECT_CODE        26U
#define TROPHY_DETECT_CONFIRM      3U
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
#define CURVE_FORWARD_SPEED        180
#define CURVE_RIGHT_SPEED           47   /* 遗留: ChassisPos_SetCurveTarget 使用 */
#define CURVE_RIGHT_MOVE_SPEED     130  /* 斜向前进横向速度 */
#define CURVE_ENCODER_COUNTS      77700  /* 前向 42 cm × 1850 */

/* 视觉对准完成后: 固定航向直进 1.2cm, 再后退 1.7cm */
#define VISION_FORWARD_SPEED     250
#define VISION_FORWARD_DISTANCE 4070   /* 2.2 cm × 1850 counts/cm */
#define VISION_BACK_DISTANCE    4995   /* 2.7 cm × 1850 counts/cm */

/* 放置物块: 视觉对准+直进3cm后, 转盘解锁→旋转到目标槽位→释放→锁定→后退3cm */
#define PLACE_DEFAULT_COLOR      21U   /* 默认放置颜色码, 后续按需修改 */
#define PLACE_TURRET_WAIT_TICKS  40    /* 转盘旋转等待 200ms */
#define PLACE_ARM_WAIT_TICKS     20    /* 释放动作等待 100ms */

/* 后对准一: 视觉对准+直进3cm完成后左移2.5cm(46250 counts, 由直进完成分支预置),
 * 到位后前进40cm(74000 counts). 纯编码器定位, 不再用后八路灰度. */
#define REAR_ALIGN_LEFT_SPEED   240

/* 后对准二: 视觉对准+直进3cm完成后后退30cm(54964 counts, 预置), 再左移8.5cm
 * (157250 counts); 左移期间前八路灰度检测到十字交叉线(GS3/GS4)可提前退出. */
#define REAR_ALIGN_2_LEFT_SPEED  320
#define REAR_ALIGN_2_BACK_SPEED  120
#define REAR_ALIGN_2_DISTANCE   157250  /* 8.5 cm lateral */

/* Final forward: 80 cm with encoder + gyro heading hold. */
#define FINAL_FORWARD_SPEED     380
#define PAUSE_TICKS              50    /* 250 ms pause */

/* Final left-forward: 30 cm at ~62° after second pause. */
#define PAUSE_2_TICKS           100    /* 0.5 s */
#define FINAL_LF_FORWARD_SPEED  180
#define FINAL_LF_LEFT_SPEED     340
#define FINAL_LF_DISTANCE     55500   /* 30 cm × 1850 */


/* Final rotate 30° CW, then forward 30 cm. */
#define FINAL_ROTATE_30_SPEED   280
#define FINAL_ROTATE_30_ANGLE    35.0f
#define FINAL_FORWARD_2_SPEED    320

/* Align to the saved left heading, move left 14.5 cm, then rotate 179° CW. */
#define PAUSE_GENERIC_TICKS      100   /* 0.5 s */
#define FINAL_ROTATE_ALIGN_SPEED 280
#define FINAL_ROTATE_ALIGN_ANGLE  48.0f
#define FINAL_LEFT_40_SPEED     320
#define FINAL_ROTATE_180_SPEED  372
#define FINAL_ROTATE_180_ANGLE  179.0f

extern volatile int wheel_target[5];
extern const int8_t line_weights[9];

void LineFollow_Init(void);
void LineFollow_ResetLinePID(void);
int  LineFollow_ComputeLinePID(int error);
void LineFollow_SetWheelTargets(int m1, int m2, int m3, int m4);
void LineFollow_Stop(void);

#endif
