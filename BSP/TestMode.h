#ifndef __TEST_MODE_H__
#define __TEST_MODE_H__

/* Exactly one dedicated test mode should normally be enabled at a time. */
#define ARM_SERVO_TEST_MODE     0   /* 大臂小臂专项测试: 上电仅CH0/CH1缓降至 FLAT */
#define CIRCLE_UART_TEST_MODE   0   /* LCD + UART4 + MaxiCam only */
#define SERVO_RANGE_TEST_MODE   0
#define COLOR_SERVO_TEST_MODE   0
#define SERVO_CHANNEL_TEST_MODE 0
#define TASK1_ROUTE_TEST_MODE   0   /* 仅跑任务1路段, 任务2扫码点停车 */
#define TASK2_ONLY_TEST_MODE    1   /* 任务2专项测试: 绕过任务1, 直接收集+放置 */
#define TASK2_COLLECT_ONLY      0   /* 任务2仅收集: 只收集不放置; 0=收集+放置 */
#define TASK2_MOTOR_DISABLE     0   /* 任务2原地测试: 1=禁用四轮, 0=正常行驶 */
#define ENCODER_ROUTE_TEST_MODE 0
#define HARDWARE_DEBUG_MODE     0
#define LITTLE_GRIPPER_SEQ_TEST 0   /* 小爪子顺序测试: 依次张开槽位0→1→2→3→4, 仅一次, 验证硬件接线 */

/* Constants used by the older servo test branches. */
#define COLOR_SERVO_CHANNEL     2U
#define COLOR_SERVO_HOME_US   850U
#define COLOR_SERVO_STEP_US   400U
#define COLOR_CAMERA_BOOT_MS  4000U

#define SERVO_TEST_LOW_US       500U
#define SERVO_TEST_HIGH_US      1600U

#endif
