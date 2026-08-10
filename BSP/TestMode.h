#ifndef __TEST_MODE_H__
#define __TEST_MODE_H__

/* Exactly one dedicated test mode should normally be enabled at a time. */
#define CIRCLE_UART_TEST_MODE   0   /* LCD + UART4 + MaxiCam only */
#define SERVO_RANGE_TEST_MODE   0
#define COLOR_SERVO_TEST_MODE   0
#define SERVO_CHANNEL_TEST_MODE 0
#define TASK1_ROUTE_TEST_MODE   1   /* 仅跑任务1路段, 任务2扫码点停车 */
#define ENCODER_ROUTE_TEST_MODE 0
#define HARDWARE_DEBUG_MODE     0

/* Constants used by the older servo test branches. */
#define COLOR_SERVO_CHANNEL     2U
#define COLOR_SERVO_HOME_US   850U
#define COLOR_SERVO_STEP_US   400U
#define COLOR_CAMERA_BOOT_MS  4000U

#define SERVO_TEST_LOW_US       500U
#define SERVO_TEST_HIGH_US      1600U

#endif
