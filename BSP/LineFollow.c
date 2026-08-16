#include "LineFollow.h"
#include "HWT101.h"
#include "HeadingHold.h"
#include "ChassisPos.h"

#define LINE_FOLLOW_KP          9.0f   /* 默认值, 运行时可经 VofaTune 调 */
#define LINE_FOLLOW_KI          0.15f
#define LINE_FOLLOW_KD          0.8f
#define LINE_FOLLOW_I_LIMIT     200

/* 八路灰度权重，索引1~8，对应传感器物理位置从左到右。
 * 循迹算法：加权位置和 → 位置式PID → 左右列差速转向。
 * line_state > 0 = 车偏左/线在右，turn > 0，左列加速/右列减速 = 车头右偏找线。 */
const int8_t line_weights[9] = {0, -12, -9, -6, -3, 3, 6, 9, 12};

volatile int wheel_target[5] = {0};

/* 运行时参数 (初值=宏默认, 与正式基线逐位一致; VofaTune 在线可调) */
volatile float linefollow_kp = LINE_FOLLOW_KP;
volatile float linefollow_ki = LINE_FOLLOW_KI;
volatile float linefollow_kd = LINE_FOLLOW_KD;
volatile int   linefollow_turn_limit = LINE_FOLLOW_TURN_LIMIT;
volatile int   linefollow_task1_speed = TRACK_FIRST_SPEED_TARGET;
volatile int   linefollow_task2_speed = TROPHY_GRAB_FAST_SPEED;
volatile int   linefollow_home_return_speed = HOME_RETURN_SPEED;

/* 各状态动作中实际使用的速度: 状态进入时从目标锁存,
 * 在线修改只影响下一次进入该状态, 不改变当前动作. */
volatile int linefollow_task1_speed_active = TRACK_FIRST_SPEED_TARGET;
volatile int linefollow_task2_speed_active = TROPHY_GRAB_FAST_SPEED;
volatile int linefollow_home_return_speed_active = HOME_RETURN_SPEED;

void LineFollow_LatchSpeeds(void)
{
    linefollow_task1_speed_active       = linefollow_task1_speed;
    linefollow_task2_speed_active       = linefollow_task2_speed;
    linefollow_home_return_speed_active = linefollow_home_return_speed;
}

/* 最近一次 PID 各项 (VofaTune watch line 用) */
volatile float linefollow_debug_p;
volatile float linefollow_debug_i;
volatile float linefollow_debug_d;
volatile int   linefollow_debug_steer;

static float linefollow_pid_integral;
static int   linefollow_pid_last_error;
static uint8_t linefollow_pid_valid;

void LineFollow_ResetLinePID(void)
{
    linefollow_pid_integral = 0.0f;
    linefollow_pid_last_error = 0;
    linefollow_pid_valid = 0;
}

/* 在线调增益: 设置后立即清积分/历史误差, 防止输出突变 */
void LineFollow_SetGains(float kp, float ki, float kd)
{
    linefollow_kp = kp;
    linefollow_ki = ki;
    linefollow_kd = kd;
    LineFollow_ResetLinePID();
}

void LineFollow_SetTurnLimit(int limit)
{
    if (limit < 0) limit = 0;
    if (limit > 900) limit = 900;
    linefollow_turn_limit = limit;
}

/* state为八路灰度权重和，0=居中。返回左右差速值。 */
int LineFollow_ComputeLinePID(int error)
{
    float derivative = 0.0f;
    int output;

    if (linefollow_pid_valid)
        derivative = (float)(error - linefollow_pid_last_error);
    linefollow_pid_last_error = error;
    linefollow_pid_valid = 1;

    linefollow_pid_integral += error;
    if (linefollow_pid_integral > LINE_FOLLOW_I_LIMIT)
        linefollow_pid_integral = LINE_FOLLOW_I_LIMIT;
    if (linefollow_pid_integral < -LINE_FOLLOW_I_LIMIT)
        linefollow_pid_integral = -LINE_FOLLOW_I_LIMIT;

    linefollow_debug_p = linefollow_kp * error;
    linefollow_debug_i = linefollow_ki * linefollow_pid_integral;
    linefollow_debug_d = linefollow_kd * derivative;

    output = (int)(linefollow_debug_p + linefollow_debug_i + linefollow_debug_d);

    if (output > linefollow_turn_limit) output = linefollow_turn_limit;
    if (output < -linefollow_turn_limit) output = -linefollow_turn_limit;
    linefollow_debug_steer = output;

    return output;
}

/* 主循环命令以单次四轮事务发布，200Hz ISR不会读到半更新的目标向量。 */
void LineFollow_SetWheelTargets(int m1, int m2, int m3, int m4)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    wheel_target[1] = m1;
    wheel_target[2] = m2;
    wheel_target[3] = m3;
    wheel_target[4] = m4;
    if (primask == 0U) __enable_irq();
}

void LineFollow_Stop(void)
{
    LineFollow_SetWheelTargets(0, 0, 0, 0);
}

void LineFollow_Init(void)
{
    ChassisPos_ResetVelocityPid();
    HWT101_UART_Init();
    HeadingHold_Init();
    ChassisPos_Init();
}
