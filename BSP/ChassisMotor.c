#include "ChassisMotor.h"
#include "Moto.h"
#include "PID.h"

static float speed_kp;
static float speed_ki;
static float speed_kd;

/* 底层急停锁存: 置位后 ControlTick 每周期强制输出 0, 直到 resume 解除.
 * VOFA stop 使用; 不破坏任何状态机变量. */
static volatile uint8_t chassis_motor_inhibit;

void ChassisMotor_SetInhibit(uint8_t inhibit)
{
    chassis_motor_inhibit = (inhibit != 0U) ? 1U : 0U;
    if (chassis_motor_inhibit)
    {
        Move_Control(0, 0, 0, 0);
        ChassisMotor_ResetSpeedPid();
    }
}

uint8_t ChassisMotor_GetInhibit(void)
{
    return chassis_motor_inhibit;
}

void ChassisMotor_ResetSpeedPid(void)
{
    PID_Init(&pid_m1, speed_kp, speed_ki, speed_kd);
    PID_Init(&pid_m2, speed_kp, speed_ki, speed_kd);
    PID_Init(&pid_m3, speed_kp, speed_ki, speed_kd);
    PID_Init(&pid_m4, speed_kp, speed_ki, speed_kd);
}

/* 在线调增益: 更新后立即重建四个速度 PID (清误差/历史/输出) */
void ChassisMotor_SetGains(float kp, float ki, float kd)
{
    speed_kp = kp;
    speed_ki = ki;
    speed_kd = kd;
    ChassisMotor_ResetSpeedPid();
}

void ChassisMotor_GetGains(float *kp, float *ki, float *kd)
{
    if (kp != NULL) *kp = speed_kp;
    if (ki != NULL) *ki = speed_ki;
    if (kd != NULL) *kd = speed_kd;
}

void ChassisMotor_ResetEncoders(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    encoder[1] = 0;
    encoder[2] = 0;
    encoder[3] = 0;
    encoder[4] = 0;
    if (primask == 0U) __enable_irq();
}

void ChassisMotor_Init(float kp, float ki, float kd)
{
    speed_kp = kp;
    speed_ki = ki;
    speed_kd = kd;
    Move_Init();
    ChassisMotor_ResetEncoders();
    ChassisMotor_ResetSpeedPid();
    Move_Control(0, 0, 0, 0);
}

void ChassisMotor_SampleEncoders(void)
{
    Move_Velocity();
}

void ChassisMotor_IntegrateEncoders(void)
{
    Move_Encoder();
}

void ChassisMotor_ControlTick(const volatile int wheel_target[5])
{
    int target_1 = wheel_target[1];
    int target_2 = wheel_target[2];
    int target_3 = wheel_target[3];
    int target_4 = wheel_target[4];

    /* 急停锁存: 每周期持续输出 0, 屏蔽任何上层目标 */
    if (chassis_motor_inhibit)
    {
        Move_Control(0, 0, 0, 0);
        return;
    }

    /* A zero command must be a real stop.  Incremental PID otherwise retains
     * its previous output and can keep driving during pause states. */
    if ((target_1 | target_2 | target_3 | target_4) == 0)
    {
        ChassisMotor_ResetSpeedPid();
        Move_Control(0, 0, 0, 0);
        return;
    }

    PID_Calc(&pid_m1, target_1, velocity[1]);
    PID_Calc(&pid_m2, target_2, velocity[2]);
    PID_Calc(&pid_m3, target_3, velocity[3]);
    PID_Calc(&pid_m4, target_4, velocity[4]);

    /* Hardware calibration: negative PWM drives the vehicle forward. */
    Move_Control(-(int)pid_m1.output,
                 -(int)pid_m2.output,
                 -(int)pid_m3.output,
                 -(int)pid_m4.output);
}
