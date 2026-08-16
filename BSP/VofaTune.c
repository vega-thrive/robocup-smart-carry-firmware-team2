#include "VofaTune.h"
#include "usart.h"
#include "LineFollow.h"
#include "ChassisMotor.h"
#include "HeadingHold.h"
#include "Moto.h"
#include "Sensor.h"
#include "HWT101.h"
#include "ProjectService.h"
#include "ChassisPos.h"
#include <stdio.h>
#include <string.h>

#if VOFA_TUNE_ENABLE

/* =================================================================== *
 *  USART2: PA2/PA3, 115200 8N1, 中断优先级 4 (低于摄像头 UART4=3).
 *  TX: 中断非阻塞. 命令回复进入发送队列(优先, 队满丢最旧);
 *      遥测直发, 忙或有回复排队时整帧丢弃.
 *  RX: 逐字节中断行缓冲, 主循环短临界区拷贝后解析.
 * =================================================================== */
#define VOFA_UART               (&huart2)
#define VOFA_UART_IRQn          USART2_IRQn
#define VOFA_UART_PRIO          4U
#define VOFA_RX_LINE_MAX        64
#define VOFA_TX_BUF_MAX         192
#define VOFA_Q_SLOTS            16
#define VOFA_Q_LEN              96
#define VOFA_SEND_PERIOD_MS     50U    /* 遥测 20Hz */

/* =================================================================== *
 *  参数双层结构: active=目标值, pending=VOFA 收到未应用
 *  增益 apply 立即生效(清对应状态); 速度 apply 只改目标,
 *  由状态机在下一次进入对应状态时锁存(LineFollow_LatchSpeeds).
 * =================================================================== */
typedef struct
{
    float line_kp;
    float line_ki;
    float line_kd;
    int   line_max_steer;
    float wheel_kp;
    float wheel_ki;
    float wheel_kd;
    float heading_kp;
    float heading_ki;
    float heading_kd;
    int   heading_max_correction;
    int   task1_line_speed;
    int   task2_line_speed;
    int   home_return_speed;
} VofaParams;

static const VofaParams vofa_defaults =
{
    9.0f, 0.15f, 0.8f, 450,                 /* 巡线 PID  (LineFollow.c 宏默认值) */
    0.62f, 0.06f, 0.00f,                    /* 轮速 PID  (main.c Init 默认值)   */
    3.0f, 3.1f, 0.12f, 125,                 /* 航向 PID  (HeadingHold.h 宏默认值)*/
    230, 230, 400                           /* task1/task2 巡线速, 回家速       */
};

static VofaParams vofa_active;
static VofaParams vofa_pending;

/* =================================================================== *
 *  串口收发状态
 * =================================================================== */
static volatile uint8_t vofa_rx_byte;
static volatile char    vofa_rx_line[VOFA_RX_LINE_MAX];
static volatile uint8_t vofa_rx_len;
static volatile uint8_t vofa_line_ready;
static volatile uint8_t vofa_rx_overflow;   /* 上一条命令未处理完又收到新行 */

static char     vofa_q[VOFA_Q_SLOTS][VOFA_Q_LEN];   /* 命令回复队列 */
static uint8_t  vofa_q_head;
static uint8_t  vofa_q_count;
static volatile uint8_t vofa_tx_active;             /* 主循环写/TxCplt中断清, 必须 volatile */
static volatile uint8_t vofa_tx_overflow;           /* 队满丢弃新回复, 主循环回报一次 */
static char     vofa_tx_buf[VOFA_Q_LEN];            /* 当前发送中的回复 (独立缓冲) */
static char     vofa_tele_buf[VOFA_TX_BUF_MAX];

static uint8_t  vofa_watch_mode = 1U;   /* 1=line 2=wheel 3=heading 4=position 5=route */
static uint32_t vofa_next_send_ms;
static uint8_t  vofa_show_index;         /* show 逐行异步发送状态机 (P_COUNT=完成) */

/* =================================================================== *
 *  参数表: 名字/合法范围/是否为浮点
 * =================================================================== */
typedef enum
{
    P_LINE_KP = 0,
    P_LINE_KI,
    P_LINE_KD,
    P_LINE_MAX_STEER,
    P_WHEEL_KP,
    P_WHEEL_KI,
    P_WHEEL_KD,
    P_HEADING_KP,
    P_HEADING_KI,
    P_HEADING_KD,
    P_HEADING_MAX_CORR,
    P_TASK1_SPEED,
    P_TASK2_SPEED,
    P_HOME_RETURN_SPEED,
    P_COUNT
} VofaParamId;

typedef struct
{
    const char *name;
    float min;
    float max;
    uint8_t is_float;
} VofaParamDef;

static const VofaParamDef vofa_param_defs[P_COUNT] =
{
    { "line_kp",                 0.0f,  30.0f, 1 },
    { "line_ki",                 0.0f,   2.0f, 1 },
    { "line_kd",                 0.0f,   5.0f, 1 },
    { "line_max_steer",          0.0f, 900.0f, 0 },
    { "wheel_kp",                0.0f,   5.0f, 1 },
    { "wheel_ki",                0.0f,   1.0f, 1 },
    { "wheel_kd",                0.0f,   2.0f, 1 },
    { "heading_kp",              0.0f,  10.0f, 1 },
    { "heading_ki",              0.0f,  10.0f, 1 },
    { "heading_kd",              0.0f,   2.0f, 1 },
    { "heading_max_correction",  0.0f, 400.0f, 0 },
    { "task1_line_speed",      100.0f, 500.0f, 0 },
    { "task2_line_speed",      100.0f, 500.0f, 0 },
    { "home_return_speed",     150.0f, 600.0f, 0 },
};

static void VofaParamSet(VofaParams *p, int id, float v)
{
    switch (id)
    {
        case P_LINE_KP:               p->line_kp = v; break;
        case P_LINE_KI:               p->line_ki = v; break;
        case P_LINE_KD:               p->line_kd = v; break;
        case P_LINE_MAX_STEER:        p->line_max_steer = (int)v; break;
        case P_WHEEL_KP:              p->wheel_kp = v; break;
        case P_WHEEL_KI:              p->wheel_ki = v; break;
        case P_WHEEL_KD:              p->wheel_kd = v; break;
        case P_HEADING_KP:            p->heading_kp = v; break;
        case P_HEADING_KI:            p->heading_ki = v; break;
        case P_HEADING_KD:            p->heading_kd = v; break;
        case P_HEADING_MAX_CORR:      p->heading_max_correction = (int)v; break;
        case P_TASK1_SPEED:           p->task1_line_speed = (int)v; break;
        case P_TASK2_SPEED:           p->task2_line_speed = (int)v; break;
        case P_HOME_RETURN_SPEED:     p->home_return_speed = (int)v; break;
        default: break;
    }
}

static float VofaParamGet(const VofaParams *p, int id)
{
    switch (id)
    {
        case P_LINE_KP:               return p->line_kp;
        case P_LINE_KI:               return p->line_ki;
        case P_LINE_KD:               return p->line_kd;
        case P_LINE_MAX_STEER:        return (float)p->line_max_steer;
        case P_WHEEL_KP:              return p->wheel_kp;
        case P_WHEEL_KI:              return p->wheel_ki;
        case P_WHEEL_KD:              return p->wheel_kd;
        case P_HEADING_KP:            return p->heading_kp;
        case P_HEADING_KI:            return p->heading_ki;
        case P_HEADING_KD:            return p->heading_kd;
        case P_HEADING_MAX_CORR:      return (float)p->heading_max_correction;
        case P_TASK1_SPEED:           return (float)p->task1_line_speed;
        case P_TASK2_SPEED:           return (float)p->task2_line_speed;
        case P_HOME_RETURN_SPEED:     return (float)p->home_return_speed;
        default:                      return 0.0f;
    }
}

/* =================================================================== *
 *  辅助: 无库依赖的浮点解析/格式化 (microlib 无 %f 支持风险)
 * =================================================================== */
static int VofaParseFloat(const char *s, float *out)
{
    float sign = 1.0f;
    float int_part = 0.0f;
    float frac = 0.0f;
    float scale = 0.1f;
    const char *p = s;
    uint8_t digits = 0U;

    if ((p == NULL) || (*p == '\0')) return 0;
    if (*p == '-') { sign = -1.0f; p++; }
    else if (*p == '+') { p++; }
    if (*p == '\0') return 0;

    while ((*p >= '0') && (*p <= '9'))
    {
        int_part = int_part * 10.0f + (float)(*p - '0');
        p++;
        digits++;
    }
    if (*p == '.')
    {
        p++;
        while ((*p >= '0') && (*p <= '9'))
        {
            frac += (float)(*p - '0') * scale;
            scale *= 0.1f;
            p++;
            digits++;
        }
    }
    if ((digits == 0U) || (*p != '\0')) return 0;
    *out = sign * (int_part + frac);
    return 1;
}

static int VofaFmtFloat(char *buf, float v)
{
    int i = (int)v;
    int frac = (int)((v - (float)i) * 1000.0f);
    if (frac < 0) frac = -frac;
    return sprintf(buf, "%d.%03d", i, frac);
}

/* =================================================================== *
 *  发送
 * =================================================================== */
/* 命令回复: 入队. 队满时拒绝新回复并置 overflow (由主循环回报一次),
 * 绝不静默删除已排队的旧回复. */
static void VofaSend(const char *text)
{
    uint16_t len = (uint16_t)strlen(text);
    char *slot;

    if (vofa_q_count >= VOFA_Q_SLOTS)
    {
        vofa_tx_overflow = 1U;
        return;
    }
    if (len > (VOFA_Q_LEN - 1U)) len = (uint16_t)(VOFA_Q_LEN - 1U);
    slot = vofa_q[(uint8_t)((vofa_q_head + vofa_q_count) % VOFA_Q_SLOTS)];
    memcpy(slot, text, len);
    slot[len] = '\0';
    vofa_q_count++;
}

/* 遥测: 只发一次构建好的行; 忙或有回复排队 -> 整帧丢弃 */
static void VofaSendTelemetry(const char *text)
{
    uint16_t len;

    if (vofa_tx_active || (vofa_q_count > 0U)) return;
    if (VOFA_UART->gState != HAL_UART_STATE_READY) return;

    len = (uint16_t)strlen(text);
    if (len > (VOFA_TX_BUF_MAX - 1U)) len = (uint16_t)(VOFA_TX_BUF_MAX - 1U);
    memcpy(vofa_tele_buf, text, len);
    vofa_tx_active = 1U;
    (void)HAL_UART_Transmit_IT(VOFA_UART, (uint8_t *)vofa_tele_buf, len);
}

/* 主循环排空回复队列: 先把队首拷贝到独立 tx_buf 再出队,
 * 发送期间队列内存可安全复用. */
static void VofaTxDrain(void)
{
    uint16_t len;

    if (vofa_tx_active) return;
    if (vofa_q_count == 0U) return;
    if (VOFA_UART->gState != HAL_UART_STATE_READY) return;

    len = (uint16_t)strlen(vofa_q[vofa_q_head]);
    if (len > (VOFA_Q_LEN - 1U)) len = (uint16_t)(VOFA_Q_LEN - 1U);
    memcpy(vofa_tx_buf, vofa_q[vofa_q_head], len);
    vofa_tx_buf[len] = '\0';

    vofa_tx_active = 1U;
    (void)HAL_UART_Transmit_IT(VOFA_UART, (uint8_t *)vofa_tx_buf, len);
    vofa_q_head = (uint8_t)((vofa_q_head + 1U) % VOFA_Q_SLOTS);
    vofa_q_count--;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        vofa_tx_active = 0U;
    }
}

/* =================================================================== *
 *  接收: ISR 只做行缓冲, 主循环短临界区拷贝后解析
 * =================================================================== */
void VofaTune_OnRxByte(void)
{
    uint8_t b = vofa_rx_byte;

    if (vofa_line_ready)
    {
        /* 上一条命令主循环还没取走: 后续所有字节直接丢弃,
         * 不写 rx_line (防止覆盖未解析命令). 换行时报告一次溢出. */
        if ((b == '\n') || (b == '\r'))
        {
            vofa_rx_overflow = 1U;
        }
    }
    else if ((b == '\n') || (b == '\r'))
    {
        if (vofa_rx_len > 0U)
        {
            vofa_rx_line[vofa_rx_len] = '\0';
            vofa_line_ready = 1U;
        }
        vofa_rx_len = 0U;
    }
    else if (vofa_rx_len < (VOFA_RX_LINE_MAX - 1U))
    {
        vofa_rx_line[vofa_rx_len++] = (char)b;
    }
    else
    {
        vofa_rx_len = 0U;   /* 超长命令丢弃 */
    }
    (void)HAL_UART_Receive_IT(VOFA_UART, (uint8_t *)&vofa_rx_byte, 1U);
}

void VofaTune_OnUartError(void)
{
    vofa_rx_len = 0U;
    (void)HAL_UART_Receive_IT(VOFA_UART, (uint8_t *)&vofa_rx_byte, 1U);
}

/* =================================================================== *
 *  命令解析 (以 \n 结尾的文本命令)
 * =================================================================== */
static int VofaFindParam(const char *name)
{
    int i;
    for (i = 0; i < P_COUNT; i++)
    {
        if (strcmp(name, vofa_param_defs[i].name) == 0) return i;
    }
    return -1;
}

/* 手工分词 (不依赖 strtok, 兼容 microlib) */
static char *VofaNextToken(char **cursor)
{
    char *p = *cursor;
    char *start;

    if (p == NULL) return NULL;
    while ((*p == ' ') || (*p == '\t') || (*p == '\r')) p++;
    if (*p == '\0')
    {
        *cursor = NULL;
        return NULL;
    }
    start = p;
    while ((*p != ' ') && (*p != '\t') && (*p != '\r') && (*p != '\0')) p++;
    if (*p != '\0')
    {
        *p = '\0';
        p++;
    }
    *cursor = p;
    return start;
}

static void VofaCmdSet(char *name, char *val)
{
    int id;
    float v;

    if ((name == NULL) || (val == NULL))
    {
        VofaSend("ERR set: usage set <name> <value>\n");
        return;
    }
    id = VofaFindParam(name);
    if (id < 0)
    {
        VofaSend("ERR set: unknown param\n");
        return;
    }
    if (!VofaParseFloat(val, &v))
    {
        VofaSend("ERR set: bad number\n");
        return;
    }
    if ((v < vofa_param_defs[id].min) || (v > vofa_param_defs[id].max))
    {
        VofaSend("ERR set: out of range\n");
        return;
    }
    VofaParamSet(&vofa_pending, id, v);
    VofaSend("OK set (pending, use apply)\n");
}

/* apply: 临界区内原子完成全部增益更新, 防止 TIM7 读到混合参数 */
static void VofaCmdApply(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    LineFollow_SetGains(vofa_pending.line_kp,
                        vofa_pending.line_ki,
                        vofa_pending.line_kd);
    LineFollow_SetTurnLimit(vofa_pending.line_max_steer);
    ChassisMotor_SetGains(vofa_pending.wheel_kp,
                          vofa_pending.wheel_ki,
                          vofa_pending.wheel_kd);
    HeadingHold_SetGains(vofa_pending.heading_kp,
                         vofa_pending.heading_ki,
                         vofa_pending.heading_kd,
                         vofa_pending.heading_max_correction);
    /* 速度只更新目标: 下一次进入对应状态时由 LineFollow_LatchSpeeds 生效 */
    linefollow_task1_speed       = vofa_pending.task1_line_speed;
    linefollow_task2_speed       = vofa_pending.task2_line_speed;
    linefollow_home_return_speed = vofa_pending.home_return_speed;
    vofa_active = vofa_pending;
    if (primask == 0U) __enable_irq();

    VofaSend("OK applied\n");
}

/* show 改为逐行异步发送: 每周期入队一行, 不撑爆队列也不丢行 */
static void VofaCmdShow(void)
{
    vofa_show_index = 0U;
    VofaSend("OK show:\n");
}

static void VofaShowTick(void)
{
    char buf[VOFA_Q_LEN];
    char va[16];
    char vp[16];
    int i;

    if (vofa_show_index >= P_COUNT) return;
    if (vofa_q_count >= (VOFA_Q_SLOTS - 1U)) return;   /* 队列满, 等下一周期 */

    i = vofa_show_index;
    VofaFmtFloat(va, VofaParamGet(&vofa_active, i));
    VofaFmtFloat(vp, VofaParamGet(&vofa_pending, i));
    sprintf(buf, "%s:%s,%s\n", vofa_param_defs[i].name, va, vp);
    VofaSend(buf);
    vofa_show_index = (uint8_t)(vofa_show_index + 1U);
}

static void VofaCmdDefaults(void)
{
    vofa_pending = vofa_defaults;
    VofaSend("OK pending=defaults (use apply)\n");
}

static void VofaCmdWatch(const char *group)
{
    if (group == NULL)
    {
        VofaSend("ERR watch: usage watch <line|wheel|heading|position|route>\n");
        return;
    }
    if (strcmp(group, "line") == 0)       { vofa_watch_mode = 1U; VofaSend("OK watch line\n"); }
    else if (strcmp(group, "wheel") == 0) { vofa_watch_mode = 2U; VofaSend("OK watch wheel\n"); }
    else if (strcmp(group, "heading") == 0) { vofa_watch_mode = 3U; VofaSend("OK watch heading\n"); }
    else if (strcmp(group, "position") == 0) { vofa_watch_mode = 4U; VofaSend("OK watch position\n"); }
    else if (strcmp(group, "route") == 0) { vofa_watch_mode = 5U; VofaSend("OK watch route\n"); }
    else
    {
        VofaSend("ERR watch: unknown group\n");
    }
}

/* stop: 底层锁存急停 (ControlTick 持续输出 0), 状态机内部变量不动.
 * 注意: 状态机计时仍继续, 长时间 stop 后 resume 可能已超时/换状态. */
static void VofaCmdStop(void)
{
    ChassisMotor_SetInhibit(1U);
    LineFollow_Stop();
    VofaSend("OK stop (inhibited; timers keep running, use resume)\n");
}

static void VofaCmdResume(void)
{
    ChassisMotor_SetInhibit(0U);
    VofaSend("OK resume (inhibit released)\n");
}

static void VofaParseLine(char *line)
{
    char *cursor = line;
    char *cmd;
    char *arg1;
    char *arg2;

    cmd  = VofaNextToken(&cursor);
    arg1 = VofaNextToken(&cursor);
    arg2 = VofaNextToken(&cursor);

    if (cmd == NULL) return;

    if (strcmp(cmd, "set") == 0)             VofaCmdSet(arg1, arg2);
    else if (strcmp(cmd, "show") == 0)       VofaCmdShow();
    else if (strcmp(cmd, "apply") == 0)      VofaCmdApply();
    else if (strcmp(cmd, "defaults") == 0)   VofaCmdDefaults();
    else if (strcmp(cmd, "watch") == 0)      VofaCmdWatch(arg1);
    else if (strcmp(cmd, "stop") == 0)       VofaCmdStop();
    else if (strcmp(cmd, "resume") == 0)     VofaCmdResume();
    else
    {
        VofaSend("ERR unknown cmd\n");
    }
}

/* =================================================================== *
 *  监控数据 (按 watch 分组, 20Hz)
 * =================================================================== */
static void VofaSendWatchLine(void)
{
    char buf[VOFA_TX_BUF_MAX];
    int line_state = 0;
    int turn = 0;
    int route = ProjectService_GetRouteState();
    int base = 0;

    if (route == ROUTE_TRACK_FIRST)            base = linefollow_task1_speed_active;
    else if (route == ROUTE_TRACK_RETURN)      base = linefollow_task2_speed_active;
    else if (route == ROUTE_HOME_RETURN_DRIVE) base = linefollow_home_return_speed_active;

    ProjectService_GetLineDebug(&line_state, &turn);
    sprintf(buf,
        "line_err:%d,gs:%u%u%u%u%u%u%u%u,p:%d,i:%d,d:%d,steer:%d,base:%d,"
        "w1:%d,w2:%d,w3:%d,w4:%d,state:%d\n",
        line_state,
        (unsigned)GS_state[1], (unsigned)GS_state[2],
        (unsigned)GS_state[3], (unsigned)GS_state[4],
        (unsigned)GS_state[5], (unsigned)GS_state[6],
        (unsigned)GS_state[7], (unsigned)GS_state[8],
        (int)linefollow_debug_p, (int)linefollow_debug_i, (int)linefollow_debug_d,
        linefollow_debug_steer, base,
        wheel_target[1], wheel_target[2], wheel_target[3], wheel_target[4],
        route);
    VofaSendTelemetry(buf);
}

static void VofaSendWatchWheel(void)
{
    char buf[VOFA_TX_BUF_MAX];
    sprintf(buf,
        "t1:%d,t2:%d,t3:%d,t4:%d,v1:%d,v2:%d,v3:%d,v4:%d,"
        "e1:%d,e2:%d,e3:%d,e4:%d,pwm1:%d,pwm2:%d,pwm3:%d,pwm4:%d,inh:%u\n",
        wheel_target[1], wheel_target[2], wheel_target[3], wheel_target[4],
        velocity[1], velocity[2], velocity[3], velocity[4],
        wheel_target[1] - velocity[1], wheel_target[2] - velocity[2],
        wheel_target[3] - velocity[3], wheel_target[4] - velocity[4],
        motor_pwm[1], motor_pwm[2], motor_pwm[3], motor_pwm[4],
        (unsigned)ChassisMotor_GetInhibit());
    VofaSendTelemetry(buf);
}

static void VofaSendWatchHeading(void)
{
    char buf[VOFA_TX_BUF_MAX];
    float angle = 0.0f;
    float abs_yaw = 0.0f;
    int correction = 0;

    ProjectService_GetHeadingDebug(&angle, &abs_yaw, &correction);
    sprintf(buf, "h_angle:%d,h_abs:%d,h_corr:%d\n",
            (int)(angle * 100.0f), (int)(abs_yaw * 100.0f), correction);
    VofaSendTelemetry(buf);
}

static void VofaSendWatchPosition(void)
{
    char buf[VOFA_TX_BUF_MAX];
    sprintf(buf,
        "pos_active:%u,pos_fault:%u,tgt_speed:%d,pos:%d,err:%d,"
        "lpos:%d,lerr:%d,left:%d\n",
        (unsigned)pos_control_active, (unsigned)pos_fault,
        debug_trap_speed, debug_trap_position, debug_trap_error,
        debug_trap_left_position, debug_trap_left_error,
        ChassisPos_GetLeftPosition());
    VofaSendTelemetry(buf);
}

static void VofaSendWatchRoute(void)
{
    char buf[VOFA_TX_BUF_MAX];
    uint32_t enter_ms = ProjectService_GetRouteStateEnterMs();
    sprintf(buf, "state:%d,sub:%u,enter_ms:%lu,elapsed_ms:%lu\n",
            ProjectService_GetRouteState(),
            (unsigned)ProjectService_GetRouteSubStep(),
            (unsigned long)enter_ms,
            (unsigned long)(HAL_GetTick() - enter_ms));
    VofaSendTelemetry(buf);
}

static void VofaSendWatch(void)
{
    switch (vofa_watch_mode)
    {
        case 1U: VofaSendWatchLine();     break;
        case 2U: VofaSendWatchWheel();    break;
        case 3U: VofaSendWatchHeading();  break;
        case 4U: VofaSendWatchPosition(); break;
        case 5U: VofaSendWatchRoute();    break;
        default: break;
    }
}

/* =================================================================== *
 *  公共接口
 * =================================================================== */
void VofaTune_Init(void)
{
    MX_USART2_UART_Init();
    HAL_NVIC_SetPriority(VOFA_UART_IRQn, VOFA_UART_PRIO, 0U);
    HAL_NVIC_EnableIRQ(VOFA_UART_IRQn);

    vofa_active  = vofa_defaults;
    vofa_pending = vofa_defaults;
    vofa_rx_len  = 0U;
    vofa_line_ready = 0U;
    vofa_rx_overflow = 0U;
    vofa_q_head = 0U;
    vofa_q_count = 0U;
    vofa_tx_active = 0U;
    vofa_tx_overflow = 0U;
    vofa_watch_mode = 1U;
    vofa_show_index = P_COUNT;
    vofa_next_send_ms = HAL_GetTick() + VOFA_SEND_PERIOD_MS;

    (void)HAL_UART_Receive_IT(VOFA_UART, (uint8_t *)&vofa_rx_byte, 1U);

    VofaSend("vofa ready (watch line)\n");
}

void VofaTune_Process(void)
{
    char parse_buf[VOFA_RX_LINE_MAX];

    /* 每周期轻量跟踪状态切换时间戳 (与 watch 分组无关) */
    ProjectService_TrackRouteStateTick();

    /* 短临界区拷贝命令行, 避免 ISR 解析期间覆盖缓冲 */
    if (vofa_line_ready)
    {
        uint32_t primask = __get_PRIMASK();
        __disable_irq();
        memcpy(parse_buf, (const void *)vofa_rx_line, VOFA_RX_LINE_MAX);
        parse_buf[VOFA_RX_LINE_MAX - 1U] = '\0';
        vofa_line_ready = 0U;
        if (vofa_rx_overflow)
        {
            vofa_rx_overflow = 0U;
            if (primask == 0U) __enable_irq();
            VofaSend("ERR rx overflow\n");
        }
        else if (primask == 0U)
        {
            __enable_irq();
        }
        VofaParseLine(parse_buf);
    }

    VofaShowTick();
    VofaTxDrain();

    if (vofa_tx_overflow)
    {
        vofa_tx_overflow = 0U;
        VofaSend("ERR tx queue full\n");
    }

    if ((int32_t)(HAL_GetTick() - vofa_next_send_ms) < 0) return;
    vofa_next_send_ms = HAL_GetTick() + VOFA_SEND_PERIOD_MS;
    VofaSendWatch();
}

#endif /* VOFA_TUNE_ENABLE */
