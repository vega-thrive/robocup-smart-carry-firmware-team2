#include "MaxiCam.h"
#include <string.h>

static UART_HandleTypeDef *maxicam_uart;       /* 摄像头绑定的UART。 */
static uint8_t maxicam_buffer[10];             /* 完整协议帧缓存。 */
static uint8_t maxicam_index;                  /* 下一字节写入位置。 */
static uint8_t maxicam_rx_byte;                /* HAL单字节中断接收缓存。 */
static volatile MaxiCam_Frame maxicam_latest;  /* 最近一帧有效数据。 */

/* 摄像头脚本需要先处理 STOP，才能可靠进入下一个识别循环。
 * 旧代码在同一毫秒连续发送 STOP 和目标模式，目标字节可能被旧循环吞掉。 */
#define MAXICAM_MODE_SWITCH_GUARD_MS 150U
#define MAXICAM_MODE_RETRY_MS         20U
static uint8_t maxicam_mode_pending;
static uint8_t maxicam_pending_mode;
static uint32_t maxicam_mode_send_at_ms;
static uint32_t maxicam_last_stop_ms;
static uint8_t maxicam_stop_guard_valid;

/* 接收诊断：分别统计字节、帧头和完整有效帧。 */
volatile uint32_t maxicam_diag_rx_bytes;
volatile uint32_t maxicam_diag_headers;
volatile uint32_t maxicam_diag_packets;
volatile uint32_t maxicam_diag_mode_tx;
volatile uint32_t maxicam_diag_mode_tx_error;

static void MaxiCam_OnRxByte(uint8_t byte);

/* 初始化软件状态。UART外设由CubeMX生成的MX_UARTx_Init()初始化。 */
void MaxiCam_Init(UART_HandleTypeDef *uart)
{
    maxicam_uart = uart;
    maxicam_index = 0U;
    maxicam_mode_pending = 0U;
    maxicam_pending_mode = 0U;
    maxicam_mode_send_at_ms = 0U;
    maxicam_last_stop_ms = 0U;
    maxicam_stop_guard_valid = 0U;
    memset((void *)&maxicam_latest, 0, sizeof(maxicam_latest));
}

/* HAL每次只接收一个字节，完成后由MaxiCam_OnRxComplete()续接。 */
HAL_StatusTypeDef MaxiCam_StartReceiveIT(void)
{
    if (maxicam_uart == NULL) return HAL_ERROR;
    return HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
}

static HAL_StatusTypeDef MaxiCam_TransmitModeByte(uint8_t command)
{
    HAL_StatusTypeDef status;

    status = HAL_UART_Transmit(maxicam_uart, &command, 1U, 20U);
    if (status == HAL_OK)
        maxicam_diag_mode_tx++;
    else
        maxicam_diag_mode_tx_error++;
    return status;
}

/* 模式枚举值就是协议命令值。STOP 立即发送；STOP 后的目标模式延迟
 * 150ms 发送，避免两个命令在摄像头旧识别循环的一次 UART 读取中粘连。 */
HAL_StatusTypeDef MaxiCam_SetMode(MaxiCam_Mode mode)
{
    uint8_t command = (uint8_t)mode;
    uint32_t now;

    if (maxicam_uart == NULL) return HAL_ERROR;
    now = HAL_GetTick();

    /* 切换模式后旧模式的最近帧立即失效，只接受切换后新收到的数据。 */
    maxicam_latest.received_at_ms = 0U;

    if (mode == MAXICAM_MODE_STOP)
    {
        maxicam_mode_pending = 0U;
        maxicam_last_stop_ms = now;
        maxicam_stop_guard_valid = 1U;
        return MaxiCam_TransmitModeByte(command);
    }

    maxicam_pending_mode = command;
    maxicam_mode_pending = 1U;
    maxicam_mode_send_at_ms = now;
    if (maxicam_stop_guard_valid &&
        ((now - maxicam_last_stop_ms) < MAXICAM_MODE_SWITCH_GUARD_MS))
    {
        maxicam_mode_send_at_ms = maxicam_last_stop_ms +
                                  MAXICAM_MODE_SWITCH_GUARD_MS;
        return HAL_OK;
    }

    if (MaxiCam_TransmitModeByte(maxicam_pending_mode) == HAL_OK)
    {
        maxicam_mode_pending = 0U;
        return HAL_OK;
    }
    maxicam_mode_send_at_ms = now + MAXICAM_MODE_RETRY_MS;
    return HAL_BUSY;
}

/* 主循环服务延迟的模式命令；发送忙时自动短延时重试。 */
void MaxiCam_Process(void)
{
    uint32_t now;

    if ((maxicam_uart == NULL) || !maxicam_mode_pending) return;
    now = HAL_GetTick();
    if ((int32_t)(now - maxicam_mode_send_at_ms) < 0) return;

    if (MaxiCam_TransmitModeByte(maxicam_pending_mode) == HAL_OK)
        maxicam_mode_pending = 0U;
    else
        maxicam_mode_send_at_ms = now + MAXICAM_MODE_RETRY_MS;
}

/* 摄像头固定发送10字节帧：
 * [0]=0xFF, [1]=0xAA, [2..7]=6字节业务数据, [8]=0xFE, [9]=0xBB。
 * 解析器能在噪声中重新寻找FF AA；只有帧尾正确才发布最新帧。 */
static void MaxiCam_OnRxByte(uint8_t byte)
{
    maxicam_diag_rx_bytes++;

    if (maxicam_index == 0U)
    {
        /* 等待第一个帧头字节0xFF，其余字节直接丢弃。 */
        if (byte == 0xFFU) maxicam_buffer[maxicam_index++] = byte;
        return;
    }
    if (maxicam_index == 1U)
    {
        /* 第二个帧头必须是0xAA；连续0xFF时保留同步机会。 */
        if (byte == 0xAAU)
        {
            maxicam_buffer[maxicam_index++] = byte;
            maxicam_diag_headers++;
        }
        else
        {
            maxicam_index = (byte == 0xFFU) ? 1U : 0U;
        }
        return;
    }

    maxicam_buffer[maxicam_index++] = byte;
    if (maxicam_index == sizeof(maxicam_buffer))
    {
        /* 收满10字节后校验帧尾；坏帧不发布并重新寻找帧头。 */
        if ((maxicam_buffer[8] == 0xFEU) &&
            (maxicam_buffer[9] == 0xBBU))
        {
            memcpy((void *)maxicam_latest.data, &maxicam_buffer[2], 6U);
            maxicam_latest.received_at_ms = HAL_GetTick();
            maxicam_latest.sequence++;
            maxicam_diag_packets++;
        }
        maxicam_index = 0U;
    }
}

/* 此函数由全局HAL_UART_RxCpltCallback()转调。 */
void MaxiCam_OnRxComplete(UART_HandleTypeDef *uart)
{
    if ((maxicam_uart == NULL) || (uart != maxicam_uart)) return;
    MaxiCam_OnRxByte(maxicam_rx_byte);
    (void)HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
}

/* 串口错误后放弃当前半帧，避免把错误前后的字节拼成一帧。 */
void MaxiCam_OnUartError(void)
{
    maxicam_index = 0U;
    if (maxicam_uart != NULL)
    {
        (void)HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
    }
}

uint8_t MaxiCam_GetLatest(MaxiCam_Frame *frame, uint32_t max_age_ms)
{
    uint32_t sequence;
    if (frame == NULL) return 0U;
    sequence = maxicam_latest.sequence;
    if (sequence == 0U) return 0U;
    if ((HAL_GetTick() - maxicam_latest.received_at_ms) > max_age_ms)
        return 0U;

    /* 先记录序号、再复制、最后复核序号。若ISR在复制期间发布新帧，
     * 本次返回0，避免主循环使用由两帧拼接而成的数据。 */
    memcpy(frame, (const void *)&maxicam_latest, sizeof(*frame));
    return (frame->sequence == sequence) ? 1U : 0U;
}
