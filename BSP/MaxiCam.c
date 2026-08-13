#include "MaxiCam.h"
#include <string.h>

static UART_HandleTypeDef *maxicam_uart;       /* 摄像头绑定的UART。 */
static uint8_t maxicam_buffer[10];             /* 完整协议帧缓存。 */
static uint8_t maxicam_index;                  /* 下一字节写入位置。 */
static uint8_t maxicam_rx_byte;                /* HAL单字节中断接收缓存。 */
static volatile MaxiCam_Frame maxicam_latest;  /* 最近一帧有效数据。 */

/* 接收诊断：分别统计字节、帧头和完整有效帧。 */
volatile uint32_t maxicam_diag_rx_bytes;
volatile uint32_t maxicam_diag_headers;
volatile uint32_t maxicam_diag_packets;

static void MaxiCam_OnRxByte(uint8_t byte);

/* 初始化软件状态。UART外设由CubeMX生成的MX_UARTx_Init()初始化。 */
void MaxiCam_Init(UART_HandleTypeDef *uart)
{
    maxicam_uart = uart;
    maxicam_index = 0U;
    memset((void *)&maxicam_latest, 0, sizeof(maxicam_latest));
}

/* HAL每次只接收一个字节，完成后由MaxiCam_OnRxComplete()续接。 */
HAL_StatusTypeDef MaxiCam_StartReceiveIT(void)
{
    if (maxicam_uart == NULL) return HAL_ERROR;
    return HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
}

/* 模式枚举值就是摄像头协议命令值，直接发送一个字节。 */
HAL_StatusTypeDef MaxiCam_SetMode(MaxiCam_Mode mode)
{
    uint8_t command = (uint8_t)mode;
    if (maxicam_uart == NULL) return HAL_ERROR;
    return HAL_UART_Transmit(maxicam_uart, &command, 1U, 20U);
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
