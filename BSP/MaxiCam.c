#include "MaxiCam.h"
#include <string.h>

static UART_HandleTypeDef *maxicam_uart;
static uint8_t maxicam_buffer[10];
static uint8_t maxicam_index;
static uint8_t maxicam_rx_byte;
static volatile MaxiCam_Frame maxicam_latest;

volatile uint32_t maxicam_diag_rx_bytes;
volatile uint32_t maxicam_diag_headers;
volatile uint32_t maxicam_diag_packets;

static void MaxiCam_OnRxByte(uint8_t byte);

void MaxiCam_Init(UART_HandleTypeDef *uart)
{
    maxicam_uart = uart;
    maxicam_index = 0U;
    memset((void *)&maxicam_latest, 0, sizeof(maxicam_latest));
}

HAL_StatusTypeDef MaxiCam_StartReceiveIT(void)
{
    if (maxicam_uart == NULL) return HAL_ERROR;
    return HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
}

HAL_StatusTypeDef MaxiCam_SetMode(MaxiCam_Mode mode)
{
    uint8_t command = (uint8_t)mode;
    if (maxicam_uart == NULL) return HAL_ERROR;
    return HAL_UART_Transmit(maxicam_uart, &command, 1U, 20U);
}

static void MaxiCam_OnRxByte(uint8_t byte)
{
    maxicam_diag_rx_bytes++;

    if (maxicam_index == 0U)
    {
        if (byte == 0xFFU) maxicam_buffer[maxicam_index++] = byte;
        return;
    }
    if (maxicam_index == 1U)
    {
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

void MaxiCam_OnRxComplete(UART_HandleTypeDef *uart)
{
    if ((maxicam_uart == NULL) || (uart != maxicam_uart)) return;
    MaxiCam_OnRxByte(maxicam_rx_byte);
    (void)HAL_UART_Receive_IT(maxicam_uart, &maxicam_rx_byte, 1U);
}

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
    memcpy(frame, (const void *)&maxicam_latest, sizeof(*frame));
    return (frame->sequence == sequence) ? 1U : 0U;
}
