#include "GM65Scanner.h"
#include <string.h>

static const uint8_t gm65_trigger_command[9] =
{
    0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD
};

static const uint8_t gm65_ack[7] =
{
    0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31
};

static UART_HandleTypeDef *gm65_uart;
static volatile GM65_Status gm65_status = GM65_IDLE;
static volatile uint16_t gm65_length;
static uint32_t gm65_deadline;
static uint32_t gm65_last_data_ms;
static char gm65_text[GM65_TEXT_CAPACITY];
static uint8_t gm65_rx_byte;
static uint8_t gm65_ack_index;
static uint8_t gm65_data_started;

/* 诊断 (扫码测试用) */
volatile uint32_t gm65_diag_rx_bytes;
volatile uint8_t  gm65_diag_last_bytes[8];
volatile uint8_t  gm65_diag_last_index;

static void GM65_StartListen(uint32_t timeout_ms)
{
    gm65_length = 0U;
    gm65_text[0] = '\0';
    gm65_ack_index = 0U;
    gm65_data_started = 0U;
    gm65_last_data_ms = 0U;
    gm65_deadline = HAL_GetTick() + timeout_ms;
    gm65_status = GM65_WAITING;
}

static void GM65_RemoveSuffix(void)
{
    if ((gm65_length >= 2U) &&
        (gm65_text[gm65_length - 2U] == 'D') &&
        (gm65_text[gm65_length - 1U] == 'T'))
    {
        gm65_length -= 2U;
    }
    gm65_text[gm65_length] = '\0';
}

void GM65_Init(UART_HandleTypeDef *uart)
{
    gm65_uart = uart;
    gm65_status = GM65_IDLE;
    gm65_length = 0U;
    gm65_text[0] = '\0';
    gm65_ack_index = 0U;
    gm65_data_started = 0U;
    gm65_last_data_ms = 0U;
}

HAL_StatusTypeDef GM65_StartReceiveIT(void)
{
    if (gm65_uart == NULL) return HAL_ERROR;
    return HAL_UART_Receive_IT(gm65_uart, &gm65_rx_byte, 1U);
}

HAL_StatusTypeDef GM65_StartScan(uint32_t timeout_ms)
{
    if (gm65_uart == NULL) return HAL_ERROR;
    GM65_StartListen(timeout_ms);
    return HAL_UART_Transmit(gm65_uart, (uint8_t *)gm65_trigger_command,
                             sizeof(gm65_trigger_command), 100U);
}

void GM65_Process(void)
{
    if ((gm65_status == GM65_WAITING) && (gm65_length != 0U) &&
        ((uint32_t)(HAL_GetTick() - gm65_last_data_ms) >= 100U))
    {
        GM65_RemoveSuffix();
        gm65_status = (gm65_length != 0U) ? GM65_READY : GM65_WAITING;
    }

    if ((gm65_status == GM65_WAITING) &&
        ((int32_t)(HAL_GetTick() - gm65_deadline) >= 0))
    {
        gm65_status = GM65_TIMEOUT;
    }
}

void GM65_OnRxComplete(UART_HandleTypeDef *uart)
{
    if ((gm65_uart == NULL) || (uart != gm65_uart)) return;
    GM65_OnRxByte(gm65_rx_byte);
    (void)HAL_UART_Receive_IT(gm65_uart, &gm65_rx_byte, 1U);
}

void GM65_OnRxByte(uint8_t byte)
{
    gm65_diag_rx_bytes++;
    gm65_diag_last_bytes[gm65_diag_last_index] = byte;
    gm65_diag_last_index = (uint8_t)((gm65_diag_last_index + 1U) & 7U);

    if (gm65_status != GM65_WAITING) return;

    if (gm65_data_started == 0U)
    {
        if (byte == gm65_ack[gm65_ack_index])
        {
            gm65_ack_index++;
            if (gm65_ack_index == sizeof(gm65_ack))
            {
                gm65_ack_index = 0U;
                gm65_data_started = 1U;
                return;
            }
        }
        else
        {
            gm65_ack_index = (byte == gm65_ack[0]) ? 1U : 0U;
        }

        if ((gm65_ack_index == 0U) && (byte >= 0x20U) && (byte <= 0x7EU))
        {
            gm65_data_started = 1U;
        }
        else
        {
            return;
        }
    }

    if ((byte == '\r') || (byte == '\n'))
    {
        if (gm65_length != 0U)
        {
            GM65_RemoveSuffix();
            gm65_status = (gm65_length != 0U) ? GM65_READY : GM65_WAITING;
        }
        return;
    }

    if (gm65_length >= (GM65_TEXT_CAPACITY - 1U))
    {
        gm65_text[GM65_TEXT_CAPACITY - 1U] = '\0';
        gm65_status = GM65_OVERFLOW;
        return;
    }

    gm65_text[gm65_length++] = (char)byte;
    gm65_text[gm65_length] = '\0';
    gm65_last_data_ms = HAL_GetTick();
}

void GM65_OnUartError(void)
{
    if (gm65_status == GM65_WAITING) gm65_status = GM65_TIMEOUT;
}

GM65_Status GM65_GetStatus(void)
{
    return gm65_status;
}

uint8_t GM65_TakeText(char *destination, uint16_t capacity)
{
    uint16_t length;
    if ((gm65_status != GM65_READY) || (destination == NULL) ||
        (capacity == 0U)) return 0U;

    length = gm65_length;
    if (length >= capacity) length = capacity - 1U;
    memcpy(destination, gm65_text, length);
    destination[length] = '\0';
    gm65_status = GM65_IDLE;
    return 1U;
}
