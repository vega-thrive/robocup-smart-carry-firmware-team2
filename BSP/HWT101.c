#include "HWT101.h"
#include "usart.h"
#include "GM65Scanner.h"
#include "MaxiCam.h"
#include "VofaTune.h"

/* WIT serial frame: 0x55, type, 8 data bytes, checksum. */
static uint8_t uart_rx_byte;
static uint8_t frame[11];
static uint8_t frame_index;
static volatile uint8_t hwt_ready;
static volatile uint8_t reset_pending = 1;
static volatile float yaw_reference;
static volatile float yaw_absolute;
static volatile uint32_t last_frame_tick;
static volatile uint32_t angle_update_count;

#define HWT101_FRAME_TIMEOUT_MS 200U

/* WIT configuration protocol: unlock -> write RRATE=100 Hz -> save.
 * The setting is stored in the module, so the command is harmless on later boots. */
#define HWT101_SET_RATE_100HZ 1U

volatile float Angle;

static float HWT101_Normalize(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static void HWT101_SetOutputRate100Hz(void)
{
#if HWT101_SET_RATE_100HZ
    static const uint8_t unlock_cmd[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    static const uint8_t rate_cmd[5]   = {0xFF, 0xAA, 0x03, 0x09, 0x00};
    static const uint8_t save_cmd[5]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    HAL_UART_Transmit(&huart5, (uint8_t *)unlock_cmd, sizeof(unlock_cmd), 20);
    HAL_Delay(60);
    HAL_UART_Transmit(&huart5, (uint8_t *)rate_cmd, sizeof(rate_cmd), 20);
    HAL_Delay(60);
    HAL_UART_Transmit(&huart5, (uint8_t *)save_cmd, sizeof(save_cmd), 20);
    HAL_Delay(60);
#endif
}

void HWT101_UART_Init(void)
{
    frame_index = 0;
    hwt_ready = 0;
    reset_pending = 1;
    last_frame_tick = 0;
    angle_update_count = 0;
    HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    HWT101_SetOutputRate100Hz();
}

void HWT101_getAngle_Reset(void)
{
    if (hwt_ready)
    {
        yaw_reference = yaw_absolute;
        Angle = 0.0f;
        reset_pending = 0;
    }
    else
    {
        reset_pending = 1;
    }
}

uint8_t HWT101_IsReady(void)
{
    return hwt_ready && (HWT101_GetFrameAgeMs() <= HWT101_FRAME_TIMEOUT_MS);
}

uint32_t HWT101_GetFrameAgeMs(void)
{
    if (!hwt_ready)
    {
        return 0xFFFFFFFFU;
    }

    return HAL_GetTick() - last_frame_tick;
}

uint32_t HWT101_GetUpdateCount(void)
{
    return angle_update_count;
}

float HWT101_GetAbsoluteYaw(void)
{
    return yaw_absolute;
}

void HWT101_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t checksum;
    uint8_t i;

    if (huart->Instance != UART5)
    {
        return;
    }

    if (frame_index == 0)
    {
        if (uart_rx_byte == 0x55)
        {
            frame[frame_index++] = uart_rx_byte;
        }
    }
    else
    {
        frame[frame_index++] = uart_rx_byte;
        if (frame_index == sizeof(frame))
        {
            checksum = 0;
            for (i = 0; i < 10; i++)
            {
                checksum += frame[i];
            }

            if ((frame[1] == 0x53) && (checksum == frame[10]))
            {
                int16_t raw_yaw = (int16_t)(((uint16_t)frame[7] << 8) | frame[6]);
                yaw_absolute = (float)raw_yaw * 180.0f / 32768.0f;

                if (reset_pending)
                {
                    yaw_reference = yaw_absolute;
                    reset_pending = 0;
                }

                Angle = HWT101_Normalize(yaw_absolute - yaw_reference);
                last_frame_tick = HAL_GetTick();
                hwt_ready = 1;
                angle_update_count++;
            }

            frame_index = 0;
        }
    }

    HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        HWT101_UART_RxCpltCallback(huart);
    }
    else if (huart->Instance == USART1)
    {
        GM65_OnRxComplete(huart);
    }
    else if (huart->Instance == UART4)
    {
        MaxiCam_OnRxComplete(huart);
    }
#if VOFA_TUNE_ENABLE
    else if (huart->Instance == USART2)
    {
        VofaTune_OnRxByte();
    }
#endif
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5)
    {
        frame_index = 0;
        HAL_UART_Receive_IT(&huart5, &uart_rx_byte, 1);
    }
    else if (huart->Instance == USART1)
    {
        GM65_OnUartError();
        (void)GM65_StartReceiveIT();
    }
    else if (huart->Instance == UART4)
    {
        MaxiCam_OnUartError();
    }
#if VOFA_TUNE_ENABLE
    else if (huart->Instance == USART2)
    {
        VofaTune_OnUartError();
    }
#endif
}
