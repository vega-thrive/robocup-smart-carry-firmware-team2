#include "CompetitionQr.h"
#include "GM65Scanner.h"
#include "QrMissionMap.h"

#define COMP_QR_ATTEMPT_TIMEOUT_MS 1000U
#define COMP_QR_MAX_ATTEMPTS          3U

static volatile CompetitionQrTask competition_qr_task;
static volatile CompetitionQrStatus competition_qr_status;
static volatile uint8_t competition_qr_code;
static volatile uint8_t competition_qr_attempts;
static volatile uint8_t competition_qr_task_codes[2];
static volatile uint8_t competition_qr_task_defaulted[2];
static volatile uint8_t competition_qr_start_pending;

static uint8_t CompetitionQr_ParseCode(const char *text, uint8_t *code)
{
    uint16_t value = 0U;
    uint8_t digits = 0U;

    if ((text == NULL) || (code == NULL)) return 0U;
    while (*text != '\0')
    {
        if ((*text < '0') || (*text > '9') || (digits >= 2U)) return 0U;
        value = (uint16_t)(value * 10U + (uint16_t)(*text - '0'));
        digits++;
        text++;
    }
    if (digits == 0U) return 0U;
    if ((competition_qr_task == COMP_QR_TASK1) &&
        ((value < 1U) || (value > 16U))) return 0U;
    if ((competition_qr_task == COMP_QR_TASK2) &&
        ((value < 1U) || (value > 6U))) return 0U;
    *code = (uint8_t)value;
    return 1U;
}

static HAL_StatusTypeDef CompetitionQr_BeginAttempt(void)
{
    competition_qr_attempts++;
    return GM65_StartScan(COMP_QR_ATTEMPT_TIMEOUT_MS);
}

void CompetitionQr_Init(UART_HandleTypeDef *uart)
{
    competition_qr_status = COMP_QR_IDLE;
    competition_qr_code = 0U;
    competition_qr_attempts = 0U;
    competition_qr_task_codes[0] = 0U;
    competition_qr_task_codes[1] = 0U;
    competition_qr_task_defaulted[0] = 0U;
    competition_qr_task_defaulted[1] = 0U;
    competition_qr_start_pending = 0U;
    GM65_Init(uart);
    (void)GM65_StartReceiveIT();
}

void CompetitionQr_RequestStart(CompetitionQrTask task)
{
    if ((task != COMP_QR_TASK1) && (task != COMP_QR_TASK2)) return;
    competition_qr_task = task;
    competition_qr_status = COMP_QR_SCANNING;
    competition_qr_code = 0U;
    competition_qr_attempts = 0U;
    competition_qr_task_codes[(uint8_t)task - 1U] = 0U;
    competition_qr_task_defaulted[(uint8_t)task - 1U] = 0U;
    competition_qr_start_pending = 1U;
}

void CompetitionQr_Process(void)
{
    GM65_Status scanner_status;
    char text[GM65_TEXT_CAPACITY];
    uint8_t parsed_code;

    if (competition_qr_status != COMP_QR_SCANNING) return;
    if (competition_qr_start_pending != 0U)
    {
        competition_qr_start_pending = 0U;
        if (CompetitionQr_BeginAttempt() != HAL_OK)
        {
            competition_qr_code = 1U;
            competition_qr_task_codes[(uint8_t)competition_qr_task - 1U] = 1U;
            competition_qr_task_defaulted[(uint8_t)competition_qr_task - 1U] =
                1U;
            competition_qr_status = COMP_QR_DEFAULTED;
        }
        return;
    }
    GM65_Process();
    scanner_status = GM65_GetStatus();

    if (scanner_status == GM65_READY)
    {
        if (GM65_TakeText(text, sizeof(text)) &&
            CompetitionQr_ParseCode(text, &parsed_code))
        {
            competition_qr_code = parsed_code;
            competition_qr_task_codes[(uint8_t)competition_qr_task - 1U] =
                parsed_code;
            competition_qr_task_defaulted[(uint8_t)competition_qr_task - 1U] =
                0U;
            competition_qr_status = COMP_QR_READY;
            return;
        }
        scanner_status = GM65_TIMEOUT; /* Invalid payload consumes one attempt. */
    }

    if ((scanner_status == GM65_TIMEOUT) ||
        (scanner_status == GM65_OVERFLOW))
    {
        if (competition_qr_attempts >= COMP_QR_MAX_ATTEMPTS)
        {
            competition_qr_code = 1U;
            competition_qr_task_codes[(uint8_t)competition_qr_task - 1U] = 1U;
            competition_qr_task_defaulted[(uint8_t)competition_qr_task - 1U] =
                1U;
            competition_qr_status = COMP_QR_DEFAULTED;
        }
        else if (CompetitionQr_BeginAttempt() != HAL_OK)
        {
            competition_qr_code = 1U;
            competition_qr_task_codes[(uint8_t)competition_qr_task - 1U] = 1U;
            competition_qr_task_defaulted[(uint8_t)competition_qr_task - 1U] =
                1U;
            competition_qr_status = COMP_QR_DEFAULTED;
        }
    }
}

CompetitionQrStatus CompetitionQr_GetStatus(void)
{
    return competition_qr_status;
}

uint8_t CompetitionQr_GetTaskCode(CompetitionQrTask task)
{
    if ((task != COMP_QR_TASK1) && (task != COMP_QR_TASK2)) return 0U;
    return competition_qr_task_codes[(uint8_t)task - 1U];
}

uint8_t CompetitionQr_GetAttemptCount(void)
{
    return competition_qr_attempts;
}

uint8_t CompetitionQr_GetTask1PlaceColor(uint8_t task_code,
                                         uint8_t place_index)
{
    return QrMissionMap_GetTask1RouteColor(task_code, place_index);
}

QrTask2Trophy CompetitionQr_GetTask2CaptureTrophy(uint8_t task_code,
                                                  uint8_t capture_index)
{
    return QrMissionMap_GetTask2CaptureTrophy(task_code, capture_index);
}

QrTask2Trophy CompetitionQr_GetTask2PlaceTrophy(uint8_t place_index)
{
    return QrMissionMap_GetTask2PlaceTrophy(place_index);
}
