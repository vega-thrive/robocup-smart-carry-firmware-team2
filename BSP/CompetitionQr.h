#ifndef __COMPETITION_QR_H__
#define __COMPETITION_QR_H__

#include "main.h"
#include "QrMissionMap.h"

typedef enum
{
    COMP_QR_TASK1 = 1,
    COMP_QR_TASK2 = 2
} CompetitionQrTask;

typedef enum
{
    COMP_QR_IDLE = 0,
    COMP_QR_SCANNING,
    COMP_QR_READY,
    COMP_QR_DEFAULTED
} CompetitionQrStatus;


void CompetitionQr_Init(UART_HandleTypeDef *uart);
void CompetitionQr_RequestStart(CompetitionQrTask task);
void CompetitionQr_Process(void);
CompetitionQrStatus CompetitionQr_GetStatus(void);
uint8_t CompetitionQr_GetTaskCode(CompetitionQrTask task);
uint8_t CompetitionQr_GetAttemptCount(void);
uint8_t CompetitionQr_GetTask1PlaceColor(uint8_t task_code,
                                         uint8_t place_index);
QrTask2Trophy CompetitionQr_GetTask2CaptureTrophy(uint8_t task_code,
                                                  uint8_t capture_index);
QrTask2Trophy CompetitionQr_GetTask2PlaceTrophy(uint8_t place_index);

#endif
