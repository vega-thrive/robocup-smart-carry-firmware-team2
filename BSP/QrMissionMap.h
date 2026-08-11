#ifndef __QR_MISSION_MAP_H__
#define __QR_MISSION_MAP_H__

#include <stdint.h>

#define QR_TASK1_CODE_MIN       1U
#define QR_TASK1_CODE_MAX      16U
#define QR_TASK1_PLACE_COUNT    5U

#define QR_TASK2_CODE_MIN       1U
#define QR_TASK2_CODE_MAX       6U
#define QR_TASK2_PLACE_COUNT    3U

/* Keep these values consistent with the MaxiCam color protocol. */
#define QR_COLOR_RED           21U
#define QR_COLOR_BLUE          22U
#define QR_COLOR_GREEN         23U
#define QR_COLOR_WHITE         24U
#define QR_COLOR_BLACK         25U
#define QR_COLOR_INVALID        0U

typedef enum
{
    QR_TASK2_POINT_A = 0,
    QR_TASK2_POINT_B,
    QR_TASK2_POINT_C,
    QR_TASK2_POINT_INVALID = 0xFF
} QrTask2Point;

uint8_t QrMissionMap_GetTask1Color(uint8_t qr_code,
                                   uint8_t place_index);
QrTask2Point QrMissionMap_GetTask2Point(uint8_t qr_code,
                                       uint8_t sequence_index);
char QrMissionMap_Task2PointToChar(QrTask2Point point);

#endif
