#include "QrMissionMap.h"

/* Rows are QR numbers 1..16; columns are task-1 locations A..E. */
static const uint8_t task1_color_map[QR_TASK1_CODE_MAX]
                                    [QR_TASK1_PLACE_COUNT] =
{
    { QR_COLOR_BLACK, QR_COLOR_WHITE, QR_COLOR_RED,   QR_COLOR_GREEN, QR_COLOR_BLUE  },
    { QR_COLOR_WHITE, QR_COLOR_BLACK, QR_COLOR_RED,   QR_COLOR_GREEN, QR_COLOR_BLUE  },
    { QR_COLOR_WHITE, QR_COLOR_BLACK, QR_COLOR_GREEN, QR_COLOR_RED,   QR_COLOR_BLUE  },
    { QR_COLOR_BLUE,  QR_COLOR_WHITE, QR_COLOR_BLACK, QR_COLOR_RED,   QR_COLOR_GREEN },
    { QR_COLOR_WHITE, QR_COLOR_RED,   QR_COLOR_BLUE,  QR_COLOR_BLACK, QR_COLOR_GREEN },
    { QR_COLOR_BLACK, QR_COLOR_RED,   QR_COLOR_BLUE,  QR_COLOR_WHITE, QR_COLOR_GREEN },
    { QR_COLOR_BLUE,  QR_COLOR_GREEN, QR_COLOR_BLACK, QR_COLOR_WHITE, QR_COLOR_RED   },
    { QR_COLOR_GREEN, QR_COLOR_WHITE, QR_COLOR_BLUE,  QR_COLOR_BLACK, QR_COLOR_RED   },
    { QR_COLOR_WHITE, QR_COLOR_GREEN, QR_COLOR_BLACK, QR_COLOR_BLUE,  QR_COLOR_RED   },
    { QR_COLOR_BLACK, QR_COLOR_RED,   QR_COLOR_BLUE,  QR_COLOR_GREEN, QR_COLOR_WHITE },
    { QR_COLOR_RED,   QR_COLOR_BLUE,  QR_COLOR_GREEN, QR_COLOR_BLACK, QR_COLOR_WHITE },
    { QR_COLOR_GREEN, QR_COLOR_RED,   QR_COLOR_BLACK, QR_COLOR_BLUE,  QR_COLOR_WHITE },
    { QR_COLOR_WHITE, QR_COLOR_RED,   QR_COLOR_BLUE,  QR_COLOR_GREEN, QR_COLOR_BLACK },
    { QR_COLOR_RED,   QR_COLOR_GREEN, QR_COLOR_WHITE, QR_COLOR_BLUE,  QR_COLOR_BLACK },
    { QR_COLOR_BLUE,  QR_COLOR_WHITE, QR_COLOR_GREEN, QR_COLOR_RED,   QR_COLOR_BLACK },
    { QR_COLOR_GREEN, QR_COLOR_BLUE,  QR_COLOR_RED,   QR_COLOR_WHITE, QR_COLOR_BLACK }
};

/* Rows are QR numbers 1..6; columns are the three task-2 placements. */
static const QrTask2Point task2_point_map[QR_TASK2_CODE_MAX]
                                             [QR_TASK2_PLACE_COUNT] =
{
    { QR_TASK2_POINT_A, QR_TASK2_POINT_B, QR_TASK2_POINT_C },
    { QR_TASK2_POINT_A, QR_TASK2_POINT_C, QR_TASK2_POINT_B },
    { QR_TASK2_POINT_B, QR_TASK2_POINT_A, QR_TASK2_POINT_C },
    { QR_TASK2_POINT_B, QR_TASK2_POINT_C, QR_TASK2_POINT_A },
    { QR_TASK2_POINT_C, QR_TASK2_POINT_A, QR_TASK2_POINT_B },
    { QR_TASK2_POINT_C, QR_TASK2_POINT_B, QR_TASK2_POINT_A }
};

uint8_t QrMissionMap_GetTask1Color(uint8_t qr_code,
                                   uint8_t place_index)
{
    if ((qr_code < QR_TASK1_CODE_MIN) ||
        (qr_code > QR_TASK1_CODE_MAX) ||
        (place_index >= QR_TASK1_PLACE_COUNT))
        return QR_COLOR_INVALID;

    return task1_color_map[qr_code - QR_TASK1_CODE_MIN][place_index];
}

QrTask2Point QrMissionMap_GetTask2Point(uint8_t qr_code,
                                       uint8_t sequence_index)
{
    if ((qr_code < QR_TASK2_CODE_MIN) ||
        (qr_code > QR_TASK2_CODE_MAX) ||
        (sequence_index >= QR_TASK2_PLACE_COUNT))
        return QR_TASK2_POINT_INVALID;

    return task2_point_map[qr_code - QR_TASK2_CODE_MIN][sequence_index];
}

char QrMissionMap_Task2PointToChar(QrTask2Point point)
{
    if (point == QR_TASK2_POINT_A) return 'A';
    if (point == QR_TASK2_POINT_B) return 'B';
    if (point == QR_TASK2_POINT_C) return 'C';
    return '?';
}
