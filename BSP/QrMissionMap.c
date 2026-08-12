#include "QrMissionMap.h"

/* =================================================================== *
 *  任务1: 颜色映射 (列顺序 A, B, C, D, E)
 * =================================================================== */
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

/* 车辆实际经过放置点的顺序: A→B→C→E→D, 映射到表格列索引 */
static const uint8_t task1_route_to_table_index[QR_TASK1_PLACE_COUNT] =
{
    0U, /* A */
    1U, /* B */
    2U, /* C */
    4U, /* E */
    3U  /* D */
};

/* =================================================================== *
 *  任务2: 奖杯映射 (列顺序 位置1, 位置2, 位置3)
 * =================================================================== */
static const QrTask2Trophy task2_trophy_map[QR_TASK2_CODE_MAX]
                                            [QR_TASK2_COLLECT_COUNT] =
{
    { QR_TASK2_TROPHY_CHAMPION,    QR_TASK2_TROPHY_RUNNER_UP,   QR_TASK2_TROPHY_THIRD_PLACE },
    { QR_TASK2_TROPHY_CHAMPION,    QR_TASK2_TROPHY_THIRD_PLACE, QR_TASK2_TROPHY_RUNNER_UP   },
    { QR_TASK2_TROPHY_RUNNER_UP,   QR_TASK2_TROPHY_CHAMPION,    QR_TASK2_TROPHY_THIRD_PLACE },
    { QR_TASK2_TROPHY_RUNNER_UP,   QR_TASK2_TROPHY_THIRD_PLACE, QR_TASK2_TROPHY_CHAMPION    },
    { QR_TASK2_TROPHY_THIRD_PLACE, QR_TASK2_TROPHY_CHAMPION,    QR_TASK2_TROPHY_RUNNER_UP   },
    { QR_TASK2_TROPHY_THIRD_PLACE, QR_TASK2_TROPHY_RUNNER_UP,   QR_TASK2_TROPHY_CHAMPION    }
};

/* 车辆收集经过顺序固定: 位置3→位置2→位置1, 倒序映射 */
static const uint8_t task2_capture_to_table_index[QR_TASK2_COLLECT_COUNT] =
{
    2U, /* 位置3 */
    1U, /* 位置2 */
    0U  /* 位置1 */
};

/* 奖杯放置顺序固定: 亚军→冠军→季军 (B→A→C) */
static const QrTask2Trophy task2_place_order[QR_TASK2_PLACE_COUNT] =
{
    QR_TASK2_TROPHY_RUNNER_UP,    /* B */
    QR_TASK2_TROPHY_CHAMPION,     /* A */
    QR_TASK2_TROPHY_THIRD_PLACE   /* C */
};

/* =================================================================== *
 *  查询接口
 * =================================================================== */

uint8_t QrMissionMap_GetTask1RouteColor(uint8_t qr_code,
                                        uint8_t place_index)
{
    uint8_t table_index;

    if ((qr_code < QR_TASK1_CODE_MIN) ||
        (qr_code > QR_TASK1_CODE_MAX) ||
        (place_index >= QR_TASK1_PLACE_COUNT))
        return QR_COLOR_INVALID;

    table_index = task1_route_to_table_index[place_index];
    return task1_color_map[qr_code - QR_TASK1_CODE_MIN][table_index];
}

QrTask2Trophy QrMissionMap_GetTask2CaptureTrophy(uint8_t qr_code,
                                                  uint8_t capture_index)
{
    uint8_t table_index;

    if ((qr_code < QR_TASK2_CODE_MIN) ||
        (qr_code > QR_TASK2_CODE_MAX) ||
        (capture_index >= QR_TASK2_COLLECT_COUNT))
        return QR_TASK2_TROPHY_INVALID;

    table_index = task2_capture_to_table_index[capture_index];
    return task2_trophy_map[qr_code - QR_TASK2_CODE_MIN][table_index];
}

QrTask2Trophy QrMissionMap_GetTask2PlaceTrophy(uint8_t place_index)
{
    if (place_index >= QR_TASK2_PLACE_COUNT)
        return QR_TASK2_TROPHY_INVALID;

    return task2_place_order[place_index];
}

char QrMissionMap_Task2TrophyToChar(QrTask2Trophy trophy)
{
    if (trophy == QR_TASK2_TROPHY_CHAMPION)    return 'A';
    if (trophy == QR_TASK2_TROPHY_RUNNER_UP)   return 'B';
    if (trophy == QR_TASK2_TROPHY_THIRD_PLACE) return 'C';
    return '?';
}
