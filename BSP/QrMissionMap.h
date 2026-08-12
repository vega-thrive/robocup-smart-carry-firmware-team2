#ifndef __QR_MISSION_MAP_H__
#define __QR_MISSION_MAP_H__

#include <stdint.h>

#define QR_TASK1_CODE_MIN       1U
#define QR_TASK1_CODE_MAX      16U
#define QR_TASK1_PLACE_COUNT    5U

#define QR_TASK2_CODE_MIN       1U
#define QR_TASK2_CODE_MAX       6U
#define QR_TASK2_COLLECT_COUNT  3U
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
    QR_TASK2_TROPHY_CHAMPION = 0,  /* A */
    QR_TASK2_TROPHY_RUNNER_UP,    /* B */
    QR_TASK2_TROPHY_THIRD_PLACE,  /* C */
    QR_TASK2_TROPHY_INVALID = 0xFF
} QrTask2Trophy;

/* 任务1: 按车辆实际经过顺序 A→B→C→E→D 查询目标颜色 */
uint8_t QrMissionMap_GetTask1RouteColor(uint8_t qr_code,
                                        uint8_t place_index);

/* 任务2: 车辆经过顺序固定为 位置3→位置2→位置1, 倒序推断奖杯等级 */
QrTask2Trophy QrMissionMap_GetTask2CaptureTrophy(uint8_t qr_code,
                                                  uint8_t capture_index);

/* 任务2: 放置顺序固定为 亚军→冠军→季军 (B→A→C) */
QrTask2Trophy QrMissionMap_GetTask2PlaceTrophy(uint8_t place_index);

char QrMissionMap_Task2TrophyToChar(QrTask2Trophy trophy);

#endif
