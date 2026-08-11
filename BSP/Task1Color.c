#include "Task1Color.h"
#include "Servo4.h"
#include "MaxiCam.h"
#include "TurntableInventory.h"

typedef enum {
    TASK1_COLOR_RED   = 21,
    TASK1_COLOR_BLUE  = 22,
    TASK1_COLOR_GREEN = 23,
    TASK1_COLOR_WHITE = 24,
    TASK1_COLOR_BLACK = 25
} Task1Color;

#define TASK1_COLOR_COUNT      5U
#define TASK1_FRAME_MAX_AGE  250U

static uint8_t active;
static uint8_t count;
static uint8_t seen[TASK1_COLOR_COUNT];
static uint32_t last_sequence;

static uint8_t ColorToIndex(uint8_t code)
{
    if ((code < TASK1_COLOR_RED) || (code > TASK1_COLOR_BLACK))
        return 0xFF;
    return code - TASK1_COLOR_RED;
}

void Task1Color_Start(void)
{
    uint8_t i;
    active = 1U;
    count = 0U;
    for (i = 0U; i < TASK1_COLOR_COUNT; i++)
        seen[i] = 0U;
    last_sequence = 0U;
    Servo4_RotationHome();
    (void)MaxiCam_SetMode(MAXICAM_MODE_COLOR);
}

void Task1Color_Stop(void)
{
    if (active)
        (void)MaxiCam_SetMode(MAXICAM_MODE_STOP);
    active = 0U;
}

void Task1Color_Process(void)
{
    MaxiCam_Frame frame;
    uint8_t idx;

    if (!active) return;
    if (count >= TASK1_COLOR_COUNT) return;

    if (!MaxiCam_GetLatest(&frame, TASK1_FRAME_MAX_AGE)) return;
    if (frame.sequence == last_sequence) return;
    last_sequence = frame.sequence;

    idx = ColorToIndex(frame.data[0]);
    if (idx >= TASK1_COLOR_COUNT) return;
    if (seen[idx]) return;

    {
        uint8_t captured_slot;
        if (!TurntableInventory_RecordColor(frame.data[0], &captured_slot))
            return;
    }

    seen[idx] = 1U;
    count++;
    Servo4_RotationSetStep(count);

    if (count >= TASK1_COLOR_COUNT)
    {
        active = 0U;
        (void)MaxiCam_SetMode(MAXICAM_MODE_CIRCLE);
    }
}
