#include "TurntableInventory.h"
#include <stddef.h>

static TurntableSlot turntable_slots[TURNTABLE_SLOT_COUNT];
static uint16_t turntable_home_pwm_us;
static uint16_t turntable_step_pwm_us;
static uint8_t turntable_next_capture_slot;
static uint8_t turntable_loaded_count;
static uint8_t turntable_captured_count;

static uint16_t TurntableInventory_CalculatePwm(uint8_t slot_index)
{
    uint32_t pwm = (uint32_t)turntable_home_pwm_us
                 + (uint32_t)turntable_step_pwm_us * slot_index;

    if (pwm > 0xFFFFU) pwm = 0xFFFFU;
    return (uint16_t)pwm;
}

static uint8_t TurntableInventory_Record(TurntableItemType type,
                                         uint8_t item_code,
                                         uint8_t *slot_index)
{
    TurntableSlot *slot;
    uint8_t index;

    if (turntable_next_capture_slot >= TURNTABLE_SLOT_COUNT) return 0U;

    index = turntable_next_capture_slot;
    slot = &turntable_slots[index];
    if (slot->valid) return 0U;

    slot->valid = 1U;
    slot->type = type;
    slot->item_code = item_code;
    slot->capture_order = (uint8_t)(turntable_captured_count + 1U);

    turntable_next_capture_slot++;
    turntable_captured_count++;
    turntable_loaded_count++;

    if (slot_index != NULL) *slot_index = index;
    return 1U;
}

void TurntableInventory_Init(uint16_t home_pwm_us, uint16_t step_pwm_us)
{
    turntable_home_pwm_us = home_pwm_us;
    turntable_step_pwm_us = step_pwm_us;
    TurntableInventory_Reset();
}

void TurntableInventory_Reset(void)
{
    uint8_t i;

    for (i = 0U; i < TURNTABLE_SLOT_COUNT; i++)
    {
        turntable_slots[i].valid = 0U;
        turntable_slots[i].type = TURNTABLE_ITEM_EMPTY;
        turntable_slots[i].item_code = 0U;
        turntable_slots[i].capture_order = 0U;
        turntable_slots[i].pwm_us = TurntableInventory_CalculatePwm(i);
    }

    turntable_next_capture_slot = 0U;
    turntable_loaded_count = 0U;
    turntable_captured_count = 0U;
}

uint8_t TurntableInventory_RecordColor(uint8_t color_code,
                                       uint8_t *slot_index)
{
    if ((color_code < TURNTABLE_COLOR_CODE_MIN) ||
        (color_code > TURNTABLE_COLOR_CODE_MAX))
        return 0U;

    return TurntableInventory_Record(TURNTABLE_ITEM_COLOR,
                                     color_code,
                                     slot_index);
}

uint8_t TurntableInventory_RecordTrophy(uint8_t *slot_index)
{
    return TurntableInventory_Record(TURNTABLE_ITEM_TROPHY,
                                     TURNTABLE_TROPHY_CODE,
                                     slot_index);
}

uint8_t TurntableInventory_RecordTrophyRank(uint8_t trophy_rank,
                                            uint8_t *slot_index)
{
    /* trophy_rank: 0=冠军, 1=亚军, 2=季军, 与 QrTask2Trophy 对齐 */
    if (trophy_rank > 2U) return 0U;
    return TurntableInventory_Record(TURNTABLE_ITEM_TROPHY,
                                     trophy_rank,
                                     slot_index);
}

uint8_t TurntableInventory_GetSlot(uint8_t slot_index,
                                   TurntableSlot *slot)
{
    if ((slot_index >= TURNTABLE_SLOT_COUNT) || (slot == NULL)) return 0U;
    *slot = turntable_slots[slot_index];
    return 1U;
}

uint8_t TurntableInventory_FindColor(uint8_t color_code,
                                     uint8_t *slot_index,
                                     uint16_t *pwm_us)
{
    uint8_t i;

    for (i = 0U; i < TURNTABLE_SLOT_COUNT; i++)
    {
        if (turntable_slots[i].valid &&
            (turntable_slots[i].type == TURNTABLE_ITEM_COLOR) &&
            (turntable_slots[i].item_code == color_code))
        {
            if (slot_index != NULL) *slot_index = i;
            if (pwm_us != NULL) *pwm_us = turntable_slots[i].pwm_us;
            return 1U;
        }
    }

    return 0U;
}

uint8_t TurntableInventory_FindTrophyRank(uint8_t trophy_rank,
                                          uint8_t *slot_index,
                                          uint16_t *pwm_us)
{
    uint8_t i;

    for (i = 0U; i < TURNTABLE_SLOT_COUNT; i++)
    {
        if (turntable_slots[i].valid &&
            (turntable_slots[i].type == TURNTABLE_ITEM_TROPHY) &&
            (turntable_slots[i].item_code == trophy_rank))
        {
            if (slot_index != NULL) *slot_index = i;
            if (pwm_us != NULL) *pwm_us = turntable_slots[i].pwm_us;
            return 1U;
        }
    }

    return 0U;
}

uint8_t TurntableInventory_MarkEmpty(uint8_t slot_index)
{
    TurntableSlot *slot;

    if (slot_index >= TURNTABLE_SLOT_COUNT) return 0U;
    slot = &turntable_slots[slot_index];
    if (!slot->valid) return 0U;

    slot->valid = 0U;
    slot->type = TURNTABLE_ITEM_EMPTY;
    slot->item_code = 0U;
    slot->capture_order = 0U;
    if (turntable_loaded_count > 0U) turntable_loaded_count--;
    return 1U;
}

uint8_t TurntableInventory_GetLoadedCount(void)
{
    return turntable_loaded_count;
}

uint8_t TurntableInventory_GetCapturedCount(void)
{
    return turntable_captured_count;
}

uint8_t TurntableInventory_HasFreeCaptureSlot(void)
{
    return (turntable_next_capture_slot < TURNTABLE_SLOT_COUNT) ? 1U : 0U;
}

uint16_t TurntableInventory_GetNextCapturePwm(void)
{
    if (!TurntableInventory_HasFreeCaptureSlot()) return 0U;
    return turntable_slots[turntable_next_capture_slot].pwm_us;
}

uint16_t TurntableInventory_GetPwmForSlot(uint8_t slot_index)
{
    if (slot_index >= TURNTABLE_SLOT_COUNT) return 0U;
    return turntable_slots[slot_index].pwm_us;
}
