#ifndef __TURNTABLE_INVENTORY_H__
#define __TURNTABLE_INVENTORY_H__

#include <stdint.h>

#define TURNTABLE_SLOT_COUNT        5U
#define TURNTABLE_COLOR_CODE_MIN   21U
#define TURNTABLE_COLOR_CODE_MAX   25U
#define TURNTABLE_TROPHY_CODE      26U

typedef enum
{
    TURNTABLE_ITEM_EMPTY = 0,
    TURNTABLE_ITEM_COLOR,
    TURNTABLE_ITEM_TROPHY
} TurntableItemType;

typedef struct
{
    uint8_t valid;
    TurntableItemType type;
    uint8_t item_code;
    uint8_t capture_order;
    uint16_t pwm_us;
} TurntableSlot;

/* Configure the mechanical zero and the PWM increment between adjacent slots,
 * then clear all remembered items.  Memory is retained only until reset or
 * power-off; no flash is written. */
void TurntableInventory_Init(uint16_t home_pwm_us, uint16_t step_pwm_us);
void TurntableInventory_Reset(void);

/* Record an item in the current sequential capture slot.  On success,
 * slot_index receives the slot that was at the collection entrance. */
uint8_t TurntableInventory_RecordColor(uint8_t color_code,
                                       uint8_t *slot_index);
uint8_t TurntableInventory_RecordTrophy(uint8_t *slot_index);
uint8_t TurntableInventory_RecordTrophyRank(uint8_t trophy_rank,
                                            uint8_t *slot_index);

/* Query helpers used later when selecting a stored item for placement. */
uint8_t TurntableInventory_GetSlot(uint8_t slot_index,
                                   TurntableSlot *slot);
uint8_t TurntableInventory_FindColor(uint8_t color_code,
                                     uint8_t *slot_index,
                                     uint16_t *pwm_us);
uint8_t TurntableInventory_FindTrophyRank(uint8_t trophy_rank,
                                          uint8_t *slot_index,
                                          uint16_t *pwm_us);
uint8_t TurntableInventory_MarkEmpty(uint8_t slot_index);

uint8_t TurntableInventory_GetLoadedCount(void);
uint8_t TurntableInventory_GetCapturedCount(void);
uint8_t TurntableInventory_HasFreeCaptureSlot(void);
uint16_t TurntableInventory_GetNextCapturePwm(void);
uint16_t TurntableInventory_GetPwmForSlot(uint8_t slot_index);

#endif
