/**
 * @file sequencer.c
 * @brief Pick-and-Place State Machine — Implementation.
 */

#include "sequencer.h"
#include "gripper.h"
#include "motor_controller.h"
#include <stdio.h>

static struct {
    PnP_State_t state;
    uint16_t pairs_remaining;
    uint32_t wait_start_tick;
    bool grip_enabled;
} seq;

void Sequencer_Init(void)
{
    seq.state = PNP_IDLE;
    seq.pairs_remaining = 0;
}

void Sequencer_Start(uint16_t pairs, bool grip_enable)
{
    seq.pairs_remaining = pairs;
    seq.grip_enabled = grip_enable;
    seq.state = PNP_MOVE_TO_PICK;
    printf("[SEQ] Starting sequence: %u pairs\r\n", pairs);
}

void Sequencer_Tick(void)
{
    // Implementation will be moved from modbus_bridge.c
    // For now, this is a placeholder to complete Phase B file structure.
}

PnP_State_t Sequencer_GetState(void)
{
    return seq.state;
}

bool Sequencer_IsDone(void)
{
    return seq.state == PNP_IDLE;
}
