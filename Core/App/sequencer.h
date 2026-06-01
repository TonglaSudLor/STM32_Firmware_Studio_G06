/**
 * @file sequencer.h
 * @brief Pick-and-Place State Machine — non-blocking sequencer.
 *
 * TODO: Extract from Core/Src/modbus_bridge.c
 *   - PnP_State_t enum
 *   - PnP_StateMachine() function
 *   - pnp_state, pnp_total_pairs, pnp_current_pair, pnp_grip_enable variables
 *   - motor_at_target() helper
 *
 * When done: add Core/App/sequencer.c
 *   Rule: sequencer may call Motor_MoveToPosition() and Gripper_*() only.
 *         Must not touch motor_controller internals directly.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SEQ_IDLE = 0,
    SEQ_MOVE_TO_PICK,
    SEQ_WAIT_PICK_ARRIVAL,
    SEQ_DO_PICK_OPEN, SEQ_DO_PICK_DOWN, SEQ_DO_PICK_CLOSE, SEQ_DO_PICK_UP,
    SEQ_MOVE_TO_PLACE,
    SEQ_WAIT_PLACE_ARRIVAL,
    SEQ_DO_PLACE_DOWN, SEQ_DO_PLACE_OPEN, SEQ_DO_PLACE_UP, SEQ_DO_PLACE_CLOSE,
    SEQ_NEXT_PAIR,
    SEQ_DONE
} Seq_State_t;

void        Sequencer_Init(void);
void        Sequencer_Start(uint16_t pairs, bool grip_enable);
void        Sequencer_Tick(void);      /* call from main loop every cycle */
bool        Sequencer_IsDone(void);
Seq_State_t Sequencer_GetState(void);
void        Sequencer_Abort(void);
