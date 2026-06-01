/**
 * @file gripper.h
 * @brief Gripper Control — relay output abstraction.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - Gripper_Up/Down/Open/Close functions
 *   - Gripper_Sequence_Pick / Gripper_Sequence_Place
 *   - gripper_seq_request deferred sequence logic (bug 1-F)
 *
 * Hardware (from hw_io.h):
 *   PA1  out_gripper_up   1=UP relay ON   → arm goes UP   (spring → DOWN)
 *   PA4  out_gripper_down 1=CLOSE relay ON → claw CLOSES  (spring → OPEN)
 *
 * All 4 states (UP/DOWN × OPEN/CLOSE) are independently controllable.
 *
 * When done: add Core/App/gripper.c
 */

#pragma once

void Gripper_Up(void);
void Gripper_Down(void);
void Gripper_Open(void);
void Gripper_Close(void);
void Gripper_Toggle(void);
void Gripper_Sequence_Pick(void);
void Gripper_Sequence_Place(void);
