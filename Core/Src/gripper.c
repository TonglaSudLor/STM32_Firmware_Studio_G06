/**
 * @file gripper.c
 * @brief Gripper relay control — extracted from motor_controller.c.
 *
 * Communicates with hardware exclusively through hw.out_gripper_* fields.
 * HW_RefreshIO() (called at 100 Hz in TIM6 ISR) writes those fields to GPIO.
 *
 * Sequences run at thread level (main loop) because they block on reed switches.
 * NEVER call sequence functions from an ISR context.
 *
 * Hardware:
 *   hw.out_gripper_up   (PA1): 1 = relay ON → arm UP   (spring-return → DOWN on 0)
 *   hw.out_gripper_down (PA4): 1 = relay ON → claw CLOSE (spring-return → OPEN on 0)
 */

#include "gripper.h"
#include "hw_io.h"
#include "main.h"
#include <stdio.h>

/* ---- Primitive relay commands ---- */

void Gripper_Up(void)    { hw.out_gripper_up   = 1; printf("Gripper: UP\r\n");   }
void Gripper_Down(void)  { hw.out_gripper_up   = 0; printf("Gripper: DOWN\r\n"); }
void Gripper_Open(void)  { hw.out_gripper_down = 0; printf("Claw: OPEN\r\n");    }
void Gripper_Close(void) { hw.out_gripper_down = 1; printf("Claw: CLOSE\r\n");   }

void Gripper_Toggle(void)
{
    static bool is_open = true;
    if (is_open) Gripper_Close();
    else         Gripper_Open();
    is_open = !is_open;
}

/* Gripper_Sequence_Pick / Gripper_Sequence_Place live in motor_controller.c
 * because they need task_pick_active / task_place_active flags that
 * modbus_bridge.c reads, and the conditional wait_for_reed_if_joystick logic. */
