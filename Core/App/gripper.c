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

/* ---- Reed switch helpers ----
 *
 * Reads the cached hw.in_reed_* values (written by HW_RefreshIO inside the
 * TIM6 ISR). Kicks the IWDG so a long gripper travel doesn't trigger a reset.
 */
static void wait_for_reed(volatile uint8_t *reed, const char *name)
{
    uint32_t t0 = HAL_GetTick();
    while (!(*reed)) {
        IWDG->KR = 0xAAAAU;
        if ((HAL_GetTick() - t0) >= REED_SW_TIMEOUT_MS) {
            printf("Reed SW timeout: %s\r\n", name);
            return;
        }
    }
    printf("Reed SW OK: %s\r\n", name);
}

/* ---- Sequences ---- */

void Gripper_Sequence_Pick(void)
{
    printf("Sequence: PICK\r\n");
    Gripper_Open();  wait_for_reed(&hw.in_reed_open,  "OPEN");
    Gripper_Down();  wait_for_reed(&hw.in_reed_down,  "DOWN");
    Gripper_Close(); wait_for_reed(&hw.in_reed_close, "CLOSE");
    Gripper_Up();    wait_for_reed(&hw.in_reed_up,    "UP");
    printf("Sequence PICK: Done\r\n");
}

void Gripper_Sequence_Place(void)
{
    printf("Sequence: PLACE\r\n");
    Gripper_Down();  wait_for_reed(&hw.in_reed_down,  "DOWN");
    Gripper_Open();  wait_for_reed(&hw.in_reed_open,  "OPEN");
    Gripper_Up();    wait_for_reed(&hw.in_reed_up,    "UP");
    Gripper_Close(); wait_for_reed(&hw.in_reed_close, "CLOSE");
    printf("Sequence PLACE: Done\r\n");
}
