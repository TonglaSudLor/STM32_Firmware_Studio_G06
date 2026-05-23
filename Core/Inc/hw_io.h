/**
 * @file hw_io.h
 * @brief Hardware I/O Manager for Custom PCB
 *
 * Handles all GPIO reads (Opto inputs) and writes (Relay/Gripper/Reed outputs).
 * Provides a flat debug struct visible in STM32CubeIDE Live Expressions.
 *
 * Pinout:
 *   Inputs  (Opto):    PA5=E-Stop, PB9=Proximity, PA6=SelectMode, PA7=Reset
 *   Inputs  (Reed SW): PB0=Up, PA4=Down, PA1=Close, PA0=Open  (1=position reached)
 *   Outputs (Relay):   PB12=MotorPower, PB11=ModeLight, PB2=StatusLight
 *   Outputs (Gripper): PC0=Up/Down, PC1=Close/Open
 */

#ifndef HW_IO_H
#define HW_IO_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Hardware I/O Debug Variables
 * Add each variable individually to STM32CubeIDE Live Expressions.
 * ============================================================================ */

/* ============================================================================
 * Hardware I/O Debug Struct
 * Add "hw" to STM32CubeIDE Live Expressions to expand and view/edit all I/O.
 * ============================================================================ */
typedef struct {
    /* --- Inputs (auto-updated every 100Hz, read-only) --- */
    volatile uint8_t in_estop;        /* PA5  E-Stop button via Opto CH1  (1=pressed) */
    volatile uint8_t in_proximity;    /* PB9  Proximity sensor via Opto CH2 (1=detected) */
    volatile uint8_t in_select_mode;  /* PA6  Mode switch via Opto CH3    (1=Joystick, 0=Base) */
    volatile uint8_t in_reset_btn;    /* PA7  Reset button via Opto CH4   (1=pressed) */
    volatile uint8_t raw_prox_bit;    /* PA6  Raw bit state (0 or 1) for debugging */
    volatile uint8_t sanity_check;   /* Should be 0xAA (170) if code is updated */

    /* --- Relay Outputs --- */
    volatile uint8_t out_relay_motor;  /* PB12 Relay CH1: Motor Power      (1=ON, 0=OFF) */
    volatile uint8_t out_relay_mode;   /* PB11 Relay CH2: Mode Lamp        (1=Joystick Blue, 0=Base Blue) */
    volatile uint8_t out_relay_status; /* PB2  Relay CH3: Status Lamp      (1=Red/Emergency, 0=Green/Ready) */

    /* --- Gripper Outputs --- */
    volatile uint8_t out_gripper_ud;  /* PC0  Relay CH4: Up/Down          (1=Down, 0=Up) */
    volatile uint8_t out_gripper_co;  /* PC1  Relay CH5: Close/Open       (1=Close, 0=Open) */

    /* --- Reed Switch Inputs (gripper position feedback) --- */
    volatile uint8_t in_reed_up;      /* PB0  Reed SW Up    (1=gripper is UP) */
    volatile uint8_t in_reed_down;    /* PA4  Reed SW Down  (1=gripper is DOWN) */
    volatile uint8_t in_reed_close;   /* PA1  Reed SW Close (1=claw is CLOSED) */
    volatile uint8_t in_reed_open;    /* PA0  Reed SW Open  (1=claw is OPEN) */

    /* --- Motor Driver Status (read-only) --- */
    volatile uint8_t out_motor_dir;   /* PA9  Direction pin state         (1=Forward, 0=Reverse) */

    /* --- Override Control --- */
    volatile uint8_t override_enabled; /* Set to 1 to manually force outputs from Live Expressions */
} HW_Debug_t;

extern HW_Debug_t hw;

/* ============================================================================
 * Public Functions
 * ============================================================================ */

/* Reed switch active logic: 1 = active-HIGH, 0 = active-LOW. Flip if wired inverted. */
#define REED_SW_ACTIVE_LEVEL   1

/* Max milliseconds to wait for a reed switch to trigger before continuing anyway. */
#define REED_SW_TIMEOUT_MS     3000

/**
 * @brief Initialize all output pins to their default safe states.
 *        Call once from Motor_Init() or main().
 */
void HW_Init(void);

/**
 * @brief Refresh all hardware I/O. Call at 100Hz from Motor_ControlLoop().
 *        Reads all Opto inputs into hw_in_* variables.
 *        Writes all hw_out_* variables to their physical GPIO pins.
 */
void HW_RefreshIO(void);

/**
 * @brief Emergency Stop handler. Call from EXTI callback (PA5 interrupt).
 *        Immediately cuts motor power relay and activates status lamp.
 */
void HW_EStop_Trigger(void);

/**
 * @brief Clear Emergency Stop state. Call when reset button is pressed.
 */
void HW_EStop_Clear(void);

#endif /* HW_IO_H */
