/**
 * @file hw_io.c
 * @brief Hardware I/O Manager for Custom PCB
 *
 * All variables here are plain globals (no volatile, no struct) so that
 * STM32CubeIDE Live Expressions can reliably read and EDIT them.
 *
 * To test hardware from Live Expressions:
 *   1. Add e.g. "hw_out_relay_motor" as an expression.
 *   2. Click the value and type 1 or 0.
 *   3. The physical relay will click on the next 100Hz tick.
 */

#include "hw_io.h"
#include "motor_controller.h"
#include <stdio.h>

/* --- Hardware Debug Struct Instance --- */
HW_Debug_t hw = {
    .override_enabled = 0,
    .sanity_check = 0xAA
};

/* ============================================================================
 * Private Helper
 * ============================================================================ */
static void apply_outputs(void)
{
    /* Always write the struct values to the physical pins.
     * When override_enabled == 1, the user edits the struct manually.
     * When override_enabled == 0, the automatic logic updates the struct.
     * Either way, we must push those values to the actual hardware.
     */
    HAL_GPIO_WritePin(Relay_MotorPower_GPIO_Port, Relay_MotorPower_Pin,
                      hw.out_relay_motor  ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Relay_Sysmode_GPIO_Port, Relay_Sysmode_Pin,
                      hw.out_relay_mode   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Relay__SysStatus_GPIO_Port, Relay__SysStatus_Pin,
                      hw.out_relay_status ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Gripper_UpDown_GPIO_Port, Gripper_UpDown_Pin,
                      hw.out_gripper_ud   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Gripper_CloseOpen_GPIO_Port, Gripper_CloseOpen_Pin,
                      hw.out_gripper_co   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    /* Reed SW pins are inputs — nothing to write */
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void HW_Init(void)
{
    /* Safe default state: Motor power OFF, status lamp GREEN (ready) */
    hw.out_relay_motor  = 0; /* Motor power OFF until system is ready */
    hw.out_relay_mode   = 0; /* Base system mode lamp */
    hw.out_relay_status = 0; /* Green = System Ready */
    hw.out_gripper_ud   = 0; /* Gripper UP */
    hw.out_gripper_co   = 0; /* Gripper OPEN */
    /* Reed SW pins are inputs — no init needed */

    apply_outputs();
}

void HW_RefreshIO(void)
{
    /* --- Read all Opto inputs (1 = Active) --- */
    /* E-Stop NO contact: Healthy = Opto OFF (Pin HIGH). Emergency = Opto ON (Pin LOW) */
    hw.in_estop       = (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin)             == GPIO_PIN_RESET) ? 1 : 0;
    hw.in_proximity   = (HAL_GPIO_ReadPin(Proximity_Sensor_GPIO_Port, Proximity_Sensor_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    hw.raw_prox_bit   = (HAL_GPIO_ReadPin(Proximity_Sensor_GPIO_Port, Proximity_Sensor_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    hw.in_select_mode = (HAL_GPIO_ReadPin(Selected_Mode_GPIO_Port, Selected_Mode_Pin) == GPIO_PIN_RESET) ? 1 : 0;

    /* --- Slide switch edge detect → toggle system mode --- */
    {
        static int8_t prev_select = -1;  /* -1 = uninitialized */
        if (prev_select < 0) {
            prev_select = (int8_t)hw.in_select_mode;  /* sync on first read; no toggle */
        } else if ((int8_t)hw.in_select_mode != prev_select) {
            prev_select = (int8_t)hw.in_select_mode;
            printf("[SLIDE SW] edge detected, toggling mode\r\n");
            Mode_Toggle();
        }
    }
    hw.in_reset_btn   = (HAL_GPIO_ReadPin(Reset_Btn_GPIO_Port, Reset_Btn_Pin)       == GPIO_PIN_RESET) ? 1 : 0;

    /* --- Read Reed Switch inputs (gripper position feedback) --- */
#if REED_SW_ACTIVE_LEVEL == 1
    hw.in_reed_up    = (HAL_GPIO_ReadPin(Reed_Up_GPIO_Port,    Reed_Up_Pin)    == GPIO_PIN_SET) ? 1 : 0;
    hw.in_reed_down  = (HAL_GPIO_ReadPin(Reed_Down_GPIO_Port,  Reed_Down_Pin)  == GPIO_PIN_SET) ? 1 : 0;
    hw.in_reed_close = (HAL_GPIO_ReadPin(Reed_Close_GPIO_Port, Reed_Close_Pin) == GPIO_PIN_SET) ? 1 : 0;
    hw.in_reed_open  = (HAL_GPIO_ReadPin(Reed_Open_GPIO_Port,  Reed_Open_Pin)  == GPIO_PIN_SET) ? 1 : 0;
#else
    hw.in_reed_up    = (HAL_GPIO_ReadPin(Reed_Up_GPIO_Port,    Reed_Up_Pin)    == GPIO_PIN_RESET) ? 1 : 0;
    hw.in_reed_down  = (HAL_GPIO_ReadPin(Reed_Down_GPIO_Port,  Reed_Down_Pin)  == GPIO_PIN_RESET) ? 1 : 0;
    hw.in_reed_close = (HAL_GPIO_ReadPin(Reed_Close_GPIO_Port, Reed_Close_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    hw.in_reed_open  = (HAL_GPIO_ReadPin(Reed_Open_GPIO_Port,  Reed_Open_Pin)  == GPIO_PIN_RESET) ? 1 : 0;
#endif

    /* --- Read motor direction pin state for monitoring --- */
    hw.out_motor_dir  = (HAL_GPIO_ReadPin(Motor_Direction_GPIO_Port, Motor_Direction_Pin) == GPIO_PIN_SET) ? 1 : 0;

    /* --- Latching Emergency Logic --- */
    if (!hw.override_enabled) {
        if (hw.in_estop) {
            /* Emergency Pressed: Safe the system immediately */
            emergency_stop = true;
            fault_code |= FAULT_ESTOP_PHYSICAL;
        } else if (hw.in_reset_btn) {
            /* Reset Pressed AND Emergency is Released: Enter Ready state */
            emergency_stop = false;
            fault_code &= ~(FAULT_ESTOP_PHYSICAL | FAULT_PROX_LOST |
                            FAULT_ESTOP_JOYSTICK | FAULT_ESTOP_DASHBOARD |
                            FAULT_ESTOP_MODBUS);
        }

        /* Update Outputs based on emergency_stop state */
        if (emergency_stop) {
            hw.out_relay_status = 1; /* Red Light ON */
            hw.out_relay_motor  = 0; /* Motor Relay OFF (NO) */
        } else {
            hw.out_relay_status = 0; /* Green Light ON */
            hw.out_relay_motor  = 1; /* Motor Relay ON (NC) */
        }
    } else {
        /* In override mode, still update emergency_stop flag but don't force outputs */
        if (hw.in_estop) { emergency_stop = true; fault_code |= FAULT_ESTOP_PHYSICAL; }
        else if (hw.in_reset_btn) emergency_stop = false;
    }

    /* --- Update Mode lamp if not overridden --- */
    extern volatile Control_SystemMode_t control_system_mode;
    if (!hw.override_enabled) {
        hw.out_relay_mode = (control_system_mode == CONTROL_MODE_JOYSTICK) ? 1 : 0;
    }

    /* --- Write all outputs to physical pins --- */
    apply_outputs();
}

extern void Motor_SendAudioCommand(char sound_code);

void HW_EStop_Trigger(void)
{
    /* Still called from EXTI, ensures immediate hardware response */
    hw.in_estop = 1; 
    hw.out_relay_status = 1;
    hw.out_relay_motor  = 0; /* Must be 0 (OFF) for safe state! */
    if (!emergency_stop) {
        emergency_stop = true;
        fault_code |= FAULT_ESTOP_PHYSICAL;
        Motor_SendAudioCommand('E');
    }
    apply_outputs();
}

void HW_EStop_Clear(void)
{
    /* Latching logic handles clearing now in HW_RefreshIO, kept for API compatibility */
}
