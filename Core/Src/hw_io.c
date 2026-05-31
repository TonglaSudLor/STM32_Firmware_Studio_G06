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
#include "current_sensor.h"
#include "params.h"
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
    HAL_GPIO_WritePin(Gripper_Up_GPIO_Port,   Gripper_Up_Pin,
                      hw.out_gripper_up   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(Gripper_Down_GPIO_Port, Gripper_Down_Pin,
                      hw.out_gripper_down ? GPIO_PIN_SET : GPIO_PIN_RESET);
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
    hw.out_gripper_up   = 0; /* Vertical: spring returns DOWN */
    hw.out_gripper_down = 0; /* Claw: spring returns OPEN */
    /* Reed SW pins are inputs — no init needed */

    apply_outputs();
}

void HW_RefreshIO(void)
{
    /* --- Read all Opto inputs (1 = Active) --- */
    /* E-Stop: PA5 has only the internal ~40k pull-up (no external pull-up
     * possible on this PCB). Motor current and relay arcs couple noise onto
     * the high-impedance pin and cause spurious LOW pulses while the motor
     * is running, which previously latched FAULT_ESTOP_PHYSICAL whenever
     * the motor moved. Mitigation:
     *   - Require 8 CONSECUTIVE LOW reads at 100 Hz (~80 ms continuous) to
     *     declare a press. EMI bursts almost never sustain LOW that long;
     *     a human finger easily does, and 80 ms is imperceptible latency.
     *   - ANY HIGH read resets the counter to 0 — no gradual decrement.
     *     A real press is a clean continuous LOW; noise is bursty and the
     *     first HIGH read in any burst restarts the count.
     * The EXTI fast-trigger path has been removed (see HAL_GPIO_EXTI_Callback
     * in main.c): a microsecond-window vote cannot tell a real press from an
     * EMI burst, so this polled debounce is now the only path that can set
     * hw.in_estop = 1 and latch the emergency. */
    {
        static uint8_t  estop_debounce = 0;
        static uint16_t motor_active_ticks = 0;   /* lingers after relay opens */
        uint8_t estop_raw = (HAL_GPIO_ReadPin(E_Stop_GPIO_Port, E_Stop_Pin) == GPIO_PIN_RESET) ? 1 : 0;

        /* Adaptive threshold: motor noise on PA5 only happens while the motor
         * power relay is ON (or just transitioned). When idle, respond fast
         * for a snappy button press. When the motor is running, demand a much
         * longer continuous LOW to reject EMI.
         *
         * Thresholds (see comment on threshold variable below for effective ms). */
        if (hw.out_relay_motor) {
            motor_active_ticks = 100;  /* hold strict mode 1 s past relay open */
        } else if (motor_active_ticks > 0) {
            motor_active_ticks--;
        }
        /* HW_RefreshIO() is called from three contexts: the 100 Hz TIM6 ISR
         * (via Motor_ControlLoop), the ~50 Hz main-loop tick, and the
         * tight wait_for_reed() poll — so estop_debounce accumulates at
         * roughly 150 Hz, not 100 Hz.  Actual window = threshold / 150 s.
         *
         * During homing or diagnostic tests, high PWM commands generate 
         * significant EMI on PA5. Use a 1.5 s window (225 samples at ~150 Hz) 
         * so the startup transients are ignored. Normal running stays at 500 ms. */
        uint16_t threshold = (motor_active_ticks > 0)
                           ? ((current_mode == MOTOR_MODE_HOMING || current_mode == MOTOR_MODE_TEST) ? 225 : 75)
                           : 8;

        if (estop_raw) {
            if (estop_debounce < threshold) estop_debounce++;
        } else {
            estop_debounce = 0;  /* any HIGH read = not a sustained press; restart */
        }
        hw.in_estop = (estop_debounce >= threshold) ? 1 : 0;
    }
    hw.in_proximity   = (HAL_GPIO_ReadPin(Proximity_Sensor_GPIO_Port, Proximity_Sensor_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    hw.raw_prox_bit   = (HAL_GPIO_ReadPin(Proximity_Sensor_GPIO_Port, Proximity_Sensor_Pin) == GPIO_PIN_RESET) ? 1 : 0;

    /* --- Slide switch debouncing (integrator) → toggle system mode ---
     * The slide switch on PA6 is susceptible to motor-current EMI.
     * Require a consistent state for 20 samples (~200ms) before toggling. */
    {
        static uint8_t  select_debounce = 0;
        static int8_t   debounced_state = -1; /* -1 = uninitialized */
        static uint16_t toggle_cooldown = 0;  /* ticks remaining before another toggle is allowed */
        uint8_t raw_select = (HAL_GPIO_ReadPin(Selected_Mode_GPIO_Port, Selected_Mode_Pin) == GPIO_PIN_RESET) ? 1 : 0;
        hw.in_select_raw = raw_select;

        if (toggle_cooldown > 0) toggle_cooldown--;

        if (debounced_state < 0) {
            debounced_state = (int8_t)raw_select;
        } else if (raw_select != (uint8_t)debounced_state) {
            select_debounce++;
            hw.select_debounce_cnt = select_debounce;
            if (select_debounce > hw.select_debounce_peak)
                hw.select_debounce_peak = select_debounce;
            if (select_debounce >= 5) {
                debounced_state = (int8_t)raw_select;
                select_debounce = 0;
                hw.select_debounce_cnt = 0;
                /* Mode change is handled level-triggered in the main loop
                 * by comparing hw.in_select_mode to control_system_mode.
                 * No Mode_Toggle() call needed here. */
            }
        } else {
            select_debounce = 0;
            hw.select_debounce_cnt = 0;
        }
        hw.in_select_mode = (uint8_t)debounced_state;
    }
    /* Debounce reset button: require 5 consecutive active reads (~100ms at 50Hz poll).
     * Prevents contact bounce from causing rapid relay oscillation. */
    {
        static uint8_t reset_debounce = 0;
        uint8_t reset_raw = (HAL_GPIO_ReadPin(Reset_Btn_GPIO_Port, Reset_Btn_Pin) == GPIO_PIN_RESET) ? 1 : 0;
        if (reset_raw) {
            if (reset_debounce < 5) reset_debounce++;
        } else {
            reset_debounce = 0;
        }
        hw.in_reset_btn = (reset_debounce >= 5) ? 1 : 0;
    }

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

    /* --- Sample WCS1800 current sensor (PA0, ADC1_IN1) --- */
    hw.current_amps    = CurrentSensor_Sample();
    hw.current_adc_raw = CurrentSensor_GetRaw();

    /* Overcurrent trip with time debounce (bug 0-E): a single filtered sample
     * over the limit must NOT latch the e-stop. Inrush, direction-reversal and
     * gripper-relay transients legitimately spike past the limit for a few ms.
     * Require the current to stay over the limit continuously for
     * OVERCURRENT_TIME_MS before cutting motor power. */
    {
        static uint32_t overcurrent_start_tick = 0;
        bool over = (hw.current_amps > OVERCURRENT_LIMIT_AMPS ||
                     hw.current_amps < -OVERCURRENT_LIMIT_AMPS);
        if (!hw.override_enabled && over) {
            if (overcurrent_start_tick == 0) overcurrent_start_tick = HAL_GetTick();
            if ((HAL_GetTick() - overcurrent_start_tick) >= OVERCURRENT_TIME_MS) {
                FAULT_SET(FAULT_OVERCURRENT);
                emergency_stop = true;
                hw.out_relay_motor  = 0;
                hw.out_relay_status = 1;
                printf("[FAULT] Overcurrent: %.1f A (limit %.1f A, >%lu ms)\r\n",
                       hw.current_amps, (float)OVERCURRENT_LIMIT_AMPS,
                       (unsigned long)OVERCURRENT_TIME_MS);
            }
        } else {
            overcurrent_start_tick = 0;   /* dropped below limit (or override) — reset */
            if (!emergency_stop) FAULT_CLR(FAULT_OVERCURRENT);
        }
    }

    /* --- Read motor direction pin state for monitoring --- */
    hw.out_motor_dir  = (HAL_GPIO_ReadPin(Motor_Direction_GPIO_Port, Motor_Direction_Pin) == GPIO_PIN_SET) ? 1 : 0;

    /* --- Latching Emergency Logic --- */
    if (!hw.override_enabled) {
        if (hw.in_estop) {
            /* Emergency Pressed: Safe the system immediately */
            emergency_stop = true;
            FAULT_SET(FAULT_ESTOP_PHYSICAL);
            /* Motor relay is about to open, encoder loses power → position
             * cannot be trusted after recovery. Force a re-home.
             *
             * Exception: if E-stop fires while ALREADY homing, we haven't
             * established position yet anyway. Do NOT set position_unknown
             * here — doing so causes Motor_ControlLoop to auto-arm homing
             * again the instant Reset is pressed, which creates a noise-
             * driven Reset→homing→E-stop→Reset infinite loop.
             * With position_unknown = false, Reset goes to STOPPED and the
             * user manually re-triggers Fine Home once the noise settles. */
            if (current_mode != MOTOR_MODE_HOMING) {
                position_unknown = true;
            }
        } else if (hw.in_reset_btn) {
            /* Reset Pressed AND Emergency is Released: Enter Ready state */
            emergency_stop = false;
            FAULT_CLR(FAULT_ESTOP_PHYSICAL | FAULT_PROX_LOST |
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
        if (hw.in_estop) {
            emergency_stop = true;
            FAULT_SET(FAULT_ESTOP_PHYSICAL);
            if (current_mode != MOTOR_MODE_HOMING) {
                position_unknown = true;
            }
        } else if (hw.in_reset_btn) emergency_stop = false;
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
        FAULT_SET(FAULT_ESTOP_PHYSICAL);
        position_unknown = true;
        Motor_SendAudioCommand('E');
    }
    apply_outputs();
}

void HW_EStop_Clear(void)
{
    /* Latching logic handles clearing now in HW_RefreshIO, kept for API compatibility */
}
