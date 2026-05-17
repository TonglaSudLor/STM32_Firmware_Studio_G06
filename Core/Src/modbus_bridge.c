/**
 * @file modbus_bridge.c
 * @brief Application-level Modbus register mapping and logic
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#include "modbus_bridge.h"
#include "modbus_rtu.h"
#include "motor_controller.h"
#include "hw_io.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/* Task flags exported from motor_controller.c */
extern volatile bool task_pick_active;
extern volatile bool task_place_active;

/* Heartbeat constants from Base System spec */
#define HB_YA   22881   /* Robot writes when waiting for reply */
#define HB_HI   18537   /* Base System writes as alive ack */

static uint32_t last_hb_ack_tick = 0;
static volatile bool base_system_alive = false;

/* === Pick & Place automated sequence === */
typedef enum {
    PNP_IDLE = 0,
    PNP_MOVE_TO_PICK,
    PNP_WAIT_PICK_ARRIVAL,
    PNP_DO_PICK_OPEN,
    PNP_DO_PICK_DOWN,
    PNP_DO_PICK_CLOSE,
    PNP_DO_PICK_UP,
    PNP_MOVE_TO_PLACE,
    PNP_WAIT_PLACE_ARRIVAL,
    PNP_DO_PLACE_DOWN,
    PNP_DO_PLACE_OPEN,
    PNP_DO_PLACE_UP,
    PNP_DO_PLACE_CLOSE,
    PNP_NEXT_PAIR,
    PNP_DONE
} PnP_State_t;

static volatile PnP_State_t pnp_state    = PNP_IDLE;
static uint16_t pnp_total_pairs          = 0;
static uint16_t pnp_current_pair         = 0;
static bool     pnp_grip_enable          = false;
static uint32_t pnp_step_start_tick      = 0;

/* Gripper step delay (each mechanical step) */
#define PNP_GRIP_STEP_MS    600U
/* Position tolerance and velocity threshold for "arrived" */
#define PNP_POS_TOL_DEG     2.0f
#define PNP_VEL_TOL_RPM     3.0f

static bool motor_at_target(void)
{
    float err = fabsf(Motor_GetPosition() - trajectory.target_pos);
    float vel = fabsf(Motor_GetSpeed());
    return (err < PNP_POS_TOL_DEG) && (vel < PNP_VEL_TOL_RPM);
}

extern UART_HandleTypeDef hlpuart1;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim3;

/* --- Modbus Configuration --- */
#define MODBUS_SLAVE_ID     21
#define MODBUS_REG_COUNT    128

/* --- Global Modbus Handle --- */
static Modbus_Handle_t hmodbus;
static Modbus_Register_t register_frame[MODBUS_REG_COUNT];
static bool modbus_initialized = false;

/* --- Debug Log --- */
static uint8_t debug_rx_log[8] = {0};
static uint8_t debug_rx_idx = 0;

/* ============================================================================
 * Internal Register Handlers
 * ============================================================================ */

/**
 * @brief Process writes from Modbus Master (PC)
 */
/* P&P state machine — runs from main loop every cycle */
static void PnP_StateMachine(void)
{
    if (pnp_state == PNP_IDLE) return;

    uint32_t elapsed = HAL_GetTick() - pnp_step_start_tick;

    switch (pnp_state) {
        case PNP_MOVE_TO_PICK: {
            int16_t pos = (int16_t)register_frame[0x12 + 2 * pnp_current_pair].U16;
            Motor_MoveToPosition(-(float)pos);  /* flip sign for Base System convention */
            pnp_state = PNP_WAIT_PICK_ARRIVAL;
            pnp_step_start_tick = HAL_GetTick();
            break;
        }
        case PNP_WAIT_PICK_ARRIVAL:
            if (motor_at_target() || elapsed > 10000) {
                if (pnp_grip_enable) {
                    Gripper_Open();
                    pnp_state = PNP_DO_PICK_OPEN;
                    pnp_step_start_tick = HAL_GetTick();
                } else {
                    pnp_state = PNP_MOVE_TO_PLACE;
                }
            }
            break;
        case PNP_DO_PICK_OPEN:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Down(); pnp_state = PNP_DO_PICK_DOWN; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PICK_DOWN:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Close(); pnp_state = PNP_DO_PICK_CLOSE; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PICK_CLOSE:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Up(); pnp_state = PNP_DO_PICK_UP; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PICK_UP:
            if (elapsed > PNP_GRIP_STEP_MS) { pnp_state = PNP_MOVE_TO_PLACE; }
            break;

        case PNP_MOVE_TO_PLACE: {
            int16_t pos = (int16_t)register_frame[0x12 + 2 * pnp_current_pair + 1].U16;
            Motor_MoveToPosition(-(float)pos);  /* flip sign for Base System convention */
            pnp_state = PNP_WAIT_PLACE_ARRIVAL;
            pnp_step_start_tick = HAL_GetTick();
            break;
        }
        case PNP_WAIT_PLACE_ARRIVAL:
            if (motor_at_target() || elapsed > 10000) {
                if (pnp_grip_enable) {
                    Gripper_Down();
                    pnp_state = PNP_DO_PLACE_DOWN;
                    pnp_step_start_tick = HAL_GetTick();
                } else {
                    pnp_state = PNP_NEXT_PAIR;
                }
            }
            break;
        case PNP_DO_PLACE_DOWN:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Open(); pnp_state = PNP_DO_PLACE_OPEN; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PLACE_OPEN:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Up(); pnp_state = PNP_DO_PLACE_UP; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PLACE_UP:
            if (elapsed > PNP_GRIP_STEP_MS) { Gripper_Close(); pnp_state = PNP_DO_PLACE_CLOSE; pnp_step_start_tick = HAL_GetTick(); }
            break;
        case PNP_DO_PLACE_CLOSE:
            if (elapsed > PNP_GRIP_STEP_MS) { pnp_state = PNP_NEXT_PAIR; }
            break;

        case PNP_NEXT_PAIR:
            pnp_current_pair++;
            if (pnp_current_pair >= pnp_total_pairs) {
                pnp_state = PNP_DONE;
            } else {
                pnp_state = PNP_MOVE_TO_PICK;
            }
            break;
        case PNP_DONE:
            printf("[P&P] Sequence finished (%u pairs)\r\n", pnp_total_pairs);
            pnp_state = PNP_IDLE;
            break;
        default:
            pnp_state = PNP_IDLE;
            break;
    }
}

static void ModbusBridge_HandleCommands(void)
{
    if (!modbus_initialized) return;

    /* 0x00: Heartbeat — Base System writes HI (18537) back when alive */
    if (register_frame[0x00].U16 == HB_HI) {
        last_hb_ack_tick = HAL_GetTick();
        base_system_alive = true;
        register_frame[0x00].U16 = HB_YA;   /* reset to YA for next ping */
    }
    /* Detect liveness timeout (3 seconds without HI reply) */
    if (base_system_alive && (HAL_GetTick() - last_hb_ack_tick > 3000)) {
        base_system_alive = false;
    }

    // 0x01: Operating Mode Bits
    if (register_frame[0x01].U16 & 0x01) { // Home (Move to 0)
        Motor_MoveToPosition(0.0f);
        register_frame[0x01].U16 &= ~0x01;
    }
    
    /* 0x01 bit 1 = Manual/Jog mode (per spec - just acknowledge, no action) */
    if (register_frame[0x01].U16 & 0x02) {
        register_frame[0x01].U16 &= ~0x02;
    }

    /* 0x02: Manual Gripper — react to value changes (per spec). Up=0, Down=1, Open=2, Close=4.
     * Edge-triggered so we don't spam printf every cycle. */
    {
        static uint16_t last_grip_v = 0xFFFF; /* impossible value at boot */
        uint16_t v = register_frame[0x02].U16;
        if (v != last_grip_v) {
            if      (v == 0) Gripper_Up();
            else if (v == 1) Gripper_Down();
            else if (v == 2) Gripper_Open();
            else if (v == 4) Gripper_Close();
            last_grip_v = v;
        }
    }

    if (register_frame[0x01].U16 & 0x04) { /* Auto bit — just clear, P&P uses 0x22 trigger */
        register_frame[0x01].U16 &= ~0x04;
    }

    /* P&P trigger: per spec, Base System writes slots first, then writes 0x22 (pair count).
     * Detect non-zero 0x22 while P&P is idle and start the sequence. */
    if (pnp_state == PNP_IDLE && register_frame[0x22].U16 > 0) {
        uint16_t pairs = register_frame[0x22].U16;
        if (pairs > 5) pairs = 5;
        pnp_total_pairs  = pairs;
        pnp_current_pair = 0;
        pnp_grip_enable  = (register_frame[0x04].U16 & 0x01) ? true : false;
        pnp_state = PNP_MOVE_TO_PICK;
        pnp_step_start_tick = HAL_GetTick();
        current_mode = MOTOR_MODE_POSITION;
        printf("[P&P] Start: %u pairs, grip=%d\r\n", pairs, pnp_grip_enable);
        register_frame[0x22].U16 = 0; /* consume trigger so it doesn't re-fire */
    }
    if (register_frame[0x01].U16 & 0x08) { // Set Home (Reset Encoder)
        __HAL_TIM_SET_COUNTER(&htim3, 0);
        encoder.absolute_counts = 0;
        encoder.current_position_deg = 0.0f;
        trajectory.target_pos = 0.0f;
        trajectory.current_setpoint_pos = 0.0f;
        register_frame[0x01].U16 &= ~0x08;
    }
    if (register_frame[0x01].U16 & 0x10) { // Test mode
        current_mode = MOTOR_MODE_TEST;
        register_frame[0x01].U16 &= ~0x10;
    }

    // 0x03: Gripper Sequence
    if (register_frame[0x03].U16 != 0) {
        uint16_t val = register_frame[0x03].U16;
        if (val == 1) Gripper_Sequence_Pick();
        else if (val == 2) Gripper_Sequence_Place();
        register_frame[0x03].U16 = 0;
    }

    /* 0x05: Jog step (Base System: +=CCW, -=CW → invert to match firmware convention) */
    if (register_frame[0x05].U16 != 0) {
        float step = -(float)((int16_t)register_frame[0x05].U16);
        Motor_MoveToPosition(encoder.current_position_deg + step);
        register_frame[0x05].U16 = 0;
    }

    /* 0x24: Point-to-Point Target (sign inverted to match firmware direction) */
    if (register_frame[0x24].U16 != 0) {
        Motor_MoveToPosition(-(float)((int16_t)register_frame[0x24].U16));
        register_frame[0x24].U16 = 0;
    }

    // 0x25: Safety / Soft Stop
    if (register_frame[0x25].U16 & 0x01) {
        emergency_stop = true;
        register_frame[0x25].U16 &= ~0x01;
    } else if (register_frame[0x25].U16 & 0x02) {
        emergency_stop = false;
        register_frame[0x25].U16 &= ~0x02;
    }
}

/* ============================================================================
 * Public API
 * ============================================================================ */

void ModbusBridge_Init(void)
{
    memset(register_frame, 0, sizeof(register_frame));
    
    // 0x00: Device ID / Heartbeat ("YA")
    register_frame[0].U16 = 22881; 
    
    // Initialize RTU Handle
    hmodbus.huart = &hlpuart1;
    hmodbus.htim = &htim16;
    hmodbus.slave_address = MODBUS_SLAVE_ID;
    hmodbus.register_count = MODBUS_REG_COUNT;
    
    Modbus_Init(&hmodbus, register_frame);
    modbus_initialized = true;
}

void ModbusBridge_Process(void)
{
    /* 1. Process Command Writes from Master */
    ModbusBridge_HandleCommands();

    /* 2. Run Pick & Place state machine (non-blocking) */
    PnP_StateMachine();

    /* 3. Run Modbus Protocol Engine */
    Modbus_Process(&hmodbus);
}

void ModbusBridge_UpdateRegisters(void)
{
    /* Sync internal state to Modbus Read-Only registers. */

    /* 0x26: Reed sensors (bit 0=up, 1=down, 2=close, 3=open).
     * Base System UI uses bits 0&1 paired for height (Up/Down)
     * and bit 2 for jaw closed. */
    uint16_t reed_bits = 0;
    if (hw.in_reed_up)    reed_bits |= (1 << 0);
    if (hw.in_reed_down)  reed_bits |= (1 << 1);
    if (hw.in_reed_close) reed_bits |= (1 << 2);
    if (hw.in_reed_open)  reed_bits |= (1 << 3);
    register_frame[0x26].U16 = reed_bits;

    /* 0x27: Current task bits per Base System spec
     *   bit 0 = Homing, 1 = Go Pick, 2 = Go Place, 3 = Go Point */
    uint16_t task_bits = 0;
    if (current_mode == MOTOR_MODE_HOMING)   task_bits |= (1 << 0);
    if (task_pick_active ||
        pnp_state == PNP_MOVE_TO_PICK ||
        pnp_state == PNP_WAIT_PICK_ARRIVAL ||
        (pnp_state >= PNP_DO_PICK_OPEN && pnp_state <= PNP_DO_PICK_UP)) {
        task_bits |= (1 << 1);  /* Go Pick */
    }
    if (task_place_active ||
        pnp_state == PNP_MOVE_TO_PLACE ||
        pnp_state == PNP_WAIT_PLACE_ARRIVAL ||
        (pnp_state >= PNP_DO_PLACE_DOWN && pnp_state <= PNP_DO_PLACE_CLOSE)) {
        task_bits |= (1 << 2);  /* Go Place */
    }
    if (current_mode == MOTOR_MODE_POSITION && pnp_state == PNP_IDLE) {
        task_bits |= (1 << 3);  /* Go Point */
    }
    register_frame[0x27].U16 = task_bits;

    /* 0x28: Current Position (×10 for UI). Sign flipped to match Base System direction. */
    register_frame[0x28].U16 = (int16_t)(-Motor_GetPosition() * 10.0f);

    /* 0x29: Current Speed (×10 for UI). Sign flipped to match Base System direction. */
    register_frame[0x29].U16 = (int16_t)(-Motor_GetSpeed() * 10.0f);

    /* 0x30: Acceleration (×10 for UI). Sign flipped to match Base System direction. */
    register_frame[0x30].U16 = (int16_t)(-trajectory.current_setpoint_accel * 10.0f);

    /* 0x31: Emergency / Safety state (bit 0) */
    register_frame[0x31].U16 = emergency_stop ? 1 : 0;
}

void ModbusBridge_RxCallback(uint8_t data)
{
    // Log for debugging
    debug_rx_log[debug_rx_idx % 8] = data;
    debug_rx_idx++;

    if (hmodbus.uart.rx_tail < MODBUS_BUFFER_SIZE) {
        hmodbus.uart.rx_buffer[hmodbus.uart.rx_tail++] = data;
    }
    
    hmodbus.state = MODBUS_STATE_RECEPTION;
    
    // Reset T3.5 Timer
    __HAL_TIM_SET_COUNTER(hmodbus.htim, 0);
    HAL_TIM_Base_Start_IT(hmodbus.htim);
}

void ModbusBridge_TimerCallback(void)
{
    hmodbus.flag_t35_timeout = 1;
    HAL_TIM_Base_Stop_IT(hmodbus.htim);
}