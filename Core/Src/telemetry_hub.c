#include "telemetry_hub.h"
#include "motor_controller.h"
#include "hw_io.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern volatile float current_pwm;

/* --- Private Definitions --- */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 512

typedef enum {
    TLM_STATE_IDLE,
    TLM_STATE_PACKET
} TLM_State_t;

static UART_HandleTypeDef *t_huart;
static TLM_State_t rx_state = TLM_STATE_IDLE;
static char rx_buffer[RX_BUFFER_SIZE];
static uint16_t rx_idx = 0;
static char tx_buffer[TX_BUFFER_SIZE];

/* --- Internal Helpers --- */
static void Telemetry_ParseCommand(char *cmd_str);
static void Telemetry_HandleSet(char *payload);
static void Telemetry_HandleCmd(char *payload);

/* --- Public Implementation --- */

static bool has_received_command = false;

bool Telemetry_HasReceivedCommand(void) {
    return has_received_command;
}

void Telemetry_Init(UART_HandleTypeDef *huart) {
    t_huart = huart;
    rx_state = TLM_STATE_IDLE;
    rx_idx = 0;
    has_received_command = false;
}

static uint32_t last_slow_sync_tick = 0;

void Telemetry_Update(void) {
    if (t_huart == NULL) return;

    /* Don't spam dashboard text on LPUART1 when in BASE_SYSTEM mode —
     * the wire is now carrying Modbus RTU binary frames and any text
     * mixed in would corrupt Modbus responses. */
    if (control_system_mode == CONTROL_MODE_BASE_SYSTEM) return;

    uint32_t now = HAL_GetTick();

    // 1. FAST TELEMETRY (Every 20ms) - Motion & Critical Status
    float pos = encoder.current_position_deg;
    float vel = encoder.filtered_rpm;
    float target = trajectory.target_pos;
    float acc = trajectory.current_setpoint_accel;
    
    float vel_set = trajectory.current_setpoint_vel;
    float acc_set = trajectory.current_setpoint_accel;

    int len = snprintf(tx_buffer, TX_BUFFER_SIZE,
        "$POS:%.2f,VEL:%.2f,ACC:%.2f,TAR:%.2f,VSET:%.2f,ASET:%.2f,PWM:%.1f,MODE:%d,SYSM:%d,JOGM:%d,JOY:%d,ESTOP:%d,FAULT:%d,PROX:%d,GHOST:%d,GUD:%d,GCO:%d*",
        pos, vel, acc, target, vel_set, acc_set, current_pwm, (int)current_mode, (int)control_system_mode, (int)jog_mode, (int)is_joystick_connected,
        (int)emergency_stop, (int)fault_code, (int)hw.raw_prox_bit, (int)current_mode == MOTOR_MODE_GHOST,
        (int)hw.out_gripper_ud, (int)hw.out_gripper_co);

    if (len > 0) {
        HAL_UART_Transmit(t_huart, (uint8_t*)tx_buffer, len, 100);
    }

    // 2. SLOW TELEMETRY (Every 1000ms) - Tuning Parameters
    if (now - last_slow_sync_tick >= 1000) {
        last_slow_sync_tick = now;
        len = snprintf(tx_buffer, TX_BUFFER_SIZE,
            "$SKP:%.3f,SKI:%.3f,SKD:%.3f,SKF:%.3f,PKP:%.3f,PKI:%.3f,PKD:%.3f,"
            "VMAX:%.1f,AMAX:%.1f,STEPC:%.1f,STEPF:%.1f,JOGF:%.1f,HOMES:%.1f,MINP:%.1f*",
            tuning.speed_Kp, tuning.speed_Ki, tuning.speed_Kd, tuning.speed_Kf,
            tuning.pos_Kp, tuning.pos_Ki, tuning.pos_Kd, 
            tuning.move_speed_coarse, tuning.max_accel, tuning.step_size_coarse, 
            tuning.step_size_fine, tuning.jog_speed_fine, tuning.move_speed_return_home, tuning.min_pwm);
            
        if (len > 0) {
            HAL_UART_Transmit(t_huart, (uint8_t*)tx_buffer, len, 100);
        }
    }
}

void Telemetry_OnByteReceived(uint8_t byte) {
    switch (rx_state) {
        case TLM_STATE_IDLE:
            if (byte == '$') {
                rx_state = TLM_STATE_PACKET;
                rx_idx = 0;
            }
            break;

        case TLM_STATE_PACKET:
            if (byte == '*') {
                rx_buffer[rx_idx] = '\0';
                Telemetry_ParseCommand(rx_buffer);
                rx_state = TLM_STATE_IDLE;
            } else {
                if (rx_idx < (RX_BUFFER_SIZE - 1)) {
                    rx_buffer[rx_idx++] = byte;
                } else {
                    rx_state = TLM_STATE_IDLE; // Buffer overflow, reset
                }
            }
            break;
    }
}

static void Telemetry_ParseCommand(char *cmd_str) {
    has_received_command = true;
    
    // Expected format: TYPE:PAYLOAD
    char *colon = strchr(cmd_str, ':');
    if (colon == NULL) return;

    *colon = '\0';
    char *type = cmd_str;
    char *payload = colon + 1;

    if (strcmp(type, "SET") == 0) {
        Telemetry_HandleSet(payload);
    } else if (strcmp(type, "CMD") == 0) {
        Telemetry_HandleCmd(payload);
    }
}

static void Telemetry_HandleSet(char *payload) {
    // Payload format: KEY=VAL,KEY=VAL...
    char *token = strtok(payload, ",");
    while (token != NULL) {
        char *equal = strchr(token, '=');
        if (equal != NULL) {
            *equal = '\0';
            char *key = token;
            float val = atof(equal + 1);

            // Apply to tuning struct
            if (strcmp(key, "POS_KP") == 0) tuning.pos_Kp = val;
            else if (strcmp(key, "POS_KI") == 0) tuning.pos_Ki = val;
            else if (strcmp(key, "POS_KD") == 0) tuning.pos_Kd = val;
            else if (strcmp(key, "SPEED_KP") == 0) tuning.speed_Kp = val;
            else if (strcmp(key, "SPEED_KI") == 0) tuning.speed_Ki = val;
            else if (strcmp(key, "SPEED_KD") == 0) tuning.speed_Kd = val;
            else if (strcmp(key, "SPEED_KF") == 0) tuning.speed_Kf = val;
            else if (strcmp(key, "V_MAX") == 0 || strcmp(key, "MOVE_COARSE") == 0) tuning.move_speed_coarse = val;
            else if (strcmp(key, "A_MAX") == 0 || strcmp(key, "MAX_ACCEL") == 0) tuning.max_accel = val;
            else if (strcmp(key, "STEP_COARSE") == 0) tuning.step_size_coarse = val;
            else if (strcmp(key, "STEP_FINE") == 0) tuning.step_size_fine = val;
            else if (strcmp(key, "MIN_PWM") == 0) tuning.min_pwm = val;
            else if (strcmp(key, "HOME_SPEED") == 0) tuning.move_speed_return_home = val;
            else if (strcmp(key, "JOG_FINE") == 0) tuning.jog_speed_fine = val;
            else if (strcmp(key, "TARGET") == 0) Motor_MoveToPosition(val);
            else if (strcmp(key, "SAFE_STALL")   == 0) safety_config.stall_prevent       = (val > 0.5f);
            else if (strcmp(key, "SAFE_ENCODER") == 0) safety_config.encoder_check       = (val > 0.5f);
            else if (strcmp(key, "SAFE_OVERROT") == 0) safety_config.over_rotation_check = (val > 0.5f);
            else if (strcmp(key, "SAFE_JOY")     == 0) safety_config.joystick_check      = (val > 0.5f);
            else if (strcmp(key, "SYS_MODE")     == 0) control_system_mode = (val > 0.5f) ? CONTROL_MODE_JOYSTICK : CONTROL_MODE_BASE_SYSTEM;
            else if (strcmp(key, "JOG_MODE")     == 0) jog_mode = (val > 0.5f) ? JOG_FINE : JOG_COARSE;
        }
        token = strtok(NULL, ",");
    }
}

static void Telemetry_HandleCmd(char *payload) {
    if (strcmp(payload, "ESTOP=1") == 0 || strcmp(payload, "ESTOP") == 0) {
        emergency_stop = true;
    } else if (strcmp(payload, "CLEAR") == 0) {
        fault_code = FAULT_NONE;
        emergency_stop = false;
    } else if (strcmp(payload, "HOME") == 0) {
        trigger_homing_sequence = true;
    } else if (strcmp(payload, "GRIP_UP") == 0) {
        hw.out_gripper_ud = 0;
    } else if (strcmp(payload, "GRIP_DN") == 0) {
        hw.out_gripper_ud = 1;
    } else if (strcmp(payload, "GRIP_OPEN") == 0) {
        hw.out_gripper_co = 0;
    } else if (strcmp(payload, "GRIP_CLOSE") == 0) {
        hw.out_gripper_co = 1;
    } else if (strcmp(payload, "TOGGLE_MODE") == 0) {
        Mode_Toggle();
    }
}
