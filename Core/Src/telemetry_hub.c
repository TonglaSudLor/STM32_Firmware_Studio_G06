#include "telemetry_hub.h"
#include "motor_controller.h"
#include "hw_io.h"
#include "kalman.h"
#include "modbus_bridge.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern Auto_Config_t auto_config;
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
volatile bool flash_save_requested = false;

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
static uint32_t last_kf_tx_tick = 0;
static uint32_t last_fast_tx_tick = 0;
static uint32_t tlm_fast_rate_ms = TELEMETRY_DEMO_RATE_MS; // Default to 20Hz for Demo stability

void Telemetry_Update(void) {
    if (t_huart == NULL) return;

    /* Don't spam dashboard text on LPUART1 when in BASE_SYSTEM mode —
     * the wire is now carrying Modbus RTU binary frames and any text
     * mixed in would corrupt Modbus responses. */
    if (control_system_mode == CONTROL_MODE_BASE_SYSTEM) return;

    uint32_t now = HAL_GetTick();

    // 1. FAST TELEMETRY - Motion & Critical Status
    if (now - last_fast_tx_tick >= tlm_fast_rate_ms) {
        last_fast_tx_tick = now;
        
        float pos = encoder.current_position_deg;
        float vel = encoder.filtered_rpm;
        float target = trajectory.target_pos;
        
        float vel_set = trajectory.current_setpoint_vel;
        float acc_set = trajectory.current_setpoint_accel;

        int len = snprintf(tx_buffer, TX_BUFFER_SIZE,
            "$POS:%.2f,VEL:%.2f,ACC:%.2f,TAR:%.2f,VSET:%.2f,ASET:%.2f,PWM:%.1f,MODE:%d,SYSM:%d,JOGM:%d,JOY:%d,ESTOP:%d,FAULT:%d,PROX:%d,GHOST:%d,GUP:%d,GDN:%d,CURR:%.2f,RSUP:%d,RSDN:%d,RSCL:%d,RSOP:%d,BSALV:%d,PNPS:%d,PUNK:%d*",
            pos, vel, acc_set, target, vel_set, acc_set, current_pwm, (int)current_mode, (int)control_system_mode, (int)jog_mode, (int)is_joystick_connected,
            (int)emergency_stop, (int)fault_code, (int)hw.raw_prox_bit, (int)current_mode == MOTOR_MODE_GHOST,
            (int)hw.out_gripper_up, (int)hw.out_gripper_down, hw.current_amps,
            (int)hw.in_reed_up, (int)hw.in_reed_down, (int)hw.in_reed_close, (int)hw.in_reed_open,
            (int)ModbusBridge_IsBaseAlive(), ModbusBridge_GetPnPState(), (int)position_unknown);

        if (len > 0) {
            HAL_UART_Transmit(t_huart, (uint8_t*)tx_buffer, len, 100);
        }
    }

    // 1b. KALMAN ESTIMATES — short packet at 5 Hz (200 ms)
    if (now - last_kf_tx_tick >= 200) {
        last_kf_tx_tick = now;
        int klen = snprintf(tx_buffer, TX_BUFFER_SIZE,
            "$KFEN:%d,KTH:%.1f,KOM:%.1f,KTL:%.2f,KIA:%.2f,KIV:%.1e,KP00:%.1e,KP11:%.1e,KP22:%.1e,KP33:%.1e,KSTH:%.1f,KSOM:%.1f*",
            (int)Kalman_GetEnabled(),
            Kalman_GetTheta() * (180.0f/3.14159265f),
            Kalman_GetOmegaRPM(),
            Kalman_GetLoadTorque(),
            Kalman_GetCurrent(),
            Kalman_GetInnovation(),
            Kalman_GetP00(), Kalman_GetP11(), Kalman_GetP22(), Kalman_GetP33(),
            Kalman_SanityGetTheta() * (180.0f/3.14159265f),
            Kalman_SanityGetOmega() * (60.0f / (2.0f*3.14159265f)));
        if (klen > 0) HAL_UART_Transmit(t_huart, (uint8_t*)tx_buffer, klen, 100);
    }

    // 2. SLOW TELEMETRY (Every 1000ms) - Tuning Parameters
    if (now - last_slow_sync_tick >= 1000) {
        last_slow_sync_tick = now;
        int len = snprintf(tx_buffer, TX_BUFFER_SIZE,
            "$SKP:%.3f,SKI:%.3f,SKD:%.3f,KVFF:%.3f,KAFF:%.3f,KTFF:%.3f,PKP:%.3f,PKI:%.3f,PKD:%.3f,VMAX:%.2f,AMAX:%.2f,JMAX:%.1f,STEPC:%.1f,STEPF:%.1f,JOGF:%.1f,HOMES:%.1f,MINP:%.1f,PLOOP:%d,SHPEN:%d,SHPWN:%.3f,SHPZT:%.4f,SHPN:%lu,GRIPS:%d,GRIPD:%u,HOFS:%.2f*",
            tuning.speed_Kp, tuning.speed_Ki, tuning.speed_Kd,
            tuning.K_vff, tuning.K_aff, tuning.K_tff,
            tuning.pos_Kp, tuning.pos_Ki, tuning.pos_Kd,
            tuning.move_speed_coarse, tuning.max_accel, tuning.max_jerk,
            tuning.step_size_coarse,
            tuning.step_size_fine, tuning.jog_speed_fine, tuning.move_speed_return_home, tuning.min_pwm,
            position_loop_enabled ? 1 : 0,
            tuning.shaper_enable ? 1 : 0,
            tuning.shaper_omega_n,
            tuning.shaper_zeta,
            (unsigned long)Motor_GetShaperDelay(),
            auto_config.use_sensors ? 1 : 0,
            auto_config.delay_ms,
            tuning.home_offset_deg);
            
        if (len > 0) {
            HAL_UART_Transmit(t_huart, (uint8_t*)tx_buffer, len, 100);
        }
    }
}

void Telemetry_ProcessByte(uint8_t byte) {
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
                    rx_state = TLM_STATE_IDLE;
                }
            }
            break;
    }
}

static void Telemetry_ParseCommand(char *cmd_str) {
    has_received_command = true;
    Motor_RefreshWatchdog();
    char *colon = strchr(cmd_str, ':');
    if (colon == NULL) return;
    *colon = '\0';
    char *type = cmd_str;
    char *payload = colon + 1;
    if (strcmp(type, "SET") == 0) { Telemetry_HandleSet(payload); }
    else if (strcmp(type, "CMD") == 0) { Telemetry_HandleCmd(payload); }
}

static void Telemetry_HandleSet(char *payload) {
    char *token = strtok(payload, ",");
    while (token != NULL) {
        char *equal = strchr(token, '=');
        if (equal != NULL) {
            *equal = '\0';
            char *key = token;
            float val = atof(equal + 1);
            if (strcmp(key, "POS_KP") == 0) tuning.pos_Kp = val;
            else if (strcmp(key, "POS_KI") == 0) tuning.pos_Ki = val;
            else if (strcmp(key, "POS_KD") == 0) tuning.pos_Kd = val;
            else if (strcmp(key, "SPEED_KP") == 0) tuning.speed_Kp = val;
            else if (strcmp(key, "SPEED_KI") == 0) tuning.speed_Ki = val;
            else if (strcmp(key, "SPEED_KD") == 0) tuning.speed_Kd = val;
            else if (strcmp(key, "POS_LOOP") == 0) {
                trajectory.target_pos             = encoder.current_position_deg;
                trajectory.current_setpoint_pos   = encoder.current_position_deg;
                trajectory.current_setpoint_vel   = 0.0f;
                trajectory.current_setpoint_accel = 0.0f;
                pid_speed.integral    = 0.0f;
                pid_speed.error_prev  = 0.0f;
                pid_speed.d_filt      = 0.0f;
                pid_position.integral = 0.0f;
                position_loop_enabled = (val > 0.5f);
            }
            else if (strcmp(key, "KF_EN")    == 0) Kalman_SetEnabled(val > 0.5f);
            else if (strcmp(key, "KF_RESET") == 0) Kalman_Reset(encoder.current_position_deg * (3.14159265f/180.0f));
            else if (strcmp(key, "KF_Q_THETA") == 0) Kalman_SetSigmaTheta(val);
            else if (strcmp(key, "KF_Q_OMEGA") == 0) Kalman_SetSigmaOmega(val);
            else if (strcmp(key, "KF_Q_TAU")   == 0) Kalman_SetSigmaTau(val);
            else if (strcmp(key, "KF_Q_I")     == 0) Kalman_SetSigmaI(val);
            else if (strcmp(key, "KF_R")       == 0) Kalman_SetR(val);
            else if (strcmp(key, "SINE_EN")  == 0) {
                sine_test_enabled = (val > 0.5f);
                if (sine_test_enabled && !emergency_stop) {
                    extern void Motor_MoveToPosition(float target_degrees);
                    Motor_MoveToPosition(encoder.current_position_deg);
                } else if (!sine_test_enabled) {
                    trajectory.target_pos          = encoder.current_position_deg;
                    trajectory.current_setpoint_pos = encoder.current_position_deg;
                    trajectory.current_setpoint_vel = 0.0f;
                    trajectory.current_setpoint_accel = 0.0f;
                    pid_speed.integral    = 0.0f;
                    pid_speed.error_prev  = 0.0f;
                    pid_speed.d_filt      = 0.0f;
                    pid_position.integral = 0.0f;
                }
            }
            else if (strcmp(key, "SINE_AMP") == 0) sine_amp_rpm  = val;
            else if (strcmp(key, "SINE_FREQ")== 0) sine_freq_hz  = val;
            else if (strcmp(key, "K_VFF")    == 0) tuning.K_vff    = val;
            else if (strcmp(key, "K_AFF")    == 0) tuning.K_aff    = val;
            else if (strcmp(key, "K_TFF")    == 0) tuning.K_tff    = val;
            else if (strcmp(key, "V_MAX") == 0 || strcmp(key, "MOVE_COARSE") == 0) tuning.move_speed_coarse = val;
            else if (strcmp(key, "A_MAX") == 0 || strcmp(key, "MAX_ACCEL") == 0) tuning.max_accel = val;
            else if (strcmp(key, "J_MAX") == 0 || strcmp(key, "MAX_JERK") == 0) tuning.max_jerk = val;
            else if (strcmp(key, "V_MAX_RAD") == 0) tuning.move_speed_coarse = val * (60.0f / (2.0f * 3.14159265f));
            else if (strcmp(key, "A_MAX_RAD") == 0) tuning.max_accel         = val * (60.0f / (2.0f * 3.14159265f));
            else if (strcmp(key, "J_MAX_RAD") == 0) tuning.max_jerk          = val * (60.0f / (2.0f * 3.14159265f));
            else if (strcmp(key, "STEP_COARSE") == 0) tuning.step_size_coarse = val;
            else if (strcmp(key, "STEP_FINE") == 0) tuning.step_size_fine = val;
            else if (strcmp(key, "MIN_PWM") == 0) tuning.min_pwm = val;
            else if (strcmp(key, "HOME_SPEED") == 0) tuning.move_speed_return_home = val;
            else if (strcmp(key, "JOG_FINE") == 0) tuning.jog_speed_fine = val;
            else if (strcmp(key, "TARGET") == 0) { extern void Motor_MoveToPosition(float target_degrees); Motor_MoveToPosition(val); }
            else if (strcmp(key, "SAFE_STALL")   == 0) safety_config.stall_prevent       = (val > 0.5f);
            else if (strcmp(key, "SAFE_ENCODER") == 0) safety_config.encoder_check       = (val > 0.5f);
            else if (strcmp(key, "SAFE_OVERROT") == 0) safety_config.over_rotation_check = (val > 0.5f);
            else if (strcmp(key, "SAFE_JOY")     == 0) safety_config.joystick_check      = (val > 0.5f);
            else if (strcmp(key, "SAFE_ESTOP")   == 0) safety_config.physical_estop_check= (val > 0.5f);
            else if (strcmp(key, "MAX_ROT")      == 0) safety_config.soft_limit_deg  = val;
            else if (strcmp(key, "STALL_PWM")    == 0) safety_config.stall_pwm_pct    = val;
            else if (strcmp(key, "STALL_VEL")    == 0) safety_config.stall_vel_rpm    = val;
            else if (strcmp(key, "STALL_TIME")   == 0) safety_config.stall_time_ms    = (uint32_t)val;
            else if (strcmp(key, "STALL_ERR")    == 0) safety_config.stall_error_deg  = val;
            else if (strcmp(key, "SYS_MODE")     == 0) {
                if (val > 1.5f) { current_mode = MOTOR_MODE_AUTO; auto_config.current_index = 0; }
                else { control_system_mode = (val > 0.5f) ? CONTROL_MODE_JOYSTICK : CONTROL_MODE_BASE_SYSTEM;
                    if (current_mode == MOTOR_MODE_AUTO) current_mode = MOTOR_MODE_STOPPED; }
            }
            else if (strcmp(key, "WP_CLR")       == 0) { auto_config.target_count = 0; auto_config.current_index = 0; }
            else if (strcmp(key, "WP_ADD")       == 0) { if (auto_config.target_count < 10) { auto_config.positions[auto_config.target_count] = val; auto_config.actions[auto_config.target_count] = AUTO_ACTION_NONE; auto_config.target_count++; } }
            else if (strcmp(key, "WP_ACT")       == 0) { if (auto_config.target_count > 0) { auto_config.actions[auto_config.target_count - 1] = (Auto_Action_t)((int)val); } }
            else if (strcmp(key, "WP_SENS")      == 0) auto_config.use_sensors = (val > 0.5f);
            else if (strcmp(key, "WP_DLY")       == 0) auto_config.delay_ms = (uint16_t)val;
            else if (strcmp(key, "JOG_MODE")     == 0) jog_mode = (val > 0.5f) ? JOG_FINE : JOG_COARSE;
            else if (strcmp(key, "SHPEN")        == 0) tuning.shaper_enable  = (val > 0.5f);
            else if (strcmp(key, "SHPWN")        == 0) { tuning.shaper_omega_n = val; Motor_ShaperRecompute(); }
            else if (strcmp(key, "SHPZT")        == 0) { tuning.shaper_zeta    = val; Motor_ShaperRecompute(); }
            else if (strcmp(key, "TLM_RATE")     == 0) tlm_fast_rate_ms = (uint32_t)val;
            else if (strcmp(key, "HOME_OFFSET")  == 0) { tuning.home_offset_deg = val; printf("[HOME] Home offset set to %.2f deg\r\n", tuning.home_offset_deg); }
        }
        token = strtok(NULL, ",");
    }
}

static void Telemetry_HandleCmd(char *payload) {
    if (strcmp(payload, "ESTOP=1") == 0 || strcmp(payload, "ESTOP") == 0) { emergency_stop = true; FAULT_SET(FAULT_ESTOP_DASHBOARD); }
    else if (strcmp(payload, "CLEAR") == 0) { startup_estop_pending = false; FAULT_CLR(FAULT_STARTUP_ESTOP); fault_code = FAULT_NONE; emergency_stop = false; }
    else if (strcmp(payload, "HOME") == 0) { trigger_homing_sequence = true; }
    else if (strcmp(payload, "GRIP_UP") == 0) { Gripper_Up(); }
    else if (strcmp(payload, "GRIP_DN") == 0) { Gripper_Down(); }
    else if (strcmp(payload, "GRIP_STOP") == 0) { hw.out_gripper_up = 0; hw.out_gripper_down = 0; }
    else if (strcmp(payload, "CLAW_OPEN") == 0) { Gripper_Open(); }
    else if (strcmp(payload, "CLAW_CLOSE") == 0) { Gripper_Close(); }
    else if (strcmp(payload, "CLAW_STOP") == 0) { hw.out_gripper_down = 0; }
    else if (strcmp(payload, "TOGGLE_MODE") == 0) { Mode_Toggle(); }
    else if (strcmp(payload, "DIAG") == 0 || strcmp(payload, "Z") == 0) { Motor_ProcessCommand('Z'); }
    else if (strcmp(payload, "SAVE") == 0) { flash_save_requested = true; printf("[SYSTEM] Flash Save Requested
"); }
    else if (strcmp(payload, "SET_HOME") == 0) { Motor_SetHomeHere(); }
}