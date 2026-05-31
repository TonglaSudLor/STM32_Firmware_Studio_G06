/**
 * @file motor_controller.h
 * @brief Professional PID Motor Controller for STM32
 * 
 * This module provides a dual-loop (position/speed) PID controller with
 * trajectory generation, autotuning, and safety monitoring features.
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* --- Hardware Configuration --- */
#define MOTOR_ENCODER_PPR           2048    /**< Encoder pulses per revolution */
#define MOTOR_GEAR_RATIO            1.0f    /**< Motor gear ratio */
#define MOTOR_CONTROL_FREQ_HZ       100     /**< Control loop frequency (Hz) */

/* --- Control Structures --- */

/**
 * @brief Motor control operating modes
 */
typedef enum {
    MOTOR_MODE_STOPPED = 0,
    MOTOR_MODE_SPEED,
    MOTOR_MODE_POSITION,
    MOTOR_MODE_AUTOTUNE,
    MOTOR_MODE_AUTOTUNE_SPEED,
    MOTOR_MODE_TEST,
    MOTOR_MODE_GHOST,
    MOTOR_MODE_HOMING
} Motor_ControlMode_t;

/**
 * @brief System control modes (Base or Joystick)
 */
typedef enum {
    CONTROL_MODE_BASE_SYSTEM = 0,
    CONTROL_MODE_JOYSTICK = 1
} Control_SystemMode_t;

/**
 * @brief System fault codes (Bitmask)
 */
typedef enum {
    FAULT_NONE              = 0x000,
    /* Automatic safety faults (gated by safety_config) */
    FAULT_MOTOR_STALLED     = 0x001,
    FAULT_ENCODER_ERROR     = 0x002,
    FAULT_JOYSTICK_LOST     = 0x004,
    FAULT_OVER_ROTATION     = 0x008,   /**< Motor exceeded rotation limit */
    /* User / external e-stop sources (always raise; informational only) */
    FAULT_ESTOP_PHYSICAL    = 0x010,   /**< Hardware E-Stop button (EXTI) */
    FAULT_PROX_LOST         = 0x020,   /**< Proximity sensor open */
    FAULT_ESTOP_JOYSTICK    = 0x040,   /**< Joystick safety button (P/X) */
    FAULT_ESTOP_DASHBOARD   = 0x080,   /**< Dashboard EMERGENCY STOP button */
    FAULT_ESTOP_MODBUS      = 0x100,   /**< Modbus 0x25 soft-stop request */
    FAULT_STARTUP_ESTOP     = 0x200,   /**< Power-on latch — cleared only when self-test passes */
    FAULT_OVERCURRENT       = 0x400    /**< WCS1800 current exceeded OVERCURRENT_LIMIT_AMPS */
} Motor_FaultCode_t;

/* Atomic fault-bit helpers (bug 1-B). fault_code |= / &= are read-modify-write
 * and are mutated from both thread context (main loop) and the TIM6 / UART RX
 * ISRs. Guard each RMW with a PRIMASK save/disable/restore so a preempting
 * context cannot lose a just-set or just-cleared bit. PRIMASK save-restore is
 * nesting-safe (works whether called from thread or ISR context). */
#define FAULT_SET(bits)  do { uint32_t _fpm = __get_PRIMASK(); __disable_irq(); \
                              fault_code = (Motor_FaultCode_t)(fault_code | (bits)); \
                              __set_PRIMASK(_fpm); } while (0)
#define FAULT_CLR(bits)  do { uint32_t _fpm = __get_PRIMASK(); __disable_irq(); \
                              fault_code = (Motor_FaultCode_t)(fault_code & ~(bits)); \
                              __set_PRIMASK(_fpm); } while (0)

/**
 * @brief Jog operation modes
 */
typedef enum {
    JOG_COARSE = 0,
    JOG_FINE = 1
} Motor_JogMode_t;

/**
 * @brief Autotune trigger states
 */
typedef enum {
    ATUNE_IDLE = 0,
    ATUNE_POS,    /**< Start Position Autotune */
    ATUNE_SPEED   /**< Start Speed Autotune */
} Motor_AutotuneTrigger_t;

/**
 * @brief Autotune execution status
 */
typedef enum {
    STATUS_IDLE = 0,
    STATUS_RUNNING_POS,
    STATUS_RUNNING_SPEED,
    STATUS_SUCCESS,
    STATUS_ERROR_LIMIT_EXCEEDED
} Motor_AutotuneStatus_t;

/**
 * @brief Tuning parameters for PID loops
 */
typedef struct {
    // PID Speed Loop
    float speed_Kp;
    float speed_Ki;
    float speed_Kd;
    float K_vff;               /**< Velocity feedforward — V per (rad/s). Multiplies trajectory v_ref. */
    float K_aff;               /**< Acceleration feedforward — V per (rad/s^2). Multiplies trajectory a_ref. */
    float K_tff;               /**< Disturbance feedforward gain (0=off, 1=full). Scales Kalman τ_L → voltage. */
    
    // PID Position Loop
    float pos_Kp;
    float pos_Ki;
    float pos_Kd;
    
    // Jog & Step Settings
    float jog_speed_fine;      /**< RPM for continuous jog */
    float move_speed_coarse;   /**< RPM for step movement */
    float step_size_coarse;    /**< Degrees per click in Coarse mode */
    float step_size_fine;      /**< Degrees per click in Fine mode */
    float move_speed_return_home; /**< Custom RPM for returning to home/origin */
    float min_pwm;             /**< Minimum PWM to overcome friction */
    float max_accel;           /**< Maximum acceleration (RPM/s) — a_max for S-curve */
    float max_jerk;            /**< Maximum jerk (RPM/s²) — j_max for S-curve */

    // ZVD Input Shaper
    bool  shaper_enable;       /**< Enable ZVD input shaper */
    float shaper_omega_n;      /**< Shaper natural frequency (rad/s) */
    float shaper_zeta;         /**< Shaper damping ratio */

    // Homing offset
    float home_offset_deg;     /**< Shift encoder zero by this many degrees after sensor homing.
                                 *   +X deg means sensor center maps to position +X, so position 0
                                 *   is X degrees before the sensor (use to trim workspace origin). */
} Motor_TuningParams_t;

/**
 * @brief Individual safety toggles
 */
typedef struct {
    bool stall_prevent;        /**< Enable E-Stop on motor stall */
    bool encoder_check;        /**< Enable E-Stop on encoder loss/inversion */
    bool over_rotation_check;  /**< Enable E-Stop on soft limit breach */
    bool joystick_check;       /**< Enable E-Stop on joystick connection loss */
    bool physical_estop_check; /**< Enable E-Stop from physical pin (PA5) */
    /* Runtime-tunable thresholds (defaults seeded from params.h #defines).
     * Settable from the dashboard via SET:MAX_ROT / STALL_PWM / STALL_VEL /
     * STALL_TIME / STALL_ERR. */
    float    soft_limit_deg;   /**< Over-rotation soft limit (deg from home) */
    float    stall_pwm_pct;    /**< Min |PWM| % to consider a stall */
    float    stall_vel_rpm;    /**< Max |RPM| to consider a stall */
    uint32_t stall_time_ms;    /**< Sustain time before stall trips */
    float    stall_error_deg;  /**< Min position error to allow stall trip */
} SafetyConfig_t;

/**
 * @brief PID Controller state and configuration
 */
typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float integral_max;
    float error_prev;
    float d_filt;              /**< Low-pass filter state for derivative term */
    float output_min, output_max;
} PID_Controller_t;

/**
 * @brief Encoder state and measurement data
 */
typedef struct {
    int32_t absolute_counts;
    uint32_t count_prev;
    float current_position_deg;
    float filtered_rpm;
} Encoder_Data_t;

/**
 * @brief Trajectory configuration parameters
 */
typedef struct {
    float max_velocity;
    float max_acceleration;
    /* jerk is controlled by tuning.max_jerk, not here */
} Trajectory_Config_t;

/**
 * @brief current trajectory state
 */
typedef struct {
    float target_pos;
    float target_vel;
    float current_setpoint_pos;
    float current_setpoint_vel;
    float current_setpoint_accel;
} Trajectory_State_t;

/* --- Ghost Mode Buffering --- */
#define GHOST_BUFFER_MAX 1000
typedef struct {
    uint32_t tick;
    int32_t pos_x100;
    int32_t target_x100;
} Ghost_Buffer_t;

/* --- Public Variables --- */
extern volatile Motor_ControlMode_t current_mode;
extern volatile Control_SystemMode_t control_system_mode;
extern volatile Motor_JogMode_t jog_mode;
extern volatile Motor_AutotuneTrigger_t autotune_trigger;
extern volatile Motor_AutotuneStatus_t autotune_status;
extern volatile int tuning_progress;
extern volatile Motor_TuningParams_t tuning;
extern volatile bool is_joystick_connected;
extern volatile bool emergency_stop;
extern volatile bool position_unknown;
extern volatile bool position_loop_enabled;
extern volatile bool  sine_test_enabled;
extern volatile float sine_amp_rpm;
extern volatile float sine_freq_hz;
extern volatile SafetyConfig_t safety_config;
extern volatile Motor_FaultCode_t fault_code;
extern volatile bool startup_estop_pending;  /**< Cleared only when self-test passes (HARDWARE OK) */
extern volatile float target_position_deg;
extern volatile float buffered_target_pos;   /**< Ghost target for S-curve testing */
extern volatile bool ghost_move_active;
extern volatile uint32_t ghost_settle_start_tick;
extern volatile float original_home_offset_deg;
extern volatile bool trigger_homing_sequence;
extern volatile uint8_t gripper_seq_request;   /**< Deferred gripper sequence: 0=none, 1=pick, 2=place (bug 1-F) */

extern Ghost_Buffer_t ghost_buffer[GHOST_BUFFER_MAX];
extern volatile uint32_t ghost_buffer_idx;
extern volatile bool ghost_dump_requested;

extern PID_Controller_t pid_speed;
extern PID_Controller_t pid_position;
extern Encoder_Data_t encoder;
extern Trajectory_State_t trajectory;

/* --- Public Functions --- */

/**
 * @brief Initialize motor control system
 */
void Motor_Init(void);

/**
 * @brief Move motor to target position in degrees
 * @param target_degrees Target position
 */
void Motor_MoveToPosition(float target_degrees);

/**
 * @brief Set voltage limits for PWM mapping
 * @param max_voltage Maximum allowed voltage (V)
 * @param supply_voltage Actual supply voltage (V)
 */
void Motor_SetVoltageLimit(float max_voltage, float supply_voltage);

/**
 * @brief Configure motion profile for position control
 * @param max_rpm Maximum speed (RPM)
 * @param max_accel Maximum acceleration (RPM/s^2)
 * @param smoothing S-Curve smoothing factor (0.0 to 1.0)
 */
void Motor_SetMotionProfile(float max_rpm, float max_accel, float smoothing);

/**
 * @brief Set target velocity for speed control
 * @param rpm Target velocity in RPM
 */
void Motor_SetJogVelocity(float rpm);

/**
 * @brief Send audio command
 * @param sound_code Code for sound to play
 */
void Motor_SendAudioCommand(char sound_code);

/**
 * @brief Process incoming command characters
 * @param cmd Command character
 */
void Motor_ProcessCommand(char cmd);
void Motor_ProcessPacket(char action, char safety, char status);
bool Motor_RunHomingSequence(void);

/**
 * @brief Drain deferred control-loop log events and gripper requests.
 *        Call from the main loop (thread context). Prints the strings that
 *        used to be printf'd inside the 100 Hz TIM6 ISR (bug 1-G).
 */
void Motor_DrainControlLog(void);

/**
 * @brief Instantly declare the current encoder position as home (position 0).
 *        Equivalent to the joystick A single-click. Safe to call from
 *        telemetry or dashboard — does nothing while E-Stop is active.
 */
void Motor_SetHomeHere(void);

/**
 * @brief Update selection button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateSelectionButton(bool pressed);

/**
 * @brief Update mode button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateModeButton(bool pressed);

/**
 * @brief Update control mode button state
 * @param pressed True if button is pressed
 */
void Motor_UpdateControlModeButton(bool pressed);

/**
 * @brief Set joystick connection status
 * @param connected True if joystick is connected
 */
void Motor_SetConnectionStatus(bool connected);

/**
 * @brief Clear the gamepad-disconnect debounce (streak + last-status latch).
 *        Call when re-arming the joystick safety check after it was disabled
 *        (e.g. at the end of DIAG) so a stale non-'C' streak cannot instantly
 *        re-trip FAULT_JOYSTICK_LOST.
 */
void Motor_ResetJoystickDebounce(void);
void Motor_RefreshWatchdog(void);

/**
 * @brief Stream telemetry data to MATLAB
 */
void Motor_SendDataToMatlab(void);

/**
 * @brief Start relay-based position autotune
 */
void Motor_StartAutotune(void);

/**
 * @brief Start relay-based speed autotune
 */
void Motor_StartAutotuneSpeed(void);

/**
 * @brief Main control loop ISR (100Hz)
 */
void Motor_ControlLoop(void);

/**
 * @brief Recompute ZVD shaper coefficients after omega_n or zeta change
 */
void Motor_ShaperRecompute(void);

/**
 * @brief Toggle between JOYSTICK (Dashboard, LPUART1=115200 8N1)
 *        and BASE_SYSTEM (Modbus, LPUART1=19200 8E1). Defined in main.c.
 */
void Mode_Toggle(void);

/**
 * @brief Get current position in degrees
 * @return float Position (deg)
 */
float Motor_GetPosition(void);

/**
 * @brief Get current speed in RPM
 * @return float Speed (RPM)
 */
float Motor_GetSpeed(void);

/* --- Gripper Functions --- */
void Gripper_Up(void);
void Gripper_Down(void);
void Gripper_Open(void);
void Gripper_Close(void);
void Gripper_Toggle(void);
void Gripper_Sequence_Pick(void);
void Gripper_Sequence_Place(void);

#endif /* MOTOR_CONTROLLER_H */