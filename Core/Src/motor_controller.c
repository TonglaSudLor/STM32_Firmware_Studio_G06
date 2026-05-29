/**
 * @file motor_controller.c
 * @brief Professional PID Motor Controller for STM32
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#include "motor_controller.h"
#include "hw_io.h"
#include "params.h"
#include "kalman.h"
#include <math.h>
#include <stdio.h>

/* ============================================================================
 * Private System Variables
 * ============================================================================ */

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart3;

volatile Motor_ControlMode_t current_mode = MOTOR_MODE_STOPPED;
volatile Control_SystemMode_t control_system_mode = CONTROL_MODE_BASE_SYSTEM;
volatile Motor_JogMode_t jog_mode = JOG_COARSE;
volatile Motor_AutotuneTrigger_t autotune_trigger = ATUNE_IDLE;
volatile Motor_AutotuneStatus_t autotune_status = STATUS_IDLE;
volatile int tuning_progress = 0;
volatile Motor_TuningParams_t tuning;
volatile bool is_joystick_connected = false;
volatile bool emergency_stop = true;
/* Set to true when the physical (hardware) E-Stop fires — the motor relay
 * opens, which cuts power to the encoder too, so the TIM3 quadrature count
 * cannot be trusted across the outage. On recovery, the firmware ignores the
 * normal "return to original_home_offset_deg" path and forces a re-homing
 * sequence instead. Cleared once homing has been triggered. */
volatile bool position_unknown = false;
/* When false, the outer position PID is bypassed and the velocity PID is fed
 * the S-curve velocity setpoint directly. Use this to tune the inner loop
 * in isolation. Trajectory generator (and so v_ref / a_ref feedforward) still runs. */
volatile bool position_loop_enabled = true;

/* Velocity-loop sine-wave test generator. Used only when position_loop_enabled == false.
 * target_rpm = sine_amp_rpm * sin(2*pi*sine_freq_hz * t). */
volatile bool  sine_test_enabled = false;
volatile float sine_amp_rpm      = 30.0f;
volatile float sine_freq_hz      = 0.5f;
static   uint32_t sine_start_tick = 0;
volatile SafetyConfig_t safety_config = {
    .stall_prevent = true,
    .encoder_check = true,
    .over_rotation_check = true,
    .joystick_check = true,
    .physical_estop_check = true
};
volatile Motor_FaultCode_t fault_code = FAULT_NONE;
volatile float target_position_deg = 0.0f;
volatile float current_pwm = 0.0f;
volatile float buffered_target_pos = 0.0f;
volatile bool ghost_move_active = false;
volatile uint32_t ghost_settle_start_tick = 0;
volatile float original_home_offset_deg = 0.0f;
volatile bool trigger_homing_sequence = false;
/* Deferred gripper sequence request (bug 1-F): the joystick command arrives in
 * the USART3 RX ISR; the blocking Pick/Place sequence must NOT run there (it
 * busy-waits on reed switches for up to ~12 s, freezing the control loop). The
 * ISR only sets this flag; the main loop runs the sequence at thread level.
 * 0=none, 1=pick, 2=place. */
volatile uint8_t gripper_seq_request = 0;

/* Deferred control-loop logging (bug 1-G). printf blocks for milliseconds and
 * must not run inside the 100 Hz TIM6 ISR. The ISR sets an event bit (and
 * latches any value the message needs); the main loop calls
 * Motor_DrainControlLog() to emit the strings at thread level. Only the TIM6
 * ISR sets bits (single producer → plain |= is safe); the main loop snapshots
 * and clears under a PRIMASK guard so a concurrent ISR set is never lost. */
#define CLOG_ESTOP_PHYS_REHOME   0x0001u
#define CLOG_ESTOP_CLEARED_HOLD  0x0002u
#define CLOG_RELAY_SETTLED       0x0004u
#define CLOG_TEST_START          0x0008u
#define CLOG_TEST_FINISH         0x0010u
#define CLOG_HOME_TEMP_SET       0x0020u
#define CLOG_HOME_MOVE_TEMP      0x0040u
#define CLOG_HOME_MOVE_ORIG      0x0080u
#define CLOG_ENCODER_INVERTED    0x0100u
#define CLOG_ENCODER_NOSIGNAL    0x0200u
#define CLOG_MOTOR_STALLED       0x0400u
#define CLOG_SOFT_LIMIT          0x0800u

static volatile uint32_t clog_events         = 0;
static volatile float    clog_hold_pos       = 0.0f;  /* CLOG_ESTOP_CLEARED_HOLD */
static volatile float    clog_temp_home_orig = 0.0f;  /* CLOG_HOME_TEMP_SET */
static volatile float    clog_move_orig_pos  = 0.0f;  /* CLOG_HOME_MOVE_ORIG */
static volatile float    clog_move_speed     = 0.0f;  /* CLOG_HOME_MOVE_* RPM */

Ghost_Buffer_t ghost_buffer[GHOST_BUFFER_MAX];
volatile uint32_t ghost_buffer_idx = 0;
volatile bool ghost_dump_requested = false;

static float control_dt = 0.01f;
static uint32_t open_loop_test_tick = 0;
static uint32_t m_button_hold_tick = 0;
static bool m_button_active = false;
static uint32_t y_button_hold_tick = 0;
static bool y_button_active = false;
static uint32_t b_button_hold_tick = 0;
static bool b_button_active = false;
static float last_rpm_for_accel = 0.0f;
static uint32_t a_press_tick = 0;    
static uint32_t stall_timer = 0;
static uint32_t encoder_fault_timer = 0;
static uint32_t homing_settle_tick    = 0;    /* relay-settle delay start tick */
static bool     homing_settle_pending = false; /* waiting to arm re-home after E-stop clear */
static uint32_t joystick_watchdog_timer = 0;
static int32_t last_absolute_counts = 0;
static bool a_button_is_held = false;
static bool a_long_press_handled = false;
static uint32_t a_button_click_count = 0;
static uint32_t a_button_last_release_tick = 0;
static bool a_button_evaluating = false;

/* Autotune State Machine */
static struct {
    float relay_output; 
    float peak_max; 
    float peak_min;          
    uint32_t last_flip_tick; 
    uint32_t period_sum; 
    float amplitude_sum;     
    int cycle_count; 
    int8_t motor_sign; 
    bool direction;          
    float center_pos; 
    float target_val;        
} atune;

PID_Controller_t pid_speed;
PID_Controller_t pid_position;
Encoder_Data_t encoder;
Trajectory_Config_t motion_config;
Trajectory_State_t trajectory;

/* ============================================================================
 * Private Function Prototypes
 * ============================================================================ */
static float PID_Compute(PID_Controller_t *pid, float setpoint, float feedback);
static float PWM_Apply(float duty_cycle);
static float Motor_DriveWithAntiWindup(float pid_out, float ff);
static void Encoder_Update(void);
static void Trajectory_Generator_Update(void);
void Motor_SendAudioCommand(char sound_code);

/* ============================================================================
 * PID Control Implementation
 * ============================================================================ */

/**
 * @brief Compute PID output
 */
static float PID_Compute(PID_Controller_t *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;
    float p_term = pid->Kp * error;
    
    pid->integral += error * control_dt;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    
    float i_term = pid->Ki * pid->integral;
    float d_input = (feedback - pid->error_prev); 
    float d_term_raw = (pid->Kd * d_input) / control_dt;
    
    // Low-pass filter for derivative term
    pid->d_filt = (0.8f * pid->d_filt) + (0.2f * d_term_raw); 
    pid->error_prev = feedback;
    
    float output = p_term + i_term - pid->d_filt;
    
    // Output clamping with anti-windup
    if (pid->output_max > 0.0f || pid->output_min < 0.0f) {
        if (output > pid->output_max) {
            output = pid->output_max;
            // Anti-windup: stop integrating in the saturated direction
            if (error > 0) pid->integral -= error * control_dt;
        } else if (output < pid->output_min) {
            output = pid->output_min;
            if (error < 0) pid->integral -= error * control_dt;
        }
    }
    
    return output;
}

/**
 * @brief Apply PWM duty cycle to motor.
 * @return The final signed duty (%) actually applied to TIM1 AFTER min_pwm
 *         friction offset and hardware saturation. This is the true actuator
 *         output; callers route it back to the Kalman observer (current_pwm)
 *         and to anti-windup so the controller and observer never desync from
 *         the physical PWM. (Fix 2-E)
 */
static float PWM_Apply(float duty_cycle)
{
    float min_p = tuning.min_pwm;

    if (fabsf(duty_cycle) < 0.1f) {
        duty_cycle = 0.0f;
    } else {
        if (duty_cycle > 0) duty_cycle += min_p;
        else duty_cycle -= min_p;
    }

    if (duty_cycle > pid_speed.output_max) duty_cycle = pid_speed.output_max;
    else if (duty_cycle < pid_speed.output_min) duty_cycle = pid_speed.output_min;

    bool forward = (duty_cycle >= 0);
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pwm_value = (uint32_t)((arr * fabsf(duty_cycle)) / 100.0f);

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
    HAL_GPIO_WritePin(Motor_Direction_GPIO_Port, Motor_Direction_Pin, forward ? GPIO_PIN_RESET : GPIO_PIN_SET);

    /* Publish the true applied voltage-equivalent to the Kalman observer.
     * Every drive path that goes through PWM_Apply now keeps the observer's
     * input u in sync with the physical PWM, including saturated commands. */
    current_pwm = duty_cycle;
    return duty_cycle;
}

/**
 * @brief Drive the inner speed loop with feedforward and back-calculation
 *        anti-windup. (Fixes 2-A)
 *
 * The speed PID clamps only its OWN output, so it is blind to the extra
 * headroom consumed by the feedforward term: pid_out may be in-range while
 * (pid_out + ff) saturates the actuator. Left uncorrected, the integrator
 * winds up against a limit it cannot observe.
 *
 * Here we form the linear command u_cmd = pid_out + ff, compute the value the
 * actuator can actually realise within its LINEAR limits (output_min/max, i.e.
 * excluding the min_pwm friction offset, which is a feedforward not a windup
 * source), and bleed the integrator by the un-realisable excess scaled by
 * 1/Ki — the standard tracking back-calculation law. Returns the true applied
 * PWM from PWM_Apply (which also clamps + adds friction offset).
 */
static float Motor_DriveWithAntiWindup(float pid_out, float ff)
{
    float u_cmd = pid_out + ff;

    float u_sat = u_cmd;
    if (u_sat > pid_speed.output_max)      u_sat = pid_speed.output_max;
    else if (u_sat < pid_speed.output_min) u_sat = pid_speed.output_min;

    if (pid_speed.Ki > 0.0f) {
        pid_speed.integral -= (u_cmd - u_sat) / pid_speed.Ki;
        if (pid_speed.integral > pid_speed.integral_max)        pid_speed.integral = pid_speed.integral_max;
        else if (pid_speed.integral < -pid_speed.integral_max)  pid_speed.integral = -pid_speed.integral_max;
    }

    return PWM_Apply(u_cmd);
}

/**
 * @brief Update encoder readings and calculate RPM
 */
static void Encoder_Update(void)
{
    uint32_t current_count = __HAL_TIM_GET_COUNTER(&htim3);
    int32_t delta = (int32_t)current_count - (int32_t)encoder.count_prev;
    
    // Handle timer rollover
    if (delta > 32767) delta -= 65536;
    else if (delta < -32768) delta += 65536;
    
    // EMI Noise Filter: Reject physically impossible jumps
    // (Max 500 RPM = ~680 counts per 10ms tick. >2000 is relay noise)
    if (delta > 2000 || delta < -2000) {
        delta = 0;
    }
    
    // Invert encoder direction to match motor phase
    delta = -delta;
    
    encoder.count_prev = current_count;
    encoder.absolute_counts += delta;
    
    float counts_per_rev = MOTOR_ENCODER_PPR * 4 * MOTOR_GEAR_RATIO;
    encoder.current_position_deg = ((float)encoder.absolute_counts / counts_per_rev) * 360.0f;
    
    float instant_rpm = ((float)delta / counts_per_rev / control_dt) * 60.0f;
    encoder.filtered_rpm = (0.15f * instant_rpm) + (0.85f * encoder.filtered_rpm);
}

/* ============================================================================
 * 7-Segment S-Curve Trajectory Generator
 *
 * Each move is pre-planned analytically into seven phases:
 *   1. J+   jerk up        (a: 0 → +a_peak)
 *   2. A+   const accel
 *   3. J-   jerk down      (a: +a_peak → 0,  v reaches v_peak)
 *   4. V    const velocity cruise
 *   5. J-   jerk down      (a: 0 → -a_peak)
 *   6. A-   const decel
 *   7. J+   jerk up        (a: -a_peak → 0,  v reaches 0 at target)
 *
 * The planner handles three reduced cases automatically:
 *   - Triangular A profile (move too short to reach a_max)
 *   - Triangular V profile (move too short to reach v_max)
 *   - Triangular A+V profile (very short move)
 *
 * Units inside the planner: degrees, deg/s, deg/s², deg/s³. Convert at the
 * boundary with the rest of the firmware (RPM, RPM/s, RPM/s²) by ×6 / ÷6.
 * ========================================================================== */

typedef struct {
    bool   active;
    int8_t direction;          /* +1 forward, -1 reverse */
    float  pos_start_deg;
    float  pos_target_deg;     /* target snapshot at plan time */
    float  distance;           /* |target - start|, degrees */
    float  t_elapsed;          /* seconds since plan began */
    /* Phase durations (seconds) */
    float  T_j, T_a, T_v;
    /* Achieved peaks (may be < limits for short moves) */
    float  a_peak_dps2;
    float  v_peak_dps;
    float  jmax_dps3;
    /* Cumulative time boundaries */
    float  t1, t2, t3, t4, t5, t6, t7;
    /* Pre-computed state at boundaries (relative pos, deg-positive) */
    float  v1_dps, v2_dps;
    float  p1_deg, p2_deg, p3_deg, p4_deg, p5_deg, p6_deg;
} SCurvePlan_t;

static SCurvePlan_t s_plan = {0};

/* ============================================================================
 * ZVD Input Shaper
 * Pre-computed amplitudes (A1, A2, A3) and delay-tick count (N) are updated
 * only when parameters change — never inside the 100 Hz ISR — so expf/sqrtf
 * never run per-tick.
 * ========================================================================== */
#define SHAPER_BUF_SIZE  100   /* 100 ticks @ 100 Hz = 1 s max total delay */

static float    shaper_buf[SHAPER_BUF_SIZE];
static float    shaper_buf_v[SHAPER_BUF_SIZE];  /* velocity FF, same delay line (Fix 2-B) */
static float    shaper_buf_a[SHAPER_BUF_SIZE];  /* accel FF,    same delay line (Fix 2-B) */
static uint32_t shaper_buf_idx = 0;
static float    shaper_A1 = 1.0f, shaper_A2 = 0.0f, shaper_A3 = 0.0f;
static uint32_t shaper_N  = 0;
static float    shaper_last_output = 0.0f;   /* last shaped setpoint, for stall check (bug 0-I) */

static void ZVD_UpdateCoefficients(void)
{
    float wn   = tuning.shaper_omega_n;
    float zeta = tuning.shaper_zeta;
    if (wn   <= 0.0f) wn   = 1.0f;
    if (zeta <  0.0f) zeta = 0.0f;
    if (zeta >= 1.0f) zeta = 0.999f;   /* underdamped only */

    float sqrt1z2 = sqrtf(1.0f - zeta * zeta);
    float K       = expf(-zeta * 3.14159265f / sqrt1z2);
    float K2      = K * K;
    float denom   = 1.0f + 2.0f * K + K2;

    shaper_A1 = 1.0f      / denom;
    shaper_A2 = 2.0f * K  / denom;
    shaper_A3 = K2        / denom;

    float    T_d = 3.14159265f / (wn * sqrt1z2);
    uint32_t N   = (uint32_t)(T_d * (float)MOTOR_CONTROL_FREQ_HZ + 0.5f);
    if (N < 1u) N = 1u;
    if (2u * N >= (uint32_t)SHAPER_BUF_SIZE) N = ((uint32_t)SHAPER_BUF_SIZE - 1u) / 2u;
    shaper_N = N;
}

static void ZVD_FlushBuffer(float val)
{
    for (int i = 0; i < SHAPER_BUF_SIZE; i++) {
        shaper_buf[i]   = val;
        shaper_buf_v[i] = 0.0f;   /* velocity/accel delay lines start at rest */
        shaper_buf_a[i] = 0.0f;
    }
}

void Motor_ShaperRecompute(void)
{
    ZVD_UpdateCoefficients();
}

/* Conversion helpers — firmware is in RPM/RPM/s/RPM/s², planner is in
 * deg/s, deg/s², deg/s³. 1 RPM = 6 deg/s. */
#define RPM_TO_DPS    6.0f
#define DPS_TO_RPM   (1.0f / 6.0f)

static void scurve_plan(float pos_start, float pos_target,
                        float vmax_dps, float amax_dps2, float jmax_dps3)
{
    s_plan.t_elapsed       = 0.0f;
    s_plan.pos_start_deg   = pos_start;
    s_plan.pos_target_deg  = pos_target;
    s_plan.distance        = fabsf(pos_target - pos_start);
    s_plan.direction       = (pos_target >= pos_start) ? +1 : -1;
    s_plan.jmax_dps3       = jmax_dps3;

    /* Trivial moves: already there or bad limits → flat plan */
    if (s_plan.distance < 0.001f ||
        vmax_dps   <= 0.0f ||
        amax_dps2  <= 0.0f ||
        jmax_dps3  <= 0.0f)
    {
        s_plan.T_j = s_plan.T_a = s_plan.T_v = 0.0f;
        s_plan.a_peak_dps2 = 0.0f;
        s_plan.v_peak_dps  = 0.0f;
        s_plan.t1 = s_plan.t2 = s_plan.t3 = s_plan.t4 =
        s_plan.t5 = s_plan.t6 = s_plan.t7 = 0.0f;
        s_plan.active = false;
        return;
    }

    /* Step 1: can the accel ramp reach a_max within the available v_max? */
    float T_j_full = amax_dps2 / jmax_dps3;
    float v_after_full_jerks = amax_dps2 * T_j_full;     /* = a²/j, both jerk phases combined */

    float T_j, T_a, a_peak, v_peak;
    if (vmax_dps >= v_after_full_jerks) {
        /* Yes — accel reaches a_max */
        T_j   = T_j_full;
        a_peak = amax_dps2;
        T_a    = vmax_dps / amax_dps2 - T_j;             /* ≥ 0 */
        v_peak = vmax_dps;                                /* tentative */
    } else {
        /* No — triangular A profile, a_peak < a_max */
        a_peak = sqrtf(vmax_dps * jmax_dps3);
        T_j    = a_peak / jmax_dps3;
        T_a    = 0.0f;
        v_peak = vmax_dps;
    }

    /* Step 2: does the move have room for a V cruise? */
    float d_accel = v_peak * (2.0f * T_j + T_a) * 0.5f;
    float T_v;
    if (2.0f * d_accel <= s_plan.distance) {
        /* Trapezoidal V profile */
        T_v = (s_plan.distance - 2.0f * d_accel) / v_peak;
    } else {
        /* No cruise. Recompute v_peak for triangular V. */
        T_v = 0.0f;

        /* Try: still reach a_max — solve  d = v_p·T_j + v_p²/a_max */
        float aTj  = amax_dps2 * T_j_full;
        float disc = aTj * aTj + 4.0f * amax_dps2 * s_plan.distance;
        float v_try = (-aTj + sqrtf(disc)) * 0.5f;

        if (v_try >= v_after_full_jerks) {
            /* Still reaches a_max */
            v_peak = v_try;
            a_peak = amax_dps2;
            T_j    = T_j_full;
            T_a    = v_peak / amax_dps2 - T_j;
        } else {
            /* Triangular A AND V: d = 2·v·sqrt(v/j) → v = (d²·j/4)^(1/3) */
            float d_sq = s_plan.distance * s_plan.distance;
            v_peak = powf(d_sq * jmax_dps3 * 0.25f, 1.0f / 3.0f);
            a_peak = sqrtf(v_peak * jmax_dps3);
            T_j    = a_peak / jmax_dps3;
            T_a    = 0.0f;
        }
    }

    /* Store */
    s_plan.T_j         = T_j;
    s_plan.T_a         = T_a;
    s_plan.T_v         = T_v;
    s_plan.a_peak_dps2 = a_peak;
    s_plan.v_peak_dps  = v_peak;

    /* Boundary times */
    s_plan.t1 = T_j;
    s_plan.t2 = s_plan.t1 + T_a;
    s_plan.t3 = s_plan.t2 + T_j;
    s_plan.t4 = s_plan.t3 + T_v;
    s_plan.t5 = s_plan.t4 + T_j;
    s_plan.t6 = s_plan.t5 + T_a;
    s_plan.t7 = s_plan.t6 + T_j;

    /* Pre-compute relative state at each boundary to avoid recomputing every tick */
    s_plan.v1_dps = 0.5f * jmax_dps3 * T_j * T_j;                                                            /* end of J+ */
    s_plan.v2_dps = s_plan.v1_dps + a_peak * T_a;                                                            /* end of A+ */
    s_plan.p1_deg = jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p2_deg = s_plan.p1_deg + s_plan.v1_dps * T_a + 0.5f * a_peak * T_a * T_a;
    s_plan.p3_deg = s_plan.p2_deg + s_plan.v2_dps * T_j + 0.5f * a_peak * T_j * T_j
                                    - jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p4_deg = s_plan.p3_deg + v_peak * T_v;
    s_plan.p5_deg = s_plan.p4_deg + v_peak * T_j - jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p6_deg = s_plan.p5_deg + s_plan.v2_dps * T_a - 0.5f * a_peak * T_a * T_a;

    s_plan.active = true;
}

/* Evaluate the plan at time t. Outputs are RELATIVE (always non-negative pos,
 * always non-negative vel/accel during accel phase). The caller applies sign
 * via plan.direction. */
static void scurve_eval(float t, float *p_rel, float *v_dps, float *a_dps2)
{
    float T_j   = s_plan.T_j;
    float a_pk  = s_plan.a_peak_dps2;
    float v_pk  = s_plan.v_peak_dps;
    float jmax  = s_plan.jmax_dps3;

    if (t <= 0.0f)             { *p_rel = 0.0f;              *v_dps = 0.0f; *a_dps2 = 0.0f; return; }
    if (t >= s_plan.t7)        { *p_rel = s_plan.distance;   *v_dps = 0.0f; *a_dps2 = 0.0f; return; }

    if (t < s_plan.t1) {                                /* J+ */
        *a_dps2 =        jmax * t;
        *v_dps  = 0.5f * jmax * t * t;
        *p_rel  =        jmax * t * t * t / 6.0f;
    } else if (t < s_plan.t2) {                         /* A+ */
        float dt = t - s_plan.t1;
        *a_dps2 = a_pk;
        *v_dps  = s_plan.v1_dps + a_pk * dt;
        *p_rel  = s_plan.p1_deg + s_plan.v1_dps * dt + 0.5f * a_pk * dt * dt;
    } else if (t < s_plan.t3) {                         /* J- (still in accel half) */
        float dt = t - s_plan.t2;
        *a_dps2 = a_pk - jmax * dt;
        *v_dps  = s_plan.v2_dps + a_pk * dt - 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p2_deg + s_plan.v2_dps * dt + 0.5f * a_pk * dt * dt
                                  - jmax * dt * dt * dt / 6.0f;
    } else if (t < s_plan.t4) {                         /* V cruise */
        float dt = t - s_plan.t3;
        *a_dps2 = 0.0f;
        *v_dps  = v_pk;
        *p_rel  = s_plan.p3_deg + v_pk * dt;
    } else if (t < s_plan.t5) {                         /* J- (decel side) */
        float dt = t - s_plan.t4;
        *a_dps2 = -jmax * dt;
        *v_dps  =  v_pk - 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p4_deg + v_pk * dt - jmax * dt * dt * dt / 6.0f;
    } else if (t < s_plan.t6) {                         /* A- */
        float dt = t - s_plan.t5;
        *a_dps2 = -a_pk;
        *v_dps  = s_plan.v2_dps - a_pk * dt;
        *p_rel  = s_plan.p5_deg + s_plan.v2_dps * dt - 0.5f * a_pk * dt * dt;
    } else {                                            /* J+ final */
        float dt = t - s_plan.t6;
        *a_dps2 = -a_pk + jmax * dt;
        *v_dps  = s_plan.v1_dps - a_pk * dt + 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p6_deg + s_plan.v1_dps * dt - 0.5f * a_pk * dt * dt
                                  + jmax * dt * dt * dt / 6.0f;
    }
}

static void Trajectory_Generator_Update(void)
{
    /* Convert RPM-units → deg/s-units for planner */
    float vmax_dps  = motion_config.max_velocity     * RPM_TO_DPS;
    float amax_dps2 = motion_config.max_acceleration * RPM_TO_DPS;
    float jmax_dps3 = tuning.max_jerk                * RPM_TO_DPS;

    /* (Re-)plan if no active plan OR the target has changed since planning.
     * The new plan starts from the current setpoint_pos so that
     * setpoint_pos is continuous across the re-plan; setpoint_vel/accel
     * snap to 0 (which causes a transient the speed PID will absorb — see
     * SCurve notes). */
    if (!s_plan.active || trajectory.target_pos != s_plan.pos_target_deg) {
        scurve_plan(trajectory.current_setpoint_pos,
                    trajectory.target_pos,
                    vmax_dps, amax_dps2, jmax_dps3);
    }

    if (!s_plan.active) {
        /* No motion — hold at target */
        trajectory.current_setpoint_pos   = trajectory.target_pos;
        trajectory.current_setpoint_vel   = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;
        return;
    }

    s_plan.t_elapsed += control_dt;

    float p_rel, v_dps, a_dps2;
    scurve_eval(s_plan.t_elapsed, &p_rel, &v_dps, &a_dps2);

    trajectory.current_setpoint_pos   = s_plan.pos_start_deg + (float)s_plan.direction * p_rel;
    trajectory.current_setpoint_vel   = (float)s_plan.direction * v_dps * DPS_TO_RPM;     /* RPM */
    trajectory.current_setpoint_accel = (float)s_plan.direction * a_dps2 * DPS_TO_RPM;    /* RPM/s */

    if (s_plan.t_elapsed >= s_plan.t7) {
        s_plan.active = false;
        trajectory.current_setpoint_pos   = s_plan.pos_target_deg;
        trajectory.current_setpoint_vel   = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;

        /* Trajectory finished. The position PID has accumulated integral
         * during the whole move because encoder always lags the setpoint
         * (finite Kp needs error to produce response). When the trajectory
         * snaps to target with vel=0, the residual integral keeps pushing
         * the motor forward → overshoot → PID then slams reverse →
         * mechanical jerk and motor-noise EMI bursts. Clear the position
         * integral right at trajectory end so only proportional + the
         * still-correct derivative term command the final settling. */
        pid_position.integral   = 0.0f;
        pid_position.d_filt     = 0.0f;
    }
}

static float resolution_step = 10.0f; 
static bool step_executed = false;   

/* Homing State Variables */
typedef enum {
    H_IDLE = 0,
    H_INIT,
    H_WIGGLE_SEARCH,
    H_FIND_EDGE_A,
    H_FIND_EDGE_B,
    H_VERIFY_EDGE_B,    /**< Re-approach edge B from the opposite side to cancel sensor hysteresis */
    H_VERIFY_EDGE_A,    /**< Continue back through the zone and re-cross edge A from the opposite side */
    H_CALCULATE_ZERO,
    H_DONE,
    H_ERROR
} HomingState_t;

#define HOMING_VERIFY_OVERSHOOT_DEG  1.5f   /**< how far past edge_b we travel before reversing */
#define HOMING_VERIFY_MAX_TRAVEL_DEG 30.0f  /**< abort verify if we travel this far without finding the next edge */

static HomingState_t h_state = H_IDLE;
static float h_edge_a = 0, h_edge_b = 0, h_edge_a_verify = 0, h_edge_b_verify = 0;
static float h_verify_reverse_start = 0;     /**< encoder pos at the moment we reverse, used as travel reference */
static float h_verify_a_start = 0;           /**< encoder pos when the edge-A verify phase begins */
static float h_wiggle_amp = 20.0f;
static float h_start_pos = 0;
static int h_direction = 1;
static bool h_verify_overshot = false;

/**
 * @brief Standalone Homing Sequence Function
 * @return true if homing is finished/successful, false while running.
 */
bool Motor_RunHomingSequence(void)
{
    // If emergency stop is active, reset homing
    if (emergency_stop) {
        h_state = H_IDLE;
        return true;
    }

    // If a previous error left state dirty, reset so a new trigger restarts cleanly
    if (h_state == H_ERROR) {
        h_state = H_IDLE;
    }

    switch (h_state) {
        case H_IDLE:
            h_state = H_INIT;
            return false;

        case H_INIT:
            h_start_pos = encoder.current_position_deg;
            h_wiggle_amp = 20.0f;
            h_direction = 1;
            current_mode = MOTOR_MODE_HOMING;
            // Set slow motion profile for homing
            Motor_SetMotionProfile(HOMING_SEARCH_RPM, 50.0f, 0.1f);
            h_state = H_WIGGLE_SEARCH;
            printf("[HOMING] Starting Smooth Wiggle Search...\r\n");
            return false;

        case H_WIGGLE_SEARCH:
            // Set smooth PID target
            trajectory.target_pos = h_start_pos + (float)h_direction * h_wiggle_amp;

            // Check sensor (PB9) - 0 means detected
            if (!hw.raw_prox_bit) { 
                h_state = H_FIND_EDGE_A;
                printf("[HOMING] Target Found! Locating Edge A...\r\n");
                return false;
            }

            // If we reached the wiggle target without finding anything, flip and expand
            float error = trajectory.target_pos - encoder.current_position_deg;
            if (fabsf(error) < 1.0f) {
                h_direction *= -1;
                h_wiggle_amp += 20.0f;
                if (h_wiggle_amp > HOMING_MAX_WIGGLE) {
                    h_state = H_ERROR;
                    printf("[HOMING] ERROR: Target not found within limits.\r\n");
                    Motor_SendAudioCommand('E');
                }
            }
            return false;

        case H_FIND_EDGE_A:
            // Creep slowly (3 RPM) until sensor triggers
            Motor_SetMotionProfile(HOMING_CREEP_RPM, 20.0f, 0.1f);
            trajectory.target_pos += (float)h_direction * 0.5f; // Constant slow move

            if (!hw.raw_prox_bit) {
                h_edge_a = encoder.current_position_deg;
                h_state = H_FIND_EDGE_B;
                printf("[HOMING] Edge A: %.2f. Finding Edge B...\r\n", h_edge_a);
            }
            return false;

        case H_FIND_EDGE_B:
            // Continue moving until sensor releases
            trajectory.target_pos += (float)h_direction * 0.5f;

            if (hw.raw_prox_bit) {
                h_edge_b = encoder.current_position_deg;
                h_verify_overshot = false;
                h_state = H_VERIFY_EDGE_B;
                printf("[HOMING] Edge B (first): %.2f. Verifying from the other side...\r\n", h_edge_b);
            }
            return false;

        case H_VERIFY_EDGE_B:
            /* Step 1: overshoot a small amount past edge B in the original
             * direction so we're cleanly outside the sensor's detection zone.
             * Step 2: reverse and creep back. The sensor should trigger again
             * at the same physical edge but from the other side; averaging
             * the two readings cancels the hysteresis offset.
             * Safety: if the reverse creep travels too far without seeing
             * the sensor (sensor too narrow, mechanical slip, etc.), abort
             * the verify and fall back to the original edge_b reading rather
             * than spin a full revolution looking for it. */
            if (!h_verify_overshot) {
                trajectory.target_pos += (float)h_direction * 0.5f;
                float traveled = (encoder.current_position_deg - h_edge_b) * (float)h_direction;
                if (traveled >= HOMING_VERIFY_OVERSHOOT_DEG) {
                    h_verify_overshot = true;
                    h_direction = -h_direction;   /* reverse */
                    h_verify_reverse_start = encoder.current_position_deg;
                    printf("[HOMING] Overshot %.2f deg, reversing to verify Edge B\r\n",
                           HOMING_VERIFY_OVERSHOOT_DEG);
                }
                return false;
            }

            /* Reversed creep — watch for the sensor to trigger again */
            trajectory.target_pos += (float)h_direction * 0.5f;
            if (!hw.raw_prox_bit) {
                h_edge_b_verify = encoder.current_position_deg;
                printf("[HOMING] Edge B (verify): %.2f.  Avg: %.2f\r\n",
                       h_edge_b_verify, 0.5f * (h_edge_b + h_edge_b_verify));
                h_edge_b = 0.5f * (h_edge_b + h_edge_b_verify);
                /* We're now inside the sensor zone heading back toward edge A.
                 * Continue and watch for the sensor to RELEASE — that's edge A
                 * from the opposite side, which lets us average out its
                 * hysteresis the same way we just did for edge B. */
                h_verify_a_start = encoder.current_position_deg;
                h_state = H_VERIFY_EDGE_A;
                return false;
            }

            /* Safety net: never let verify travel more than the cap. If we
             * exhaust it, skip the verify and use the original edges. */
            float back_traveled = (h_verify_reverse_start - encoder.current_position_deg)
                                  * (float)(-h_direction);
            if (back_traveled >= HOMING_VERIFY_MAX_TRAVEL_DEG) {
                printf("[HOMING] Verify abort (B): traveled %.2f deg without re-detecting sensor. "
                       "Falling back to Edge B = %.2f\r\n", back_traveled, h_edge_b);
                h_state = H_CALCULATE_ZERO;
            }
            return false;

        case H_VERIFY_EDGE_A:
            /* Same reversed direction as during H_VERIFY_EDGE_B. We're inside
             * the sensor zone. Keep going; the sensor will release as we exit
             * across edge A from the opposite side. */
            trajectory.target_pos += (float)h_direction * 0.5f;
            if (hw.raw_prox_bit) {
                h_edge_a_verify = encoder.current_position_deg;
                printf("[HOMING] Edge A (verify): %.2f.  Avg: %.2f\r\n",
                       h_edge_a_verify, 0.5f * (h_edge_a + h_edge_a_verify));
                h_edge_a = 0.5f * (h_edge_a + h_edge_a_verify);
                h_state = H_CALCULATE_ZERO;
                return false;
            }

            /* Safety net for edge A verify too. */
            float a_traveled = (h_verify_a_start - encoder.current_position_deg)
                               * (float)(-h_direction);
            if (a_traveled >= HOMING_VERIFY_MAX_TRAVEL_DEG) {
                printf("[HOMING] Verify abort (A): traveled %.2f deg without releasing. "
                       "Falling back to Edge A = %.2f\r\n", a_traveled, h_edge_a);
                h_state = H_CALCULATE_ZERO;
            }
            return false;

        case H_CALCULATE_ZERO:
            {
                float center = (h_edge_a + h_edge_b) / 2.0f;
                /* Distance from current physical position to sensor centre
                 * in the OLD encoder frame. */
                float offset = encoder.current_position_deg - center;

                /* Apply home_offset_deg — shift position 0 relative to sensor centre.
                 *
                 *   sensor centre maps to position  (-home_offset_deg)
                 *   position 0 lands at             (sensor_centre + home_offset_deg)
                 *
                 * Sign rule (simple):
                 *   +offset → position 0 is that many degrees PAST  the sensor
                 *   -offset → position 0 is that many degrees BEFORE the sensor
                 *
                 * Examples:
                 *   offset =  0.0  → home = sensor centre (default)
                 *   offset = +5.0  → arm drives 5° past sensor, stops there as home
                 *   offset = -5.0  → arm stops 5° before sensor as home
                 */
                encoder.absolute_counts = (int32_t)(
                    ((offset - tuning.home_offset_deg) / 360.0f) * (MOTOR_ENCODER_PPR * 4));
                Encoder_Update(); // Force recalculation

                trajectory.target_pos = 0.0f;
                trajectory.current_setpoint_pos = encoder.current_position_deg;
                trajectory.current_setpoint_vel = 0.0f;

                // Reset PID Integrals to prevent windup spikes
                pid_speed.integral = 0.0f;
                pid_speed.error_prev = 0.0f;
                pid_position.integral = 0.0f;
                pid_position.error_prev = 0.0f;

                current_mode = MOTOR_MODE_POSITION;
                original_home_offset_deg = 0.0f; // Sync temp and original home
                Motor_SetMotionProfile(tuning.move_speed_coarse, tuning.max_accel, 0.1f); // Restore speeds

                printf("[HOMING] SUCCESS. Center=%.2f, Offset=%.2f, Home=0.0\r\n",
                       center, tuning.home_offset_deg);
                Motor_SendAudioCommand('H');
                h_state = H_IDLE;  // ready for next trigger
            }
            return true;  // signal done — clears trigger_homing_sequence

        case H_DONE:
            h_state = H_IDLE;
            return true;

        case H_ERROR:
            current_mode = MOTOR_MODE_STOPPED;
            return true;
            
        default: break;
    }
    return false;
}

/* ============================================================================
 * Public API Implementation
 * ============================================================================ */

void Motor_Init(void)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encoder.count_prev = 0; 
    encoder.absolute_counts = 0;
    
    // Initialize parameters from defaults
    tuning.speed_Kp = DEFAULT_SPEED_KP; 
    tuning.speed_Ki = DEFAULT_SPEED_KI; 
    tuning.speed_Kd = DEFAULT_SPEED_KD;
    tuning.pos_Kp = DEFAULT_POS_KP; 
    tuning.pos_Ki = DEFAULT_POS_KI; 
    tuning.pos_Kd = DEFAULT_POS_KD;
    tuning.K_vff    = DEFAULT_K_VFF;
    tuning.K_aff    = DEFAULT_K_AFF;
    tuning.K_tff    = DEFAULT_K_TFF;
    
    tuning.jog_speed_fine = JOG_SPEED_FINE; 
    tuning.move_speed_coarse = MOVE_SPEED_COARSE;
    tuning.move_speed_return_home = MOVE_SPEED_RETURN_HOME;
    tuning.step_size_coarse = STEP_SIZE_COARSE; 
    tuning.step_size_fine = STEP_SIZE_FINE;
    tuning.min_pwm = DEFAULT_MIN_PWM;
    tuning.max_accel = DEFAULT_MAX_ACCEL;
    tuning.max_jerk  = DEFAULT_MAX_JERK;

    tuning.shaper_enable  = false;
    tuning.shaper_omega_n = DEFAULT_SHAPER_OMEGA_N;
    tuning.shaper_zeta    = DEFAULT_SHAPER_ZETA;
    tuning.home_offset_deg = DEFAULT_HOME_OFFSET;
    ZVD_UpdateCoefficients();
    ZVD_FlushBuffer(0.0f);

    resolution_step = tuning.step_size_coarse;
    
    // Initialize PID instances
    pid_speed.Kp = tuning.speed_Kp; 
    pid_speed.Ki = tuning.speed_Ki; 
    pid_speed.Kd = tuning.speed_Kd;
    pid_speed.integral_max = PID_INTEGRAL_MAX; 
    pid_speed.output_max = 100.0f; 
    pid_speed.output_min = -100.0f;
    
    pid_position.Kp = tuning.pos_Kp; 
    pid_position.Ki = tuning.pos_Ki; 
    pid_position.Kd = tuning.pos_Kd;
    pid_position.integral_max = POS_INTEGRAL_MAX;
    pid_position.output_max = 200.0f;  // Max RPM command from position loop
    pid_position.output_min = -200.0f;
    
    // Initialize motion profile defaults with user-calculated S-curve params
    motion_config.max_velocity = tuning.move_speed_coarse;
    motion_config.max_acceleration = tuning.max_accel;
    
    // Start hardware peripherals
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_Base_Start_IT(&htim6);
}

void Motor_MoveToPosition(float target_degrees)
{
    if (emergency_stop) return;
    trajectory.target_pos = target_degrees;
    current_mode = MOTOR_MODE_POSITION;
    motion_config.max_velocity    = tuning.move_speed_coarse;
    motion_config.max_acceleration = tuning.max_accel;
}

void Motor_Stop(void)
{
    PWM_Apply(0.0f);
    current_mode = MOTOR_MODE_STOPPED;
}

/**
 * @brief Instantly declare the current encoder position as home (position 0).
 *        Safe to call from telemetry CMD handler or any non-ISR context.
 *        Does nothing while E-Stop is active.
 */
void Motor_SetHomeHere(void)
{
    if (emergency_stop) return;

    Motor_SendAudioCommand('T');

    /* Accumulate the shift into original_home_offset_deg so a subsequent
     * triple-click (or Go-Original-Home) still returns to the sensor-based
     * homed position, not the new manual zero. */
    original_home_offset_deg -= encoder.current_position_deg;

    /* Zero the encoder counter and all derived state at the current position */
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    encoder.count_prev            = 0;
    encoder.absolute_counts       = 0;
    encoder.current_position_deg  = 0.0f;
    encoder.filtered_rpm          = 0.0f;

    /* Snap trajectory to the new zero so no position error is commanded */
    trajectory.target_pos             = 0.0f;
    trajectory.current_setpoint_pos   = 0.0f;
    trajectory.current_setpoint_vel   = 0.0f;
    buffered_target_pos               = 0.0f;

    /* Clear PID state to avoid integral windup spike */
    pid_speed.integral    = 0.0f;
    pid_speed.error_prev  = 0.0f;
    pid_speed.d_filt      = 0.0f;
    pid_position.integral = 0.0f;
    pid_position.error_prev = 0.0f;
    pid_position.d_filt   = 0.0f;

    printf("[HOME] Home set here (was %.2f deg). Original home now at %.2f deg.\r\n",
           -original_home_offset_deg, original_home_offset_deg);
}

void Motor_SendAudioCommand(char sound_code)
{
    /* Enqueue for the main-loop TX drain — never block here. This is called
     * from the 100 Hz control ISR and the USART3 RX ISR (bug 0-C). */
    uint8_t pkt[2] = {'@', (uint8_t)sound_code};
    USART3_QueueTx(pkt, 2);
}

void Motor_SetVoltageLimit(float max_voltage, float supply_voltage)
{
    if (supply_voltage <= 0.0f) return;
    float max_pwm = (max_voltage / supply_voltage) * 100.0f;
    pid_speed.output_max = max_pwm; 
    pid_speed.output_min = -max_pwm;
}

void Motor_SetMotionProfile(float max_rpm, float max_accel, float smoothing)
{
    // (Fix: Do not overwrite tuning struct here, only update active motion_config)
    (void)smoothing; /* smoothing param kept for API compat; jerk is controlled by tuning.max_jerk */
    
    // Update active trajectory limits
    motion_config.max_velocity = max_rpm;
    motion_config.max_acceleration = max_accel;
}

void Motor_SetJogVelocity(float rpm)
{
    if (emergency_stop) return;
    trajectory.target_vel = rpm;
    current_mode = MOTOR_MODE_SPEED;
}

void Motor_UpdateSelectionButton(bool pressed)
{
    if (pressed) {
        if (!y_button_active) {
            y_button_hold_tick = HAL_GetTick();
            y_button_active = true;
        }
    } else {
        y_button_active = false;
    }
}

void Motor_UpdateModeButton(bool pressed)
{
    if (pressed) {
        if (!m_button_active) {
            m_button_hold_tick = HAL_GetTick();
            m_button_active = true;
        }
    } else {
        m_button_active = false;
    }
}

void Motor_UpdateControlModeButton(bool pressed)
{
    if (pressed) {
        if (!b_button_active) {
            b_button_hold_tick = HAL_GetTick();
            b_button_active = true;
        }
    } else {
        b_button_active = false;
    }
}

void Motor_SetConnectionStatus(bool connected) 
{ 
    is_joystick_connected = connected; 
}

void Motor_SendDataToMatlab(void)
{
    float ghost_pos = (current_mode == MOTOR_MODE_GHOST) ? buffered_target_pos : trajectory.target_pos;

    if (ghost_move_active) {
        // BUFFERING MODE: Store data silently
        if (ghost_buffer_idx < GHOST_BUFFER_MAX) {
            ghost_buffer[ghost_buffer_idx].tick = HAL_GetTick();
            ghost_buffer[ghost_buffer_idx].pos_x100 = (long)(encoder.current_position_deg * 100.0f);
            ghost_buffer[ghost_buffer_idx].target_x100 = (long)(trajectory.target_pos * 100.0f);
            ghost_buffer_idx++;
        }
    } else if (ghost_dump_requested) {
        // DUMPING MODE: Send the whole buffer as fast as possible
        for (uint32_t i = 0; i < ghost_buffer_idx; i++) {
            printf("DATA,%lu,%ld,%ld\r\n", 
                   ghost_buffer[i].tick, 
                   ghost_buffer[i].pos_x100, 
                   ghost_buffer[i].target_x100);
        }
        printf("END\r\n");
        ghost_dump_requested = false;
        ghost_buffer_idx = 0;
    } else {
        // PREVIEW MODE: Send only target and current pos for the live preview
        printf("PREVIEW,%ld,%ld\r\n", 
               (long)(encoder.current_position_deg * 100.0f),
               (long)(ghost_pos * 100.0f));
    }
}

void Motor_StartAutotune(void)
{
    if (emergency_stop) return;
    atune.relay_output = 25.0f; 
    atune.center_pos = encoder.current_position_deg; 
    atune.peak_max = -999.0f; 
    atune.peak_min = 999.0f; 
    atune.cycle_count = -1; 
    atune.amplitude_sum = 0.0f; 
    atune.period_sum = 0; 
    atune.last_flip_tick = HAL_GetTick();
    atune.direction = true; 
    atune.motor_sign = 1;
    tuning_progress = 0; 
    autotune_status = STATUS_RUNNING_POS;
    current_mode = MOTOR_MODE_AUTOTUNE;
}

void Motor_StartAutotuneSpeed(void)
{
    if (emergency_stop) return;
    atune.relay_output = 25.0f; 
    atune.center_pos = encoder.current_position_deg;
    atune.target_val = 5.0f; 
    atune.peak_max = -999.0f; 
    atune.peak_min = 999.0f;
    atune.cycle_count = -1; 
    atune.amplitude_sum = 0.0f; 
    atune.period_sum = 0;
    atune.last_flip_tick = HAL_GetTick(); 
    atune.direction = true; 
    atune.motor_sign = 1;
    tuning_progress = 0; 
    autotune_status = STATUS_RUNNING_SPEED;
    current_mode = MOTOR_MODE_AUTOTUNE_SPEED;
}

static bool last_joystick_status = false;
static char last_safety_char = 'O';

void Motor_ProcessPacket(char action, char safety, char status)
{
    bool current_status = (status == 'C');

    /* Any validated packet (printable 3 bytes) means the ESP32 link itself is
     * alive — refresh the silence watchdog unconditionally. Motor_ControlLoop
     * uses this timestamp to detect a *silent* link (cable pulled / ESP32 hung). */
    joystick_watchdog_timer = HAL_GetTick();

    /* Debounce the gamepad-side connection flag the ESP32 reports in the status
     * byte (bug 0-A). A single non-'C' — a noise byte, a dropped-byte frame
     * shift, or a momentary BT hiccup — must NOT latch the e-stop. Require
     * JOYSTICK_DISCONNECT_STREAK consecutive non-'C' packets before declaring
     * the gamepad lost. The JOYSTICK_TIMEOUT_MS silence watchdog in the control
     * loop handles a hard/dead link (bug 0-D). */
    static uint8_t disconnect_streak = 0;

    if (current_status) {
        disconnect_streak = 0;
        if (!last_joystick_status) {
        }
        last_joystick_status  = true;
        is_joystick_connected = true;
        FAULT_CLR(FAULT_JOYSTICK_LOST);
    } else {
        if (disconnect_streak < 255) disconnect_streak++;
        if (disconnect_streak >= JOYSTICK_DISCONNECT_STREAK) {
            if (last_joystick_status) {
                printf("\r\n[SYSTEM] !!! JOYSTICK DISCONNECTED !!!\r\n");
            }
            last_joystick_status  = false;
            is_joystick_connected = false;
            if (safety_config.joystick_check) {
                FAULT_SET(FAULT_JOYSTICK_LOST);
                emergency_stop = true;
            } else {
                FAULT_CLR(FAULT_JOYSTICK_LOST);
            }
        }
        /* Below the streak threshold: ignore this packet's status and keep
         * running. Fall through so the safety-button and action fields are
         * still processed — a single transient must not freeze control input. */
    }

    // 3. Safety / Emergency Button Toggle Logic (Rising Edge)
    if (safety == 'P' && last_safety_char == 'O') {
        // Toggle Emergency Stop
        if (!emergency_stop) {
            emergency_stop = true;
            FAULT_SET(FAULT_ESTOP_JOYSTICK);
            Motor_SendAudioCommand('E');
            printf("[SAFETY] E-Stop LATCHED via Joystick\r\n");
        } else if (!hw.in_estop && hw.raw_prox_bit) {
            // Only clear if physical hardware is also safe
            emergency_stop = false;
            FAULT_CLR(FAULT_ESTOP_PHYSICAL | FAULT_PROX_LOST |
                      FAULT_ESTOP_JOYSTICK | FAULT_ESTOP_DASHBOARD |
                      FAULT_ESTOP_MODBUS);
            Motor_SendAudioCommand('C');
            printf("[SAFETY] E-Stop CLEARED via Joystick\r\n");
        }
    }
    last_safety_char = safety;

    // 4. Action Command
    Motor_ProcessCommand(action);
}

void Motor_ProcessCommand(char cmd)
{
    // RESET WATCHDOG: Receiving any character indicates the joystick is alive
    is_joystick_connected = true;
    joystick_watchdog_timer = HAL_GetTick();
    FAULT_CLR(FAULT_JOYSTICK_LOST);

    // PRIORITY 1: Manual Emergency Stop Command
    if (cmd == 'P' || cmd == 'X') {
        if (!emergency_stop) {
            printf("[SAFETY] Manual E-Stop triggered via Joystick (cmd=%c)\r\n", cmd);
        }
        emergency_stop = true;
        FAULT_SET(FAULT_ESTOP_JOYSTICK);
        Motor_SendAudioCommand('E');
        trajectory.current_setpoint_vel = 0.0f;
        trajectory.current_setpoint_pos = encoder.current_position_deg;
        trajectory.target_pos = encoder.current_position_deg;
        pid_speed.integral = 0.0f; 
        pid_position.integral = 0.0f;
        return;
    }

    // PRIORITY 2: Config & Mode Buttons
    if (cmd == 'A') {
        if (!a_button_is_held) { 
            a_press_tick = HAL_GetTick(); 
            a_button_is_held = true; 
            a_long_press_handled = false;
        }
    } else if (cmd == 'O') {
        if (a_button_is_held) {
            if (!a_long_press_handled) {
                uint32_t duration = HAL_GetTick() - a_press_tick;
                if (duration < 2000) {
                    a_button_click_count++;
                    a_button_last_release_tick = HAL_GetTick();
                    a_button_evaluating = true;
                }
            }
            a_button_is_held = false; 
            a_long_press_handled = false;
        }
    }
    
    // B Button (Control Mode Toggle)
    if (cmd == 'B') {
        if (!b_button_active) {
            b_button_hold_tick = HAL_GetTick();
            b_button_active = true;
        }
    } else if (cmd == 'O') {
        b_button_active = false;
    }

    // Y Button (Ghost Mode Toggle / Execute)
    if (cmd == 'Y') {
        if (!y_button_active) {
            y_button_hold_tick = HAL_GetTick();
            y_button_active = true;
        }
    } else if (cmd == 'O') {
        if (y_button_active) {
            uint32_t duration = HAL_GetTick() - y_button_hold_tick;
            if (duration < 1000 && current_mode == MOTOR_MODE_GHOST) {
                // SHORT PRESS in Ghost Mode -> EXECUTE MOVE
                trajectory.target_pos = buffered_target_pos;
                ghost_move_active = true;
                ghost_buffer_idx = 0;
                printf("START,%.2f\r\n", buffered_target_pos);
            }
            y_button_active = false;
        }
    }

    // M Button (Jog Mode & Autotune)
    if (cmd == 'M') { 
        if (!m_button_active) {
            m_button_hold_tick = HAL_GetTick();
            m_button_active = true;

            if (jog_mode == JOG_COARSE) { 
                jog_mode = JOG_FINE; 
                resolution_step = tuning.step_size_fine; 
            } else { 
                jog_mode = JOG_COARSE; 
                resolution_step = tuning.step_size_coarse; 
            }
            Motor_SendAudioCommand('M');
        }
        return; 
    } else if (cmd == 'O') {
        m_button_active = false;
    }

    // PRIORITY 3: Base System Check
    if (control_system_mode == CONTROL_MODE_BASE_SYSTEM) return;

    // PRIORITY 4: Motion Commands (Blocked if E-Stop)
    if (emergency_stop) {
        if (cmd != 'O') {
            Motor_SendAudioCommand('E');
        }
        return;
    }

    switch (cmd)
    {
    case 'U': Gripper_Up(); break;
    case 'D': Gripper_Down(); break;
    case 'F': 
        if (ghost_move_active) {
            ghost_move_active = false;
            ghost_dump_requested = true;
            printf("MANUAL STOP\r\n");
        } else {
            Gripper_Toggle(); 
        }
        break;
    /* Defer the blocking sequences to the main loop (bug 1-F) — this runs in
     * the USART3 RX ISR and must never busy-wait on reed switches here. */
    case 'S': gripper_seq_request = 1; break;
    case 'G': gripper_seq_request = 2; break;
    case 'L': 
        if (current_mode == MOTOR_MODE_GHOST) {
            buffered_target_pos -= resolution_step;
        } else if (jog_mode == JOG_COARSE) {
            if (!step_executed) { 
                trajectory.target_pos -= tuning.step_size_coarse; 
                current_mode = MOTOR_MODE_POSITION; 
                step_executed = true; 
            }
        } else { 
            trajectory.target_vel = -tuning.jog_speed_fine; 
            current_mode = MOTOR_MODE_SPEED; 
        }
        break;
    case 'R': 
        if (current_mode == MOTOR_MODE_GHOST) {
            buffered_target_pos += resolution_step;
        } else if (jog_mode == JOG_COARSE) {
            if (!step_executed) { 
                trajectory.target_pos += tuning.step_size_coarse; 
                current_mode = MOTOR_MODE_POSITION; 
                step_executed = true; 
            }
        } else { 
            trajectory.target_vel = tuning.jog_speed_fine; 
            current_mode = MOTOR_MODE_SPEED; 
        }
        break;

    case 'O': 
        step_executed = false;
        if (current_mode == MOTOR_MODE_SPEED) {
            if (jog_mode == JOG_FINE) {
                current_mode = MOTOR_MODE_STOPPED;
            } else {
                current_mode = MOTOR_MODE_POSITION; 
                trajectory.target_pos = encoder.current_position_deg;
                trajectory.current_setpoint_pos = encoder.current_position_deg;
                trajectory.current_setpoint_vel = 0.0f; 
            }
        }
        break;
    case 'Y': 
        if (current_mode == MOTOR_MODE_GHOST) {
            trajectory.target_pos = buffered_target_pos;
            ghost_buffer_idx = 0; 
            ghost_move_active = true;
            ghost_settle_start_tick = 0;
        }
        break;
    case 'J': 
        current_mode = MOTOR_MODE_POSITION; 
        trajectory.target_pos = encoder.current_position_deg; 
        break;
    default: break;
    }
}

void Motor_ControlLoop(void)
{
    static float current_applied_pwm = 0.0f;
    static bool last_emergency_stop = false;

    HW_RefreshIO();  // Sync all hardware I/O with debug struct
    Encoder_Update();

    // --- Real-time Parameter Sync (Allows Live Expressions Tuning) ---
    pid_speed.Kp = tuning.speed_Kp;
    pid_speed.Ki = tuning.speed_Ki;
    pid_speed.Kd = tuning.speed_Kd;
    
    pid_position.Kp = tuning.pos_Kp;
    pid_position.Ki = tuning.pos_Ki;
    pid_position.Kd = tuning.pos_Kd;

    // Sync acceleration only — velocity is set explicitly by each caller
    // (Motor_MoveToPosition, Motor_SetMotionProfile, homing state machine).
    // Do NOT overwrite max_velocity here: that would cancel any slow-speed
    // profile set for return-home or fine-jog moves.
    if (current_mode == MOTOR_MODE_POSITION || current_mode == MOTOR_MODE_GHOST) {
        motion_config.max_acceleration = tuning.max_accel;
    }

    target_position_deg = trajectory.target_pos;

    // Check if E-Stop just cleared
    if (last_emergency_stop && !emergency_stop) {
        /* Always reset PID state on E-Stop clear regardless of path.
         * Without this, wound-up integrals from before the stop apply
         * full PWM the instant the relay re-energises, which causes the
         * encoder signal-loss timer to start (high PWM, zero RPM for one tick)
         * and can fire FAULT_ENCODER_ERROR even though the encoder is fine. */
        pid_speed.integral    = 0.0f;
        pid_speed.error_prev  = 0.0f;
        pid_speed.d_filt      = 0.0f;
        pid_position.integral = 0.0f;
        pid_position.error_prev = 0.0f;
        pid_position.d_filt   = 0.0f;
        trajectory.current_setpoint_vel   = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;
        FAULT_CLR(FAULT_ENCODER_ERROR);  /* clear stale encoder fault from E-Stop relay bounce */

        if (position_unknown) {
            /* Recovery from a physical (hardware) E-stop: motor relay was
             * open, encoder lost power, TIM3 quadrature counts are stale.
             * Don't trust current_position_deg — kick off a re-home.
             *
             * DO NOT arm homing immediately.  During the creep phase of homing
             * the PID commands very high PWM at very low speed, causing strong
             * PWM-switching noise on PA5 (E-Stop input, only 40 kΩ pull-up).
             * If homing starts the instant the relay closes, noise accumulates
             * for >300 ms and fires FAULT_ESTOP_PHYSICAL again — creating the
             * Reset → E-stop → Reset infinite loop the user sees.
             *
             * Fix: hold motor STOPPED (PWM = 0) for 500 ms.  No switching means
             * no noise → estop_debounce drains to 0 → relay fully settles.
             * The settle timer cancels if emergency_stop re-triggers during the
             * wait, so a real E-stop press is never masked. */
            position_unknown      = false;
            current_mode          = MOTOR_MODE_STOPPED;
            trajectory.target_pos            = encoder.current_position_deg;
            trajectory.current_setpoint_pos  = encoder.current_position_deg;
            Motor_SetMotionProfile(tuning.move_speed_return_home, tuning.max_accel, 0.1f);
            homing_settle_tick    = HAL_GetTick();
            homing_settle_pending = true;
            clog_events |= CLOG_ESTOP_PHYS_REHOME;
        } else {
            /* Soft E-stop (or boot): motor relay just closed this tick and the
             * user only asked to clear the latch — they did not request motion.
             * Snap trajectory to the current encoder position so no PWM is
             * commanded. The user must explicitly issue the next move. This
             * also avoids stall faults when the relay hasn't physically
             * settled and any commanded PWM produces no rotation. */
            current_mode = MOTOR_MODE_STOPPED;
            trajectory.target_pos             = encoder.current_position_deg;
            trajectory.current_setpoint_pos   = encoder.current_position_deg;
            clog_hold_pos = encoder.current_position_deg; clog_events |= CLOG_ESTOP_CLEARED_HOLD;
        }
        /* Flush ZVD buffer so stale pre-E-Stop setpoints cannot leak into
         * the shaper output on the first tick after resume. */
        ZVD_FlushBuffer(encoder.current_position_deg);
    }
    last_emergency_stop = emergency_stop;

    /* ---- Relay-settle re-home: fire once the E-Stop pin is quiet for 500 ms -- */
    if (homing_settle_pending) {
        if (emergency_stop) {
            /* E-stop re-triggered during settle (noise persisted or real press).
             * hw_io.c has already set position_unknown = true again.
             * Cancel the timer — it restarts on the next Reset clear. */
            homing_settle_pending = false;
        } else if ((HAL_GetTick() - homing_settle_tick) >= 500) {
            homing_settle_pending = false;
            if (current_mode == MOTOR_MODE_STOPPED) {  /* still in settle-hold state */
                trigger_homing_sequence = true;
                current_mode = MOTOR_MODE_POSITION;    /* RunHomingSequence() switches to HOMING */
                clog_events |= CLOG_RELAY_SETTLED;
            }
        }
    }

    // M-button long press (3s) -> Test Mode
    if (m_button_active && current_mode != MOTOR_MODE_TEST && !emergency_stop) {
        if (HAL_GetTick() - m_button_hold_tick >= 3000) {
            clog_events |= CLOG_TEST_START;
            open_loop_test_tick = HAL_GetTick();
            current_mode = MOTOR_MODE_TEST;
            m_button_active = false;
        }
    }

    // Y-button long press (1s) -> Toggle Ghost Mode
    if (y_button_active && !emergency_stop) {
        if (HAL_GetTick() - y_button_hold_tick >= 1000) {
            if (current_mode == MOTOR_MODE_GHOST) {
                current_mode = MOTOR_MODE_POSITION;
                Motor_SendAudioCommand('g');
            } else {
                current_mode = MOTOR_MODE_GHOST;
                buffered_target_pos = trajectory.target_pos;
                Motor_SendAudioCommand('G');
            }
            y_button_active = false;
        }
    }

    // B-button long press (1s) -> Toggle Control Mode (also reconfigures LPUART1)
    if (b_button_active && !emergency_stop) {
        if (HAL_GetTick() - b_button_hold_tick >= 1000) {
            Mode_Toggle();
            b_button_active = false;
        }
    }

    // Handle autotune triggers
    if (autotune_trigger == ATUNE_POS) { 
        Motor_StartAutotune(); 
        autotune_trigger = ATUNE_IDLE; 
    } else if (autotune_trigger == ATUNE_SPEED) { 
        Motor_StartAutotuneSpeed(); 
        autotune_trigger = ATUNE_IDLE; 
    }

    // Handle A Button State Machine (Multi-click & Long press)
    if (a_button_is_held && !a_long_press_handled && (HAL_GetTick() - a_press_tick >= 2000)) {
        // LONG PRESS: Trigger Homing Sequence
        if (current_mode != MOTOR_MODE_HOMING) {
            trigger_homing_sequence = true;
            Motor_SendAudioCommand('h');
        }
        a_long_press_handled = true;
        a_button_click_count = 0;
        a_button_evaluating = false;
    } else if (!a_button_is_held && a_button_evaluating && (HAL_GetTick() - a_button_last_release_tick > 400)) {
        // TIMEOUT REACHED: Evaluate clicks
        if (a_button_click_count == 1) {
            // SINGLE CLICK: Set Temporary Home
            Motor_SendAudioCommand('T');
            original_home_offset_deg -= encoder.current_position_deg;
            
            __HAL_TIM_SET_COUNTER(&htim3, 0);
            encoder.count_prev = 0;
            encoder.absolute_counts = 0;
            encoder.current_position_deg = 0.0f;
            encoder.filtered_rpm = 0.0f;
            last_rpm_for_accel = 0.0f;
            
            trajectory.target_pos = 0.0f;
            trajectory.current_setpoint_pos = 0.0f;
            trajectory.current_setpoint_vel = 0.0f;
            buffered_target_pos = 0.0f;
            
            pid_speed.integral = 0.0f;
            pid_speed.error_prev = 0.0f;
            pid_speed.d_filt = 0.0f;
            
            pid_position.integral = 0.0f;
            pid_position.error_prev = 0.0f;
            pid_position.d_filt = 0.0f;
            
            clog_temp_home_orig = original_home_offset_deg; clog_events |= CLOG_HOME_TEMP_SET;
        } else if (a_button_click_count == 2) {
            // DOUBLE CLICK: Go to Temporary Home
            Motor_SendAudioCommand('2');
            trajectory.target_pos = 0.0f;
            current_mode = MOTOR_MODE_POSITION;
            Motor_SetMotionProfile(tuning.move_speed_return_home, tuning.max_accel, 0.1f);
            clog_move_speed = tuning.move_speed_return_home; clog_events |= CLOG_HOME_MOVE_TEMP;
        } else if (a_button_click_count >= 3) {
            // TRIPLE CLICK: Go to Original Home
            Motor_SendAudioCommand('3');
            trajectory.target_pos = original_home_offset_deg;
            current_mode = MOTOR_MODE_POSITION;
            Motor_SetMotionProfile(tuning.move_speed_return_home, tuning.max_accel, 0.1f);
            clog_move_orig_pos = original_home_offset_deg; clog_move_speed = tuning.move_speed_return_home; clog_events |= CLOG_HOME_MOVE_ORIG;
        }
        
        a_button_click_count = 0;
        a_button_evaluating = false;
    }

    /* Link silence watchdog (bug 0-D): declare the joystick link dead if no
     * validated packet has arrived for JOYSTICK_TIMEOUT_MS, regardless of what
     * the ESP32 status byte said. This is the authoritative detector for a
     * physically dead/unplugged link. Only armed in JOYSTICK control mode so it
     * does not trigger spuriously in BASE_SYSTEM (Modbus) mode. */
    if (control_system_mode == CONTROL_MODE_JOYSTICK &&
        (HAL_GetTick() - joystick_watchdog_timer) > JOYSTICK_TIMEOUT_MS) {
        is_joystick_connected = false;
    }

    // Joystick fault from ESP32 disconnect signal only (no timer).
    // Honour the "Joystick Check" safety toggle: when disabled, neither raise
    // the fault bit nor force STOPPED mode — the motor keeps running.
    if (!is_joystick_connected && safety_config.joystick_check) {
        FAULT_SET(FAULT_JOYSTICK_LOST);
        /* Do NOT interrupt the homing state machine — it drives MOTOR_MODE_HOMING
         * intentionally and must complete to set encoder zero.  Any other active
         * mode (POSITION, SPEED, JOG …) is stopped as before. */
        if (current_mode != MOTOR_MODE_STOPPED && current_mode != MOTOR_MODE_HOMING) {
            current_mode = MOTOR_MODE_STOPPED;
            PWM_Apply(0.0f);
        }
    } else {
        FAULT_CLR(FAULT_JOYSTICK_LOST);
    }

    /* --- Safety Monitoring --- */
    bool stall_condition = false;

    // 1. Stall Detection
    if (fabsf(current_applied_pwm) >= STALL_PWM_THRESHOLD && fabsf(encoder.filtered_rpm) < STALL_VELOCITY_THRESHOLD) {
        if (current_mode == MOTOR_MODE_POSITION || current_mode == MOTOR_MODE_GHOST) {
            /* Compare against the SHAPED (commanded) setpoint, not the final
             * target (bug 0-I). The ZVD shaper delays the command by up to 2N
             * ticks (~0.5 s at default wn); measuring error against the un-shaped
             * final target causes the 2 s stall window to elapse while the shaper
             * is still ramping up the command → spurious FAULT_MOTOR_STALLED.
             * shaper_last_output holds the shaped setpoint from the previous tick
             * (10 ms stale — negligible). When the shaper is disabled,
             * shaper_last_output == trajectory.current_setpoint_pos, which is also
             * correct (and stricter than using target_pos). */
            float pos_error = fabsf(shaper_last_output - encoder.current_position_deg);
            if (pos_error > STALL_SETTLING_ERROR_DEG) stall_condition = true;
        } else if (current_mode == MOTOR_MODE_SPEED) {
            stall_condition = true;
        }
    }

    // 2. Encoder Phase Inversion
    // The check fires only when ALL of these are true for 30 consecutive
    // ticks (~300 ms):
    //   - PWM is well above the fault threshold (motor is actively driven)
    //   - PWM direction has been stable for ≥ 500 ms (no recent reversal)
    //   - count_delta sign opposes PWM sign by a clear margin
    // This kills the entire family of normal-physics transients:
    //   * direction reversal during overshoot recovery (inertia carries
    //     the motor the old way while PWM is already commanding the new)
    //   * trajectory deceleration where PWM may briefly counter-pulse
    //   * settling-region small motions where count_delta is noisy
    // A genuinely miswired encoder will keep counts opposite to PWM
    // forever, so the 800 ms (500 + 300) total latency is still safe.
    int32_t count_delta = encoder.absolute_counts - last_absolute_counts;
    static int8_t   last_pwm_sign = 0;
    static uint32_t pwm_dir_stable_tick = 0;
    int8_t pwm_sign_now =
        (current_applied_pwm >  ENCODER_FAULT_PWM_THRESHOLD) ?  1 :
        (current_applied_pwm < -ENCODER_FAULT_PWM_THRESHOLD) ? -1 : 0;
    if (pwm_sign_now != last_pwm_sign) {
        pwm_dir_stable_tick = HAL_GetTick();   /* reset stability window */
        last_pwm_sign = pwm_sign_now;
    }
    bool pwm_dir_settled = (pwm_sign_now != 0) &&
                           ((HAL_GetTick() - pwm_dir_stable_tick) >= 500);

    /* Also suppress during homing: wiggle search keeps flipping h_direction,
     * creep speeds are 1–10 RPM, and breakaway static friction can make
     * count_delta cross zero while PWM is held high — none of that is a
     * wiring fault. */
    bool instant_inverted = pwm_dir_settled &&
                            (current_mode != MOTOR_MODE_HOMING) && (
        (pwm_sign_now ==  1 && count_delta < -10) ||
        (pwm_sign_now == -1 && count_delta >  10));

    static uint8_t inversion_streak = 0;
    if (instant_inverted) {
        if (inversion_streak < 255) inversion_streak++;
    } else {
        inversion_streak = 0;
    }
    bool encoder_inverted = (inversion_streak >= 30);   /* 300 ms continuous */

    if (safety_config.encoder_check && encoder_inverted) {
        FAULT_SET(FAULT_ENCODER_ERROR);
        emergency_stop = true;
        clog_events |= CLOG_ENCODER_INVERTED;
        PWM_Apply(0.0f);
    }

    // 3. Encoder Signal Loss
    // Suppressed during homing: search/creep speeds are 1–10 RPM, so static-
    // friction breakaway at the start of a wiggle/creep move legitimately
    // produces high PWM with ~0 RPM and counts barely changing for hundreds
    // of milliseconds — that looks identical to "encoder cable unplugged",
    // but it isn't, it's just slow start-up. Homing has its own failure
    // detection (H_ERROR via HOMING_MAX_WIGGLE) so a real stuck condition
    // is still caught.
    bool encoder_signal_lost = false;
    if (current_mode == MOTOR_MODE_HOMING) {
        /* Don't arm the timer at all — and re-baseline last_absolute_counts
         * so it doesn't see a stale "no change" the moment homing ends. */
        encoder_fault_timer = 0;
        last_absolute_counts = encoder.absolute_counts;
    } else if (fabsf(current_applied_pwm) > ENCODER_FAULT_PWM_THRESHOLD &&
        fabsf(encoder.filtered_rpm) < STALL_VELOCITY_THRESHOLD &&
        encoder.absolute_counts == last_absolute_counts) {

        if (encoder_fault_timer == 0) encoder_fault_timer = HAL_GetTick();
        else if (HAL_GetTick() - encoder_fault_timer >= 1000) {
            encoder_signal_lost = true;
        }
    } else {
        encoder_fault_timer = 0;
        last_absolute_counts = encoder.absolute_counts;
    }

    if (safety_config.encoder_check && encoder_signal_lost) {
        FAULT_SET(FAULT_ENCODER_ERROR);
        emergency_stop = true;
        clog_events |= CLOG_ENCODER_NOSIGNAL;
        PWM_Apply(0.0f);
    }

    // Clear stale encoder fault when:
    //  - the user disables the check (so the dashboard stops showing it), OR
    //  - neither inversion nor signal-loss conditions are currently active
    //    and the e-stop has been cleared (so a recovered link drops the fault).
    if (!safety_config.encoder_check) {
        FAULT_CLR(FAULT_ENCODER_ERROR);
    } else if (!encoder_inverted && !encoder_signal_lost && !emergency_stop) {
        FAULT_CLR(FAULT_ENCODER_ERROR);
    }

    // Stall Timer
    if (stall_condition) {
        if (stall_timer == 0) stall_timer = HAL_GetTick();
        else if (HAL_GetTick() - stall_timer >= STALL_TIME_MS) {
            if (safety_config.stall_prevent) {
                FAULT_SET(FAULT_MOTOR_STALLED);
                emergency_stop = true;
                clog_events |= CLOG_MOTOR_STALLED;
                PWM_Apply(0.0f);
            }
        }
    } else {
        stall_timer = 0;
    }

    // Clear stale stall fault when the check is disabled or condition has cleared and e-stop is released
    if (!safety_config.stall_prevent) {
        FAULT_CLR(FAULT_MOTOR_STALLED);
    } else if (!stall_condition && !emergency_stop) {
        FAULT_CLR(FAULT_MOTOR_STALLED);
    }

    // 4. Over-Rotation Protection (Virtual Wall Notification)
    if (fabsf(encoder.current_position_deg) > SOFT_LIMIT_DEG) {
        if (!(fault_code & FAULT_OVER_ROTATION)) {
            FAULT_SET(FAULT_OVER_ROTATION);
            clog_events |= CLOG_SOFT_LIMIT;
        }
    } else {
        FAULT_CLR(FAULT_OVER_ROTATION);
    }

    /* Fault-bit hygiene: clear stale automatic-safety fault bits when the
     * user disables that check, even if e-stop is latched. Without this the
     * dashboard keeps showing "Encoder Error" after the user unchecks
     * Encoder Check because the clear-path that lives later in this function
     * is unreachable while e-stop is active. */
    if (!safety_config.encoder_check) FAULT_CLR(FAULT_ENCODER_ERROR);
    if (!safety_config.stall_prevent) FAULT_CLR(FAULT_MOTOR_STALLED);
    if (!safety_config.joystick_check) FAULT_CLR(FAULT_JOYSTICK_LOST);

    /* If e-stop was raised solely by an automatic safety fault that the user
     * has now disabled, and no other fault (automatic or manual) remains,
     * auto-release the e-stop so the motor can run again. Manual e-stop
     * sources (physical button, dashboard, joystick button, Modbus) stay
     * latched until explicitly cleared. */
    if (emergency_stop && fault_code == FAULT_NONE && !hw.in_estop) {
        emergency_stop = false;
    }

    // Stop motor if e-stop or stopped mode
    if (emergency_stop || current_mode == MOTOR_MODE_STOPPED) {
        if (ghost_move_active) {
            ghost_move_active = false;
            ghost_dump_requested = true;
        }
        PWM_Apply(0.0f);
        /* CRITICAL: also reset the module-level PWM monitor variable. Without
         * this, current_applied_pwm holds the last large value commanded just
         * before the fault (e.g. -80% from the corrective brake), the early-
         * return below skips the PID compute that would normally overwrite
         * it, and every subsequent tick the safety checks above see a stale
         * "high PWM" alongside zero RPM → encoder-signal-loss timer arms
         * → FAULT_ENCODER_ERROR re-fires within 1 s → user can never clear
         * the emergency. */
        current_applied_pwm = 0.0f;
        current_pwm = 0.0f;

        // Continuously sync trajectory during E-Stop so the motor does not violently snap
        // back to an old target position when the E-Stop state is cleared.
        trajectory.target_pos = encoder.current_position_deg;
        trajectory.current_setpoint_pos = encoder.current_position_deg;
        trajectory.current_setpoint_vel = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;
        pid_speed.integral = 0.0f;
        pid_position.integral = 0.0f;
        /* Also re-baseline the signal-loss tracker so that when emergency
         * clears, the very next tick doesn't see a stale "count not changing
         * while PWM is high" condition. (Inversion-streak naturally resets
         * because pwm_sign_now == 0 once current_applied_pwm is 0.) */
        encoder_fault_timer = 0;
        last_absolute_counts = encoder.absolute_counts;
        
        return; 
    }

    /* --- Autotune: Position --- */
    if (current_mode == MOTOR_MODE_AUTOTUNE) {
        if (fabsf(encoder.current_position_deg - atune.center_pos) > 25.0f) { 
            emergency_stop = true; 
            autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            return; 
        }
        if (atune.cycle_count == -1) { 
            PWM_Apply(15.0f); 
            if (HAL_GetTick() - atune.last_flip_tick > 300) { 
                atune.motor_sign = ((encoder.current_position_deg - atune.center_pos) >= 0) ? 1 : -1;
                atune.cycle_count = 0; 
                atune.last_flip_tick = HAL_GetTick(); 
            } 
            return; 
        }
        bool crossed = (atune.direction && (encoder.current_position_deg > atune.center_pos)) || 
                       (!atune.direction && (encoder.current_position_deg < atune.center_pos));
        if (crossed) { 
            atune.direction = !atune.direction; 
            if (atune.cycle_count > 2) { 
                atune.amplitude_sum += (atune.peak_max - atune.peak_min); 
                atune.period_sum += (HAL_GetTick() - atune.last_flip_tick); 
            }
            atune.cycle_count++; 
            atune.last_flip_tick = HAL_GetTick(); 
            tuning_progress = atune.cycle_count; 
            atune.peak_max = -999.0f; 
            atune.peak_min = 999.0f; 
        }
        if (encoder.current_position_deg > atune.peak_max) atune.peak_max = encoder.current_position_deg;
        if (encoder.current_position_deg < atune.peak_min) atune.peak_min = encoder.current_position_deg;
        PWM_Apply((atune.direction ? atune.relay_output : -atune.relay_output) * atune.motor_sign);
        if (atune.cycle_count >= 10) { 
            float avg_A = (atune.amplitude_sum / (atune.cycle_count - 2)) / 2.0f; 
            if (avg_A > 0.1f) { 
                float Ku = (4.0f * atune.relay_output) / (3.14159f * avg_A);
                tuning.pos_Kp = 0.20f * Ku; 
                tuning.pos_Ki = 0.10f * tuning.pos_Kp; 
                tuning.pos_Kd = 0.01f;
                if (tuning.pos_Kp > 2.5f) tuning.pos_Kp = 2.5f; 
                autotune_status = STATUS_SUCCESS;
            } else { 
                autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            }
            current_mode = MOTOR_MODE_STOPPED;
        }
        return;
    }

    /* --- Autotune: Speed --- */
    if (current_mode == MOTOR_MODE_AUTOTUNE_SPEED) {
        if (fabsf(encoder.current_position_deg - atune.center_pos) > 60.0f) { 
            emergency_stop = true; 
            autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            return; 
        }
        if (atune.cycle_count == -1) { 
            PWM_Apply(20.0f); 
            if (HAL_GetTick() - atune.last_flip_tick > 300) { 
                atune.motor_sign = (encoder.filtered_rpm >= 0) ? 1 : -1; 
                atune.cycle_count = 0; 
                atune.last_flip_tick = HAL_GetTick(); 
            } 
            return; 
        }
        bool crossed = (atune.direction && (encoder.filtered_rpm > atune.target_val)) || 
                       (!atune.direction && (encoder.filtered_rpm < -atune.target_val));
        if (crossed) { 
            atune.direction = !atune.direction; 
            if (atune.cycle_count > 2) { 
                atune.amplitude_sum += (atune.peak_max - atune.peak_min); 
                atune.period_sum += (HAL_GetTick() - atune.last_flip_tick); 
            }
            atune.cycle_count++; 
            atune.last_flip_tick = HAL_GetTick(); 
            tuning_progress = atune.cycle_count; 
            atune.peak_max = -999.0f; 
            atune.peak_min = 999.0f; 
        }
        if (encoder.filtered_rpm > atune.peak_max) atune.peak_max = encoder.filtered_rpm;
        if (encoder.filtered_rpm < atune.peak_min) atune.peak_min = encoder.filtered_rpm;
        PWM_Apply((atune.direction ? atune.relay_output : -atune.relay_output) * atune.motor_sign);
        if (atune.cycle_count >= 15) { 
            float avg_A = (atune.amplitude_sum / (atune.cycle_count - 2)) / 2.0f; 
            if (avg_A > 0.5f) {
                float Ku = (4.0f * atune.relay_output) / (3.14159f * avg_A);
                tuning.speed_Kp = 0.20f * Ku; 
                tuning.speed_Ki = 0.50f * tuning.speed_Kp;
                if (tuning.speed_Kp > 6.0f) tuning.speed_Kp = 6.0f; 
                autotune_status = STATUS_SUCCESS;
            } else { 
                autotune_status = STATUS_ERROR_LIMIT_EXCEEDED; 
            }
            current_mode = MOTOR_MODE_STOPPED;
        }
        return;
    }
    
    // Update active PID gains
    pid_speed.Kp = tuning.speed_Kp; 
    pid_speed.Ki = tuning.speed_Ki; 
    pid_speed.Kd = tuning.speed_Kd;
    pid_position.Kp = tuning.pos_Kp; 
    pid_position.Ki = tuning.pos_Ki; 
    pid_position.Kd = tuning.pos_Kd;

    /* --- Open Loop Test Mode --- */
    if (current_mode == MOTOR_MODE_TEST) {
        if (HAL_GetTick() - open_loop_test_tick < 1000) {
            PWM_Apply(25.0f);
        } else {
            PWM_Apply(0.0f);
            clog_events |= CLOG_TEST_FINISH;
            tuning.move_speed_coarse = 5.0f;
            trajectory.target_pos = 0.0f;
            current_mode = MOTOR_MODE_POSITION;
        }
        return;
    }

    /* --- Velocity Loop --- */
    if (current_mode == MOTOR_MODE_SPEED) {
        // VIRTUAL HARD STOPS: Prevent further motion in the direction of the limit
        if (encoder.current_position_deg >= SOFT_LIMIT_DEG && trajectory.target_vel > 0.0f) {
            trajectory.target_vel = 0.0f;
        } else if (encoder.current_position_deg <= -SOFT_LIMIT_DEG && trajectory.target_vel < 0.0f) {
            trajectory.target_vel = 0.0f;
        }

        /* Trajectory FF (cascade-control diagram).
         *   v_ref [RPM]  -> rad/s ; * K_vff [V/(rad/s)]   -> Volts
         *   a_ref [RPM/s]-> rad/s²; * K_aff [V/(rad/s^2)] -> Volts
         *   Volts -> PWM% via (100 / SUPPLY_VOLTAGE).
         * In pure SPEED mode there's no S-curve a_ref, so K_aff term is 0. */
        const float RPM_TO_RADS = 0.10471975512f;       // 2*pi/60
        const float V_TO_PWM    = 100.0f / SUPPLY_VOLTAGE;
        float v_ref_rads = trajectory.target_vel * RPM_TO_RADS;
        float tau_L_s    = Kalman_GetEnabled() ? Kalman_GetLoadTorque() : 0.0f;
        float ff_dist_s  = tuning.K_tff * (tau_L_s * MOT_R_ARM / (MOT_N_GEAR * MOT_ETA_GB * MOT_K_T)) * V_TO_PWM;
        float ff_volts   = tuning.K_vff * v_ref_rads;
        float ff = ff_volts * V_TO_PWM + ff_dist_s;
        float pid_out = PID_Compute(&pid_speed, trajectory.target_vel, encoder.filtered_rpm);
        current_applied_pwm = Motor_DriveWithAntiWindup(pid_out, ff);
        trajectory.target_pos = encoder.current_position_deg;
        trajectory.current_setpoint_pos = encoder.current_position_deg;
    }
    /* --- Position Loop --- */
    else if (current_mode == MOTOR_MODE_POSITION || current_mode == MOTOR_MODE_GHOST || current_mode == MOTOR_MODE_HOMING) {
        // VIRTUAL HARD STOPS: Clamp target position
        if (trajectory.target_pos > SOFT_LIMIT_DEG) {
            trajectory.target_pos = SOFT_LIMIT_DEG;
            static uint32_t last_warn_up = 0;
            if (HAL_GetTick() - last_warn_up > 1000) { Motor_SendAudioCommand('W'); last_warn_up = HAL_GetTick(); }
        }
        if (trajectory.target_pos < -SOFT_LIMIT_DEG) {
            trajectory.target_pos = -SOFT_LIMIT_DEG;
            static uint32_t last_warn_dn = 0;
            if (HAL_GetTick() - last_warn_dn > 1000) { Motor_SendAudioCommand('W'); last_warn_dn = HAL_GetTick(); }
        }
        Trajectory_Generator_Update();

        /* --- ZVD Input Shaper ---
         * Filters the S-curve position setpoint before the outer PID so that
         * the two-impulse (A2) and four-impulse (A3) delayed copies cancel the
         * first resonant swing. Coefficients are pre-computed in
         * ZVD_UpdateCoefficients() and never involve sqrtf/expf here. */
        shaper_buf[shaper_buf_idx]   = trajectory.current_setpoint_pos;
        shaper_buf_v[shaper_buf_idx] = trajectory.current_setpoint_vel;
        shaper_buf_a[shaper_buf_idx] = trajectory.current_setpoint_accel;
        float shaped_pos;
        float shaped_vel;
        float shaped_acc;
        if (tuning.shaper_enable && shaper_N > 0u) {
            uint32_t i_n  = (shaper_buf_idx + (uint32_t)SHAPER_BUF_SIZE - shaper_N)
                            % (uint32_t)SHAPER_BUF_SIZE;
            uint32_t i_2n = (shaper_buf_idx + (uint32_t)SHAPER_BUF_SIZE - 2u * shaper_N)
                            % (uint32_t)SHAPER_BUF_SIZE;
            shaped_pos = shaper_A1 * shaper_buf[shaper_buf_idx]
                       + shaper_A2 * shaper_buf[i_n]
                       + shaper_A3 * shaper_buf[i_2n];
            /* The ZVD shaper is a linear FIR filter, so the shaped velocity and
             * acceleration are exactly the same convolution applied to the
             * trajectory derivatives. Routing these (instead of the raw
             * trajectory v/a) into the feedforward keeps the FF phase-aligned
             * with shaped_pos — they share the same group delay. (Fix 2-B) */
            shaped_vel = shaper_A1 * shaper_buf_v[shaper_buf_idx]
                       + shaper_A2 * shaper_buf_v[i_n]
                       + shaper_A3 * shaper_buf_v[i_2n];
            shaped_acc = shaper_A1 * shaper_buf_a[shaper_buf_idx]
                       + shaper_A2 * shaper_buf_a[i_n]
                       + shaper_A3 * shaper_buf_a[i_2n];
        } else {
            shaped_pos = trajectory.current_setpoint_pos;
            shaped_vel = trajectory.current_setpoint_vel;
            shaped_acc = trajectory.current_setpoint_accel;
        }
        shaper_buf_idx = (shaper_buf_idx + 1u) % (uint32_t)SHAPER_BUF_SIZE;
        shaper_last_output = shaped_pos;   /* expose commanded setpoint to the stall check (bug 0-I) */

        // (Dynamic speed recovery removed — it was overwriting Live Expressions tuning)

        float target_rpm;
        /* Shaper-delayed trajectory derivatives so the velocity/accel
         * feedforward stays phase-aligned with shaped_pos. (Fix 2-B)
         * The sine-test bypass below overrides these with analytic values. */
        float v_ref_rpm = shaped_vel;
        float a_ref_rpmps = shaped_acc;

        if (position_loop_enabled) {
            target_rpm = PID_Compute(&pid_position,
                                     shaped_pos,
                                     encoder.current_position_deg);
        } else {
            /* Position loop bypassed — tune the velocity loop in isolation. */
            if (sine_test_enabled) {
                if (sine_start_tick == 0) sine_start_tick = HAL_GetTick();
                float t  = (HAL_GetTick() - sine_start_tick) * 0.001f;
                float w  = 2.0f * 3.14159265f * sine_freq_hz; // rad/s
                target_rpm  = sine_amp_rpm * sinf(w * t);
                /* Analytic derivatives of the sine command, so the trajectory
                 * feedforward tracks the actual velocity command instead of
                 * the (idle) S-curve outputs. */
                v_ref_rpm   = target_rpm;                          // RPM
                a_ref_rpmps = sine_amp_rpm * w * cosf(w * t);      // RPM/s
            } else {
                sine_start_tick = 0;
                target_rpm = trajectory.current_setpoint_vel;
            }
            pid_position.integral   = 0.0f;
            pid_position.error_prev = 0.0f;
            pid_position.d_filt     = 0.0f;
        }

        /* Trajectory FF on velocity-PID output (matches cascade-control diagram). */
        const float RPM_TO_RADS = 0.10471975512f;       // 2*pi/60
        const float V_TO_PWM    = 100.0f / SUPPLY_VOLTAGE;
        float v_ref_rads = v_ref_rpm   * RPM_TO_RADS;
        float a_ref_rads = a_ref_rpmps * RPM_TO_RADS;
        /* Disturbance feedforward: use Kalman τ_L estimate to pre-compensate load.
         * V_ff = τ_L * R / (N * η * Kt)  →  PWM% via V_TO_PWM. */
        float tau_L      = Kalman_GetEnabled() ? Kalman_GetLoadTorque() : 0.0f;
        float ff_dist    = tuning.K_tff * (tau_L * MOT_R_ARM / (MOT_N_GEAR * MOT_ETA_GB * MOT_K_T)) * V_TO_PWM;
        float ff_volts   = tuning.K_vff * v_ref_rads + tuning.K_aff * a_ref_rads;
        float ff = ff_volts * V_TO_PWM + ff_dist;

        float pid_out = PID_Compute(&pid_speed, target_rpm, encoder.filtered_rpm);
        current_applied_pwm = Motor_DriveWithAntiWindup(pid_out, ff);

        // Ghost Mode settle check: Must be within 0.5 deg AND < 1.0 RPM for 3 seconds
        if (ghost_move_active) {
            float pos_error = fabsf(encoder.current_position_deg - trajectory.target_pos);
            float vel_error = fabsf(encoder.filtered_rpm);
            
            if (pos_error <= 0.5f && vel_error < 1.0f) {
                if (ghost_settle_start_tick == 0) {
                    ghost_settle_start_tick = HAL_GetTick();
                } else if (HAL_GetTick() - ghost_settle_start_tick >= 3000) {
                    ghost_move_active = false;
                    ghost_dump_requested = true;
                    ghost_settle_start_tick = 0;
                }
            } else {
                ghost_settle_start_tick = 0;
            }
        }
    }
}

float Motor_GetPosition(void) { return encoder.current_position_deg; }
float Motor_GetSpeed(void) { return encoder.filtered_rpm; }

/* Emit the strings deferred by the 100 Hz control ISR (bug 1-G). Call from the
 * main loop. Snapshot+clear the event word atomically so a TIM6-ISR set that
 * lands between the read and the clear is preserved for the next drain. */
void Motor_DrainControlLog(void)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    uint32_t ev      = clog_events;
    clog_events      = 0;
    float hold       = clog_hold_pos;
    float th_orig    = clog_temp_home_orig;
    float mo_pos     = clog_move_orig_pos;
    float spd        = clog_move_speed;
    __set_PRIMASK(pm);

    if (ev == 0) return;

    if (ev & CLOG_ESTOP_PHYS_REHOME)  printf("[SAFETY] Physical E-Stop cleared — re-homing in 500 ms (relay settle).\r\n");
    if (ev & CLOG_ESTOP_CLEARED_HOLD) printf("[SAFETY] E-Stop Cleared. Holding at current position (%.2f).\r\n", hold);
    if (ev & CLOG_RELAY_SETTLED)      printf("[SAFETY] Relay settled — starting re-home.\r\n");
    if (ev & CLOG_TEST_START)         printf("START\r\n");
    if (ev & CLOG_HOME_TEMP_SET)      printf("[HOME] Temporary Home Set. Original Home is now at %.2f deg.\r\n", th_orig);
    if (ev & CLOG_HOME_MOVE_TEMP)     printf("[HOME] Moving to Temporary Home (0.0) at %.1f RPM\r\n", spd);
    if (ev & CLOG_HOME_MOVE_ORIG)     printf("[HOME] Moving to Original Home (%.2f) at %.1f RPM\r\n", mo_pos, spd);
    if (ev & CLOG_ENCODER_INVERTED)   printf("CRITICAL: ENCODER INVERTED / PHASE ERROR\r\n");
    if (ev & CLOG_ENCODER_NOSIGNAL)   printf("CRITICAL: ENCODER DISCONNECTED / NO SIGNAL\r\n");
    if (ev & CLOG_MOTOR_STALLED)      printf("CRITICAL: MOTOR STALLED\r\n");
    if (ev & CLOG_SOFT_LIMIT)         printf("WARNING: SOFT LIMIT REACHED (HARD STOP ACTIVE)\r\n");
    if (ev & CLOG_TEST_FINISH)        printf("FINISH\r\n");
}

/* ============================================================================
 * Gripper & Sequence Functions
 * ============================================================================ */

void Gripper_Up(void)    { hw.out_gripper_up = 1; printf("Gripper: UP\r\n"); }
void Gripper_Down(void)  { hw.out_gripper_up = 0; printf("Gripper: DOWN\r\n"); }
void Gripper_Open(void)  { hw.out_gripper_down = 0; printf("Claw: OPEN\r\n"); }
void Gripper_Close(void) { hw.out_gripper_down = 1; printf("Claw: CLOSE\r\n"); }

void Gripper_Toggle(void)
{
    static bool is_open = true;
    if (is_open) Gripper_Close();
    else Gripper_Open();
    is_open = !is_open;
}

/* Wait for a reed switch to read 1, or bail out after REED_SW_TIMEOUT_MS.
 * Runs at thread level (main loop) after bug 1-F. Reads the cached *reed value,
 * which the 100 Hz TIM6 ISR refreshes via HW_RefreshIO — we must NOT call
 * HW_RefreshIO here (bug 1-A): it is not reentrant and is owned by the ISR. */
static void wait_for_reed(volatile uint8_t *reed, const char *name)
{
    uint32_t t0 = HAL_GetTick();
    while (!(*reed)) {
        if ((HAL_GetTick() - t0) >= REED_SW_TIMEOUT_MS) {
            printf("Reed SW timeout: %s\r\n", name);
            return;
        }
    }
    printf("Reed SW OK: %s\r\n", name);
}

/* Task state flags for Modbus 0x27 (Go Pick / Go Place bits) */
volatile bool task_pick_active  = false;
volatile bool task_place_active = false;

/* Conditional wait — only block when control_system_mode == JOYSTICK
 * so Modbus master in BASE_SYSTEM mode never gets stuck waiting. */
static void wait_for_reed_if_joystick(volatile uint8_t *reed, const char *name)
{
    extern volatile Control_SystemMode_t control_system_mode;
    if (control_system_mode != CONTROL_MODE_JOYSTICK) return;
    wait_for_reed(reed, name);
}

void Gripper_Sequence_Pick(void)
{
    printf("Starting Sequence: PICK\r\n");
    task_pick_active = true;
    Gripper_Open();
    wait_for_reed_if_joystick(&hw.in_reed_open, "OPEN");
    Gripper_Down();
    wait_for_reed_if_joystick(&hw.in_reed_down, "DOWN");
    Gripper_Close();
    wait_for_reed_if_joystick(&hw.in_reed_close, "CLOSE");
    Gripper_Up();
    wait_for_reed_if_joystick(&hw.in_reed_up, "UP");
    task_pick_active = false;
    printf("Sequence PICK: Done\r\n");
}

void Gripper_Sequence_Place(void)
{
    printf("Starting Sequence: PLACE\r\n");
    task_place_active = true;
    Gripper_Down();
    wait_for_reed_if_joystick(&hw.in_reed_down, "DOWN");
    Gripper_Open();
    wait_for_reed_if_joystick(&hw.in_reed_open, "OPEN");
    Gripper_Up();
    wait_for_reed_if_joystick(&hw.in_reed_up, "UP");
    Gripper_Close();
    wait_for_reed_if_joystick(&hw.in_reed_close, "CLOSE");
    task_place_active = false;
    printf("Sequence PLACE: Done\r\n");
}
