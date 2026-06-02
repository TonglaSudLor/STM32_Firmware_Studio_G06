/**
 * @file motor_homing.c
 * @brief Homing sequence — sensor-based centre-finding with hysteresis correction.
 *
 * Extracted from motor_controller.c (แบบ A — extern globals, no interface change).
 *
 * State machine flow:
 *   IDLE → INIT → WIGGLE_SEARCH → FIND_EDGE_A → FIND_EDGE_B
 *        → VERIFY_EDGE_B → VERIFY_EDGE_A → CALCULATE_ZERO → IDLE
 *
 * Call Motor_RunHomingSequence() every 100 Hz until it returns true.
 */

#include "motor_controller.h"
#include "hw_io.h"
#include "params.h"
#include "stm32g4xx_hal.h"
#include <math.h>
#include <stdio.h>

/* ---- Globals owned by motor_controller.c ---- */
extern volatile bool                  emergency_stop;
extern          Encoder_Data_t        encoder;
extern volatile Motor_TuningParams_t  tuning;
extern          Trajectory_State_t    trajectory;
extern volatile Motor_ControlMode_t   current_mode;
extern          PID_Controller_t      pid_position;
extern          PID_Controller_t      pid_speed;
extern volatile float                 original_home_offset_deg;
extern          TIM_HandleTypeDef     htim3;

#define DPS_TO_RPM  (1.0f / 6.0f)

/* ---- Homing State ---- */
typedef enum {
    H_IDLE = 0,
    H_INIT,
    H_WIGGLE_SEARCH,
    H_FIND_EDGE_A,
    H_FIND_EDGE_B,
    H_VERIFY_EDGE_B,
    H_VERIFY_EDGE_A,
    H_CALCULATE_ZERO,
    H_DONE,
    H_ERROR
} HomingState_t;

#define HOMING_VERIFY_OVERSHOOT_DEG  1.5f
#define HOMING_VERIFY_MAX_TRAVEL_DEG 30.0f

static HomingState_t h_state         = H_IDLE;
static float h_edge_a = 0, h_edge_b = 0, h_edge_a_verify = 0, h_edge_b_verify = 0;
static float h_verify_reverse_start  = 0;
static float h_verify_a_start        = 0;
static float h_wiggle_amp            = 20.0f;
static float h_start_pos             = 0;
static int   h_direction             = 1;
static bool  h_verify_overshot       = false;
static uint32_t h_wiggle_start_ms    = 0u;

bool Motor_Homing_IsWiggling(void) { return h_state == H_WIGGLE_SEARCH; }

bool Motor_RunHomingSequence(void)
{
    if (emergency_stop) {
        if (h_state != H_IDLE)
            printf("[HOMING] ABORTED: Emergency Stop is active.\r\n");
        h_state = H_IDLE;
        return true;
    }

    if (h_state == H_ERROR)
        h_state = H_IDLE;

    switch (h_state) {
        case H_IDLE:
            h_state = H_INIT;
            return false;

        case H_INIT:
            h_start_pos       = encoder.current_position_deg;
            h_wiggle_amp      = 20.0f;
            h_direction       = 1;
            h_wiggle_start_ms = 0u;
            current_mode = MOTOR_MODE_HOMING;
            Motor_SetMotionProfile(HOMING_SEARCH_RPM, 50.0f, 0.1f);
            h_state = H_WIGGLE_SEARCH;
            printf("[HOMING] Starting Smooth Wiggle Search...\r\n");
            return false;

        case H_WIGGLE_SEARCH:
        {
            if (h_wiggle_start_ms == 0u) h_wiggle_start_ms = HAL_GetTick();
            float t_s = (float)(HAL_GetTick() - h_wiggle_start_ms) * 0.001f;

            const float HOMING_AMP_RATE = 8.0f;
            h_wiggle_amp = HOMING_AMP_RATE * t_s;
            if (h_wiggle_amp < 1.0f) h_wiggle_amp = 1.0f;

            if (h_wiggle_amp >= HOMING_MAX_WIGGLE) {
                h_wiggle_start_ms = 0u;
                h_wiggle_amp      = 20.0f;
                h_state = H_ERROR;
                printf("[HOMING] ERROR: Target not found within limits.\r\n");
                Motor_SendAudioCommand('E');
                return false;
            }

            const float SWEEP_OMEGA = 2.0f * 3.14159265f * 0.2f;
            float sine_pos = h_start_pos + h_wiggle_amp * sinf(SWEEP_OMEGA * t_s);
            if (sine_pos >  SOFT_LIMIT_DEG) sine_pos =  SOFT_LIMIT_DEG;
            if (sine_pos < -SOFT_LIMIT_DEG) sine_pos = -SOFT_LIMIT_DEG;

            trajectory.target_pos           = sine_pos;
            trajectory.current_setpoint_pos = sine_pos;

            float sine_vel_dps = HOMING_AMP_RATE * sinf(SWEEP_OMEGA * t_s)
                               + h_wiggle_amp * SWEEP_OMEGA * cosf(SWEEP_OMEGA * t_s);
            trajectory.current_setpoint_vel   = sine_vel_dps * DPS_TO_RPM;
            trajectory.current_setpoint_accel = 0.0f;

            if (!hw.raw_prox_bit) {
                h_direction = (encoder.filtered_rpm >= 0.0f) ? 1 : -1;
                h_wiggle_start_ms = 0u;
                h_wiggle_amp      = 20.0f;

                pid_position.integral  = 0.0f;
                pid_position.error_prev = encoder.current_position_deg;
                pid_speed.integral     = 0.0f;
                pid_speed.error_prev   = encoder.filtered_rpm;
                trajectory.current_setpoint_pos   = encoder.current_position_deg;
                trajectory.current_setpoint_vel   = 0.0f;
                trajectory.current_setpoint_accel = 0.0f;
                ZVD_FlushBuffer(encoder.current_position_deg);

                Motor_SetMotionProfile(HOMING_CREEP_RPM, 20.0f, 0.1f);
                trajectory.target_pos = encoder.current_position_deg + ((float)h_direction * 360.0f);
                h_state = H_FIND_EDGE_A;
                printf("[HOMING] Target Found! Locating Edge A...\r\n");
            }
            return false;
        }

        case H_FIND_EDGE_A:
            h_edge_a = encoder.current_position_deg;
            h_state  = H_FIND_EDGE_B;
            printf("[HOMING] Edge A: %.2f. Finding Edge B...\r\n", h_edge_a);
            return false;

        case H_FIND_EDGE_B:
            if (hw.raw_prox_bit) {
                h_edge_b         = encoder.current_position_deg;
                h_verify_overshot = false;
                h_state           = H_VERIFY_EDGE_B;
                printf("[HOMING] Edge B (first): %.2f. Verifying...\r\n", h_edge_b);
            }
            return false;

        case H_VERIFY_EDGE_B:
            if (!h_verify_overshot) {
                float traveled = (encoder.current_position_deg - h_edge_b) * (float)h_direction;
                if (traveled >= HOMING_VERIFY_OVERSHOOT_DEG) {
                    h_verify_overshot    = true;
                    h_direction          = -h_direction;
                    h_verify_reverse_start = encoder.current_position_deg;
                    trajectory.current_setpoint_pos = encoder.current_position_deg;
                    trajectory.target_pos = encoder.current_position_deg + ((float)h_direction * 360.0f);
                    printf("[HOMING] Overshot %.2f deg, reversing to verify Edge B\r\n",
                           HOMING_VERIFY_OVERSHOOT_DEG);
                }
                return false;
            }
            if (!hw.raw_prox_bit) {
                h_edge_b_verify = encoder.current_position_deg;
                printf("[HOMING] Edge B (verify): %.2f.  Avg: %.2f\r\n",
                       h_edge_b_verify, 0.5f * (h_edge_b + h_edge_b_verify));
                h_edge_b       = 0.5f * (h_edge_b + h_edge_b_verify);
                h_verify_a_start = encoder.current_position_deg;
                h_state          = H_VERIFY_EDGE_A;
                return false;
            }
            {
                float back_traveled = (h_verify_reverse_start - encoder.current_position_deg)
                                      * (float)(-h_direction);
                if (back_traveled >= HOMING_VERIFY_MAX_TRAVEL_DEG) {
                    printf("[HOMING] Verify abort (B): fell back to Edge B = %.2f\r\n", h_edge_b);
                    h_state = H_CALCULATE_ZERO;
                }
            }
            return false;

        case H_VERIFY_EDGE_A:
            if (hw.raw_prox_bit) {
                h_edge_a_verify = encoder.current_position_deg;
                printf("[HOMING] Edge A (verify): %.2f.  Avg: %.2f\r\n",
                       h_edge_a_verify, 0.5f * (h_edge_a + h_edge_a_verify));
                h_edge_a = 0.5f * (h_edge_a + h_edge_a_verify);
                h_state  = H_CALCULATE_ZERO;
                return false;
            }
            {
                float a_traveled = (h_verify_a_start - encoder.current_position_deg)
                                   * (float)(-h_direction);
                if (a_traveled >= HOMING_VERIFY_MAX_TRAVEL_DEG) {
                    printf("[HOMING] Verify abort (A): fell back to Edge A = %.2f\r\n", h_edge_a);
                    h_state = H_CALCULATE_ZERO;
                }
            }
            return false;

        case H_CALCULATE_ZERO:
        {
            float center = (h_edge_a + h_edge_b) / 2.0f;
            float offset = encoder.current_position_deg - center;

            encoder.absolute_counts = (int32_t)(
                ((offset - tuning.home_offset_deg) / 360.0f) * (MOTOR_ENCODER_PPR * 4));
            Motor_SyncEncoderPosition();

            trajectory.target_pos             = 0.0f;
            trajectory.current_setpoint_pos   = encoder.current_position_deg;
            trajectory.current_setpoint_vel   = 0.0f;

            pid_speed.integral    = 0.0f;
            pid_speed.error_prev  = 0.0f;
            pid_position.integral = 0.0f;
            pid_position.error_prev = 0.0f;

            current_mode = MOTOR_MODE_POSITION;
            original_home_offset_deg = 0.0f;
            Motor_SetMotionProfile(tuning.move_speed_coarse, tuning.max_accel, 0.1f);

            printf("[HOMING] SUCCESS. Center=%.2f, Offset=%.2f, Home=0.0\r\n",
                   center, tuning.home_offset_deg);
            Motor_SendAudioCommand('H');
            h_state = H_IDLE;
        }
        return true;

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
