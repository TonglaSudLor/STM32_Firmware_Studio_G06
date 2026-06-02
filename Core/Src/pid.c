/**
 * @file pid.c
 * @brief PID Controller — pure math, zero HAL dependency.
 *
 * Extracted from Core/Src/motor_controller.c (PID_Compute, anti-windup).
 * Derivative is computed on the error signal with a first-order IIR low-pass
 * (alpha=0.2) to suppress high-frequency noise.
 */

#include "pid.h"

void PID_Reset(PID_t *pid)
{
    pid->integral   = 0.0f;
    pid->error_prev = 0.0f;
    pid->d_filt     = 0.0f;
}

float PID_Update(PID_t *pid, float error, float dt)
{
    if (dt <= 0.0f) return 0.0f;

    /* Proportional */
    float p_term = pid->Kp * error;

    /* Integral with symmetric clamping */
    pid->integral += error * dt;
    if      (pid->integral >  pid->integral_max) pid->integral =  pid->integral_max;
    else if (pid->integral < -pid->integral_max) pid->integral = -pid->integral_max;
    float i_term = pid->Ki * pid->integral;

    /* Derivative on error, IIR low-pass (alpha=0.2) */
    float d_raw  = pid->Kd * (error - pid->error_prev) / dt;
    pid->d_filt  = 0.8f * pid->d_filt + 0.2f * d_raw;
    pid->error_prev = error;

    /* d_filt is negative when error is decreasing (approaching setpoint) → damping */
    float output = p_term + i_term + pid->d_filt;

    /* Output clamping + clamping-based anti-windup (undo last integral step) */
    if (pid->output_max > 0.0f || pid->output_min < 0.0f) {
        if (output > pid->output_max) {
            output = pid->output_max;
            if (error > 0.0f) pid->integral -= error * dt;
        } else if (output < pid->output_min) {
            output = pid->output_min;
            if (error < 0.0f) pid->integral -= error * dt;
        }
    }

    return output;
}
