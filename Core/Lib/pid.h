/**
 * @file pid.h
 * @brief PID Controller — pure math, zero HAL dependency.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - PID_Controller_t struct (already in motor_controller.h)
 *   - PID update logic inside Motor_ControlLoop()
 *   - Anti-windup, derivative filter
 *
 * When done: add Core/Lib/pid.c + tests/test_pid.c
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float integral_max;
    float error_prev;
    float d_filt;           /* derivative low-pass filter state */
    float output_min, output_max;
} PID_t;

/**
 * @brief Reset PID state (integral, derivative filter, previous error).
 */
void PID_Reset(PID_t *pid);

/**
 * @brief Compute one PID step.
 * @param pid   Controller state.
 * @param error Setpoint − measurement.
 * @param dt    Time step in seconds.
 * @return      Controller output (clamped to output_min / output_max).
 */
float PID_Update(PID_t *pid, float error, float dt);
