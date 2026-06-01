/**
 * @file input_shaper.h
 * @brief ZVD Input Shaper — pure math, zero HAL dependency.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - Shaper ring buffer (100 ticks)
 *   - Impulse coefficients A1, A2, A3 + delays T1, T2, T3
 *   - Motor_ShaperRecompute() logic
 *   - Push logic inside Motor_ControlLoop()
 *
 * Params used (from params.h):
 *   DEFAULT_SHAPER_OMEGA_N  = 12.13 rad/s
 *   DEFAULT_SHAPER_ZETA     = 0.041
 *
 * ZVD impulse timing:
 *   T_d = pi / (omega_n * sqrt(1 - zeta^2))
 *   Delay N ticks = round(T_d * 100Hz)
 *
 * When done: add Core/Lib/input_shaper.c + tests/test_input_shaper.c
 */

#pragma once
#include <stdbool.h>

#define SHAPER_BUFFER_SIZE 100   /* 1 s max delay at 100 Hz */

typedef struct {
    float  buf[SHAPER_BUFFER_SIZE];
    int    head;
    float  A[3];    /* impulse amplitudes [A1, A2, A3] — sum to 1.0 */
    int    T[3];    /* impulse delays in ticks [0, N, 2N] */
    bool   enabled;
} Shaper_t;

void  Shaper_Init(Shaper_t *s, float omega_n, float zeta, float dt);
void  Shaper_Recompute(Shaper_t *s, float omega_n, float zeta);
float Shaper_Push(Shaper_t *s, float setpoint);
