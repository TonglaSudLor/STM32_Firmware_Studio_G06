/**
 * @file kalman_lib.h
 * @brief 4-State DC Motor Kalman Filter — HAL-decoupled version.
 *
 * TODO: Refactor from Core/Inc/kalman.h + Core/Src/kalman.c
 *   - Remove HAL_GetTick() dependency: inject dt as parameter
 *   - Change: Kalman_Tick(u_volts, theta_rad)
 *     → Kalman_Tick(u_volts, theta_rad, float dt)
 *   - Keep RK4 integration, P matrix, Q/R tuning
 *   - Remove TIM7 reference (caller provides dt)
 *
 * State vector (output-shaft frame):
 *   x = [theta (rad), omega (rad/s), tau_L (N·m), i_a (A)]
 *
 * Motor model params: MOT_* in params.h
 * Noise params: KF_SIGMA_* in params.h
 *
 * When done:
 *   - Move kalman.c → Core/Lib/kalman_lib.c
 *   - Update Core/Src/main.c to pass dt from TIM6
 *   - Add tests/test_kalman.c
 */

#pragma once

/* This file is a placeholder — use Core/Inc/kalman.h until refactor is done. */
#include "kalman.h"
