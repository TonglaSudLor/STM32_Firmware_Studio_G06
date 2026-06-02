/**
 * @file kalman_lib.h
 * @brief 4-state DC motor Kalman Filter — Pure C99, Zero HAL.
 */

#ifndef KALMAN_LIB_H
#define KALMAN_LIB_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Kalman Filter state and covariance
 */
typedef struct {
    // States: [theta (rad), omega (rad/s), tau_L (Nm), i_a (A)]
    float x[4];
    
    // Covariance matrix P (4x4, symmetric)
    // We store only the diagonal for simplicity if full matrix is too heavy for M4
    // But since we have RK4 integration, let's keep it robust.
    float P[4][4];
    
    // Process noise (Q) and Measurement noise (R)
    float sigma_theta, sigma_omega, sigma_tau, sigma_i;
    float R;
    
    float innovation;
    bool enabled;
} Kalman_t;

/**
 * @brief Initialize Kalman filter with system-identified parameters
 */
void Kalman_Init(Kalman_t *k, float initial_theta_rad);

/**
 * @brief Prediction and Update step (call at fixed dt)
 * @param u Input voltage (V)
 * @param z Measurement theta (rad)
 * @param dt Time step (s)
 */
void Kalman_Step(Kalman_t *k, float u, float z, float dt);

/**
 * @brief Reset states and covariance
 */
void Kalman_Reset(Kalman_t *k, float theta_rad);

#endif /* KALMAN_LIB_H */
