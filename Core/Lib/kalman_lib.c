/**
 * @file kalman_lib.c
 * @brief 4-state DC motor Kalman Filter — Implementation using RK4.
 */

#include "kalman_lib.h"
#include "params.h"
#include <math.h>
#include <string.h>

// Helper to clear matrix
static void mat_eye(float m[4][4], float val) {
    memset(m, 0, sizeof(float)*16);
    for(int i=0; i<4; i++) m[i][i] = val;
}

void Kalman_Init(Kalman_t *k, float initial_theta_rad)
{
    Kalman_Reset(k, initial_theta_rad);
    k->sigma_theta = KF_SIGMA_THETA_DEF;
    k->sigma_omega = KF_SIGMA_OMEGA_DEF;
    k->sigma_tau   = KF_SIGMA_TAU_DEF;
    k->sigma_i     = KF_SIGMA_I_DEF;
    k->R           = KF_R_DEFAULT;
    k->enabled     = true;
}

void Kalman_Reset(Kalman_t *k, float theta_rad)
{
    memset(k->x, 0, sizeof(k->x));
    k->x[0] = theta_rad;
    mat_eye(k->P, 1.0f); // Initial uncertainty
    k->innovation = 0.0f;
}

/**
 * @brief Continuous-time system dynamics: dx/dt = f(x, u)
 * States: 0:theta, 1:omega, 2:tau_L, 3:i_a
 */
static void f(const float x[4], float u, float dxdt[4])
{
    // Parameters from params.h
    const float Ra = MOT_R_ARM;
    const float La = MOT_L_ARM;
    const float Ke = MOT_K_E;
    const float Kt = MOT_K_T;
    const float J  = MOT_J_INERTIA;
    const float B  = MOT_B_VISC;
    const float N  = MOT_N_GEAR;
    const float eta = MOT_ETA_GB;

    dxdt[0] = x[1]; // dtheta = omega
    
    // domega = (Kt*N*eta*ia - B*omega - tau_L) / J
    dxdt[1] = (Kt * N * eta * x[3] - B * x[1] - x[2]) / J;
    
    dxdt[2] = 0.0f; // dtau_L = 0 (random walk modeled in Q)
    
    // dia = (u - Ke*N*omega - Ra*ia) / La
    dxdt[3] = (u - Ke * N * x[1] - Ra * x[3]) / La;
}

void Kalman_Step(Kalman_t *k, float u, float z, float dt)
{
    if (!k->enabled) {
        k->x[0] = z; // Latch to measurement
        return;
    }

    // --- 1. Predict (RK4 Integration) ---
    float k1[4], k2[4], k3[4], k4[4], xt[4];
    
    f(k->x, u, k1);
    for(int i=0; i<4; i++) xt[i] = k->x[i] + k1[i]*dt*0.5f;
    f(xt, u, k2);
    for(int i=0; i<4; i++) xt[i] = k->x[i] + k2[i]*dt*0.5f;
    f(xt, u, k3);
    for(int i=0; i<4; i++) xt[i] = k->x[i] + k3[i]*dt;
    f(xt, u, k4);
    
    for(int i=0; i<4; i++) {
        k->x[i] += (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]) * dt / 6.0f;
    }

    // --- 2. Predict Covariance (P = P + Q*dt) ---
    // Simplified: we add Q to diagonal. In a full EKF we'd use Jacobian.
    k->P[0][0] += k->sigma_theta * k->sigma_theta * dt;
    k->P[1][1] += k->sigma_omega * k->sigma_omega * dt;
    k->P[2][2] += k->sigma_tau   * k->sigma_tau   * dt;
    k->P[3][3] += k->sigma_i     * k->sigma_i     * dt;

    // --- 3. Update (Measurement z = theta) ---
    // H = [1 0 0 0]
    // y = z - Hx
    k->innovation = z - k->x[0];
    
    // S = HPH' + R = P00 + R
    float S = k->P[0][0] + k->R;
    
    // K = PH' / S
    float K[4];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;
    K[2] = k->P[2][0] / S;
    K[3] = k->P[3][0] / S;
    
    // x = x + Ky
    for(int i=0; i<4; i++) k->x[i] += K[i] * k->innovation;
    
    // P = (I - KH)P
    // Row 0: P0j = (1-K0)P0j
    // Row 1: P1j = P1j - K1*P0j ...
    for(int j=0; j<4; j++) {
        float p0j = k->P[0][j];
        k->P[0][j] = (1.0f - K[0]) * p0j;
        k->P[1][j] = k->P[1][j] - K[1] * p0j;
        k->P[2][j] = k->P[2][j] - K[2] * p0j;
        k->P[3][j] = k->P[3][j] - K[3] * p0j;
    }
}
