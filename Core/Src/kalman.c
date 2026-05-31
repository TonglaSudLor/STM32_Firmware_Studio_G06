/**
 * @file kalman.c
 * @brief 4-state linear Kalman filter for the DC motor.
 *
 * Approach: integrate the continuous-time model with RK4 inside a 1 kHz tick.
 *   State derivative:    dx/dt = A x + B u
 *   Covariance ODE:      dP/dt = A P + P A^T + Qc
 * Update step (measurement = theta, H = [1 0 0 0]):
 *   y = z - x[0]
 *   S = P[0][0] + R
 *   K = P[:,0] / S
 *   x += K * y
 *   P  = (I - K H) P
 *
 * All tunable parameters and state estimates are named global volatile scalars
 * so STM32CubeMonitor can read and write them directly by name.
 */

#include "kalman.h"
#include "params.h"
#include <string.h>
#include <math.h>

/* --------------------------------------------------------------------------
 * Model derived from System Identification (see params.h MOT_*).
 * Continuous-time A and B (constant for an LTI plant). Indices match the
 * state vector [theta, omega, tau_L, i_a].
 * -------------------------------------------------------------------------- */
#define A11  1.0f                                                       /* dtheta/dt = omega */
#define A21  (-MOT_B_VISC / MOT_J_INERTIA)                              /* domega term in omega */
#define A22  (-1.0f / MOT_J_INERTIA)                                    /* domega term in tau_L */
#define A23  (MOT_N_GEAR * MOT_ETA_GB * MOT_K_T / MOT_J_INERTIA)        /* domega term in i_a */
#define A41  (-MOT_K_E * MOT_N_GEAR / MOT_L_ARM)                        /* di/dt term in omega */
#define A43  (-MOT_R_ARM / MOT_L_ARM)                                   /* di/dt term in i_a */
#define B4   (1.0f / MOT_L_ARM)                                         /* di/dt term in u */

/* ==========================================================================
 * Named global variables — STM32CubeMonitor: add these by exact name.
 *
 * NOTE on units for Q variables:
 *   CubeMonitor writes VARIANCE (σ²) directly.
 *   The dashboard SET command (KF_Q_THETA=x) takes SIGMA (σ) and squares it.
 *   Example: dashboard KF_Q_THETA=0.001 → CubeMonitor shows kf_q_theta=1e-6
 * ========================================================================== */

/* --- Kalman state estimates (read-only in normal use) --- */
volatile float kf_theta      = 0.0f;  /* angle              (rad)   */
volatile float kf_omega      = 0.0f;  /* angular velocity   (rad/s) */
volatile float kf_tau_l      = 0.0f;  /* load torque        (N·m)   */
volatile float kf_ia         = 0.0f;  /* armature current   (A)     */

/* --- Covariance diagonal: filter confidence (read-only) --- */
volatile float kf_p00        = 1.0f;
volatile float kf_p11        = 100.0f;
volatile float kf_p22        = 100.0f;
volatile float kf_p33        = 100.0f;

/* --- Process noise variances (read/write from CubeMonitor) --- */
volatile float kf_q_theta    = 0.0f;  /* position drift variance    (rad²/s)    */
volatile float kf_q_omega    = 0.0f;  /* velocity noise variance    (rad²/s³)   */
volatile float kf_q_tau      = 0.0f;  /* load-torque random walk    (N²m²/s)    */
volatile float kf_q_ia       = 0.0f;  /* current imperfection var.  (A²/s)      */

/* --- Measurement noise variance (read/write from CubeMonitor) --- */
volatile float kf_r          = KF_R_DEFAULT;   /* encoder quantisation  (rad²) */

/* --- Control flags (read/write from CubeMonitor) --- */
volatile uint8_t kf_enable   = 0;     /* 0 = filter off, 1 = filter on */

/* --- Diagnostic (read-only) --- */
volatile float kf_innovation = 0.0f;  /* latest measurement residual (rad) */

/* --- Open-loop sanity model (read-only) --- */
volatile float kf_sanity_theta = 0.0f;  /* model-predicted angle      (rad)   */
volatile float kf_sanity_omega = 0.0f;  /* model-predicted velocity   (rad/s) */

/* --------------------------------------------------------------------------
 * Internal-only storage (not needed in CubeMonitor)
 * -------------------------------------------------------------------------- */
static volatile float P[4][4];   /* full 4×4 covariance matrix */
static volatile float xs[4];     /* full sanity-model state vector */

/* --------------------------------------------------------------------------
 * Continuous-time derivatives
 * -------------------------------------------------------------------------- */
static inline void state_deriv(const float xi[4], float u, float dx[4])
{
    dx[0] = A11 * xi[1];
    dx[1] = A21 * xi[1] + A22 * xi[2] + A23 * xi[3];
    dx[2] = 0.0f;
    dx[3] = A41 * xi[1] + A43 * xi[3] + B4 * u;
}

static inline void cov_deriv(const float Pin[4][4], float dP[4][4])
{
    float AP[4][4];
    for (int j = 0; j < 4; j++) {
        AP[0][j] = Pin[1][j];
        AP[1][j] = A21 * Pin[1][j] + A22 * Pin[2][j] + A23 * Pin[3][j];
        AP[2][j] = 0.0f;
        AP[3][j] = A41 * Pin[1][j] + A43 * Pin[3][j];
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dP[i][j] = AP[i][j] + AP[j][i];
        }
    }
    /* Add process noise from named globals (variances) */
    dP[0][0] += kf_q_theta;
    dP[1][1] += kf_q_omega;
    dP[2][2] += kf_q_tau;
    dP[3][3] += kf_q_ia;
}

/* --------------------------------------------------------------------------
 * RK4 step over dt
 * -------------------------------------------------------------------------- */
static void rk4_state(float xi[4], float u, float dt)
{
    float k1[4], k2[4], k3[4], k4[4], xt[4];
    state_deriv(xi, u, k1);
    for (int i = 0; i < 4; i++) xt[i] = xi[i] + 0.5f * dt * k1[i];
    state_deriv(xt, u, k2);
    for (int i = 0; i < 4; i++) xt[i] = xi[i] + 0.5f * dt * k2[i];
    state_deriv(xt, u, k3);
    for (int i = 0; i < 4; i++) xt[i] = xi[i] + dt * k3[i];
    state_deriv(xt, u, k4);
    for (int i = 0; i < 4; i++)
        xi[i] += (dt / 6.0f) * (k1[i] + 2.0f * k2[i] + 2.0f * k3[i] + k4[i]);
}

static void rk4_cov(float Pi[4][4], float dt)
{
    float k1[4][4], k2[4][4], k3[4][4], k4[4][4], Pt[4][4];
    cov_deriv(Pi, k1);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) Pt[i][j] = Pi[i][j] + 0.5f * dt * k1[i][j];
    cov_deriv(Pt, k2);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) Pt[i][j] = Pi[i][j] + 0.5f * dt * k2[i][j];
    cov_deriv(Pt, k3);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) Pt[i][j] = Pi[i][j] + dt * k3[i][j];
    cov_deriv(Pt, k4);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Pi[i][j] += (dt / 6.0f) * (k1[i][j] + 2.0f * k2[i][j] + 2.0f * k3[i][j] + k4[i][j]);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */
void Kalman_Init(void)
{
    kf_theta = 0.0f; kf_omega = 0.0f; kf_tau_l = 0.0f; kf_ia = 0.0f;

    for (int i = 0; i < 4; i++) {
        xs[i] = 0.0f;
        for (int j = 0; j < 4; j++) P[i][j] = 0.0f;
    }
    P[0][0] = 1.0f; P[1][1] = 100.0f; P[2][2] = 100.0f; P[3][3] = 100.0f;
    kf_p00 = 1.0f; kf_p11 = 100.0f; kf_p22 = 100.0f; kf_p33 = 100.0f;

    kf_q_theta = KF_SIGMA_THETA_DEF * KF_SIGMA_THETA_DEF;
    kf_q_omega = KF_SIGMA_OMEGA_DEF * KF_SIGMA_OMEGA_DEF;
    kf_q_tau   = KF_SIGMA_TAU_DEF   * KF_SIGMA_TAU_DEF;
    kf_q_ia    = KF_SIGMA_I_DEF     * KF_SIGMA_I_DEF;
    kf_r       = KF_R_DEFAULT;

    kf_enable    = 0;
    kf_innovation = 0.0f;
    kf_sanity_theta = 0.0f;
    kf_sanity_omega = 0.0f;
}

void Kalman_Reset(float theta_meas_rad)
{
    kf_theta = theta_meas_rad; kf_omega = 0.0f; kf_tau_l = 0.0f; kf_ia = 0.0f;

    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) P[i][j] = 0.0f;

    P[0][0] = kf_r * 10.0f;
    P[1][1] = 100.0f; P[2][2] = 100.0f; P[3][3] = 100.0f;
    kf_p00 = P[0][0]; kf_p11 = 100.0f; kf_p22 = 100.0f; kf_p33 = 100.0f;

    kf_innovation = 0.0f;
}

void Kalman_Tick(float u_volts, float theta_meas_rad)
{
    /* --- Predict --- */
    float xl[4] = { kf_theta, kf_omega, kf_tau_l, kf_ia };
    float Pl[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) Pl[i][j] = P[i][j];

    rk4_state(xl, u_volts, KF_DT);
    rk4_cov(Pl, KF_DT);

    /* --- Update (H = [1 0 0 0]) --- */
    float y = theta_meas_rad - xl[0];
    float S = Pl[0][0] + kf_r;
    if (S < 1e-12f) S = 1e-12f;
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = Pl[i][0] / S;

    for (int i = 0; i < 4; i++) xl[i] += K[i] * y;

    /* --- Covariance update: Joseph stabilized form --- */
    float MP[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            MP[i][j] = Pl[i][j] - K[i] * Pl[0][j];

    float Pjos[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Pjos[i][j] = MP[i][j] - K[j] * MP[i][0] + kf_r * K[i] * K[j];

    /* Symmetrize and write back */
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            P[i][j] = 0.5f * (Pjos[i][j] + Pjos[j][i]);

    /* Publish state and diagnostics to named globals */
    kf_theta = xl[0]; kf_omega = xl[1]; kf_tau_l = xl[2]; kf_ia = xl[3];
    kf_p00 = P[0][0]; kf_p11 = P[1][1]; kf_p22 = P[2][2]; kf_p33 = P[3][3];
    kf_innovation = y;
}

void Kalman_SanityTick(float u_volts)
{
    float xl[4];
    for (int i = 0; i < 4; i++) xl[i] = xs[i];
    rk4_state(xl, u_volts, KF_DT);
    for (int i = 0; i < 4; i++) xs[i] = xl[i];
    kf_sanity_theta = xs[0];
    kf_sanity_omega = xs[1];
}

void Kalman_SanityReset(float theta_meas_rad)
{
    for (int i = 0; i < 4; i++) xs[i] = 0.0f;
    xs[0] = theta_meas_rad;
    kf_sanity_theta = theta_meas_rad;
    kf_sanity_omega = 0.0f;
}

/* --- Enable / disable --- */
void  Kalman_SetEnabled(bool en) { kf_enable = en ? 1u : 0u; }
bool  Kalman_GetEnabled(void)    { return kf_enable != 0u; }

/* --- Noise setters: dashboard takes sigma, we store variance --- */
void  Kalman_SetSigmaTheta(float s) { kf_q_theta = s * s; }
void  Kalman_SetSigmaOmega(float s) { kf_q_omega = s * s; }
void  Kalman_SetSigmaTau  (float s) { kf_q_tau   = s * s; }
void  Kalman_SetSigmaI    (float s) { kf_q_ia    = s * s; }
void  Kalman_SetR         (float r) { kf_r = (r > 1e-15f) ? r : 1e-15f; }

/* --- State getters --- */
float Kalman_GetTheta(void)      { return kf_theta; }
float Kalman_GetOmega(void)      { return kf_omega; }
float Kalman_GetOmegaRPM(void)   { return kf_omega * (60.0f / (2.0f * 3.14159265f)); }
float Kalman_GetLoadTorque(void) { return kf_tau_l; }
float Kalman_GetCurrent(void)    { return kf_ia; }
float Kalman_GetInnovation(void) { return kf_innovation; }

float Kalman_GetP00(void) { return kf_p00; }
float Kalman_GetP11(void) { return kf_p11; }
float Kalman_GetP22(void) { return kf_p22; }
float Kalman_GetP33(void) { return kf_p33; }

float Kalman_SanityGetTheta(void) { return kf_sanity_theta; }
float Kalman_SanityGetOmega(void) { return kf_sanity_omega; }
