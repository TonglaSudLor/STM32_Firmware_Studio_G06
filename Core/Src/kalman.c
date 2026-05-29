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

/* --------------------------------------------------------------------------
 * State, covariance, and tunables
 * -------------------------------------------------------------------------- */
static volatile float x[4];           /* theta, omega, tau_L, i_a */
static volatile float P[4][4];
static volatile float Qc_diag[4];     /* continuous process noise (variances) */
static volatile float R_meas;
static volatile bool  enabled = false;
static volatile float last_innovation = 0.0f;

/* Open-loop sanity model (same RK4, no correction) */
static volatile float xs[4];

/* --------------------------------------------------------------------------
 * Continuous-time derivatives
 * -------------------------------------------------------------------------- */
static inline void state_deriv(const float xi[4], float u, float dx[4])
{
    /* dtheta/dt = omega                                                   */
    dx[0] = A11 * xi[1];
    /* domega/dt = -b/J * omega - 1/J * tau_L + N*eta*Kt/J * i             */
    dx[1] = A21 * xi[1] + A22 * xi[2] + A23 * xi[3];
    /* dtau_L/dt = 0 (random walk; the noise drives it, mean is zero)       */
    dx[2] = 0.0f;
    /* di/dt = -Ke*N/L * omega - R/L * i + 1/L * u                          */
    dx[3] = A41 * xi[1] + A43 * xi[3] + B4 * u;
}

/* dP/dt = A P + P A^T + Qc.
 * We never explicitly store A as a matrix; we open up the non-zero rows
 * inline. A is structurally:
 *   row0 = [0, 1, 0, 0]
 *   row1 = [0, A21, A22, A23]
 *   row2 = [0, 0,   0,   0]
 *   row3 = [0, A41, 0,   A43]
 * So (A P)[i][j] = sum_k A[i][k] * P[k][j]. */
static inline void cov_deriv(const float Pin[4][4], float dP[4][4])
{
    float AP[4][4];
    /* AP rows */
    for (int j = 0; j < 4; j++) {
        AP[0][j] = Pin[1][j];                                                       /* A row 0 = e1 */
        AP[1][j] = A21 * Pin[1][j] + A22 * Pin[2][j] + A23 * Pin[3][j];
        AP[2][j] = 0.0f;
        AP[3][j] = A41 * Pin[1][j] + A43 * Pin[3][j];
    }
    /* dP = AP + AP^T + diag(Qc) */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            dP[i][j] = AP[i][j] + AP[j][i];
        }
        dP[i][i] += Qc_diag[i];
    }
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
    for (int i = 0; i < 4; i++) {
        x[i]  = 0.0f;
        xs[i] = 0.0f;
        for (int j = 0; j < 4; j++) P[i][j] = 0.0f;
    }
    /* Big initial uncertainty so the first encoder reading slams the state */
    P[0][0] = 1.0f;
    P[1][1] = 100.0f;
    P[2][2] = 100.0f;
    P[3][3] = 100.0f;

    Qc_diag[0] = KF_SIGMA_THETA_DEF * KF_SIGMA_THETA_DEF;
    Qc_diag[1] = KF_SIGMA_OMEGA_DEF * KF_SIGMA_OMEGA_DEF;
    Qc_diag[2] = KF_SIGMA_TAU_DEF   * KF_SIGMA_TAU_DEF;
    Qc_diag[3] = KF_SIGMA_I_DEF     * KF_SIGMA_I_DEF;
    R_meas = KF_R_DEFAULT;
    enabled = false;
    last_innovation = 0.0f;
}

void Kalman_Reset(float theta_meas_rad)
{
    for (int i = 0; i < 4; i++) {
        x[i] = 0.0f;
        for (int j = 0; j < 4; j++) P[i][j] = 0.0f;
    }
    x[0] = theta_meas_rad;
    P[0][0] = R_meas * 10.0f;
    P[1][1] = 100.0f;
    P[2][2] = 100.0f;
    P[3][3] = 100.0f;
    last_innovation = 0.0f;
}

void Kalman_Tick(float u_volts, float theta_meas_rad)
{
    /* --- Predict --- */
    float xl[4];
    float Pl[4][4];
    for (int i = 0; i < 4; i++) {
        xl[i] = x[i];
        for (int j = 0; j < 4; j++) Pl[i][j] = P[i][j];
    }
    rk4_state(xl, u_volts, KF_DT);
    rk4_cov(Pl, KF_DT);

    /* --- Update (H = [1 0 0 0]) --- */
    float y = theta_meas_rad - xl[0];           /* innovation */
    float S = Pl[0][0] + R_meas;
    if (S < 1e-12f) S = 1e-12f;                 /* numerical guard */
    float K[4];
    for (int i = 0; i < 4; i++) K[i] = Pl[i][0] / S;

    for (int i = 0; i < 4; i++) xl[i] += K[i] * y;

    /* --- Covariance update: Joseph stabilized form ---
     * P = (I - K H) P (I - K H)^T + K R K^T
     * This is algebraically identical to the simple form P = (I - K H) P, but
     * remains symmetric positive-semidefinite under float32 round-off even when
     * R (4.9e-8) is many orders of magnitude smaller than the predicted P
     * entries (~100). With H = [1 0 0 0], (I - K H) is the identity with its
     * first COLUMN replaced by (e0 - K). We exploit that sparsity:
     *   MP = (I - K H) P          -> MP[i][j] = P[i][j] - K[i] * P[0][j]
     *   Then right-multiply by (I - K H)^T, whose first ROW is (e0 - K):
     *     Pjos[i][j] = MP[i][j] - K[j] * MP[i][0] + R * K[i] * K[j]
     */
    float MP[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            MP[i][j] = Pl[i][j] - K[i] * Pl[0][j];

    float Pjos[4][4];
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            Pjos[i][j] = MP[i][j] - K[j] * MP[i][0] + R_meas * K[i] * K[j];

    /* Symmetrize to fight residual numerical drift */
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            P[i][j] = 0.5f * (Pjos[i][j] + Pjos[j][i]);
        }
        x[i] = xl[i];
    }
    last_innovation = y;
}

void Kalman_SanityTick(float u_volts)
{
    float xl[4];
    for (int i = 0; i < 4; i++) xl[i] = xs[i];
    rk4_state(xl, u_volts, KF_DT);
    for (int i = 0; i < 4; i++) xs[i] = xl[i];
}

void Kalman_SanityReset(float theta_meas_rad)
{
    for (int i = 0; i < 4; i++) xs[i] = 0.0f;
    xs[0] = theta_meas_rad;
}

void  Kalman_SetEnabled(bool en) { enabled = en; }
bool  Kalman_GetEnabled(void)    { return enabled; }

void  Kalman_SetSigmaTheta(float s) { Qc_diag[0] = s * s; }
void  Kalman_SetSigmaOmega(float s) { Qc_diag[1] = s * s; }
void  Kalman_SetSigmaTau  (float s) { Qc_diag[2] = s * s; }
void  Kalman_SetSigmaI    (float s) { Qc_diag[3] = s * s; }
void  Kalman_SetR         (float r) { R_meas = (r > 1e-15f) ? r : 1e-15f; }

float Kalman_GetTheta(void)       { return x[0]; }
float Kalman_GetOmega(void)       { return x[1]; }
float Kalman_GetOmegaRPM(void)    { return x[1] * (60.0f / (2.0f * 3.14159265f)); }
float Kalman_GetLoadTorque(void)  { return x[2]; }
float Kalman_GetCurrent(void)     { return x[3]; }
float Kalman_GetInnovation(void)  { return last_innovation; }

float Kalman_GetP00(void) { return P[0][0]; }
float Kalman_GetP11(void) { return P[1][1]; }
float Kalman_GetP22(void) { return P[2][2]; }
float Kalman_GetP33(void) { return P[3][3]; }

float Kalman_SanityGetTheta(void) { return xs[0]; }
float Kalman_SanityGetOmega(void) { return xs[1]; }
