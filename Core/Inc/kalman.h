/**
 * @file kalman.h
 * @brief 4-state Kalman filter for the DC motor.
 *
 * State vector (output-shaft frame):
 *   x[0] = theta   (rad)         angular position
 *   x[1] = omega   (rad/s)       angular velocity
 *   x[2] = tau_L   (N·m)         load torque (random-walk disturbance)
 *   x[3] = i_a     (A)           armature current
 *
 * Input:    u    motor terminal voltage (V), signed.
 * Measure:  z    encoder angle (rad).
 *
 * The continuous-time model is integrated with RK4 inside a 1 kHz TIM7
 * interrupt. Q_c (process noise) and R (measurement noise) are tunable at
 * runtime via SET commands from the dashboard.
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <stdbool.h>
#include <stdint.h>

/* ==========================================================================
 * Named global variables — add these by exact name in STM32CubeMonitor.
 *
 * Q variables store VARIANCE (σ²).  The dashboard SET command takes σ and
 * squares it internally, so KF_Q_THETA=0.001 → kf_q_theta = 1e-6 here.
 * When writing directly in CubeMonitor, write the variance, not sigma.
 * ========================================================================== */

/* Kalman state estimates */
extern volatile float    kf_theta;        /* angle (rad) */
extern volatile float    kf_omega;        /* velocity (rad/s) */
extern volatile float    kf_tau_l;        /* load torque (N·m) */
extern volatile float    kf_ia;           /* armature current (A) */

/* Covariance diagonal (filter confidence) */
extern volatile float    kf_p00;
extern volatile float    kf_p11;
extern volatile float    kf_p22;
extern volatile float    kf_p33;

/* Process noise variances — WRITE to tune (CubeMonitor writes variance σ²) */
extern volatile float    kf_q_theta;      /* position drift */
extern volatile float    kf_q_omega;      /* velocity noise */
extern volatile float    kf_q_tau;        /* load-torque random walk */
extern volatile float    kf_q_ia;         /* current imperfection */

/* Measurement noise variance */
extern volatile float    kf_r;            /* encoder quantisation (rad²) */

/* Control */
extern volatile uint8_t  kf_enable;       /* 0 = off, 1 = on */

/* Diagnostic */
extern volatile float    kf_innovation;   /* measurement residual (rad) */

/* Open-loop sanity model */
extern volatile float    kf_sanity_theta; /* model angle (rad) */
extern volatile float    kf_sanity_omega; /* model velocity (rad/s) */

void  Kalman_Init(void);
void  Kalman_Tick(float u_volts, float theta_meas_rad);   /* call at 1 kHz */
void  Kalman_Reset(float theta_meas_rad);                 /* re-anchor to encoder */

/* Enable / disable. When disabled, the filter still runs (so we can compare
 * against the lowpass), but downstream code should ignore the estimates. */
void  Kalman_SetEnabled(bool en);
bool  Kalman_GetEnabled(void);

/* Tunable noise parameters (each takes the *standard deviation*, not variance) */
void  Kalman_SetSigmaTheta(float s);
void  Kalman_SetSigmaOmega(float s);
void  Kalman_SetSigmaTau(float s);
void  Kalman_SetSigmaI(float s);
void  Kalman_SetR(float r);

/* Getters for the dashboard / control loop */
float Kalman_GetTheta(void);
float Kalman_GetOmega(void);          /* rad/s */
float Kalman_GetOmegaRPM(void);       /* convenience: rad/s -> RPM */
float Kalman_GetLoadTorque(void);
float Kalman_GetCurrent(void);
float Kalman_GetInnovation(void);

/* P diagonal for inspection (variances of each state estimate) */
float Kalman_GetP00(void);
float Kalman_GetP11(void);
float Kalman_GetP22(void);
float Kalman_GetP33(void);

/* Open-loop "sanity check" model — same RK4 integration but never corrected
 * by measurement. The dashboard plots this so a wrong physical parameter
 * shows up before the KF is engaged. */
void  Kalman_SanityTick(float u_volts);
void  Kalman_SanityReset(float theta_meas_rad);
float Kalman_SanityGetTheta(void);
float Kalman_SanityGetOmega(void);

#endif /* KALMAN_H */
