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
