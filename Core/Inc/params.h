/**
 * @file params.h
 * @brief Global adjustable parameters for the PID Motor Controller.
 */

#ifndef PARAMS_H
#define PARAMS_H

/* ============================================================================
 * PID GAINS (Tuning)
 * ============================================================================ */

/* Speed Loop (Inner) */
#define DEFAULT_SPEED_KP    1.0f
#define DEFAULT_SPEED_KI    2.0f
#define DEFAULT_SPEED_KD    0.0f
/* Trajectory feedforward (matches cascade-control block diagram).
 *   Kvff units: V / (rad/s)   — compensates back-EMF + viscous damping
 *   Kaff units: V / (rad/s^2) — compensates inertia
 * Firmware converts trajectory RPM/(RPM/s) into rad/s/(rad/s^2) and Volts into PWM%.
 */
#define DEFAULT_K_VFF       0.0f
#define DEFAULT_K_AFF       0.0f

/* Position Loop (Outer) */
#define DEFAULT_POS_KP      0.4f
#define DEFAULT_POS_KI      0.05f
#define DEFAULT_POS_KD      0.1f

/* ============================================================================
 * MOTION & JOG SPEEDS
 * ============================================================================ */

#define JOG_SPEED_FINE      10.0f   /**< Increased for better feedback */
#define MOVE_SPEED_COARSE   69.74f  /**< 7.304 rad/s converted to RPM (v_max for S-curve) */
#define MOVE_SPEED_RETURN_HOME 30.0f /**< Custom speed for returning to home/origin */

#define STEP_SIZE_COARSE    10.0f
#define STEP_SIZE_FINE      1.0f

/* ============================================================================
 * HARDWARE LIMITS
 * ============================================================================ */

#define MAX_VOLTAGE_LIMIT   24.0f
#define SUPPLY_VOLTAGE      24.0f

#define PID_INTEGRAL_MAX    50.0f
#define POS_INTEGRAL_MAX    200.0f

#define DEFAULT_MIN_PWM     0.0f    /**< Increased to overcome static friction */
#define DEFAULT_MAX_ACCEL   262.5f   /**< 27.49 rad/s^2 converted to RPM/s   (a_max for S-curve) */
#define DEFAULT_MAX_JERK    5252.0f  /**< 550 rad/s^3 converted to RPM/s² (j_max for S-curve, T_j ≈ 50ms) */

/* ============================================================================
 * SAFETY PROTOCOLS
 * ============================================================================ */

#define STALL_PWM_THRESHOLD      25.0f   /**< Increased from 15% */
#define STALL_VELOCITY_THRESHOLD 0.5f    /**< More sensitive low-speed detection */
#define STALL_TIME_MS            2000    /**< Give it 2 seconds to start moving */
#define STALL_SETTLING_ERROR_DEG 5.0f    /**< Don't trigger stall if error < 5 deg */

#define ENCODER_FAULT_PWM_THRESHOLD 50.0f   /**< PWM threshold for hardware check */
#define ENCODER_INVERSION_RPM_LIMIT 5.0f    /**< RPM threshold for inversion check */

#define SOFT_LIMIT_DEG           720.0f  /**< 2 full rounds limit from home */

/* ============================================================================
 * SYSTEM TIMING & HOMING
 * ============================================================================ */
#define HOME_HOLD_TIME_MS   1000

#define HOMING_SEARCH_RPM    10.0f    /**< Speed for wiggle search */
#define HOMING_CREEP_RPM     1.0f    /**< Speed for fine edge detection */
#define HOMING_MAX_WIGGLE    180.0f  /**< Max search amplitude to protect cables */

/* ============================================================================
 * MOTOR PHYSICAL MODEL (System Identification)
 * Used by the Kalman filter. All mechanical terms are referred to the
 * OUTPUT SHAFT (where the encoder is mounted). Electrical terms are at the
 * motor terminals.
 * ============================================================================ */
#define MOT_R_ARM       1.4534f     /**< Armature resistance (Ohm) */
#define MOT_L_ARM       0.001448f   /**< Armature inductance (H) */
#define MOT_K_E         0.04165f    /**< Back-EMF constant (V·s/rad, motor shaft) */
#define MOT_K_T         0.04065f    /**< Torque constant (N·m/A, motor shaft) */
#define MOT_B_VISC      0.19279f    /**< Viscous damping at output (N·m·s/rad) */
#define MOT_J_INERTIA   0.72762f    /**< Total inertia at output (kg·m^2) */
#define MOT_N_GEAR      70.0f       /**< Total gear ratio (motor / output) */
#define MOT_ETA_GB      0.836f      /**< Gearbox efficiency */

/* ============================================================================
 * KALMAN FILTER
 * State x = [theta, omega, tau_L, i_a] in output-shaft frame.
 * Encoder bin = 360°/8192 ≈ 0.04395° ≈ 7.67e-4 rad.
 * R_default = bin^2 / 12 ≈ 4.9e-8 rad^2.
 * Q_c (continuous) is diagonal; the four sigma values are tunable live.
 * ============================================================================ */
#define KF_DT               0.001f       /**< KF tick = 1 kHz on TIM7 */
#define KF_R_DEFAULT        4.9e-8f      /**< Encoder quantization variance (rad^2) */
#define KF_SIGMA_THETA_DEF  1.0e-3f      /**< sqrt(Qc[0,0]) — position drift (rad/sqrt(s)) */
#define KF_SIGMA_OMEGA_DEF  1.0e-1f      /**< sqrt(Qc[1,1]) — unmodeled friction (rad/s/sqrt(s)) */
#define KF_SIGMA_TAU_DEF    5.0e-1f      /**< sqrt(Qc[2,2]) — load-torque random walk (N·m/sqrt(s)) */
#define KF_SIGMA_I_DEF      5.0e-1f      /**< sqrt(Qc[3,3]) — voltage/PWM imperfections (A/sqrt(s)) */

#endif /* PARAMS_H */
