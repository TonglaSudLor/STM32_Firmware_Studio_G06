/**
 * @file trajectory.c
 * @brief 7-segment S-curve trajectory planner.
 *
 * Extracted from motor_controller.c (แบบ A — extern globals, no interface change).
 * ZVD shaper state stays in motor_controller.c (tightly coupled to Motor_ControlLoop).
 *
 * Entry points:
 *   Trajectory_Generator_Update() — call every 100 Hz tick from Motor_ControlLoop
 */

#include "motor_controller.h"
#include "params.h"
#include <math.h>

/* ---- Globals owned by motor_controller.c ---- */
extern volatile Motor_TuningParams_t tuning;
extern          Trajectory_State_t   trajectory;
extern          Trajectory_Config_t  motion_config;
extern          PID_Controller_t     pid_position;

#define CTRL_DT      0.01f   /* 100 Hz — sync with Motor_ControlLoop TIM6 period */
#define RPM_TO_DPS   6.0f
#define DPS_TO_RPM   (1.0f / 6.0f)

/* ============================================================================
 * S-Curve Plan State
 * ============================================================================ */
typedef struct {
    bool   active;
    int8_t direction;
    float  pos_start_deg;
    float  pos_target_deg;
    float  distance;
    float  t_elapsed;
    float  T_j, T_a, T_v;
    float  a_peak_dps2;
    float  v_peak_dps;
    float  jmax_dps3;
    float  t1, t2, t3, t4, t5, t6, t7;
    float  v1_dps, v2_dps;
    float  p1_deg, p2_deg, p3_deg, p4_deg, p5_deg, p6_deg;
} SCurvePlan_t;

static SCurvePlan_t s_plan = {0};

/* ============================================================================
 * Planner — analytical 7-phase computation
 * ============================================================================ */
static void scurve_plan(float pos_start, float pos_target,
                        float vmax_dps, float amax_dps2, float jmax_dps3)
{
    s_plan.t_elapsed      = 0.0f;
    s_plan.pos_start_deg  = pos_start;
    s_plan.pos_target_deg = pos_target;
    s_plan.distance       = fabsf(pos_target - pos_start);
    s_plan.direction      = (pos_target >= pos_start) ? +1 : -1;
    s_plan.jmax_dps3      = jmax_dps3;

    if (s_plan.distance < 0.001f ||
        vmax_dps  <= 0.0f ||
        amax_dps2 <= 0.0f ||
        jmax_dps3 <= 0.0f)
    {
        s_plan.T_j = s_plan.T_a = s_plan.T_v = 0.0f;
        s_plan.a_peak_dps2 = 0.0f;
        s_plan.v_peak_dps  = 0.0f;
        s_plan.t1 = s_plan.t2 = s_plan.t3 = s_plan.t4 =
        s_plan.t5 = s_plan.t6 = s_plan.t7 = 0.0f;
        s_plan.active = false;
        return;
    }

    float T_j_full       = amax_dps2 / jmax_dps3;
    float v_after_jerks  = amax_dps2 * T_j_full;
    float T_j, T_a, a_peak, v_peak;

    if (vmax_dps >= v_after_jerks) {
        T_j    = T_j_full;
        a_peak = amax_dps2;
        T_a    = vmax_dps / amax_dps2 - T_j;
        v_peak = vmax_dps;
    } else {
        a_peak = sqrtf(vmax_dps * jmax_dps3);
        T_j    = a_peak / jmax_dps3;
        T_a    = 0.0f;
        v_peak = vmax_dps;
    }

    float d_accel = v_peak * (2.0f * T_j + T_a) * 0.5f;
    float T_v;
    if (2.0f * d_accel <= s_plan.distance) {
        T_v = (s_plan.distance - 2.0f * d_accel) / v_peak;
    } else {
        T_v = 0.0f;
        float aTj  = amax_dps2 * T_j_full;
        float disc = aTj * aTj + 4.0f * amax_dps2 * s_plan.distance;
        float v_try = (-aTj + sqrtf(disc)) * 0.5f;

        if (v_try >= v_after_jerks) {
            v_peak = v_try;
            a_peak = amax_dps2;
            T_j    = T_j_full;
            T_a    = v_peak / amax_dps2 - T_j;
        } else {
            float d_sq = s_plan.distance * s_plan.distance;
            v_peak = powf(d_sq * jmax_dps3 * 0.25f, 1.0f / 3.0f);
            a_peak = sqrtf(v_peak * jmax_dps3);
            T_j    = a_peak / jmax_dps3;
            T_a    = 0.0f;
        }
    }

    s_plan.T_j = T_j; s_plan.T_a = T_a; s_plan.T_v = T_v;
    s_plan.a_peak_dps2 = a_peak;
    s_plan.v_peak_dps  = v_peak;

    s_plan.t1 = T_j;
    s_plan.t2 = s_plan.t1 + T_a;
    s_plan.t3 = s_plan.t2 + T_j;
    s_plan.t4 = s_plan.t3 + T_v;
    s_plan.t5 = s_plan.t4 + T_j;
    s_plan.t6 = s_plan.t5 + T_a;
    s_plan.t7 = s_plan.t6 + T_j;

    s_plan.v1_dps = 0.5f * jmax_dps3 * T_j * T_j;
    s_plan.v2_dps = s_plan.v1_dps + a_peak * T_a;
    s_plan.p1_deg = jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p2_deg = s_plan.p1_deg + s_plan.v1_dps * T_a + 0.5f * a_peak * T_a * T_a;
    s_plan.p3_deg = s_plan.p2_deg + s_plan.v2_dps * T_j + 0.5f * a_peak * T_j * T_j
                                    - jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p4_deg = s_plan.p3_deg + v_peak * T_v;
    s_plan.p5_deg = s_plan.p4_deg + v_peak * T_j - jmax_dps3 * T_j * T_j * T_j / 6.0f;
    s_plan.p6_deg = s_plan.p5_deg + s_plan.v2_dps * T_a - 0.5f * a_peak * T_a * T_a;
    s_plan.active = true;
}

/* ============================================================================
 * Evaluator — piecewise kinematics at time t
 * ============================================================================ */
static void scurve_eval(float t, float *p_rel, float *v_dps, float *a_dps2)
{
    float T_j  = s_plan.T_j;
    float a_pk = s_plan.a_peak_dps2;
    float v_pk = s_plan.v_peak_dps;
    float jmax = s_plan.jmax_dps3;

    if (t <= 0.0f)      { *p_rel = 0.0f;            *v_dps = 0.0f; *a_dps2 = 0.0f; return; }
    if (t >= s_plan.t7) { *p_rel = s_plan.distance; *v_dps = 0.0f; *a_dps2 = 0.0f; return; }

    if (t < s_plan.t1) {
        *a_dps2 =        jmax * t;
        *v_dps  = 0.5f * jmax * t * t;
        *p_rel  =        jmax * t * t * t / 6.0f;
    } else if (t < s_plan.t2) {
        float dt = t - s_plan.t1;
        *a_dps2 = a_pk;
        *v_dps  = s_plan.v1_dps + a_pk * dt;
        *p_rel  = s_plan.p1_deg + s_plan.v1_dps * dt + 0.5f * a_pk * dt * dt;
    } else if (t < s_plan.t3) {
        float dt = t - s_plan.t2;
        *a_dps2 = a_pk - jmax * dt;
        *v_dps  = s_plan.v2_dps + a_pk * dt - 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p2_deg + s_plan.v2_dps * dt + 0.5f * a_pk * dt * dt
                                  - jmax * dt * dt * dt / 6.0f;
    } else if (t < s_plan.t4) {
        float dt = t - s_plan.t3;
        *a_dps2 = 0.0f;
        *v_dps  = v_pk;
        *p_rel  = s_plan.p3_deg + v_pk * dt;
    } else if (t < s_plan.t5) {
        float dt = t - s_plan.t4;
        *a_dps2 = -jmax * dt;
        *v_dps  =  v_pk - 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p4_deg + v_pk * dt - jmax * dt * dt * dt / 6.0f;
    } else if (t < s_plan.t6) {
        float dt = t - s_plan.t5;
        *a_dps2 = -a_pk;
        *v_dps  = s_plan.v2_dps - a_pk * dt;
        *p_rel  = s_plan.p5_deg + s_plan.v2_dps * dt - 0.5f * a_pk * dt * dt;
    } else {
        float dt = t - s_plan.t6;
        *a_dps2 = -a_pk + jmax * dt;
        *v_dps  = s_plan.v1_dps - a_pk * dt + 0.5f * jmax * dt * dt;
        *p_rel  = s_plan.p6_deg + s_plan.v1_dps * dt - 0.5f * a_pk * dt * dt
                                  + jmax * dt * dt * dt / 6.0f;
    }
}

/* ============================================================================
 * Public — called every 100 Hz tick from Motor_ControlLoop()
 * ============================================================================ */
void Trajectory_Generator_Update(void)
{
    float vmax_dps  = motion_config.max_velocity     * RPM_TO_DPS;
    float amax_dps2 = motion_config.max_acceleration * RPM_TO_DPS;
    float jmax_dps3 = tuning.max_jerk                * RPM_TO_DPS;

    if (!s_plan.active || trajectory.target_pos != s_plan.pos_target_deg) {
        scurve_plan(trajectory.current_setpoint_pos,
                    trajectory.target_pos,
                    vmax_dps, amax_dps2, jmax_dps3);
    }

    if (!s_plan.active) {
        trajectory.current_setpoint_pos   = trajectory.target_pos;
        trajectory.current_setpoint_vel   = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;
        return;
    }

    s_plan.t_elapsed += CTRL_DT;

    float p_rel, v_dps, a_dps2;
    scurve_eval(s_plan.t_elapsed, &p_rel, &v_dps, &a_dps2);

    trajectory.current_setpoint_pos   = s_plan.pos_start_deg + (float)s_plan.direction * p_rel;
    trajectory.current_setpoint_vel   = (float)s_plan.direction * v_dps * DPS_TO_RPM;
    trajectory.current_setpoint_accel = (float)s_plan.direction * a_dps2 * DPS_TO_RPM;

    if (s_plan.t_elapsed >= s_plan.t7) {
        s_plan.active = false;
        trajectory.current_setpoint_pos   = s_plan.pos_target_deg;
        trajectory.current_setpoint_vel   = 0.0f;
        trajectory.current_setpoint_accel = 0.0f;
        pid_position.integral = 0.0f;
        pid_position.d_filt   = 0.0f;
    }
}
