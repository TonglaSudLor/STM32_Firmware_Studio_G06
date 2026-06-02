/**
 * @file scurve.c
 * @brief 7-segment S-Curve Trajectory Generator — Implementation.
 */

#include "scurve.h"
#include <math.h>

void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max)
{
    sc->v_max = v_max;
    sc->a_max = a_max;
    sc->j_max = j_max;
    sc->active = false;
}

void SCurve_Plan(SCurve_t *sc, float current_pos, float target_pos)
{
    sc->t_elapsed = 0.0f;
    sc->pos_start = current_pos;
    sc->pos_target = target_pos;
    sc->distance = fabsf(target_pos - current_pos);
    sc->direction = (target_pos >= current_pos) ? 1 : -1;

    if (sc->distance < 0.0001f || sc->v_max <= 0.0f || sc->a_max <= 0.0f || sc->j_max <= 0.0f) {
        sc->active = false;
        return;
    }

    // 1. Determine if max acceleration is reached (T_a > 0)
    float T_j_full = sc->a_max / sc->j_max;
    float v_at_accel_limit = sc->a_max * T_j_full; // v = a^2 / j

    float T_j, T_a, a_peak, v_peak;

    if (sc->v_max >= v_at_accel_limit) {
        T_j = T_j_full;
        a_peak = sc->a_max;
        T_a = sc->v_max / sc->a_max - T_j;
        v_peak = sc->v_max;
    } else {
        a_peak = sqrtf(sc->v_max * sc->j_max);
        T_j = a_peak / sc->j_max;
        T_a = 0.0f;
        v_peak = sc->v_max;
    }

    // 2. Determine if max velocity is reached (T_v > 0)
    float d_accel = v_peak * (2.0f * T_j + T_a) * 0.5f;
    float T_v;

    if (2.0f * d_accel <= sc->distance) {
        T_v = (sc->distance - 2.0f * d_accel) / v_peak;
    } else {
        // Distance is too short to reach v_max
        T_v = 0.0f;
        // Recompute v_peak based on distance
        // Try to reach a_max first
        float aTj = sc->a_max * T_j_full;
        float disc = aTj * aTj + 4.0f * sc->a_max * sc->distance;
        float v_try = (-aTj + sqrtf(disc)) * 0.5f;

        if (v_try >= v_at_accel_limit) {
            v_peak = v_try;
            a_peak = sc->a_max;
            T_j = T_j_full;
            T_a = v_peak / sc->a_max - T_j;
        } else {
            // Can't even reach a_max
            float d_sq = sc->distance * sc->distance;
            v_peak = powf(d_sq * sc->j_max * 0.25f, 1.0f / 3.0f);
            a_peak = sqrtf(v_peak * sc->j_max);
            T_j = a_peak / sc->j_max;
            T_a = 0.0f;
        }
    }

    sc->T_j = T_j; sc->T_a = T_a; sc->T_v = T_v;
    sc->a_peak = a_peak; sc->v_peak = v_peak;

    // Phases
    sc->t1 = T_j;
    sc->t2 = sc->t1 + T_a;
    sc->t3 = sc->t2 + T_j;
    sc->t4 = sc->t3 + T_v;
    sc->t5 = sc->t4 + T_j;
    sc->t6 = sc->t5 + T_a;
    sc->t7 = sc->t6 + T_j;

    // Anchor values for each phase
    sc->v1 = 0.5f * sc->j_max * T_j * T_j;
    sc->v2 = sc->v1 + a_peak * T_a;
    sc->p1 = sc->j_max * T_j * T_j * T_j / 6.0f;
    sc->p2 = sc->p1 + sc->v1 * T_a + 0.5f * a_peak * T_a * T_a;
    sc->p3 = sc->p2 + sc->v2 * T_j + 0.5f * a_peak * T_j * T_j - sc->j_max * T_j * T_j * T_j / 6.0f;
    sc->p4 = sc->p3 + v_peak * T_v;
    sc->p5 = sc->p4 + v_peak * T_j - sc->j_max * T_j * T_j * T_j / 6.0f;
    sc->p6 = sc->p5 + sc->v2 * T_a - 0.5f * a_peak * T_a * T_a;

    sc->active = true;
}

void SCurve_Step(SCurve_t *sc, float dt, float *p_out, float *v_out, float *a_out)
{
    if (!sc->active) {
        *p_out = sc->pos_target;
        *v_out = 0.0f;
        *a_out = 0.0f;
        return;
    }

    sc->t_elapsed += dt;
    float t = sc->t_elapsed;
    float p_rel = 0.0f, v_rel = 0.0f, a_rel = 0.0f;

    if (t >= sc->t7) {
        sc->active = false;
        p_rel = sc->distance;
    } else {
        if (t < sc->t1) {
            a_rel = sc->j_max * t;
            v_rel = 0.5f * sc->j_max * t * t;
            p_rel = sc->j_max * t * t * t / 6.0f;
        } else if (t < sc->t2) {
            float dt_p = t - sc->t1;
            a_rel = sc->a_peak;
            v_rel = sc->v1 + sc->a_peak * dt_p;
            p_rel = sc->p1 + sc->v1 * dt_p + 0.5f * sc->a_peak * dt_p * dt_p;
        } else if (t < sc->t3) {
            float dt_p = t - sc->t2;
            a_rel = sc->a_peak - sc->j_max * dt_p;
            v_rel = sc->v2 + sc->a_peak * dt_p - 0.5f * sc->j_max * dt_p * dt_p;
            p_rel = sc->p2 + sc->v2 * dt_p + 0.5f * sc->a_peak * dt_p * dt_p - sc->j_max * dt_p * dt_p * dt_p / 6.0f;
        } else if (t < sc->t4) {
            float dt_p = t - sc->t3;
            a_rel = 0.0f;
            v_rel = sc->v_peak;
            p_rel = sc->p3 + sc->v_peak * dt_p;
        } else if (t < sc->t5) {
            float dt_p = t - sc->t4;
            a_rel = -sc->j_max * dt_p;
            v_rel = sc->v_peak - 0.5f * sc->j_max * dt_p * dt_p;
            p_rel = sc->p4 + sc->v_peak * dt_p - sc->j_max * dt_p * dt_p * dt_p / 6.0f;
        } else if (t < sc->t6) {
            float dt_p = t - sc->t5;
            a_rel = -sc->a_peak;
            v_rel = sc->v2 - sc->a_peak * dt_p;
            p_rel = sc->p5 + sc->v2 * dt_p - 0.5f * sc->a_peak * dt_p * dt_p;
        } else {
            float dt_p = t - sc->t6;
            a_rel = -sc->a_peak + sc->j_max * dt_p;
            v_rel = sc->v1 - sc->a_peak * dt_p + 0.5f * sc->j_max * dt_p * dt_p;
            p_rel = sc->p6 + sc->v1 * dt_p - 0.5f * sc->a_peak * dt_p * dt_p + sc->j_max * dt_p * dt_p * dt_p / 6.0f;
        }
    }

    *p_out = sc->pos_start + (float)sc->direction * p_rel;
    *v_out = (float)sc->direction * v_rel;
    *a_out = (float)sc->direction * a_rel;
}

void SCurve_Stop(SCurve_t *sc)
{
    sc->active = false;
}
