/**
 * @file scurve.h
 * @brief 7-segment S-Curve Trajectory Generator — Pure C99, Zero HAL.
 */

#ifndef SCURVE_H
#define SCURVE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief S-Curve planner state
 */
typedef struct {
    // Limits
    float v_max;
    float a_max;
    float j_max;

    // Targets
    float pos_start;
    float pos_target;
    float distance;
    int8_t direction;

    // Timings (phases)
    float t_elapsed;
    float T_j, T_a, T_v;
    float t1, t2, t3, t4, t5, t6, t7;

    // Phase anchor values
    float a_peak;
    float v_peak;
    float v1, v2;
    float p1, p2, p3, p4, p5, p6;

    bool active;
} SCurve_t;

/**
 * @brief Initialize S-curve structure with motion limits
 */
void SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max);

/**
 * @brief Start a new move to target position from current position
 */
void SCurve_Plan(SCurve_t *sc, float current_pos, float target_pos);

/**
 * @brief Advance trajectory by one time step
 */
void SCurve_Step(SCurve_t *sc, float dt, float *p_out, float *v_out, float *a_out);

/**
 * @brief Force-stop the trajectory at current output
 */
void SCurve_Stop(SCurve_t *sc);

#endif /* SCURVE_H */
