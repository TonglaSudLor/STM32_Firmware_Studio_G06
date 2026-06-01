/**
 * @file scurve.h
 * @brief S-Curve Trajectory Generator — pure math, zero HAL dependency.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - Trajectory_State_t struct (already in motor_controller.h)
 *   - S-curve step logic inside Motor_ControlLoop()
 *   - Jerk-limited 7-segment profile
 *
 * Params used (from params.h):
 *   MOVE_SPEED_COARSE  → v_max
 *   DEFAULT_MAX_ACCEL  → a_max
 *   DEFAULT_MAX_JERK   → j_max
 *
 * When done: add Core/Lib/scurve.c + tests/test_scurve.c
 */

#pragma once
#include <stdbool.h>

typedef struct {
    float v_max;
    float a_max;
    float j_max;
    float target_pos;
    float target_vel;
    float pos;
    float vel;
    float accel;
} SCurve_t;

void  SCurve_Init(SCurve_t *sc, float v_max, float a_max, float j_max);
void  SCurve_SetTarget(SCurve_t *sc, float pos, float vel);
void  SCurve_Step(SCurve_t *sc, float dt);
float SCurve_GetPos(SCurve_t *sc);
float SCurve_GetVel(SCurve_t *sc);
float SCurve_GetAccel(SCurve_t *sc);
bool  SCurve_IsDone(SCurve_t *sc);
