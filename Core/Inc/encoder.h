/**
 * @file encoder.h
 * @brief Quadrature Encoder Driver — TIM3 wrapper, 2048 PPR × 4 = 8192 counts/rev.
 *
 * Layered design:
 *   encoder.c     — pure math (no HAL), unit-testable on PC
 *   encoder_hal.c — thin HAL adapter (reads htim3), firmware only
 *
 * Rule: only encoder_hal.c may touch htim3.
 */

#pragma once
#include <stdint.h>

/* Hardware constants — override via -D flags if hardware changes */
#ifndef ENCODER_PPR
#define ENCODER_PPR       2048u   /* pulses per revolution */
#endif
#ifndef ENCODER_GEAR
#define ENCODER_GEAR      1u      /* gear ratio (integer) */
#endif
#ifndef ENCODER_PHASE_INVERTED
#define ENCODER_PHASE_INVERTED  1  /* set 0 in test builds */
#endif

#define ENCODER_COUNTS_PER_REV  ((uint32_t)(ENCODER_PPR * 4u * ENCODER_GEAR))
#define ENCODER_NOISE_THRESHOLD 2000   /* reject |delta| > this (EMI spike) */
#define ENCODER_RPM_ALPHA       0.15f  /* IIR weight for new RPM sample */

typedef struct {
    int32_t  absolute_counts;
    uint32_t count_prev;       /* last raw TIM3 count (for delta) */
    float    position_deg;
    float    filtered_rpm;
} Encoder_State_t;

/* ---- Pure functions (encoder.c) — unit-testable ---- */
void  Encoder_UpdateWithRaw(Encoder_State_t *enc, uint16_t raw_count, float dt);
void  Encoder_Reset(Encoder_State_t *enc);
float Encoder_GetPositionDeg(const Encoder_State_t *enc);
float Encoder_GetRPM(const Encoder_State_t *enc);

/* ---- HAL-dependent (encoder_hal.c) — firmware only ---- */
void  Encoder_Init(void);
void  Encoder_Update(Encoder_State_t *enc);  /* reads htim3, calls UpdateWithRaw at 100 Hz */
