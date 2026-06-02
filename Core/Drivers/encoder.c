/**
 * @file encoder.c
 * @brief Quadrature encoder math — pure C99, zero HAL dependency.
 *
 * Handles 16-bit TIM3 counter rollover, EMI noise rejection,
 * phase inversion, and IIR-filtered RPM.
 *
 * Call Encoder_UpdateWithRaw() from encoder_hal.c (firmware)
 * or directly from unit tests on a host PC.
 */

#include "encoder.h"
#include <string.h>

void Encoder_Reset(Encoder_State_t *enc)
{
    enc->absolute_counts = 0;
    enc->count_prev      = 0;
    enc->position_deg    = 0.0f;
    enc->filtered_rpm    = 0.0f;
}

void Encoder_UpdateWithRaw(Encoder_State_t *enc, uint16_t raw, float dt)
{
    /* Delta in 16-bit space (handles timer rollover 0→65535 and back) */
    int32_t delta = (int32_t)raw - (int32_t)(uint16_t)enc->count_prev;
    if (delta >  32767) delta -= 65536;
    if (delta < -32768) delta += 65536;

    /* Reject EMI spikes: >2000 counts per 10ms → ~1500 RPM, physically impossible */
    if (delta > ENCODER_NOISE_THRESHOLD || delta < -ENCODER_NOISE_THRESHOLD)
        delta = 0;

#if ENCODER_PHASE_INVERTED
    delta = -delta;
#endif

    enc->count_prev       = raw;
    enc->absolute_counts += delta;

    const float cpr = (float)ENCODER_COUNTS_PER_REV;
    enc->position_deg = ((float)enc->absolute_counts / cpr) * 360.0f;

    float instant_rpm = ((float)delta / cpr / dt) * 60.0f;
    enc->filtered_rpm = ENCODER_RPM_ALPHA * instant_rpm
                      + (1.0f - ENCODER_RPM_ALPHA) * enc->filtered_rpm;
}

float Encoder_GetPositionDeg(const Encoder_State_t *enc) { return enc->position_deg; }
float Encoder_GetRPM(const Encoder_State_t *enc)         { return enc->filtered_rpm; }
