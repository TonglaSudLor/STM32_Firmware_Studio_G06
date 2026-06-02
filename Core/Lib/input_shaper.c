/**
 * @file input_shaper.c
 * @brief ZVD (Zero-Vibration-Derivative) Input Shaper — Implementation.
 */

#include "input_shaper.h"
#include <math.h>
#include <string.h>

void Shaper_Init(Shaper_t *s, float omega_n, float zeta, float sample_freq_hz)
{
    s->enabled = false;
    s->head = 0;
    Shaper_Update(s, omega_n, zeta, sample_freq_hz);
    Shaper_Flush(s, 0.0f);
}

void Shaper_Update(Shaper_t *s, float omega_n, float zeta, float sample_freq_hz)
{
    if (omega_n <= 0.0f) {
        s->N = 0;
        s->A1 = 1.0f; s->A2 = 0.0f; s->A3 = 0.0f;
        return;
    }

    float wd = omega_n * sqrtf(1.0f - zeta * zeta);
    float T_d = 3.14159265f / wd;
    
    uint32_t n = (uint32_t)roundf(T_d * sample_freq_hz);
    if (n == 0) n = 1;
    if (2 * n >= SHAPER_BUF_SIZE) n = (SHAPER_BUF_SIZE - 1) / 2;
    s->N = n;

    float K = expf(-zeta * 3.14159265f / sqrtf(1.0f - zeta * zeta));
    float denom = (1.0f + K) * (1.0f + K);
    
    s->A1 = 1.0f / denom;
    s->A2 = 2.0f * K / denom;
    s->A3 = K * K / denom;
}

float Shaper_Process(Shaper_t *s, float input)
{
    s->buf[s->head] = input;
    
    float output;
    if (s->enabled && s->N > 0) {
        uint32_t i1 = (s->head + SHAPER_BUF_SIZE - s->N) % SHAPER_BUF_SIZE;
        uint32_t i2 = (s->head + SHAPER_BUF_SIZE - 2 * s->N) % SHAPER_BUF_SIZE;
        
        output = s->A1 * s->buf[s->head] + 
                 s->A2 * s->buf[i1] + 
                 s->A3 * s->buf[i2];
    } else {
        output = input;
    }
    
    s->head = (s->head + 1) % SHAPER_BUF_SIZE;
    return output;
}

void Shaper_Flush(Shaper_t *s, float val)
{
    for (int i = 0; i < SHAPER_BUF_SIZE; i++) {
        s->buf[i] = val;
    }
}
