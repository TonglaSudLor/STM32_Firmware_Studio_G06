/**
 * @file input_shaper.h
 * @brief ZVD (Zero-Vibration-Derivative) Input Shaper — Pure C99, Zero HAL.
 */

#ifndef INPUT_SHAPER_H
#define INPUT_SHAPER_H

#include <stdint.h>
#include <stdbool.h>

#define SHAPER_BUF_SIZE 256

/**
 * @brief ZVD Shaper state
 */
typedef struct {
    float buf[SHAPER_BUF_SIZE];
    uint32_t head;
    uint32_t N; // Delay in ticks
    
    // Coefficients
    float A1, A2, A3;
    
    bool enabled;
} Shaper_t;

/**
 * @brief Initialize shaper
 */
void Shaper_Init(Shaper_t *s, float omega_n, float zeta, float sample_freq_hz);

/**
 * @brief Update coefficients (call when omega_n or zeta changes)
 */
void Shaper_Update(Shaper_t *s, float omega_n, float zeta, float sample_freq_hz);

/**
 * @brief Push a new raw setpoint and get the shaped value
 */
float Shaper_Process(Shaper_t *s, float input);

/**
 * @brief Reset/Flush buffer with a specific value (to prevent jumps)
 */
void Shaper_Flush(Shaper_t *s, float val);

#endif /* INPUT_SHAPER_H */
