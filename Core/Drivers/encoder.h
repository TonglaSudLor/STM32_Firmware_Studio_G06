/**
 * @file encoder.h
 * @brief Quadrature Encoder Driver — HAL wrapper for TIM3.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - TIM3 read + absolute count tracking (Encoder_Data_t)
 *   - encoder.absolute_counts update logic in Motor_ControlLoop()
 *   - encoder.current_position_deg calculation
 *   - encoder.filtered_rpm calculation
 *
 * Hardware: TIM3, 2048 PPR, x4 = 8192 counts/rev
 *
 * When done: add Core/Drivers/encoder.c
 *   Rule: only this file may touch htim3 directly.
 */

#pragma once
#include <stdint.h>

typedef struct {
    int32_t absolute_counts;
    float   position_deg;
    float   filtered_rpm;
} Encoder_State_t;

void  Encoder_Init(void);
void  Encoder_Update(Encoder_State_t *enc);   /* call at 100 Hz */
void  Encoder_Reset(Encoder_State_t *enc);
float Encoder_GetPositionDeg(const Encoder_State_t *enc);
float Encoder_GetRPM(const Encoder_State_t *enc);
