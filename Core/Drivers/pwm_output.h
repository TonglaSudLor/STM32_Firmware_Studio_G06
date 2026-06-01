/**
 * @file pwm_output.h
 * @brief H-Bridge PWM Output Driver — HAL wrapper for TIM1.
 *
 * TODO: Extract from Core/Src/motor_controller.c
 *   - PWM set logic at the bottom of Motor_ControlLoop()
 *   - Direction pin handling
 *   - Voltage → duty cycle conversion
 *
 * Hardware: TIM1 (CH1/CH1N), 24V supply
 *   duty = voltage / SUPPLY_VOLTAGE * 100%
 *
 * When done: add Core/Drivers/pwm_output.c
 *   Rule: only this file may touch htim1 PWM registers directly.
 */

#pragma once

void PWM_Init(void);
void PWM_SetDuty(float percent);   /* -100.0 to +100.0, sign = direction */
void PWM_Disable(void);            /* coast: both channels low */
void PWM_Brake(void);              /* active brake: both channels high */
