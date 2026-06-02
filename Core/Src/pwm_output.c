/**
 * @file pwm_output.c
 * @brief H-Bridge PWM driver — HAL wrapper for TIM1 + direction GPIO.
 *
 * Hardware:
 *   PWM    : TIM1 CH1 (auto-reload from CubeMX)
 *   DIR    : Motor_Direction_GPIO_Port / Motor_Direction_Pin (PA9, defined in main.h)
 *   Logic  : DIR=RESET → forward,  DIR=SET → reverse  (active-low forward convention)
 *
 * This is the ONLY file that may touch htim1 CCR directly.
 */

#include "pwm_output.h"
#include "main.h"
#include "stm32g4xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim1;

void PWM_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    PWM_Disable();
}

void PWM_SetDuty(float percent)
{
    /* Clamp to valid range */
    if (percent >  100.0f) percent =  100.0f;
    if (percent < -100.0f) percent = -100.0f;

    bool forward = (percent >= 0.0f);
    HAL_GPIO_WritePin(Motor_Direction_GPIO_Port, Motor_Direction_Pin,
                      forward ? GPIO_PIN_RESET : GPIO_PIN_SET);

    uint32_t arr       = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t pwm_value = (uint32_t)((arr * fabsf(percent)) / 100.0f);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm_value);
}

void PWM_Disable(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
}

void PWM_Brake(void)
{
    /* Active brake: full duty with direction held — causes short-circuit braking.
     * Works with DRV8833 in PHASE/ENABLE mode. If H-bridge differs, use PWM_Disable(). */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, __HAL_TIM_GET_AUTORELOAD(&htim1));
}
