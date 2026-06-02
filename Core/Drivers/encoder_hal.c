/**
 * @file encoder_hal.c
 * @brief HAL adapter for encoder — reads TIM3 counter, calls encoder.c math.
 *
 * This is the ONLY file that may touch htim3.
 * Not compiled in unit tests (uses real stm32g4xx_hal.h).
 */

#include "encoder.h"
#include "stm32g4xx_hal.h"

extern TIM_HandleTypeDef htim3;

#define CONTROL_DT  0.01f   /* 100 Hz ISR period */

void Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

void Encoder_Update(Encoder_State_t *enc)
{
    uint16_t raw = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    Encoder_UpdateWithRaw(enc, raw, CONTROL_DT);
}
