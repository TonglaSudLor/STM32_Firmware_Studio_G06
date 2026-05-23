/**
 * @file current_sensor.c
 * @brief WCS1800 Hall Current Sensor driver implementation.
 *
 * Conversion pipeline (one Sample() call):
 *   ADC raw → V_adc → V_sensor (undo divider) → I_raw (WCS1800 equation) → EMA filter
 *
 * All intermediate values in SI units (Volts, Amperes).
 */

#include "current_sensor.h"

static ADC_HandleTypeDef *_hadc       = NULL;
static float              _amps_filt  = 0.0f;
static uint32_t           _raw        = 0;
static float              _v_zero     = CS_VOFFSET;  /* runtime-calibrated zero-current voltage */

void CurrentSensor_Init(ADC_HandleTypeDef *hadc)
{
    _hadc = hadc;
    HAL_ADCEx_Calibration_Start(hadc, ADC_SINGLE_ENDED);

    /* Auto-calibrate zero point: average 64 samples while motor is off at startup.
     * Eliminates offset error from sensor VCC tolerance or resistor mismatch. */
    uint32_t sum = 0;
    for (int i = 0; i < 64; i++) {
        HAL_ADC_Start(_hadc);
        if (HAL_ADC_PollForConversion(_hadc, 20) == HAL_OK)
            sum += HAL_ADC_GetValue(_hadc);
        HAL_ADC_Stop(_hadc);
    }
    float raw_avg = (float)sum / 64.0f;
    _v_zero = (raw_avg * (CS_VREF / CS_ADC_COUNTS)) / CS_DIV_RATIO;
}

float CurrentSensor_Sample(void)
{
    if (_hadc == NULL)
        return 0.0f;

    HAL_ADC_Start(_hadc);

    /* Timeout accounts for 16× oversampling: 16 × 653 cycles @ 42.5 MHz ≈ 246 µs.
     * 20 ms is a safe upper bound; if it exceeds that, hardware is broken. */
    if (HAL_ADC_PollForConversion(_hadc, 20) == HAL_OK)
    {
        _raw = HAL_ADC_GetValue(_hadc);   /* 12-bit after hardware right-shift */

        /* Step 1: ADC counts → voltage at PA0 pin */
        float v_adc = (float)_raw * (CS_VREF / CS_ADC_COUNTS);

        /* Step 2: undo the voltage divider to recover the sensor output voltage */
        float v_sensor = v_adc / CS_DIV_RATIO;

        /* Step 3: WCS1800 linear transfer function (using runtime-calibrated zero) */
        float i_raw = (v_sensor - _v_zero) / CS_SENSITIVITY;

        /* Step 4: EMA low-pass filter to suppress PWM switching noise */
        _amps_filt = CS_EMA_ALPHA * i_raw + (1.0f - CS_EMA_ALPHA) * _amps_filt;
    }

    HAL_ADC_Stop(_hadc);
    return _amps_filt;
}

float    CurrentSensor_GetAmps(void)  { return _amps_filt; }
uint32_t CurrentSensor_GetRaw(void)   { return _raw; }
float    CurrentSensor_GetVzero(void) { return _v_zero; }
