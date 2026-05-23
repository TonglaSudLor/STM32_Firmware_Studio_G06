/**
 * @file current_sensor.h
 * @brief WCS1800 Hall Current Sensor driver (35A, analog output on PA0).
 *
 * Hardware:
 *   WCS1800 VCC → +5V
 *   WCS1800 GND → GND
 *   WCS1800 OUT → R1(1kΩ) → PA0 (ADC1_IN1)
 *                             │
 *                          R2(1.8kΩ)
 *                             │
 *                            GND
 *
 * Sensor model:   V_out = 2.5V + I × 0.066 V/A   (at VCC=5V, zero=2.5V)
 * Divider ratio:  k = 1.8/(1+1.8) = 0.6429
 * ADC input:      V_adc = V_out × k   → max 3.09V at +35A  (safe for 3.3V ADC)
 */

#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

/* ---- Sensor constants (do not change unless hardware changes) ------------ */
#define CS_VREF             3.3f      /* STM32 ADC reference voltage (V) */
#define CS_ADC_COUNTS       4096.0f   /* 12-bit full-scale (after oversampling shift) */
#define CS_VOFFSET          2.5f      /* WCS1800 zero-current output voltage (V) */
#define CS_SENSITIVITY      0.066f    /* WCS1800 sensitivity (V/A) */
#define CS_DIV_RATIO        0.6429f   /* Voltage divider: R2/(R1+R2) = 1.8/2.8 */

/* ---- Tunable filter ------------------------------------------------------- */
/* EMA weight for the software low-pass filter.
 * Lower α = smoother but slower response.
 * 0.2 gives a ~4-sample time constant (40 ms at 100 Hz). */
#define CS_EMA_ALPHA        0.2f

/* ---- Public API ----------------------------------------------------------- */

/**
 * @brief Initialise ADC and run self-calibration. Call once from main()
 *        after MX_ADC1_Init().
 * @param hadc  Pointer to the configured ADC1 handle.
 */
void CurrentSensor_Init(ADC_HandleTypeDef *hadc);

/**
 * @brief Trigger one ADC conversion, apply voltage-divider inverse and
 *        WCS1800 transfer function, update EMA filter.
 *        Call at 100 Hz from HW_RefreshIO().
 * @return Filtered motor current in Amperes (negative = reverse direction).
 */
float CurrentSensor_Sample(void);

/**
 * @brief Return last filtered current reading without triggering a new conversion.
 */
float CurrentSensor_GetAmps(void);

/**
 * @brief Return last raw ADC value (after hardware oversampling shift).
 *        Useful for calibration and Live Expressions debugging.
 */
uint32_t CurrentSensor_GetRaw(void);

/**
 * @brief Return the runtime-calibrated zero-current voltage (V_sensor at 0 A).
 *        Should be close to CS_VOFFSET (2.5 V). Use in Live Expressions to
 *        verify calibration: expected range 2.3–2.7 V.
 */
float CurrentSensor_GetVzero(void);

#endif /* CURRENT_SENSOR_H */
