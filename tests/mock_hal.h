/**
 * @file mock_hal.h
 * @brief Stub types to replace stm32g4xx_hal.h when compiling on a host PC.
 *
 * Include this BEFORE any firmware header that pulls in HAL types,
 * but ONLY in test builds (gcc on Mac/Linux). Never include in firmware.
 */

#ifndef MOCK_HAL_H
#define MOCK_HAL_H

#include <stdint.h>
#include <stdbool.h>

/* STM32 HAL handle stubs — opaque pointers are enough for unit tests */
typedef void* UART_HandleTypeDef;
typedef void* TIM_HandleTypeDef;
typedef void* ADC_HandleTypeDef;

/* HAL status (not used by modbus_frame but pulled in transitively) */
typedef enum { HAL_OK = 0, HAL_ERROR = 1, HAL_BUSY = 2, HAL_TIMEOUT = 3 } HAL_StatusTypeDef;

#define HAL_GetTick()  ((uint32_t)0)
#define __disable_irq()
#define __enable_irq()
#define __get_PRIMASK() ((uint32_t)0)
#define __set_PRIMASK(x) ((void)(x))

#endif /* MOCK_HAL_H */
