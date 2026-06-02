/**
 * @file safety.h
 * @brief Fault Manager — centralised E-Stop and fault code handling.
 *
 * All fault bit operations MUST go through Safety_SetFault/ClearFault.
 * These are ISR-safe (PRIMASK-guarded). Never write fault_code directly.
 *
 * PRIMASK intrinsics (__get_PRIMASK, __disable_irq, __set_PRIMASK) must be
 * provided by the build:
 *   - Firmware  : stm32g4xx_hal.h (included upstream)
 *   - Unit tests : -include mock_hal.h (see tests/Makefile)
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    FAULT_NONE              = 0x000,
    FAULT_MOTOR_STALLED     = 0x001,
    FAULT_ENCODER_ERROR     = 0x002,
    FAULT_JOYSTICK_LOST     = 0x004,
    FAULT_OVER_ROTATION     = 0x008,
    FAULT_ESTOP_PHYSICAL    = 0x010,
    FAULT_PROX_LOST         = 0x020,
    FAULT_ESTOP_JOYSTICK    = 0x040,
    FAULT_ESTOP_DASHBOARD   = 0x080,
    FAULT_ESTOP_MODBUS      = 0x100,
    FAULT_STARTUP_ESTOP     = 0x200,   /* power-on latch — cleared by self-test */
    FAULT_OVERCURRENT       = 0x400,
} FaultCode_t;

void        Safety_Init(void);
void        Safety_SetFault(FaultCode_t bits);   /* ISR-safe (PRIMASK) */
void        Safety_ClearFault(FaultCode_t bits); /* ISR-safe (PRIMASK) */
FaultCode_t Safety_GetFaults(void);
bool        Safety_IsEStop(void);                /* true if ANY fault is active */

/* TODO: implement when extracting stall/over-rot/joystick checks from motor_controller.c */
void        Safety_CheckAll(void);               /* call at 100 Hz */
