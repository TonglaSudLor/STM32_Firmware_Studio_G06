/**
 * @file safety.h
 * @brief Fault Manager — centralised E-Stop and fault code handling.
 *
 * TODO: Extract from Core/Inc/motor_controller.h + Core/Src/motor_controller.c
 *   - Motor_FaultCode_t enum
 *   - FAULT_SET / FAULT_CLR macros (PRIMASK-guarded RMW)
 *   - fault_code volatile variable
 *   - emergency_stop volatile variable
 *   - All safety check logic in Motor_ControlLoop():
 *       stall detection, encoder check, over-rotation, overcurrent
 *
 * When done: add Core/App/safety.c
 *   Rule: all fault bit operations must go through Safety_SetFault/ClearFault.
 *         Never write fault_code directly from outside this module.
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
    FAULT_ESTOP_MODBUS      = 0x100
} FaultCode_t;

void       Safety_Init(void);
void       Safety_SetFault(FaultCode_t bits);    /* ISR-safe */
void       Safety_ClearFault(FaultCode_t bits);  /* ISR-safe */
FaultCode_t Safety_GetFaults(void);
bool       Safety_IsEStop(void);
void       Safety_CheckAll(void);                /* call at 100 Hz */
