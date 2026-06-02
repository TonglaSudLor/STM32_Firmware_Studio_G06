/**
 * @file safety.c
 * @brief Fault Manager — centralised E-Stop and fault code handling.
 *
 * Holds the single authoritative fault bitmask. All RMW operations are guarded
 * by PRIMASK save/disable/restore so ISR and thread context can both call
 * Safety_SetFault/ClearFault safely without losing bits.
 */

#include "safety.h"
#include "stm32g4xx_hal.h"

static volatile uint32_t s_faults = 0u;

void Safety_Init(void)
{
    s_faults = 0u;
    /* Power-on latch: motor blocked until self-test (DIAG) passes */
    Safety_SetFault(FAULT_STARTUP_ESTOP);
}

void Safety_SetFault(FaultCode_t bits)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_faults |= (uint32_t)bits;
    __set_PRIMASK(pm);
}

void Safety_ClearFault(FaultCode_t bits)
{
    uint32_t pm = __get_PRIMASK();
    __disable_irq();
    s_faults &= ~(uint32_t)bits;
    __set_PRIMASK(pm);
}

FaultCode_t Safety_GetFaults(void)
{
    return (FaultCode_t)s_faults;
}

bool Safety_IsEStop(void)
{
    return s_faults != 0u;
}

void Safety_CheckAll(void)
{
    /* TODO: migrate stall, over-rotation, joystick watchdog checks
     * from motor_controller.c Motor_ControlLoop() into here.
     * Pass motor state via a Safety_Inputs_t parameter struct. */
}
