/**
 * @file modbus_bridge.h
 * @brief Application-level Modbus register mapping and logic
 * 
 * @author Gemini CLI (Refactored)
 * @date May 2026
 */

#ifndef MODBUS_BRIDGE_H
#define MODBUS_BRIDGE_H

#include "main.h"
#include <stdbool.h>

/**
 * @brief Initialize the Modbus bridge and register map
 */
void ModbusBridge_Init(void);

/**
 * @brief Process Modbus protocol state machine
 */
void ModbusBridge_Process(void);

/**
 * @brief Sync internal motor variables to Modbus registers
 * Should be called periodically (e.g., in the 100Hz loop)
 */
void ModbusBridge_UpdateRegisters(void);

/**
 * @brief UART Rx Callback for Modbus data
 * @param data Received byte
 */
void ModbusBridge_RxCallback(uint8_t data);

/**
 * @brief Timer callback for Modbus T3.5 timeout
 */
void ModbusBridge_TimerCallback(void);

/**
 * @brief Returns true if the Modbus master (base system) replied to the
 *        heartbeat within the last 3 seconds.
 */
bool ModbusBridge_IsBaseAlive(void);

/**
 * @brief Returns the current Pick & Place state machine index (PnP_State_t
 *        cast to int). 0 = IDLE, see modbus_bridge.c for the full enum.
 */
int ModbusBridge_GetPnPState(void);

/**
 * @brief Call from HAL_UART_ErrorCallback when LPUART1 is in Modbus mode.
 *        Resets the protocol state machine and re-arms RX — fixes field hangs.
 */
void ModbusBridge_UartErrorRecovery(void);

#endif /* MODBUS_BRIDGE_H */