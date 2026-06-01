#ifndef __TELEMETRY_HUB_H
#define __TELEMETRY_HUB_H

#include <stdbool.h>
#include "main.h"

/**
 * @brief Initialize Telemetry Hub with a specific UART handle
 */
void Telemetry_Init(UART_HandleTypeDef *huart);

/**
 * @brief Periodic update function to stream data to the dashboard
 * Suggested call rate: 20ms - 50ms
 */
void Telemetry_Update(void);

/**
 * @brief Byte-by-byte parser for incoming dashboard commands
 * Should be called from HAL_UART_RxCpltCallback
 */
void Telemetry_ProcessByte(uint8_t byte);

/**
 * @brief Returns true if at least one valid command has been received from the dashboard.
 */
bool Telemetry_HasReceivedCommand(void);

#endif /* __TELEMETRY_HUB_H */
