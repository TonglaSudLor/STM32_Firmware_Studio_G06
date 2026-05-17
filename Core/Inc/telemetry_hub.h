#ifndef __TELEMETRY_HUB_H
#define __TELEMETRY_HUB_H

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

#endif /* __TELEMETRY_HUB_H */
