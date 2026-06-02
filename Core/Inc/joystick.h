/**
 * @file joystick.h
 * @brief ESP32 Joystick Packet Parser — USART3 middleware.
 *
 * TODO: Extract from Core/Src/main.c
 *   - USART3 RX callback (HAL_UART_RxCpltCallback, huart3 branch)
 *   - rx_packet[] parsing + Motor_ProcessPacket() dispatch
 *   - Joystick timeout / disconnect logic (JOYSTICK_TIMEOUT_MS)
 *   - u3_txq TX queue (USART3_QueueTx)
 *
 * Packet format from ESP32: 3 chars [action, safety, status]
 *   action: 'U'=up 'D'=down 'L'=left 'R'=right 'S'=stop 'H'=home ...
 *   safety: 'S'=safe 'E'=estop
 *   status: 'C'=connected 'D'=disconnected
 *
 * When done: add Core/Middleware/joystick.c
 *   Rule: only this file may touch huart3 RX path.
 */

#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    char action;
    char safety;
    char status;
} Joystick_Pkt_t;

void Joystick_Init(void);
void Joystick_RxByte(uint8_t byte);           /* call from USART3 RX ISR */
bool Joystick_GetPacket(Joystick_Pkt_t *out); /* poll from main loop */
bool Joystick_IsConnected(void);
void Joystick_TxByte(uint8_t byte);           /* enqueue echo/audio byte */
