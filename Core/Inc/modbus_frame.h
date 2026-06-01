/**
 * @file modbus_frame.h
 * @brief Modbus RTU frame parser and builder — pure C99, zero HAL dependency.
 *
 * This layer handles only byte-level framing:
 *   - CRC-16 calculation
 *   - FC03 Read Holding Registers
 *   - FC06 Write Single Register
 *   - Exception responses
 *
 * It knows nothing about UARTs, timers, or STM32. Unit-testable on any host.
 */

#ifndef MODBUS_FRAME_H
#define MODBUS_FRAME_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* -------------------------------------------------------------------------
 * Types
 * ---------------------------------------------------------------------- */

/** 16-bit register: access as U16 or two U8 bytes (big-endian Modbus order). */
typedef union {
    uint16_t U16;
    uint8_t  U8[2];
} Modbus_Register_t;

/** Context passed into every frame call — no HAL fields. */
typedef struct {
    uint8_t            slave_address;
    Modbus_Register_t *registers;
    uint32_t           register_count;
} Modbus_Frame_Ctx_t;

/* -------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------- */

/**
 * @brief Calculate Modbus CRC-16 over a byte buffer.
 * @param data  Pointer to data.
 * @param len   Number of bytes.
 * @return      CRC-16 value (hi byte first in Modbus wire order).
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t len);

/**
 * @brief Parse an incoming Modbus frame and build the response.
 *
 * Validates CRC, slave address, and function code.
 * On success writes the complete response frame (including address + CRC)
 * into tx_buf and sets *tx_len.
 *
 * @param ctx     Register context (slave address + register map).
 * @param rx_buf  Raw received bytes (address + PDU + CRC).
 * @param rx_len  Number of received bytes (must be >= 4).
 * @param tx_buf  Output buffer (caller must provide >= 256 bytes).
 * @param tx_len  Set to response length on success, 0 on failure.
 * @return true   Response ready in tx_buf — caller should transmit.
 * @return false  Frame ignored (bad CRC, wrong address, or rx_len < 4).
 */
bool Modbus_BuildResponse(Modbus_Frame_Ctx_t *ctx,
                          const uint8_t *rx_buf, uint16_t rx_len,
                          uint8_t       *tx_buf, uint16_t *tx_len);

#endif /* MODBUS_FRAME_H */
