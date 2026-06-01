/**
 * @file test_modbus_frame.c
 * @brief Unit tests for modbus_frame.c — runs on host PC with gcc.
 *
 * Build & run:
 *   cd tests && make test
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "mock_hal.h"
#include "../Core/Inc/modbus_frame.h"

/* -------------------------------------------------------------------------
 * Minimal test framework
 * ---------------------------------------------------------------------- */
static int g_passed = 0;
static int g_failed = 0;

#define TEST(name, expr) do {                                    \
    if (expr) {                                                  \
        printf("  [PASS] %s\n", name);                           \
        g_passed++;                                              \
    } else {                                                     \
        printf("  [FAIL] %s  (line %d)\n", name, __LINE__);     \
        g_failed++;                                              \
    }                                                            \
} while (0)

/* Build a complete Modbus frame (addr + PDU + CRC) into buf, return length */
static uint16_t make_frame(uint8_t *buf, uint8_t addr,
                            const uint8_t *pdu, uint16_t pdu_len)
{
    buf[0] = addr;
    memcpy(&buf[1], pdu, pdu_len);
    uint16_t body_len = 1 + pdu_len;
    uint16_t crc = Modbus_CRC16(buf, body_len);
    Modbus_Register_t r; r.U16 = crc;
    buf[body_len]     = r.U8[0];
    buf[body_len + 1] = r.U8[1];
    return body_len + 2;
}

/* -------------------------------------------------------------------------
 * Test: CRC-16
 * ---------------------------------------------------------------------- */
static void test_crc16(void)
{
    printf("\n[CRC-16]\n");

    /* Known vector: FC03 request for slave 1, reg 0, count 1
     * Expected CRC: 0x840A (lo=0x0A, hi=0x84 in wire order) */
    uint8_t frame[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    uint16_t crc = Modbus_CRC16(frame, 6);
    TEST("known vector FC03", crc == 0x0A84);

    /* Empty input: CRC of zero bytes = 0xFFFF (initial value) */
    TEST("empty input = 0xFFFF", Modbus_CRC16(frame, 0) == 0xFFFF);

    /* Single byte 0x00 */
    uint8_t b0 = 0x00;
    TEST("single byte 0x00", Modbus_CRC16(&b0, 1) == 0x40BF);
}

/* -------------------------------------------------------------------------
 * Test: FC03 Read Holding Registers
 * ---------------------------------------------------------------------- */
static void test_fc03(void)
{
    printf("\n[FC03 Read Holding Registers]\n");

    Modbus_Register_t regs[8];
    for (int i = 0; i < 8; i++) regs[i].U16 = (uint16_t)(100 + i);

    Modbus_Frame_Ctx_t ctx = {.slave_address = 1,
                               .registers     = regs,
                               .register_count = 8};
    uint8_t rx[16], tx[64];
    uint16_t rx_len, tx_len;

    /* Read reg[0], count=1 */
    uint8_t pdu1[] = {0x03, 0x00, 0x00, 0x00, 0x01};
    rx_len = make_frame(rx, 1, pdu1, sizeof(pdu1));
    bool ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);

    TEST("FC03 returns true",      ok);
    TEST("FC03 slave addr in tx",  tx[0] == 1);
    TEST("FC03 function code",     tx[1] == 0x03);
    TEST("FC03 byte count",        tx[2] == 2);
    TEST("FC03 reg[0] hi byte",    tx[3] == 0);
    TEST("FC03 reg[0] lo byte",    tx[4] == 100);
    TEST("FC03 total length",      tx_len == 7); /* addr+FC+BC+2data+CRC */

    /* Read reg[2..3], count=2 */
    uint8_t pdu2[] = {0x03, 0x00, 0x02, 0x00, 0x02};
    rx_len = make_frame(rx, 1, pdu2, sizeof(pdu2));
    ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);
    TEST("FC03 multi read ok",     ok);
    TEST("FC03 multi byte count",  tx[2] == 4);
    TEST("FC03 reg[2] value",      tx[4] == 102);
    TEST("FC03 reg[3] value",      tx[6] == 103);
}

/* -------------------------------------------------------------------------
 * Test: FC06 Write Single Register
 * ---------------------------------------------------------------------- */
static void test_fc06(void)
{
    printf("\n[FC06 Write Single Register]\n");

    Modbus_Register_t regs[4] = {0};
    Modbus_Frame_Ctx_t ctx = {.slave_address = 1,
                               .registers     = regs,
                               .register_count = 4};
    uint8_t rx[16], tx[64];
    uint16_t rx_len, tx_len;

    /* Write 0x1234 to reg[1] */
    uint8_t pdu[] = {0x06, 0x00, 0x01, 0x12, 0x34};
    rx_len = make_frame(rx, 1, pdu, sizeof(pdu));
    bool ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);

    TEST("FC06 returns true",       ok);
    TEST("FC06 register updated",   regs[1].U16 == 0x1234);
    TEST("FC06 echo addr in tx",    tx[0] == 1);
    TEST("FC06 echo FC in tx",      tx[1] == 0x06);
    TEST("FC06 echo reg addr hi",   tx[2] == 0x00);
    TEST("FC06 echo reg addr lo",   tx[3] == 0x01);
    TEST("FC06 echo value hi",      tx[4] == 0x12);
    TEST("FC06 echo value lo",      tx[5] == 0x34);
    TEST("FC06 total length",       tx_len == 8); /* addr+FC+addr2+val2+CRC2 */
}

/* -------------------------------------------------------------------------
 * Test: Error cases
 * ---------------------------------------------------------------------- */
static void test_errors(void)
{
    printf("\n[Error cases]\n");

    Modbus_Register_t regs[4] = {0};
    Modbus_Frame_Ctx_t ctx = {.slave_address = 21,
                               .registers     = regs,
                               .register_count = 4};
    uint8_t rx[16], tx[64];
    uint16_t rx_len, tx_len;

    /* Bad CRC */
    uint8_t pdu_r[] = {0x03, 0x00, 0x00, 0x00, 0x01};
    rx_len = make_frame(rx, 21, pdu_r, sizeof(pdu_r));
    rx[rx_len - 1] ^= 0xFF; /* corrupt CRC */
    TEST("bad CRC returns false",
         !Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len));

    /* Wrong slave address */
    rx_len = make_frame(rx, 99, pdu_r, sizeof(pdu_r));
    TEST("wrong address returns false",
         !Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len));

    /* Frame too short */
    TEST("rx_len < 4 returns false",
         !Modbus_BuildResponse(&ctx, rx, 3, tx, &tx_len));

    /* FC03: address out of range → exception 0x02 */
    uint8_t pdu_oor[] = {0x03, 0x00, 0x10, 0x00, 0x01}; /* reg 0x10 = 16, count=4 */
    rx_len = make_frame(rx, 21, pdu_oor, sizeof(pdu_oor));
    bool ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);
    TEST("OOR returns true (exception)", ok);
    TEST("OOR exception function code",  tx[1] == (0x03 | 0x80));
    TEST("OOR exception code 0x02",      tx[2] == 0x02);

    /* FC03: count = 0 → exception 0x03 */
    uint8_t pdu_zero[] = {0x03, 0x00, 0x00, 0x00, 0x00};
    rx_len = make_frame(rx, 21, pdu_zero, sizeof(pdu_zero));
    ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);
    TEST("count=0 returns true (exception)", ok);
    TEST("count=0 exception code 0x03",      tx[2] == 0x03);

    /* Unknown function code → exception 0x01 */
    uint8_t pdu_unk[] = {0x10, 0x00, 0x00, 0x00, 0x01};
    rx_len = make_frame(rx, 21, pdu_unk, sizeof(pdu_unk));
    ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);
    TEST("unknown FC returns true (exception)", ok);
    TEST("unknown FC exception code 0x01",      tx[2] == 0x01);

    /* FC06: address out of range → exception 0x02 */
    uint8_t pdu_w_oor[] = {0x06, 0x00, 0x10, 0x00, 0x01};
    rx_len = make_frame(rx, 21, pdu_w_oor, sizeof(pdu_w_oor));
    ok = Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);
    TEST("FC06 OOR exception 0x02", tx[2] == 0x02);
}

/* -------------------------------------------------------------------------
 * Test: Response CRC is valid
 * ---------------------------------------------------------------------- */
static void test_response_crc(void)
{
    printf("\n[Response CRC integrity]\n");

    Modbus_Register_t regs[4];
    regs[0].U16 = 0xABCD;
    Modbus_Frame_Ctx_t ctx = {.slave_address = 5,
                               .registers     = regs,
                               .register_count = 4};
    uint8_t rx[16], tx[64];
    uint16_t rx_len, tx_len;

    uint8_t pdu[] = {0x03, 0x00, 0x00, 0x00, 0x01};
    rx_len = make_frame(rx, 5, pdu, sizeof(pdu));
    Modbus_BuildResponse(&ctx, rx, rx_len, tx, &tx_len);

    /* Verify the CRC appended to tx is correct */
    uint16_t expected_crc = Modbus_CRC16(tx, tx_len - 2);
    Modbus_Register_t got; got.U8[0] = tx[tx_len-2]; got.U8[1] = tx[tx_len-1];
    TEST("response CRC is valid", expected_crc == got.U16);
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */
int main(void)
{
    printf("=== modbus_frame unit tests ===\n");

    test_crc16();
    test_fc03();
    test_fc06();
    test_errors();
    test_response_crc();

    printf("\n==============================\n");
    printf("  Passed: %d\n", g_passed);
    printf("  Failed: %d\n", g_failed);
    printf("==============================\n");

    return (g_failed > 0) ? 1 : 0;
}
