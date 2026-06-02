/**
 * @file test_safety.c
 * @brief Unit tests for Core/App/safety.c — runs on host PC with gcc.
 *
 * Compiled with -include mock_hal.h which provides PRIMASK stubs.
 *
 * Build & run:
 *   cd tests && make test
 */

#include <stdio.h>
#include <stdbool.h>
#include "../Core/Inc/safety.h"

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

/* -------------------------------------------------------------------------
 * Test: Init sets STARTUP_ESTOP latch
 * ---------------------------------------------------------------------- */
static void test_init(void)
{
    printf("\n[Init]\n");

    Safety_Init();
    FaultCode_t f = Safety_GetFaults();

    TEST("init: STARTUP_ESTOP set",    (f & FAULT_STARTUP_ESTOP) != 0);
    TEST("init: IsEStop = true",       Safety_IsEStop());
    TEST("init: no other faults",      (f & ~FAULT_STARTUP_ESTOP) == 0);
}

/* -------------------------------------------------------------------------
 * Test: SetFault / ClearFault basic
 * ---------------------------------------------------------------------- */
static void test_set_clear(void)
{
    printf("\n[SetFault / ClearFault]\n");

    /* Start clean (clear startup latch) */
    Safety_ClearFault((FaultCode_t)0xFFF);
    TEST("cleared: no faults",         Safety_GetFaults() == FAULT_NONE);
    TEST("cleared: IsEStop = false",   !Safety_IsEStop());

    /* Set a single fault */
    Safety_SetFault(FAULT_MOTOR_STALLED);
    TEST("stall set",                  (Safety_GetFaults() & FAULT_MOTOR_STALLED) != 0);
    TEST("stall: IsEStop = true",      Safety_IsEStop());

    /* Set a second fault — both coexist */
    Safety_SetFault(FAULT_ENCODER_ERROR);
    TEST("two faults coexist",         (Safety_GetFaults() & (FAULT_MOTOR_STALLED | FAULT_ENCODER_ERROR))
                                       == (FAULT_MOTOR_STALLED | FAULT_ENCODER_ERROR));

    /* Clear only one */
    Safety_ClearFault(FAULT_MOTOR_STALLED);
    TEST("stall cleared, enc remains", (Safety_GetFaults() & FAULT_ENCODER_ERROR) != 0);
    TEST("stall gone",                 (Safety_GetFaults() & FAULT_MOTOR_STALLED) == 0);

    /* Clear all */
    Safety_ClearFault(FAULT_ENCODER_ERROR);
    TEST("all cleared",                Safety_GetFaults() == FAULT_NONE);
    TEST("IsEStop = false again",      !Safety_IsEStop());
}

/* -------------------------------------------------------------------------
 * Test: Multiple faults bitmask — set/clear groups
 * ---------------------------------------------------------------------- */
static void test_bitmask(void)
{
    printf("\n[Bitmask operations]\n");

    Safety_ClearFault((FaultCode_t)0xFFF);

    /* Set all ESTOP sources at once */
    FaultCode_t estops = (FaultCode_t)(FAULT_ESTOP_PHYSICAL | FAULT_ESTOP_JOYSTICK |
                                       FAULT_ESTOP_DASHBOARD | FAULT_ESTOP_MODBUS);
    Safety_SetFault(estops);
    TEST("all estops set",             (Safety_GetFaults() & estops) == (uint32_t)estops);

    /* Clear only joystick estop */
    Safety_ClearFault(FAULT_ESTOP_JOYSTICK);
    FaultCode_t remaining = (FaultCode_t)(FAULT_ESTOP_PHYSICAL | FAULT_ESTOP_DASHBOARD |
                                          FAULT_ESTOP_MODBUS);
    TEST("joy cleared, rest remain",   (Safety_GetFaults() & estops) == (uint32_t)remaining);

    /* SetFault is idempotent */
    Safety_SetFault(FAULT_ESTOP_PHYSICAL);  /* already set */
    TEST("double-set is idempotent",   (Safety_GetFaults() & FAULT_ESTOP_PHYSICAL) != 0);

    /* ClearFault on unset bit is safe */
    Safety_ClearFault(FAULT_MOTOR_STALLED);  /* not set */
    TEST("clear unset bit: safe",      (Safety_GetFaults() & FAULT_ESTOP_PHYSICAL) != 0);

    Safety_ClearFault((FaultCode_t)0xFFF);
    TEST("clean after bitmask tests",  Safety_GetFaults() == FAULT_NONE);
}

/* -------------------------------------------------------------------------
 * Test: IsEStop behaviour
 * ---------------------------------------------------------------------- */
static void test_estop(void)
{
    printf("\n[IsEStop]\n");

    Safety_ClearFault((FaultCode_t)0xFFF);

    TEST("no fault: IsEStop false",    !Safety_IsEStop());

    /* Any single fault → IsEStop true */
    Safety_SetFault(FAULT_OVERCURRENT);
    TEST("overcurrent: IsEStop true",  Safety_IsEStop());

    Safety_ClearFault(FAULT_OVERCURRENT);
    TEST("cleared: IsEStop false",     !Safety_IsEStop());

    /* FAULT_NONE (0) → IsEStop false */
    Safety_SetFault(FAULT_NONE);  /* no-op */
    TEST("set FAULT_NONE: still false", !Safety_IsEStop());
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */
int main(void)
{
    printf("=== safety unit tests ===\n");

    test_init();
    test_set_clear();
    test_bitmask();
    test_estop();

    printf("\n==============================\n");
    printf("  Passed: %d\n", g_passed);
    printf("  Failed: %d\n", g_failed);
    printf("==============================\n");

    return (g_failed > 0) ? 1 : 0;
}
