/**
 * @file test_encoder.c
 * @brief Unit tests for Core/Drivers/encoder.c — runs on host PC with gcc.
 *
 * Compiled with -DENCODER_PHASE_INVERTED=0 (no inversion).
 * Max valid delta per 10ms step = 2000 counts (~1464 RPM).
 * Tests use deltas within this limit; large moves split across multiple steps.
 *
 * Build & run:
 *   cd tests && make test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "../Core/Inc/encoder.h"

static int g_passed = 0;
static int g_failed = 0;

#define NEAR(a, b, tol) (fabsf((a) - (b)) < (tol))

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
 * Test: Reset
 * ---------------------------------------------------------------------- */
static void test_reset(void)
{
    printf("\n[Reset]\n");

    Encoder_State_t enc;
    enc.absolute_counts = 1234;
    enc.count_prev = 9999;
    enc.position_deg = 45.0f;
    enc.filtered_rpm = 100.0f;

    Encoder_Reset(&enc);
    TEST("reset: absolute_counts = 0", enc.absolute_counts == 0);
    TEST("reset: count_prev = 0",      enc.count_prev == 0);
    TEST("reset: position_deg = 0",    NEAR(enc.position_deg, 0.0f, 1e-6f));
    TEST("reset: filtered_rpm = 0",    NEAR(enc.filtered_rpm, 0.0f, 1e-6f));
}

/* -------------------------------------------------------------------------
 * Test: Position accumulation (multi-step to stay under noise threshold)
 *
 * Max valid delta = 2000 counts. 1 rev = 8192 counts, so split into steps.
 * -------------------------------------------------------------------------
 */
static void test_position(void)
{
    printf("\n[Position accumulation]\n");

    Encoder_State_t enc;
    Encoder_Reset(&enc);

    /* 1000 counts = (1000/8192)*360 = 43.945 deg */
    Encoder_UpdateWithRaw(&enc, 1000, 0.01f);
    TEST("1000 counts ≈ 43.9 deg",    NEAR(enc.position_deg, 43.945f, 0.01f));
    TEST("absolute_counts = 1000",    enc.absolute_counts == 1000);

    /* Add 1000 more → 2000 counts = 87.89 deg */
    Encoder_UpdateWithRaw(&enc, 2000, 0.01f);
    TEST("2000 counts ≈ 87.9 deg",    NEAR(enc.position_deg, 87.89f, 0.01f));

    /* Full revolution via multiple steps of 1000:
     * 8 steps × 1000 = 8000 counts = 351.6 deg (close enough to 360) */
    Encoder_Reset(&enc);
    uint16_t raw = 0;
    for (int i = 0; i < 8; i++) {
        raw += 1000;
        Encoder_UpdateWithRaw(&enc, raw, 0.01f);
    }
    TEST("8000 counts ≈ 351.6 deg",   NEAR(enc.position_deg, 351.56f, 0.02f));

    /* Negative direction: prev=1000, raw=500 → delta=-500 → pos decreases */
    Encoder_Reset(&enc);
    enc.count_prev = 1000;
    Encoder_UpdateWithRaw(&enc, 500, 0.01f);
    TEST("negative delta: pos < 0",   enc.position_deg < 0.0f);
    TEST("negative 500 counts",       enc.absolute_counts == -500);
}

/* -------------------------------------------------------------------------
 * Test: 16-bit rollover handling
 * ---------------------------------------------------------------------- */
static void test_rollover(void)
{
    printf("\n[16-bit rollover]\n");

    Encoder_State_t enc;
    Encoder_Reset(&enc);

    /* raw=65530, prev=0: delta=65530 → >32767 → -6 */
    Encoder_UpdateWithRaw(&enc, 65530, 0.01f);
    TEST("rollover neg: -6 counts", enc.absolute_counts == -6);

    /* raw=5, prev=65535: delta=5-65535=-65530 → <-32768 → +6 */
    Encoder_Reset(&enc);
    enc.count_prev = 65535;
    Encoder_UpdateWithRaw(&enc, 5, 0.01f);
    TEST("rollover fwd: +6 counts", enc.absolute_counts == 6);

    /* rollover by 1: prev=65535, raw=0 → delta=-65535 → +1 */
    Encoder_Reset(&enc);
    enc.count_prev = 65535;
    Encoder_UpdateWithRaw(&enc, 0, 0.01f);
    TEST("rollover by 1",           enc.absolute_counts == 1);
}

/* -------------------------------------------------------------------------
 * Test: Noise filter — reject |delta| > 2000
 * ---------------------------------------------------------------------- */
static void test_noise_filter(void)
{
    printf("\n[Noise filter]\n");

    Encoder_State_t enc;
    Encoder_Reset(&enc);

    /* Positive spike: delta=5000 > 2000 → rejected */
    Encoder_UpdateWithRaw(&enc, 5000, 0.01f);
    TEST("pos spike rejected: counts=0", enc.absolute_counts == 0);
    TEST("pos spike rejected: pos=0",    NEAR(enc.position_deg, 0.0f, 1e-6f));

    /* Boundary: delta=2000 exactly → accepted (> not >=) */
    Encoder_Reset(&enc);
    Encoder_UpdateWithRaw(&enc, 2000, 0.01f);
    TEST("delta=2000 accepted",         enc.absolute_counts == 2000);

    /* Boundary: delta=2001 → rejected */
    Encoder_Reset(&enc);
    Encoder_UpdateWithRaw(&enc, 2001, 0.01f);
    TEST("delta=2001 rejected",         enc.absolute_counts == 0);

    /* Valid move of 500 counts is accepted */
    Encoder_Reset(&enc);
    Encoder_UpdateWithRaw(&enc, 500, 0.01f);
    TEST("delta=500 accepted",          enc.absolute_counts == 500);

    /* Negative spike via rollover: prev=0, raw=60536 → delta=-5000 → rejected */
    Encoder_Reset(&enc);
    Encoder_UpdateWithRaw(&enc, 60536, 0.01f);
    TEST("neg spike rejected",          enc.absolute_counts == 0);
}

/* -------------------------------------------------------------------------
 * Test: RPM calculation
 *
 * 500 RPM = 500/60 rev/s = 8.333 rev/s → per 10ms: 0.08333 rev = 682.6 counts
 * First filtered: 0.15 * 500 + 0.85 * 0 = 75 RPM
 * ---------------------------------------------------------------------- */
static void test_rpm(void)
{
    printf("\n[RPM calculation]\n");

    Encoder_State_t enc;
    Encoder_Reset(&enc);

    /* 682 counts in 10ms → instant = (682/8192/0.01)*60 = 499.5 RPM
     * first filtered = 0.15 * 499.5 = 74.9 RPM */
    Encoder_UpdateWithRaw(&enc, 682, 0.01f);
    TEST("RPM: first sample ≈ 75",    NEAR(enc.filtered_rpm, 74.9f, 1.0f));

    /* After many identical steps, converges to ~500 RPM */
    Encoder_Reset(&enc);
    uint16_t raw = 0;
    for (int i = 0; i < 200; i++) {
        raw = (uint16_t)(raw + 682u);
        Encoder_UpdateWithRaw(&enc, raw, 0.01f);
    }
    TEST("RPM: converges to ~500",     NEAR(enc.filtered_rpm, 499.5f, 5.0f));

    /* Zero delta → RPM decays: filtered = 0.85 * prev */
    Encoder_State_t enc2;
    Encoder_Reset(&enc2);
    enc2.filtered_rpm = 1000.0f;
    enc2.count_prev = 500;
    Encoder_UpdateWithRaw(&enc2, 500, 0.01f);   /* delta = 0 */
    TEST("RPM decays on zero delta",   enc2.filtered_rpm < 1000.0f);
    TEST("RPM decay: 0.85 * 1000",    NEAR(enc2.filtered_rpm, 850.0f, 1.0f));
}

/* -------------------------------------------------------------------------
 * Test: Accessors
 * ---------------------------------------------------------------------- */
static void test_accessors(void)
{
    printf("\n[Accessors]\n");

    Encoder_State_t enc;
    Encoder_Reset(&enc);

    /* 1000 counts = 43.945 deg */
    Encoder_UpdateWithRaw(&enc, 1000, 0.01f);

    TEST("GetPositionDeg matches struct", NEAR(Encoder_GetPositionDeg(&enc), enc.position_deg, 1e-6f));
    TEST("GetRPM matches struct",         NEAR(Encoder_GetRPM(&enc),         enc.filtered_rpm,  1e-6f));
    TEST("GetPositionDeg ≈ 43.9 deg",    NEAR(Encoder_GetPositionDeg(&enc), 43.945f, 0.01f));
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */
int main(void)
{
    printf("=== encoder unit tests (PHASE_INVERTED=0) ===\n");

    test_reset();
    test_position();
    test_rollover();
    test_noise_filter();
    test_rpm();
    test_accessors();

    printf("\n==============================\n");
    printf("  Passed: %d\n", g_passed);
    printf("  Failed: %d\n", g_failed);
    printf("==============================\n");

    return (g_failed > 0) ? 1 : 0;
}
