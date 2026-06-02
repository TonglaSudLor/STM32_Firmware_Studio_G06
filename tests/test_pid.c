/**
 * @file test_pid.c
 * @brief Unit tests for Core/Lib/pid.c — runs on host PC with gcc.
 *
 * Build & run:
 *   cd tests && make test
 */

#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "../Core/Lib/pid.h"

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
 * Test: Proportional only (Ki=Kd=0)
 * ---------------------------------------------------------------------- */
static void test_proportional(void)
{
    printf("\n[Proportional only]\n");

    PID_t pid = {.Kp = 2.0f, .Ki = 0.0f, .Kd = 0.0f,
                 .integral_max = 100.0f,
                 .output_min = -200.0f, .output_max = 200.0f};
    PID_Reset(&pid);

    float out = PID_Update(&pid, 5.0f, 0.01f);
    TEST("P: output = Kp * error",  NEAR(out, 10.0f, 1e-4f));

    out = PID_Update(&pid, -3.0f, 0.01f);
    TEST("P: negative error",       NEAR(out, -6.0f, 1e-4f));

    out = PID_Update(&pid, 0.0f, 0.01f);
    TEST("P: zero error = 0",       NEAR(out, 0.0f, 1e-4f));
}

/* -------------------------------------------------------------------------
 * Test: Integral accumulation
 * ---------------------------------------------------------------------- */
static void test_integral(void)
{
    printf("\n[Integral accumulation]\n");

    PID_t pid = {.Kp = 0.0f, .Ki = 1.0f, .Kd = 0.0f,
                 .integral_max = 1000.0f,
                 .output_min = -1e6f, .output_max = 1e6f};
    PID_Reset(&pid);

    /* 10 steps of error=1 at dt=0.01 → integral = 0.1 → output = 0.1 */
    float out = 0.0f;
    for (int i = 0; i < 10; i++) out = PID_Update(&pid, 1.0f, 0.01f);
    TEST("I: accumulated = 0.1",    NEAR(out, 0.1f, 1e-4f));

    /* 10 more steps of error=-1 → integral decreases back toward 0 */
    for (int i = 0; i < 10; i++) out = PID_Update(&pid, -1.0f, 0.01f);
    TEST("I: unwound back to 0",    NEAR(out, 0.0f, 1e-4f));
}

/* -------------------------------------------------------------------------
 * Test: Integral clamping
 * ---------------------------------------------------------------------- */
static void test_integral_clamp(void)
{
    printf("\n[Integral clamping]\n");

    PID_t pid = {.Kp = 0.0f, .Ki = 1.0f, .Kd = 0.0f,
                 .integral_max = 0.05f,   /* tight clamp */
                 .output_min = -1e6f, .output_max = 1e6f};
    PID_Reset(&pid);

    /* Drive many steps — integral must stay <= 0.05 */
    float out = 0.0f;
    for (int i = 0; i < 1000; i++) out = PID_Update(&pid, 1.0f, 0.01f);
    TEST("I: clamped at integral_max", NEAR(out, 0.05f, 1e-4f));
    TEST("I: integral stored clamped", NEAR(pid.integral, 0.05f, 1e-4f));
}

/* -------------------------------------------------------------------------
 * Test: Output clamping
 * ---------------------------------------------------------------------- */
static void test_output_clamp(void)
{
    printf("\n[Output clamping]\n");

    PID_t pid = {.Kp = 10.0f, .Ki = 0.0f, .Kd = 0.0f,
                 .integral_max = 100.0f,
                 .output_min = -5.0f, .output_max = 5.0f};
    PID_Reset(&pid);

    float out = PID_Update(&pid, 100.0f, 0.01f); /* would be 1000 without clamp */
    TEST("clamp max", NEAR(out, 5.0f, 1e-4f));

    out = PID_Update(&pid, -100.0f, 0.01f);
    TEST("clamp min", NEAR(out, -5.0f, 1e-4f));
}

/* -------------------------------------------------------------------------
 * Test: Anti-windup — integral must not keep growing when saturated
 * ---------------------------------------------------------------------- */
static void test_antiwindup(void)
{
    printf("\n[Anti-windup]\n");

    PID_t pid = {.Kp = 0.0f, .Ki = 100.0f, .Kd = 0.0f,
                 .integral_max = 1000.0f,
                 .output_min = -5.0f, .output_max = 5.0f};
    PID_Reset(&pid);

    /* Push into saturation for 100 steps */
    for (int i = 0; i < 100; i++) PID_Update(&pid, 1.0f, 0.01f);
    float integral_saturated = pid.integral;

    /* Now apply opposing error — output should respond immediately */
    float out = PID_Update(&pid, -1.0f, 0.01f);

    /* With proper anti-windup integral doesn't wind up uncontrollably */
    TEST("anti-windup: integral bounded", integral_saturated < 1.0f);
    TEST("anti-windup: responds to neg error", out < 5.0f);
}

/* -------------------------------------------------------------------------
 * Test: Derivative term damps oscillation
 * ---------------------------------------------------------------------- */
static void test_derivative(void)
{
    printf("\n[Derivative term]\n");

    PID_t pid = {.Kp = 0.0f, .Ki = 0.0f, .Kd = 1.0f,
                 .integral_max = 100.0f,
                 .output_min = -1e6f, .output_max = 1e6f};
    PID_Reset(&pid);

    /* Step change: error goes from 0 to 1 → derivative fires */
    float out1 = PID_Update(&pid, 1.0f, 0.01f); /* d_raw = Kd*(1-0)/dt = 100 */
    /* After IIR filter: d_filt = 0.2 * 100 = 20 */
    TEST("D: fires on step (positive d_filt)", out1 > 0.0f);

    /* Constant error → derivative decays toward 0 */
    float out_prev = out1;
    for (int i = 0; i < 20; i++) out_prev = PID_Update(&pid, 1.0f, 0.01f);
    TEST("D: decays to 0 on constant error", NEAR(out_prev, 0.0f, 0.5f));

    /* Error decreasing → derivative is negative (damping action) */
    PID_Reset(&pid);
    pid.error_prev = 1.0f;
    float out_dec = PID_Update(&pid, 0.5f, 0.01f); /* error dropped → d_raw < 0 */
    TEST("D: negative when error decreasing", out_dec < 0.0f);
}

/* -------------------------------------------------------------------------
 * Test: Reset clears all state
 * ---------------------------------------------------------------------- */
static void test_reset(void)
{
    printf("\n[Reset]\n");

    PID_t pid = {.Kp = 1.0f, .Ki = 1.0f, .Kd = 1.0f,
                 .integral_max = 100.0f,
                 .output_min = -1e6f, .output_max = 1e6f};
    PID_Reset(&pid);

    for (int i = 0; i < 10; i++) PID_Update(&pid, 5.0f, 0.01f);

    PID_Reset(&pid);
    TEST("reset: integral = 0",   NEAR(pid.integral,   0.0f, 1e-9f));
    TEST("reset: error_prev = 0", NEAR(pid.error_prev, 0.0f, 1e-9f));
    TEST("reset: d_filt = 0",     NEAR(pid.d_filt,     0.0f, 1e-9f));

    /* After reset, P-only response should be exact */
    float out = PID_Update(&pid, 3.0f, 0.01f);
    /* P=3, I≈0, D fires once from 0→3 but filter attenuates */
    /* Just verify P term dominates: out in reasonable range */
    TEST("post-reset first step >= Kp*error", out >= 3.0f);
}

/* -------------------------------------------------------------------------
 * Test: dt=0 guard
 * ---------------------------------------------------------------------- */
static void test_dt_zero(void)
{
    printf("\n[dt=0 guard]\n");

    PID_t pid = {.Kp = 10.0f, .Ki = 1.0f, .Kd = 1.0f,
                 .integral_max = 100.0f,
                 .output_min = -1e6f, .output_max = 1e6f};
    PID_Reset(&pid);

    float out = PID_Update(&pid, 5.0f, 0.0f);
    TEST("dt=0 returns 0 (no div-by-zero)", NEAR(out, 0.0f, 1e-9f));
}

/* -------------------------------------------------------------------------
 * main
 * ---------------------------------------------------------------------- */
int main(void)
{
    printf("=== pid unit tests ===\n");

    test_proportional();
    test_integral();
    test_integral_clamp();
    test_output_clamp();
    test_antiwindup();
    test_derivative();
    test_reset();
    test_dt_zero();

    printf("\n==============================\n");
    printf("  Passed: %d\n", g_passed);
    printf("  Failed: %d\n", g_failed);
    printf("==============================\n");

    return (g_failed > 0) ? 1 : 0;
}
