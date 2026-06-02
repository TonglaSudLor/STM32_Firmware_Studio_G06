#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../Core/Inc/input_shaper.h"

void test_shaper_sum_to_one() {
    Shaper_t s;
    Shaper_Init(&s, 12.13f, 0.041f, 100.0f);
    
    float sum = s.A1 + s.A2 + s.A3;
    printf("Shaper coefficients: A1=%.3f, A2=%.3f, A3=%.3f, Sum=%.4f\n", s.A1, s.A2, s.A3, sum);
    assert(fabsf(sum - 1.0f) < 0.0001f);
}

void test_shaper_step_response() {
    Shaper_t s;
    Shaper_Init(&s, 10.0f, 0.1f, 100.0f);
    s.enabled = true;
    
    // Step input from 0 to 1
    float output = 0.0f;
    for (int i = 0; i < 200; i++) {
        output = Shaper_Process(&s, 100.0f);
    }
    
    printf("Shaper Step Response Final: %.2f\n", output);
    assert(fabsf(output - 100.0f) < 0.01f);
}

int main() {
    printf("Running Input Shaper Tests...\n");
    test_shaper_sum_to_one();
    test_shaper_step_response();
    printf("Input Shaper Tests Passed!\n");
    return 0;
}
