#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../Core/Inc/scurve.h"

void test_scurve_basic() {
    SCurve_t sc;
    float v_max = 60.0f; // units/s
    float a_max = 100.0f; // units/s^2
    float j_max = 1000.0f; // units/s^3
    
    SCurve_Init(&sc, v_max, a_max, j_max);
    SCurve_Plan(&sc, 0.0f, 100.0f);
    
    assert(sc.active == true);
    assert(sc.direction == 1);
    
    float p, v, a;
    float dt = 0.01f;
    float t = 0.0f;
    
    // Simulate until done
    while (sc.active) {
        SCurve_Step(&sc, dt, &p, &v, &a);
        t += dt;
        
        // Sanity checks
        assert(fabsf(v) <= v_max + 0.1f);
        assert(fabsf(a) <= a_max + 0.1f);
    }
    
    printf("S-Curve Move: Target 100.0, Result %.2f, Time %.2f s\n", p, t);
    assert(fabsf(p - 100.0f) < 0.1f);
}

void test_scurve_short_move() {
    SCurve_t sc;
    SCurve_Init(&sc, 100.0f, 100.0f, 1000.0f);
    SCurve_Plan(&sc, 0.0f, 0.5f); // Very short move
    
    float p, v, a;
    while (sc.active) {
        SCurve_Step(&sc, 0.01f, &p, &v, &a);
    }
    printf("S-Curve Short Move: Target 0.5, Result %.2f\n", p);
    assert(fabsf(p - 0.5f) < 0.01f);
}

int main() {
    printf("Running S-Curve Tests...\n");
    test_scurve_basic();
    test_scurve_short_move();
    printf("S-Curve Tests Passed!\n");
    return 0;
}
