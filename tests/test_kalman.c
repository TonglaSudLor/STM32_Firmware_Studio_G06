#include <stdio.h>
#include <assert.h>
#include <math.h>
#include "../Core/Lib/kalman_lib.h"

void test_kalman_steady_state() {
    Kalman_t k;
    Kalman_Init(&k, 0.0f);
    
    // Constant measurement 10.0 rad
    float z = 10.0f * (M_PI / 180.0f);
    float dt = 0.001f;
    
    for (int i = 0; i < 1000; i++) {
        Kalman_Step(&k, 0.0f, z, dt);
    }
    
    printf("Kalman Steady State: Measured 10.0 deg, Estimated %.2f deg\n", k.x[0] * (180.0f/M_PI));
    assert(fabsf(k.x[0] - z) < 0.01f);
}

void test_kalman_dynamic_response() {
    Kalman_t k;
    Kalman_Init(&k, 0.0f);
    
    // Apply 24V step, simulate simple movement
    float u = 24.0f;
    float dt = 0.001f;
    float z = 0.0f;
    
    for (int i = 0; i < 100; i++) {
        // Idealized movement: theta += omega*dt
        z += 0.1f * dt; 
        Kalman_Step(&k, u, z, dt);
    }
    
    printf("Kalman Dynamic: State Theta=%.4f, Omega=%.2f, TauL=%.4f, Ia=%.2f\n", 
           k.x[0], k.x[1], k.x[2], k.x[3]);
    assert(k.P[0][0] < 1.0f); // Uncertainty should decrease
}

int main() {
    printf("Running Kalman Filter Tests...\n");
    test_kalman_steady_state();
    test_kalman_dynamic_response();
    printf("Kalman Filter Tests Passed!\n");
    return 0;
}
