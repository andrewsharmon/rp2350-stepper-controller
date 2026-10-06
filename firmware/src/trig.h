#pragma once

// Sine / cosine without libm. The Pico SDK's float sinf/cosf returned wrong
// values near multiples of pi/2 on the RP2350, so motion code uses these
// Taylor polynomials instead (host and target alike, so the host tests cover
// them).

#define TRIG_PI 3.14159265358979323846

// sin(x) for |x| <= pi/2: Taylor to x^11, error < 6e-8.
static inline float trig_sin_core(float x) {
    float x2 = x * x;
    return x * (1.0f + x2 * (-1.0f / 6.0f + x2 * (1.0f / 120.0f + x2 * (-1.0f / 5040.0f +
           x2 * (1.0f / 362880.0f + x2 * (-1.0f / 39916800.0f))))));
}

// sin(x) for |x| <= pi/2 in double: Taylor to x^19, error < 1e-13.
static inline double trig_sin_core_d(double x) {
    double x2 = x * x, term = x, sum = x;
    for (int k = 1; k <= 9; k++) {
        term *= -x2 / (double)((2 * k) * (2 * k + 1));
        sum += term;
    }
    return sum;
}

// sin(x) for any x.
static inline float trig_sinf(float x) {
    const float two_pi = (float)(2.0 * TRIG_PI), half_pi = (float)(0.5 * TRIG_PI);
    float r = x - two_pi * (float)(int)(x / two_pi);  // (-2pi, 2pi)
    if (r > (float)TRIG_PI)
        r -= two_pi;
    else if (r < -(float)TRIG_PI)
        r += two_pi;
    // Fold [-pi, pi] onto [-pi/2, pi/2]: sin(pi - r) = sin(r).
    if (r > half_pi)
        r = (float)TRIG_PI - r;
    else if (r < -half_pi)
        r = -(float)TRIG_PI - r;
    return trig_sin_core(r);
}

static inline float trig_cosf(float x) {
    return trig_sinf(x + (float)(0.5 * TRIG_PI));
}

// cos(pi * u) for u in [0, 1], in double.
static inline double trig_cos_pi_d(double u) {
    return trig_sin_core_d(TRIG_PI * (0.5 - u));
}
