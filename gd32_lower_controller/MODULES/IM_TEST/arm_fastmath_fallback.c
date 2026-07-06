#include "arm_math.h"

/*
 * Fallback implementations for CMSIS-DSP Fast Math.
 *
 * Your project includes CMSIS-DSP headers (arm_math.h) and calls
 * arm_sin_f32 / arm_cos_f32 from Simulink-generated code, but does not link
 * the official CMSIS-DSP library sources.
 *
 * These implementations are lightweight approximations (range reduction +
 * low-order polynomials) so the linker has symbols to resolve.
 *
 * If you later add official CMSIS-DSP FastMathFunctions, its strong symbols
 * can override these weak definitions.
 */

#ifndef ARM_MATH_PI_F32
#define ARM_MATH_PI_F32        (3.14159265358979323846f)
#endif
#ifndef ARM_MATH_TWO_PI_F32
#define ARM_MATH_TWO_PI_F32    (6.28318530717958647692f)
#endif
#ifndef ARM_MATH_HALF_PI_F32
#define ARM_MATH_HALF_PI_F32   (1.57079632679489661923f)
#endif

static float32_t wrap_pi_f32(float32_t x)
{
    /* Bring x to [-pi, pi] without libm (good enough for typical embedded ranges). */
    int32_t k = (int32_t)(x / ARM_MATH_TWO_PI_F32);
    x = x - (float32_t)k * ARM_MATH_TWO_PI_F32;

    if (x > ARM_MATH_PI_F32)
        x -= ARM_MATH_TWO_PI_F32;
    else if (x < -ARM_MATH_PI_F32)
        x += ARM_MATH_TWO_PI_F32;

    return x;
}

static float32_t sin_poly_f32(float32_t x)
{
    /* sin(x) ~ x - x^3/6 + x^5/120 - x^7/5040  (|x| <= pi/2) */
    float32_t x2 = x * x;
    float32_t x3 = x * x2;
    float32_t x5 = x3 * x2;
    float32_t x7 = x5 * x2;
    return x - (x3 * (1.0f / 6.0f)) + (x5 * (1.0f / 120.0f)) - (x7 * (1.0f / 5040.0f));
}

static float32_t cos_poly_f32(float32_t x)
{
    /* cos(x) ~ 1 - x^2/2 + x^4/24 - x^6/720  (|x| <= pi/2) */
    float32_t x2 = x * x;
    float32_t x4 = x2 * x2;
    float32_t x6 = x4 * x2;
    return 1.0f - (x2 * 0.5f) + (x4 * (1.0f / 24.0f)) - (x6 * (1.0f / 720.0f));
}

#if defined(__CC_ARM)
__weak
#else
__attribute__((weak))
#endif
float32_t arm_sin_f32(float32_t x)
{
    x = wrap_pi_f32(x);

    /* Reduce to [-pi/2, pi/2] using symmetries. */
    if (x > ARM_MATH_HALF_PI_F32) {
        x = ARM_MATH_PI_F32 - x;
    } else if (x < -ARM_MATH_HALF_PI_F32) {
        x = -ARM_MATH_PI_F32 - x;
    }

    return sin_poly_f32(x);
}

#if defined(__CC_ARM)
__weak
#else
__attribute__((weak))
#endif
float32_t arm_cos_f32(float32_t x)
{
    x = wrap_pi_f32(x);

    /* Reduce to [-pi/2, pi/2] using symmetries, tracking sign. */
    float32_t sign = 1.0f;
    if (x > ARM_MATH_HALF_PI_F32) {
        x = ARM_MATH_PI_F32 - x;
        sign = -1.0f;
    } else if (x < -ARM_MATH_HALF_PI_F32) {
        x = ARM_MATH_PI_F32 + x;
        sign = -1.0f;
    }

    return sign * cos_poly_f32(x);
}
