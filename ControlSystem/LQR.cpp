#include "LQR.hpp"
#include "param.hpp"

static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

float LQR::compute(float theta, float ff_angle, float theta_dot,
                   float wheel_rpm, uint8_t esc_index)
{
    uint8_t idx = esc_index % 3u;
    const float *k = g_params.lqr_k[idx];
    float        ff = g_params.lqr_ff[idx];

    float current_cmd_A = k[0] * theta
                        + k[1] * theta_dot
                        + k[2] * wheel_rpm * 0.1047f;

    current_cmd_A += ff * ff_angle;

    current_cmd_A = clampf(current_cmd_A, -g_params.lqr_max_out, g_params.lqr_max_out);

    return current_cmd_A;
}
