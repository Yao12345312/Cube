#include "LQR_Control.hpp"

#include <cmath>
#include <stdint.h>

volatile float K_lqr[3] = {
    -200.0f,
    -18.0f,
    -0.0f
};

namespace {
	
//电流环输出限幅
static constexpr float max_output = 30.0f;

static inline float rpm_to_rad(float rpm)
{
    return rpm * 2.0f * PI / 60.0f;
}

static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}
} // namespace

void LQR_ResetState(float u_prev_cmd_x100)
{
    (void)u_prev_cmd_x100;
}

float LQR_Compute(float theta, float theta_dot, float wheel_rpm)
{
    const float omega = rpm_to_rad(wheel_rpm);

    float current_cmd_A = (K_lqr[0] * theta + K_lqr[1] * theta_dot + K_lqr[2] * omega);

	current_cmd_A = clampf(current_cmd_A ,-max_output ,max_output);
    
    return current_cmd_A;
}
