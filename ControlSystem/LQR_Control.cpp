#include "LQR_Control.hpp"
#include <cmath>

//反馈系数 [ESC_index][0=angle, 1=rate, 2=wheel_speed]
volatile float K_lqr[3][3] = {
    {-120.0f, -13.0f, 0.00004f},  // ESC1 -> X
    {   0.0f,  13.0f, 0.00002f},  // ESC2 -> Z
    {-120.0f, -13.0f, 0.00004f},  // ESC3 -> Y
};

//前馈系数 [ESC_index][0=angle, 1=rate, 2=wheel_self, 3=wheel_cross]
volatile float K_ff[3][4] = {
    {  0.0f  },  // ESC1
    {  0.0f  },  // ESC2
    {  0.0f  },  // ESC3
};


namespace {
	
//输出限幅
static constexpr float max_output = 20.0f;
	
static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

} // namespace

float LQR_Compute(float theta, float ff_angle, float theta_dot, float wheel_rpm, uint8_t esc_index)
{	
	
    // LQR反馈回路
    float current_cmd_A = K_lqr[esc_index][0] * theta
                        + K_lqr[esc_index][1] * theta_dot
                        + K_lqr[esc_index][2] * wheel_rpm * 0.1047f;
	
    // 前馈回路
    current_cmd_A += K_ff[esc_index][0] * ff_angle;

    current_cmd_A = clampf(current_cmd_A, -max_output, max_output);

    return current_cmd_A;
}

