#include "LQR_Control.hpp"
#include <cmath>


volatile float K_lqr[3][3] = {
    {-160.0f, -16.0f, 0.00003f},  //电调1参数-> x
	{ 0.0f,  14.0f, 0.00003f},  //电调2参数-> z
	{-160.0f, -16.0f, 0.00003f},  //电调3参数-> y
	

};


namespace {
	
//电流环输出限幅
static constexpr float max_output = 20.0f;

static inline float clampf(float v, float lo, float hi)
{
    if (v > hi) return hi;
    if (v < lo) return lo;
    return v;
}

} // namespace

float LQR_Compute(float theta, float theta_dot, float wheel_rpm, uint8_t esc_index)
{
//+  K_lqr[esc_index][2] * wheel_rpm  * 0.1047
    float current_cmd_A = (K_lqr[esc_index][0] * theta + K_lqr[esc_index][1] * theta_dot );
	//电流限幅
	current_cmd_A = clampf(current_cmd_A ,-max_output ,max_output);
    
    return current_cmd_A;
}
