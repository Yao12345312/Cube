#include "LQR_Control.hpp"
#include <cmath>


volatile float K_lqr[3][3] = {
    {-130.0f, -14.0f, 0.00004f},  //电调1参数-> x
	{   0.0f,  12.0f, 0.00002f},  //电调2参数-> z 
	{-130.0f, -14.0f, 0.00004f},  //电调3参数-> y
	

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
    float current_cmd_A = (K_lqr[esc_index][0] * theta + K_lqr[esc_index][1] * theta_dot +  K_lqr[esc_index][2] * wheel_rpm  * 0.1047);
    current_cmd_A = clampf(current_cmd_A ,-max_output ,max_output);

    return current_cmd_A;
}

// ============================================================
// 级联控制器：外环角度P + 内环角速度PID
// ============================================================

// 控制器增益（可通过实验调整）
// [0]=ESC1/roll, [1]=ESC2/yaw, [2]=ESC3/pitch
static const float g_K_ang[3]   = {-10.0f, 0.0f, -10.0f};  // 角度P增益 (yaw=0:无角度环)
static const float g_Kp_rate[3] = {-16.0f, -20.0f, -16.0f}; // 角速度P增益
static const float g_Ki_rate[3] = {-3.0f,  -6.0f,  -3.0f};  // 角速度I增益 (yaw更大以消除自旋)
static const float g_Kd_rate[3] = {-0.5f,  -0.3f,  -0.5f};  // 角速度D增益
static const float g_K_wheel[3] = {0.005f, 0.003f, 0.005f}; // 飞轮转速前馈

static const float RATE_INTEGRAL_LIMIT = 10.0f; // 积分抗饱和
static const float CONTROL_DT = 0.005f;          // 控制周期 (200Hz)

struct RatePID {
	float integral;
	float prev_error;
};
static RatePID g_rate_pid[3] = {0};

float CascadedControl(float angle_measured, float rate, float wheel_rpm, uint8_t axis)
{
	if (axis > 2) return 0.0f;

	// ---- 外环：角度P -> 角速度设定点 ----
	// angle_measured = (平衡点角度 - 当前角度), 即 ref=0 时的误差
	float rate_sp = g_K_ang[axis] * (0.0f - angle_measured);

	// ---- 内环：角速度PID ----
	float rate_error = rate_sp - rate;

	RatePID &st = g_rate_pid[axis];
	st.integral += rate_error * CONTROL_DT;
	if (st.integral >  RATE_INTEGRAL_LIMIT) st.integral =  RATE_INTEGRAL_LIMIT;
	if (st.integral < -RATE_INTEGRAL_LIMIT) st.integral = -RATE_INTEGRAL_LIMIT;

	float derivative = (rate_error - st.prev_error) / CONTROL_DT;
	st.prev_error = rate_error;

	float out = g_Kp_rate[axis] * rate_error
	          + g_Ki_rate[axis] * st.integral
	          + g_Kd_rate[axis] * derivative
	          + g_K_wheel[axis] * wheel_rpm;

	return clampf(out, -max_output, max_output);
}