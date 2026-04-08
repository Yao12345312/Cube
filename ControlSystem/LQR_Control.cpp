#include "LQR_Control.hpp"
#include <cmath>
#include <stdint.h>

// LQR 增益(MATLAB参数离线计算获得）
static float K[3] = {
    15.0f,   // k_theta
    1.5f,    // k_theta_dot
    0.02f    // k_wheel_speed
};

//rpm → rad/s
static inline float rpm_to_rad(float rpm)
{
    return rpm * 2.0f * 3.1415926f / 60.0f;
}

//LQR 一阶控制函数
float LQR_Compute(float theta, float theta_dot, float wheel_rpm)
{
    float omega = rpm_to_rad(wheel_rpm);

    // 状态向量
    float x1 = theta;
    float x2 = theta_dot;
    float x3 = omega;

    // LQR 控制律 u = -Kx
    float u = -(K[0]*x1 + K[1]*x2 + K[2]*x3);
	
	//电调输出限幅
	if (u > 1.0f) return 1.0f;
    if (u < -1.0f) return -1.0f;
    return u;

}









