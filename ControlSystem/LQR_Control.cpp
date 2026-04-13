#include "LQR_Control.hpp"
#include <cmath>
#include <stdint.h>

// LQR 增益(MATLAB参数离线计算获得）
static float K[3] = {
    -74739.0f,   // k_theta
    -12162.0f,    // k_theta_dot
    -3.8280f    // k_wheel_speed
};

//rpm → rad/s
static inline float rpm_to_rad(float rpm)
{
    return rpm * 2.0f * PI / 60.0f;
}

static inline float rad_to_rpm(float rad)
{
    return rad * 60.0f/(2.0f * PI);
}

//LQR 一阶控制函数
float LQR_Compute(float theta, float theta_dot, float wheel_rpm)
{
    float omega = rpm_to_rad(wheel_rpm);

    // 状态向量
    float x1 = theta; //rad
    float x2 = theta_dot;	//rad/s
    float x3 = omega;	//rad/s

    // LQR 控制律 u = -Kx，输出电机电流
    float omega_target = (K[0]*x1 + K[1]*x2 + K[2]*x3);
	
	int32_t rpm_target = static_cast<int32_t> (rad_to_rpm(omega_target));
	
    return rpm_target;

}









