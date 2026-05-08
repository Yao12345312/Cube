#pragma once

#define PI 3.1415926f
#define D2R 0.017453

#include <stdint.h>

extern volatile float K_lqr[3][3];

float LQR_Compute(float theta, float theta_dot, float wheel_rpm, uint8_t esc_index);

// Cascaded controller: outer angle P + inner rate PID
// Reference: open-source cubli projects (xuxiandi/cubli-firmware, EthanLawlor/RWIP)
// angle_measured: angle deviation from balance point (rad)
// rate:           angular velocity (rad/s)
// wheel_rpm:      reaction wheel speed (RPM), for damping feedforward
// axis:           [0=ESC1/roll, 1=ESC2/yaw, 2=ESC3/pitch]
float CascadedControl(float angle_measured, float rate, float wheel_rpm, uint8_t axis);
