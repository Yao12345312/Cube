#pragma once

#define PI 3.1415926f
#define D2R 0.017453

#include <stdint.h>

extern volatile float K_lqr[3][3];

float LQR_Compute(float theta, float theta_dot, float wheel_rpm, uint8_t esc_index);
