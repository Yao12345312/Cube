#pragma once

#define PI 3.1415926f
#define D2R 0.017453

#include <stdint.h>

extern volatile float K_lqr[3][3];

// Feedforward trim coefficients: [ESC][0=angle, 1=rate, 2=wheel_self, 3=wheel_cross]
extern volatile float K_ff[3][4];

// LQR feedback + feedforward trim, clamped to ±max_output
// ff_* params are the raw sensor values multiplied by K_ff internally
float LQR_Compute(float theta, float ff_angle, float theta_dot, float wheel_rpm, uint8_t esc_index);


