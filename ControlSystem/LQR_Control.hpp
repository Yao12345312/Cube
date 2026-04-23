#pragma once

#define PI 3.1415926f

extern volatile float K_lqr[3]; 
	
float LQR_Compute(float theta, float theta_dot, float wheel_rpm);
void LQR_ResetState(float u_prev_cmd_x100 = 0.0f);
