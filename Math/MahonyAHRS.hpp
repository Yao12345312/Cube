#pragma once
#include <cmath>

#define DEG_TO_RAD 0.01745329

class MahonyAHRS
{
public:
    MahonyAHRS(float sampleFreq = 1000.0f,
               float kp = 2.0f,
               float ki = 0.0f);

    void update(float gx, float gy, float gz,
                float ax, float ay, float az,
                float mx, float my, float mz);

    void getEuler(float &roll, float &pitch, float &yaw);

private:
    float invSqrt(float x);

    float sampleFreq;
    float twoKp;
    float twoKi;

    float q0, q1, q2, q3;
    float integralFBx, integralFBy, integralFBz;
};