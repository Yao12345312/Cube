#include "MahonyAHRS.hpp"
#include "uart3Driver.hpp"
//构造函数
MahonyAHRS::MahonyAHRS(float freq, float kp, float ki)
{
    sampleFreq = freq;
    twoKp = 2.0f * kp;
    twoKi = 2.0f * ki;
	
    q0 = 1.0f;
    q1 = q2 = q3 = 0.0f;

    integralFBx = integralFBy = integralFBz = 0.0f;
}

//快速平方根倒数
float MahonyAHRS::invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void MahonyAHRS::update(float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)
{
    float recipNorm;
    float hx, hy, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float ex, ey, ez;
	
	const float magWeight = 0.005f;  // 磁力计降低到0.5%的权重
    // 归一化加速度
    recipNorm = invSqrt(ax*ax + ay*ay + az*az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 归一化磁力计
    recipNorm = invSqrt(mx*mx + my*my + mz*mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // 参考方向计算
    hx = 2.0f * (mx*(0.5f - q2*q2 - q3*q3) + my*(q1*q2 - q0*q3) + mz*(q1*q3 + q0*q2));
    hy = 2.0f * (mx*(q1*q2 + q0*q3) + my*(0.5f - q1*q1 - q3*q3) + mz*(q2*q3 - q0*q1));
    bx = sqrtf(hx*hx + hy*hy);
    bz = 2.0f * (mx*(q1*q3 - q0*q2) + my*(q2*q3 + q0*q1) + mz*(0.5f - q1*q1 - q2*q2));

    // 估计重力
    vx = 2.0f*(q1*q3 - q0*q2);
    vy = 2.0f*(q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    
    // 预测磁场方向
	wx = 2.0f*bx*(0.5f - q2*q2 - q3*q3) + 2.0f*bz*(q1*q3 - q0*q2);
	wy = 2.0f*bx*(q1*q2 - q0*q3) + 2.0f*bz*(q0*q1 + q2*q3);
	wz = 2.0f*bx*(q0*q2 + q1*q3) + 2.0f*bz*(0.5f - q1*q1 - q2*q2);

	// 误差 (加速度 + 磁力计)
	ex = (ay*vz - az*vy)+  magWeight*(my*wz - mz*wy);
	ey = (az*vx - ax*vz)+  magWeight*(mz*wx - mx*wz);
	ez = (ax*vy - ay*vx)+  magWeight*(mx*wy - my*wx);
	
	//if(fabsf(ez)>0.2) ez = (ax*vy - ay*vx); //误差较大时，忽略磁力计
		
    // 积分项
    integralFBx += twoKi * ex * (1.0f / sampleFreq);
    integralFBy += twoKi * ey * (1.0f / sampleFreq);
    integralFBz += twoKi * ez * (1.0f / sampleFreq);

    gx += twoKp * ex + integralFBx;
    gy += twoKp * ey + integralFBy;
    gz += twoKp * ez + integralFBz;

    gx *= (0.5f / sampleFreq);
    gy *= (0.5f / sampleFreq);
    gz *= (0.5f / sampleFreq);

    float qa = q0;
    float qb = q1;
    float qc = q2;

    q0 += (-qb*gx - qc*gy - q3*gz);
    q1 += (qa*gx + qc*gz - q3*gy);
    q2 += (qa*gy - qb*gz + q3*gx);
    q3 += (qa*gz + qb*gy - qc*gx);

    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
	
	//printf("%.4f,%.4f,%.4f\n",mx,my,ez);
}

void MahonyAHRS::getEuler(float &roll, float &pitch, float &yaw)
{
    roll  = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    pitch = asinf(2*(q0*q2 - q3*q1));
    yaw   = atan2f(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    roll  *= 57.29578f;
    pitch *= 57.29578f;
    yaw   *= 57.29578f;
}