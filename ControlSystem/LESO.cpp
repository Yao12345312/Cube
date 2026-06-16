#include "LESO.hpp"

// ESO带宽参数 (rad/s)，根据500Hz控制频率设定
// wo = 100 rad/s，离散化dt = 0.002s时数值稳定
#define ESO_BETA1 300.0f     // 3 * wo
#define ESO_BETA2 30000.0f   // 3 * wo^2
#define ESO_BETA3 1000000.0f // wo^3

//控制增益b0参数：这个参数越小，抗扰性能越强，太小会引入震动，实际系统控制需要调试此参数
volatile float ESO_B0 =400.0f;

//初始化角度ESO结构体，清零状态并设置增益
void ESO_Init(ESO_t *eso)
{
    eso->z1 = 0.0f;
    eso->z2 = 0.0f;
    eso->z3 = 0.0f;
    eso->b0 = ESO_B0;
    eso->beta1 = ESO_BETA1;
    eso->beta2 = ESO_BETA2;
    eso->beta3 = ESO_BETA3;
}

// 欧拉法离散化ESO状态更新
// eso: ESO实例指针
// theta: 当前角度测量值
// u: 当前控制量
// dt: 离散时间步长
void ESO_Update(ESO_t *eso, float theta, float u, float dt)
{
    // 观测误差: 实际角度 - 估计角度
    float e = theta - eso->z1;

    // z1: 角度估计更新
    eso->z1 += dt * (eso->z2 + eso->beta1 * e);

    // z2: 角速度估计更新，包含控制输入b0*u
    eso->z2 += dt * (eso->z3 + eso->b0 * u + eso->beta2 * e);

    // z3: 总扰动估计更新 (扩张状态)
    eso->z3 += dt * (eso->beta3 * e);
}


// 二阶速率ESO增益系数 (wo = 100 rad/s)
#define ESO_RATE_BETA1 200.0f    // 2 * wo
#define ESO_RATE_BETA2 10000.0f  // wo^2

// 初始化速率ESO结构体，清零状态并设置增益
void ESO_Rate_Init(ESO_Rate_t *eso)
{
    eso->z1 = 0.0f;
    eso->z2 = 0.0f;
    eso->b0 = ESO_B0;
    eso->beta1 = ESO_RATE_BETA1;
    eso->beta2 = ESO_RATE_BETA2;
}

// 二阶速率ESO状态更新 (欧拉离散化)
// rate: 实际角速率测量值 (rad/s)
// u: 当前控制量 (A)
// dt: 离散时间步长 (s)
void ESO_Rate_Update(ESO_Rate_t *eso, float rate, float u, float dt)
{
    // 观测误差: 实际角速率 - 估计角速率
    float e = rate - eso->z1;

    // z1: 角速率估计更新，包含控制输入b0*u
    eso->z1 += dt * (eso->b0 * u + eso->z2 + eso->beta1 * e);

    // z2: 总扰动估计更新 (扩张状态)
    eso->z2 += dt * (eso->beta2 * e);
}
