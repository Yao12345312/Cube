#include "LESO.hpp"

// ESO带宽参数 (rad/s)，根据500Hz控制频率设定
// wo = 100 rad/s，离散化dt = 0.002s时数值稳定
volatile float ESO_BETA1 = 300.0f;    // 3 * wo
volatile float ESO_BETA2 = 30000.0f;  // 3 * wo^2
volatile float ESO_BETA3 = 1000000.0f; // wo^3

// 控制增益b0，近似k_m/J_p (Nm/A / kg*m^2)
//实际系统控制需要调试此参数
volatile float ESO_B0 =400.0f;

// 初始化ESO结构体，清零状态并设置增益
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
    eso->z1 += dt * (
        eso->z2
        + eso->beta1 * e);

    // z2: 角速度估计更新，包含控制输入b0*u
    eso->z2 += dt * (
        eso->z3
        + eso->b0 * u
        + eso->beta2 * e);

    // z3: 总扰动估计更新 (扩张状态)
    eso->z3 += dt * (
        eso->beta3 * e);
}
