#pragma once

#ifdef __cplusplus
extern "C" {
#endif



// 线性扩张状态观测器状态结构体
typedef struct
{
    float z1;  // 角度估计 (rad)
    float z2;  // 角速度估计 (rad/s)
    float z3;  // 总扰动估计 (rad/s^2)

    float beta1;  // ESO增益1 = 3*wo
    float beta2;  // ESO增益2 = 3*wo^2
    float beta3;  // ESO增益3 = wo^3

    float b0;  // 控制增益 (rad/s^2 per A)
} ESO_t;

// ESO初始化，设置增益与控制增益
void ESO_Init(ESO_t *eso);

// ESO状态更新 (欧拉离散化)
// theta: 实际角度测量值 (rad)
// u: 控制输入 (A)
// dt: 控制周期 (s)，本系统为0.002s
void ESO_Update(ESO_t *eso, float theta, float u, float dt);

#ifdef __cplusplus
}
#endif
