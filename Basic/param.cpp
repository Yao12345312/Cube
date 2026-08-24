#include "param.hpp"
#include <string.h>

// =============================================================================
// 默认参数配置
// 数值来源:
//   ESO_B0/beta   -> LESO.cpp (ESO_B0=400, wo=100 => beta1=300 beta2=30000 beta3=1000000)
//   K_lqr         -> LQR_Control.cpp
//   K_ff          -> LQR_Control.cpp (全 0)
//   max_output    -> LQR_Control.cpp (20.0)
//   mech_mid      -> task_Control.cpp mechanics_medium[3]
//   arm/bias/z    -> task_Control.cpp 各控制函数内硬编码常量
// =============================================================================
Params g_params =
{
    /*eso_b0*/        400.0f,
    /*eso_wo*/        100.0f,
    /*eso_wo_rate*/   100.0f,

    /*lqr_k[3][3]*/   {
        {-120.0f, -13.0f, 0.00004f},   // ESC1 -> X轴
        {   0.0f,  13.0f, 0.00002f},   // ESC2 -> Z轴
        {-120.0f, -13.0f, 0.00004f},   // ESC3 -> Y轴
    },
    /*lqr_ff[3]*/     { 0.0f, 0.0f, 0.0f },
    /*lqr_max_out*/   20.0f,

    /*mech_mid[3]*/   { -2.35f, -2.35f, -0.64f },

    /*max_balance_angle*/ 0.20f,
    /*arm_angle_single*/  0.10f,
    /*arm_angle_point*/   0.04f,
    /*arm_rate*/          0.50f,
    /*arm_count_need*/    20.0f,

    /*bias_gain*/     0.0001f,
    /*bias_k*/        0.0002f,
    /*bias_leak*/     0.0001f,
    /*bias_limit*/    0.02f,

    /*z_axis_comp*/      0.15f,
    /*rc_yaw_gain*/      2.0f,
    /*rc_yaw_deadzone*/  0.04f,
    /*rc_cross_gain*/    0.15f,
};

volatile bool g_eso_dirty = true;   // 启动时强制重算一次 beta

// =============================================================================
// 参数注册表
//
// 顺序即 MAVLink PARAM_VALUE.param_index (0 起)。新增参数追加到表尾即可,
// 地面站通过 PARAM_REQUEST_LIST 自动发现全表, 无需改 GCS 代码。
// =============================================================================
const ParamEntry g_param_table[] =
{
    // ---- ESO ----
    {"eso_b0",      &g_params.eso_b0,           PARAM_TYPE_REAL32, false},
    {"eso_wo",      &g_params.eso_wo,           PARAM_TYPE_REAL32, false},
    {"eso_worate",  &g_params.eso_wo_rate,      PARAM_TYPE_REAL32, false},

    // ---- LQR 反馈增益 [ESC][0=角度 1=角速度 2=轮速] ----
    {"lqr_k00",     &g_params.lqr_k[0][0],      PARAM_TYPE_REAL32, false},
    {"lqr_k01",     &g_params.lqr_k[0][1],      PARAM_TYPE_REAL32, false},
    {"lqr_k02",     &g_params.lqr_k[0][2],      PARAM_TYPE_REAL32, false},
    {"lqr_k10",     &g_params.lqr_k[1][0],      PARAM_TYPE_REAL32, false},
    {"lqr_k11",     &g_params.lqr_k[1][1],      PARAM_TYPE_REAL32, false},
    {"lqr_k12",     &g_params.lqr_k[1][2],      PARAM_TYPE_REAL32, false},
    {"lqr_k20",     &g_params.lqr_k[2][0],      PARAM_TYPE_REAL32, false},
    {"lqr_k21",     &g_params.lqr_k[2][1],      PARAM_TYPE_REAL32, false},
    {"lqr_k22",     &g_params.lqr_k[2][2],      PARAM_TYPE_REAL32, false},

    // ---- LQR 前馈 / 限幅 ----
    {"lqr_ff0",     &g_params.lqr_ff[0],        PARAM_TYPE_REAL32, false},
    {"lqr_ff1",     &g_params.lqr_ff[1],        PARAM_TYPE_REAL32, false},
    {"lqr_ff2",     &g_params.lqr_ff[2],        PARAM_TYPE_REAL32, false},
    {"lqr_maxout",  &g_params.lqr_max_out,      PARAM_TYPE_REAL32, false},

    // ---- 机械中值 ----
    {"mech_mid0",   &g_params.mech_mid[0],      PARAM_TYPE_REAL32, false},
    {"mech_mid1",   &g_params.mech_mid[1],      PARAM_TYPE_REAL32, false},
    {"mech_mid2",   &g_params.mech_mid[2],      PARAM_TYPE_REAL32, false},

    // ---- 保护 / 解锁 ----
    {"max_bal_ang", &g_params.max_balance_angle, PARAM_TYPE_REAL32, false},
    {"arm_ang_s",   &g_params.arm_angle_single,  PARAM_TYPE_REAL32, false},
    {"arm_ang_p",   &g_params.arm_angle_point,   PARAM_TYPE_REAL32, false},
    {"arm_rate",    &g_params.arm_rate,          PARAM_TYPE_REAL32, false},
    {"arm_cnt",     &g_params.arm_count_need,    PARAM_TYPE_REAL32, false},

    // ---- 重心自适应 ----
    {"bias_gain",   &g_params.bias_gain,         PARAM_TYPE_REAL32, false},
    {"bias_k",      &g_params.bias_k,            PARAM_TYPE_REAL32, false},
    {"bias_leak",   &g_params.bias_leak,         PARAM_TYPE_REAL32, false},
    {"bias_limit",  &g_params.bias_limit,        PARAM_TYPE_REAL32, false},

    // ---- 前馈 / 遥控补偿 ----
    {"z_axis_comp", &g_params.z_axis_comp,       PARAM_TYPE_REAL32, false},
    {"rc_yaw_gain", &g_params.rc_yaw_gain,       PARAM_TYPE_REAL32, false},
    {"rc_yaw_dz",   &g_params.rc_yaw_deadzone,   PARAM_TYPE_REAL32, false},
    {"rc_cross",    &g_params.rc_cross_gain,     PARAM_TYPE_REAL32, false},
};

const uint16_t g_param_count = (uint16_t)(sizeof(g_param_table) / sizeof(g_param_table[0]));

uint16_t param_count(void)
{
    return g_param_count;
}

int16_t param_find(const char *name)
{
    if (!name)
        return -1;
    for (uint16_t i = 0; i < g_param_count; i++)
    {
        // name 最长 16 字符, strncmp 比较 16 字符即可 (不足部分表内为 '\0')
        if (strncmp(g_param_table[i].name, name, 16) == 0)
            return (int16_t)i;
    }
    return -1;
}

float param_get(uint16_t index)
{
    if (index >= g_param_count)
        return 0.0f;
    return *g_param_table[index].ptr;
}

bool param_set(uint16_t index, float value)
{
    if (index >= g_param_count)
        return false;
    if (g_param_table[index].readonly)
        return false;

    *g_param_table[index].ptr = value;

    // ESO 带宽变更 -> 置 dirty, LESO 下次 update() 重算 beta
    float *p = g_param_table[index].ptr;
    if (p == &g_params.eso_wo || p == &g_params.eso_wo_rate)
        g_eso_dirty = true;

    return true;
}
