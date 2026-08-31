#pragma once

#include <stdint.h>

// =============================================================================
// 参数类型 (与 MAVLink MAV_PARAM_TYPE 对齐, 这里本地定义避免 Basic 反向依赖
// Drivers/middleWare/MAVlink)
// =============================================================================
#define PARAM_TYPE_UINT8   1
#define PARAM_TYPE_INT8    2
#define PARAM_TYPE_UINT16  3
#define PARAM_TYPE_INT16   4
#define PARAM_TYPE_UINT32  5
#define PARAM_TYPE_INT32   6
#define PARAM_TYPE_REAL32  9   // MAV_PARAM_TYPE_REAL32, 本表全部使用 float

// =============================================================================
// 全局参数存储
//
// 所有控制器/观测器参数集中在此结构, 默认值在 param.cpp。
// 控制环热路径直接读字段 (单 float 读在 Cortex-M7 上原子), MAVLink 在线修改后
// 下一个控制周期立即生效。
// =============================================================================
struct Params
{
    // ---- ESO (线性扩张状态观测器) ----
    float eso_b0;          // 控制有效性 (rad/s^2 per A)  [参考: 400.0]
    float eso_wo;          // 角度ESO带宽 wo (rad/s), beta1=3wo beta2=3wo^2 beta3=wo^3
    float eso_wo_rate;     // 角速度ESO带宽 wo (rad/s), beta1=2wo beta2=wo^2

    // ---- LQR (3路电调: ESC1=X轴, ESC2=Z轴, ESC3=Y轴) ----
    float lqr_k[3][3];     // 反馈增益 [ESC][角度, 角速度, 轮速]
    float lqr_ff[3];       // 前馈角度增益 [ESC] (对应参考 K_ff[esc][0])
    float lqr_max_out;     // 电流指令限幅 (A)

    // ---- 机械中值 (弧度, 3轴) ----
    float mech_mid[3];     // {单边X轴, 单点X轴, 单点Y轴}

    // ---- 保护 / 解锁 ----
    float max_balance_angle; // 失衡保护角度 (rad), 超出强制 disarm
    float arm_angle_single;  // 单边解锁角度阈值 (rad)
    float arm_angle_point;   // 单点解锁角度阈值 (rad)
    float arm_rate;          // 起跳落地切平衡的角速度阈值 (rad/s), 需同时满足才预解锁
    float arm_count_need;    // 解锁需持续的周期数 (500Hz)

    // ---- 重心自适应 ----
    float bias_gain;     // 单边: 稳态重心积分增益
    float bias_k;        // 单点: 重心学习率
    float bias_leak;     // 单点: 泄漏系数
    float bias_limit;    // 单点: 积分限幅

    // ---- 前馈 / 遥控补偿 ----
    float z_axis_comp;      // Z轴共模补偿电流 (A), 同时叠加到3路输出
    float rc_yaw_gain;      // 遥控偏航指令增益
    float rc_yaw_deadzone;  // 遥控死区
    float rc_cross_gain;    // X/Y轮偏航交叉耦合系数

    // ---- 起跳 (单边/单点共用) ----
    float jump_flat_roll;   // 平放检测 roll 角 (rad), 平放稳定后触发起跳
    float jump_spin_cur;    // 储能电流幅值 (A), 决定储能快慢
    float jump_kick_cur;    // 起跳电流幅值 (A), 即起跳力度
    float jump_spin_rpm;    // 储能目标转速 (rpm), 需低于转速保护阈值
    float jump_spin_ms;     // 储能阶段超时 (ms), 超时直接进入起跳
    float jump_kick_ms;     // 反向加速时长 (ms), 决定反力矩冲量
    float jump_settle_ms;   // 起跳后最短腾空时间 (ms), 之后角度回到平衡角附近才切入平衡
    float jump_arm_ang;     // 起跳等待/切入平衡的姿态阈值 (rad, 相对机械中值)

    // ---- 电机方向标定 [ESC1, ESC2, ESC3] ----
    float esc_dir_sign[3];  // 正电流指令电机转向: 逆时针=1, 顺时针=-1
};

extern Params g_params;

// 编译期出厂默认值 (与 g_params 初始值同源, 恢复出厂参数用)
extern const Params k_param_defaults;

// 参数脏标志: param_set() 置位, 通信任务固化到 Flash 成功后清除
extern volatile bool g_params_dirty;

// =============================================================================
// 参数注册表
//
// 每个字段对应一条 entry, name 即 MAVLink param_id (<=16 字符, 不含 '\0')。
// 新增参数只需在 g_param_table 追加条目 -> 地面站自动发现, 无需改 GCS 代码。
// =============================================================================
struct ParamEntry
{
    const char name[16];   // 参数名 (不足16字符以 '\0' 填充)
    float     *ptr;        // 指向 g_params 内字段
    uint8_t    type;       // PARAM_TYPE_*
    bool       readonly;   // true = 仅广播不可在线写
};

extern const ParamEntry g_param_table[];
extern const uint16_t   g_param_count;

// =============================================================================
// 查询/修改接口 (线程安全: 控制环只读 g_params, 通信环通过本接口写)
// =============================================================================
uint16_t param_count(void);
int16_t  param_find(const char *name);         // 返回索引, -1 = 未找到
float    param_get(uint16_t index);            // 越界返回 0
bool     param_set(uint16_t index, float value);   // 写入并触发派生标志, 成功 true

// ESO 带宽变更标志: param_set 改 eso_wo / eso_wo_rate 后置位,
// LESO 在下次 update() 重算 beta (避免控制热路径做乘方运算)
extern volatile bool g_eso_dirty;
