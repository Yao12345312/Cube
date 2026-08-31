#include "task_Control.hpp"
#include "sensors/drv_BMI088.hpp"
#include "drv_ina226.hpp"
#include "drv_CubeFOC.hpp"
#include "drv_buzzer.hpp"
#include "MahonyAHRS.hpp"
#include "LESO.hpp"
#include "LQR.hpp"
#include "param.hpp"

#include "stm32h743xx.h"   // DWT / SystemCoreClock

#include <cmath>

// =============================================================================
// 全局共享对象 (声明于 task_Control.hpp, 控制任务创建, 通信/显示任务消费)
// =============================================================================
osMessageQueueId_t g_mavSensorQueue  = NULL;
osMessageQueueId_t g_dispSensorQueue = NULL;
osSemaphoreId_t    g_controlModeSem  = NULL;

volatile uint8_t g_rc_control_mode = 0xFF;   // 上电默认停机
volatile float   g_rc_speed_target = 0.0f;
volatile float   g_rc_manual_y     = 0.0f;

// 菜单本地选择的控制模式 (0xFF=未选择)
// 0=单边 1=单点 2=单边起跳 3=单点起跳 4=转速测试
uint8_t g_selected_control_mode = 0xFF;

// 电调调准页激活标志: true 期间空闲分支暂停零电流广播
volatile bool g_esc_calib_menu_active = false;

// 共享姿态 (供显示菜单读取)
Attitude_t  g_attitude = {0};
osMutexId_t g_att_mutex = NULL;

// 校准同步
osSemaphoreId_t g_calibSem = NULL;
uint8_t         g_calib_command = 0;     // 0=idle, 1=陀螺仪零偏
bool            g_gyro_calibrated = false;

namespace {
	// 最短角差 (处理 ±π 翻转)
	static inline float AngleDiffRad(float a, float b)
	{
		return atan2f(sinf(a - b), cosf(a - b));
	}
	
	// =========================================================================
	// 起跳状态机 (第一跳: 平放 -> 单边)
	// 先反向大电流把飞轮加速到高转速储存动量, 再反转大电流,
	// 反力矩瞬间作用于立方体本体实现起跳, 轮子减速由反转电流完成
	// =========================================================================
	enum JumpPhase : uint8_t
	{
		JUMP_IDLE     = 0,   // 等待立方体平放 (roll≈jump_flat_roll, pitch≈0), 稳定后自动触发起跳
		JUMP_SPIN     = 1,   // 反向大电流储存动量
		JUMP_KICK     = 2,   // 反转大电流, 反力矩起跳 (平放 -> 单边)
		JUMP_SETTLE   = 3,   // 腾空等待, 落地角度+角速度回平衡点附近, 预解锁切入平衡
		JUMP_PREP2    = 4,   // 单边平衡保持
		JUMP_SPIN2    = 5,   // ESC2/ESC3 储能, ESC1 保持单边平衡
		JUMP_KICK2    = 6,   // ESC2/ESC3 反力矩翻转到单点姿态
		JUMP_SETTLE2  = 7,   // 等待角度接近 mid_x / mid_y
		JUMP_BALANCE  = 8,   // 到达平衡点运行平衡控制函数
	};

	struct JumpState
	{
		uint8_t  phase;
		uint32_t timer;    // 当前相位已持续控制周期数 (500Hz -> 2ms/周期)
	};

	// 模式2/3互斥, 共用一个状态机
	static JumpState g_jump_state = {JUMP_IDLE, 0};

	static inline void JumpStateReset()
	{
		g_jump_state.phase = JUMP_IDLE;
		g_jump_state.timer = 0;
	}

	// 平衡观测器 (文件级共享, 起跳落地切入平衡时复位, 避免陈旧状态引起补偿瞬跳)
	static LESO      g_eso_side;   // 单边平衡 X轴
	static LESO      g_eso_pt_x;   // 单点平衡 X轴
	static LESO      g_eso_pt_y;   // 单点平衡 Y轴
	static LESO_Rate g_eso_pt_z;   // 单点平衡 Z轴角速度
} // namespace

// =============================================================================
// 电机方向映射
// 标定基准统一为: 三个电调正电流指令均为逆时针
//   电机实际转向 逆时针 -> esc_dir_sign 填 1, 顺时针 -> 填 -1
//   (参数 esc_dir0/1/2, 上位机可动态修改)
// 当前: ESC1(顺时针) ESC2(逆时针) ESC3(逆时针)
//
// kEscOldSign: 控制律内部符号沿用旧结构约定 (ESC1逆 ESC2逆 ESC3顺),
// 记录旧结构相对统一基准(全逆时针)的符号差, 固定值勿动
// =============================================================================
static const int32_t kEscOldSign[3] = { 1, 1, -1 };   // [ESC1, ESC2, ESC3]

static void send_esc_cmds(CubeFOC *esc, const int32_t *cmd, uint8_t len)
{
    int32_t out[3] = {0, 0, 0};
    for (uint8_t i = 0; i < len && i < 3; i++)
        out[i] = cmd[i] * (int32_t)g_params.esc_dir_sign[i] * kEscOldSign[i];
    esc->send_esc_current_commands(out, len);
}

// =============================================================================
// 单边平衡控制 (仅 ESC1 / X轴)
// =============================================================================
static void SingleSideBalanceControl(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    uint32_t &adjust_counter,
    bool &control_armed,
    LQR &lqr)
{
    auto *buzzer = drv_buzzer();

    // 自适应重心变量
    static float theta_bias = 0.0f;

    // 温度保护
    if (esc_status[ESC1_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC2_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC3_Index].temperature > ESC_MAX_Temperature)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        esc_current_cmd[0] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 转速保护
    if (esc_status[ESC1_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC2_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC3_Index].rpm > ESC_MAX_SpeedRPM)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        esc_current_cmd[0] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 未校准则持续发校准指令
    if (!esc_status[ESC1_Index].calib_flag)
    {
        esc_current_cmd[0] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
            esc->calib_esc_command((uint8_t)(ESC1_Index + 1));
    }
    else
    {
        calib_counter = 0;

        // 未解锁
        if (!control_armed)
        {
            if (fabsf(theta[0]) < g_params.arm_angle_single)
                arm_counter++;
            else
                arm_counter = 0;

            if (arm_counter >= (uint32_t)g_params.arm_count_need)
            {
                if (buzzer) buzzer->beep(500, 200);
                control_armed = true;
                g_eso_side.reset(theta[0]);
            }

            esc_current_cmd[0] = 0;
        }
        // 角度超过可控范围, 上锁
        else if (fabsf(theta[0]) > g_params.max_balance_angle)
        {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else
        {
            // 稳定状态下积分消除重心偏差
            if (fabsf(theta[0]) < 0.05f &&
                fabsf(theta_dot[0]) < 0.1f &&
                fabsf((float)wheel_speed[0]) < 800.0f)
            {
                theta_bias += g_params.bias_gain * theta[0];
            }

            // 速度反馈
            float theta_corr = theta[0] - g_params.lqr_k[0][2] * wheel_speed[0];

            float u = lqr.compute(theta_corr, theta[0], theta_dot[0], wheel_speed[0], ESC1_Index);

            // ESO 状态更新与扰动补偿
            g_eso_side.update(theta[0], u, 0.002f);
            float u_comp = u - g_eso_side.z3() / g_params.eso_b0;

            esc_current_cmd[0] = (int32_t)(u_comp * 1000.0f);
        }
        // 发送电调油门指令
        send_esc_cmds(esc, esc_current_cmd, 3);
    }
}

// =============================================================================
// 单点平衡控制 (ESC1=X轴, ESC2=Z轴, ESC3=Y轴)
// =============================================================================
static void SinglePointBalanceControl(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    float yaw,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    uint32_t &adjust_counter,
    bool &control_armed,
    LQR &lqr)
{
    auto *buzzer = drv_buzzer();

    // X/Y轴角度偏差积分
    static float theta_bias_x = 0.0f;
    static float theta_bias_y = 0.0f;

    // 温度保护
    if (esc_status[ESC1_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC2_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC3_Index].temperature > ESC_MAX_Temperature)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 转速保护
    if (esc_status[ESC1_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC2_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC3_Index].rpm > ESC_MAX_SpeedRPM)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        esc_current_cmd[0] = 0;
		esc_current_cmd[1] = 0;
		esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 电机未校准, 停止输出并校准
    if (!(esc_status[ESC1_Index].calib_flag &&
          esc_status[ESC2_Index].calib_flag &&
          esc_status[ESC3_Index].calib_flag))
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
        {
            const uint8_t esc_id = (uint8_t)((calib_counter / 20U) % 3U);
            esc->calib_esc_command((uint8_t)(esc_id + 1U));
        }
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;

    // 到平衡点附近解锁控制
    if (!control_armed)
    {
        if (fabsf(theta[1]) < g_params.arm_angle_point &&
            fabsf(theta[2]) < g_params.arm_angle_point)
            arm_counter++;
        else
            arm_counter = 0;

        if (arm_counter >= (uint32_t)g_params.arm_count_need)
        {
            if (buzzer) buzzer->beep(500, 200);
            control_armed = true;
            g_eso_pt_x.reset(theta[1]);
            g_eso_pt_y.reset(theta[2]);
            g_eso_pt_z.reset(theta_dot[2]);
        }
        esc_current_cmd[0] = 0;
		esc_current_cmd[1] = 0;
		esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 超过可控角度限制, 上锁
    if (fabsf(theta[1]) > g_params.max_balance_angle ||
        fabsf(theta[2]) > g_params.max_balance_angle)
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;

        adjust_counter = 0;
        theta_bias_x = 0.0f;
        theta_bias_y = 0.0f;

        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 获取角度误差
    const float theta_x = theta[1]; // - theta_bias_x;
    const float theta_y = theta[2]; // - theta_bias_y;

    const float theta_corr_x =  theta_x - g_params.lqr_k[ESC1_Index][2] * wheel_speed[0] - g_params.lqr_k[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_y = -theta_y - g_params.lqr_k[ESC3_Index][2] * wheel_speed[2] - g_params.lqr_k[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_z = -g_params.lqr_k[ESC2_Index][2] * wheel_speed[1];

    // 遥控器通道输入值
    float rc_yaw = g_rc_manual_y;
    if (fabsf(rc_yaw) < g_params.rc_yaw_deadzone) rc_yaw = 0.0f;
    const float rc_yaw_cmd = rc_yaw * g_params.rc_yaw_gain;

    // LQR 控制器输出
    float out_x = lqr.compute(theta_corr_x, (theta[1] - g_params.mech_mid[1]), theta_dot[0], wheel_speed[0], ESC1_Index);
    float out_z = lqr.compute(theta_corr_z, 0.0f, theta_dot[2], wheel_speed[1], ESC2_Index) + rc_yaw_cmd;
    float out_y = lqr.compute(theta_corr_y, -(theta[2] - g_params.mech_mid[2]), theta_dot[1], wheel_speed[2], ESC3_Index);

    // 遥控器输入值缩放后叠加到输出
    out_x += g_params.rc_cross_gain * rc_yaw_cmd;
    out_y -= g_params.rc_cross_gain * rc_yaw_cmd;

    // ESO 状态更新
    g_eso_pt_x.update(theta[1], out_x, 0.002f);
    g_eso_pt_y.update(theta[2], out_y, 0.002f);

    // Z轴速率ESO: 以theta_dot[2]为观测量, 仅补偿yaw速率扰动
    g_eso_pt_z.update(theta_dot[2], out_z, 0.002f);

    // 扰动补偿项
    const float comp_x = g_eso_pt_x.z3() / g_params.eso_b0;
    const float comp_y = g_eso_pt_y.z3() / g_params.eso_b0;
    const float comp_z = g_eso_pt_z.z2() / g_params.eso_b0;

    // 解锁后接入控制
    if (control_armed)
    {
        esc_current_cmd[0] = (int32_t)((out_x + g_params.z_axis_comp - comp_x) * 1000.0f);
        esc_current_cmd[1] = (int32_t)((out_z + g_params.z_axis_comp - comp_z) * 1000.0f);
        esc_current_cmd[2] = (int32_t)((out_y + g_params.z_axis_comp - comp_y) * 1000.0f);
    }

    send_esc_cmds(esc, esc_current_cmd, 3);
}

// =============================================================================
// 单边起跳序列步进 (IDLE→SPIN→KICK→SETTLE), 模式2与模式3第一跳共用
// IDLE: 等待立方体平放 (roll≈jump_flat_roll, pitch≈0), 平放稳定后自动触发起跳
// SPIN: ESC1 反向大电流储存动量 -> KICK: ESC1 反转大电流产生反力矩起跳 (平放 -> 单边)
// SETTLE: 落地角度+角速度回平衡点附近后预解锁转入 settle_next, 立即接入平衡
//   模式2 (单边起跳) -> JUMP_BALANCE (单边平衡)
//   模式3 (单点起跳) -> JUMP_PREP2  (第二跳准备)
// 无主动刹车结构: 轮子减速由反转起跳电流完成
// =============================================================================
static void SideJumpSequenceStep(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    float roll,
    float pitch,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    uint32_t &adjust_counter,
    bool &control_armed,
    LQR &lqr,
    uint8_t settle_next)
{
    auto *buzzer = drv_buzzer();

    switch (g_jump_state.phase)
    {
    // 前提状态: 等待立方体平放 (roll≈jump_flat_roll, pitch≈0), 稳定后自动触发第一跳
    case JUMP_IDLE:
    {
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;

        if (fabsf(AngleDiffRad(roll, g_params.jump_flat_roll)) < g_params.jump_arm_ang &&
            fabsf(pitch) < g_params.jump_arm_ang)
            g_jump_state.timer++;
        else
            g_jump_state.timer = 0;

        if (g_jump_state.timer >= (uint32_t)g_params.arm_count_need)
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_SPIN;
            if (buzzer) buzzer->beep(300, 100);
        }

        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 第一跳: ESC1 反向大电流把飞轮加速到目标转速储存动量
    case JUMP_SPIN:
    {
        esc_current_cmd[0] = -(int32_t)(g_params.jump_spin_cur * 1000.0f);
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;

        if (wheel_speed[0] <= -(int32_t)g_params.jump_spin_rpm ||
            g_jump_state.timer >= (uint32_t)(g_params.jump_spin_ms * 0.5f))
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_KICK;
        }
        break;
    }

    // 第一跳: ESC1 反转大电流加速, 反力矩使立方体绕棱边翻起 (平放 -> 单边)
    case JUMP_KICK:
    {
        esc_current_cmd[0] = (int32_t)(g_params.jump_kick_cur * 1000.0f);
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;

        if (g_jump_state.timer >= (uint32_t)(g_params.jump_kick_ms * 0.5f))
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_SETTLE;
        }
        break;
    }

    // 第一跳腾空输出清零; 角度回 mid_x 附近且角速度充分衰减 (已落地站稳,
    // 而非空中掠过平衡点) 时预解锁并转入 settle_next, 立即接入平衡避免过冲
    case JUMP_SETTLE:
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;

        if (g_jump_state.timer >= (uint32_t)(g_params.jump_settle_ms * 0.5f) &&
            fabsf(theta[0]) < g_params.jump_arm_ang &&
            fabsf(theta_dot[0]) < g_params.arm_rate)
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = settle_next;
            control_armed = true;      // 角度/角速度已双重确认, 跳过解锁计数
            arm_counter = 0;
            g_eso_side.reset(theta[0]);
        }
        break;
    }

    default:
        return;
    }

    g_jump_state.timer++;
    send_esc_cmds(esc, esc_current_cmd, 3);
}

// =============================================================================
// 单边起跳控制 (模式2, 仅 ESC1 / X轴, 只包含单边起跳本身, 一次操作独立完成)
// 流程: 平放检测 (roll≈jump_flat_roll, pitch≈0) -> ESC1 反向储能 -> ESC1 反转反力矩起跳
//       -> 落地角度+角速度回 mid_x 附近预解锁切入单边平衡, 此后一直运行单边平衡控制
// =============================================================================
static void SingleSideJumpControl(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    float roll,
    float pitch,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    uint32_t &adjust_counter,
    bool &control_armed,
    LQR &lqr)
{
    // 温度 / 转速保护
    if (esc_status[ESC1_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC2_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC3_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC1_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC2_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC3_Index].rpm > ESC_MAX_SpeedRPM)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        JumpStateReset();
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 未校准
    if (!esc_status[ESC1_Index].calib_flag)
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;
        JumpStateReset();

        if ((calib_counter++ % 20U) == 0U)
            esc->calib_esc_command((uint8_t)(ESC1_Index + 1));
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;

    // 第一跳: 单边起跳序列 (平放 -> 单边)
    if (g_jump_state.phase <= JUMP_SETTLE)
    {
        SideJumpSequenceStep(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, roll, pitch, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr, JUMP_BALANCE);
        return;
    }

    // 已切入平衡: 此后一直运行单边平衡控制, 不再返回起跳流程
    SingleSideBalanceControl(
        esc, esc_status, theta, theta_dot, wheel_speed,
        esc_current_cmd, calib_counter, arm_counter,
        adjust_counter, control_armed, lqr);
}

// =============================================================================
// 单点起跳控制 (模式3, 一次操作独立完成: 第一跳单边起跳 + 第二跳单点起跳)
// [第一跳] 平放检测 (roll≈jump_flat_roll, pitch≈0) -> ESC1 反向储能 -> ESC1 反转反力矩起跳
//          -> 落地角度+角速度回 mid_x 附近, 预解锁重建单边平衡
// [第二跳] 重建单边平衡 (roll=mid_x), 稳定后 ESC2/ESC3 储能 (ESC1 持续稳住滚转)
//          -> ESC2/ESC3 反力矩绕棱边翻转到单点姿态 -> 角度接近 mid_x / mid_y 切入单点平衡
// 此后一直运行单点平衡控制, 不再返回起跳流程
// =============================================================================
static void SinglePointJumpControl(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    float roll,
    float pitch,
    float yaw,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    uint32_t &adjust_counter,
    bool &control_armed,
    LQR &lqr)
{
    auto *buzzer = drv_buzzer();

    // 温度 / 转速保护
    if (esc_status[ESC1_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC2_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC3_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC1_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC2_Index].rpm > ESC_MAX_SpeedRPM ||
        esc_status[ESC3_Index].rpm > ESC_MAX_SpeedRPM)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        JumpStateReset();
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 未校准
    if (!(esc_status[ESC1_Index].calib_flag &&
          esc_status[ESC2_Index].calib_flag &&
          esc_status[ESC3_Index].calib_flag))
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;
        JumpStateReset();

        if ((calib_counter++ % 20U) == 0U)
        {
            const uint8_t esc_id = (uint8_t)((calib_counter / 20U) % 3U);
            esc->calib_esc_command((uint8_t)(esc_id + 1U));
        }
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;

    // 第一跳: 单边起跳序列 (平放 -> 单边), 落地后进入第二跳准备
    if (g_jump_state.phase <= JUMP_SETTLE)
    {
        SideJumpSequenceStep(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, roll, pitch, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr, JUMP_PREP2);
        return;
    }

    const int32_t spin_rpm = (int32_t)g_params.jump_spin_rpm;

    switch (g_jump_state.phase)
    {
    // 第一跳落地后重建单边平衡, 稳定后自动触发第二跳
    case JUMP_PREP2:
    {
        // 清除残留起跳指令, ESC2/ESC3 输出保持为零
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;

        if (control_armed && fabsf(theta[0]) < g_params.jump_arm_ang)
            g_jump_state.timer++;
        else
            g_jump_state.timer = 0;

        if (g_jump_state.timer >= (uint32_t)g_params.arm_count_need)
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_SPIN2;
            if (buzzer) buzzer->beep(300, 100);
        }

        // ESC1 单边平衡 (内部计算并发送指令)
        SingleSideBalanceControl(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr);
        return;
    }

    // 第二跳: ESC2/ESC3 正向大电流储能, ESC1 持续单边平衡稳住滚转
    case JUMP_SPIN2:
    {
        // 先装载 ESC2/ESC3 储能指令, 再由单边平衡计算 ESC1 并统一发送
        const int32_t spin_cmd = (int32_t)(g_params.jump_spin_cur * 1000.0f);
        esc_current_cmd[1] = spin_cmd;
        esc_current_cmd[2] = spin_cmd;

        if ((wheel_speed[1] >= spin_rpm &&
             wheel_speed[2] >= spin_rpm) ||
            g_jump_state.timer >= (uint32_t)(g_params.jump_spin_ms * 0.5f))
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_KICK2;
        }

        g_jump_state.timer++;
        SingleSideBalanceControl(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr);

        // 单边失衡则中止第二跳, 回 PREP2 重建平衡后自动重试
        if (!control_armed)
        {
            g_jump_state.phase = JUMP_PREP2;
            g_jump_state.timer = 0;
            esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
            send_esc_cmds(esc, esc_current_cmd, 3);
        }
        return;
    }

    // 第二跳: ESC2/ESC3 反向大电流加速, 合反力矩绕棱边起跳 (单边 -> 单点)
    case JUMP_KICK2:
    {
        const int32_t kick_cmd = -(int32_t)(g_params.jump_kick_cur * 1000.0f);
        esc_current_cmd[1] = kick_cmd;
        esc_current_cmd[2] = kick_cmd;

        if (g_jump_state.timer >= (uint32_t)(g_params.jump_kick_ms * 0.5f))
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_SETTLE2;
        }

        g_jump_state.timer++;
        SingleSideBalanceControl(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr);

        // 单边失衡则中止第二跳, 回 PREP2 重建平衡后自动重试
        if (!control_armed)
        {
            g_jump_state.phase = JUMP_PREP2;
            g_jump_state.timer = 0;
            esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
            send_esc_cmds(esc, esc_current_cmd, 3);
        }
        return;
    }

    // 第二跳翻转/腾空输出清零; 角度接近 mid_x / mid_y 且角速度充分衰减
    // (已落地站稳) 时预解锁切入单点平衡, 立即接入平衡避免过冲
    case JUMP_SETTLE2:
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;

        if (g_jump_state.timer >= (uint32_t)(g_params.jump_settle_ms * 0.5f) &&
            fabsf(theta[1]) < g_params.jump_arm_ang &&
            fabsf(theta[2]) < g_params.jump_arm_ang &&
            fabsf(theta_dot[0]) < g_params.arm_rate &&
            fabsf(theta_dot[1]) < g_params.arm_rate)
        {
            g_jump_state.timer = 0;
            g_jump_state.phase = JUMP_BALANCE;
            control_armed = true;      // 角度/角速度已双重确认, 跳过解锁计数
            arm_counter = 0;
            g_eso_pt_x.reset(theta[1]);
            g_eso_pt_y.reset(theta[2]);
            g_eso_pt_z.reset(theta_dot[2]);
        }
        break;
    }

    // 已切入平衡: 此后一直运行单点平衡控制, 不再返回起跳流程
    case JUMP_BALANCE:
    default:
        SinglePointBalanceControl(
            esc, esc_status, theta, theta_dot, wheel_speed,
            esc_current_cmd, yaw, calib_counter, arm_counter,
            adjust_counter, control_armed, lqr);
        return;
    }

    g_jump_state.timer++;
    send_esc_cmds(esc, esc_current_cmd, 3);
}

// =============================================================================
// 转速测试模式 (3路电调交替正反转，需要将3路电调都接入CAN总线且设置好ID)
// =============================================================================
static void WheelSpeedTestControl(
    CubeFOC *esc,
    CubeFOC::ESCStatusCache esc_status[],
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    bool &control_armed)
{
    // 温度保护
    if (esc_status[ESC1_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC2_Index].temperature > ESC_MAX_Temperature ||
        esc_status[ESC3_Index].temperature > ESC_MAX_Temperature)
    {
        g_selected_control_mode = 0xFF;
        g_rc_control_mode = 0xFF;
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }

    // 未校准
    if (!(esc_status[ESC1_Index].calib_flag &&
          esc_status[ESC2_Index].calib_flag &&
          esc_status[ESC3_Index].calib_flag))
    {
        esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
        {
            const uint8_t esc_id = (uint8_t)((calib_counter / 20U) % 3U);
            esc->calib_esc_command((uint8_t)(esc_id + 1U));
        }
        send_esc_cmds(esc, esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;

    static uint32_t test_timer = 0;
    static bool forward_phase = true;
    const uint32_t phase_duration = 2500U;
    const int32_t test_current = 500;

    if (forward_phase)
    {
        esc_current_cmd[0] =  test_current;
        esc_current_cmd[1] =  test_current;
        esc_current_cmd[2] =  test_current;
    }
    else
    {
        esc_current_cmd[0] = -test_current;
        esc_current_cmd[1] = -test_current;
        esc_current_cmd[2] = -test_current;
    }

    if (++test_timer >= phase_duration)
    {
        test_timer = 0;
        forward_phase = !forward_phase;
    }
    send_esc_cmds(esc, esc_current_cmd, 3);
}

// =============================================================================
// 控制任务主循环 (500Hz / 2ms)
// =============================================================================
void StartControlTask(void *argument)
{

    auto *imu    = drv_bmi088();
    auto *esc    = drv_cubefoc();
    auto *ina226 = drv_ina226();
	auto *buzzer = drv_buzzer();
	
    // AHRS: 500Hz, Kp=2.0, Ki=0.001
    MahonyAHRS ahrs(500.0f, 2.0f, 0.001f);

    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float roll = 0, pitch = 0, yaw = 0;

    CubeFOC::ESCStatusCache esc_status[Max_ESC_Num] = {};
    int32_t esc_current_cmd[3] = {0};

    uint32_t calib_counter  = 0;
    uint32_t arm_counter    = 0;
    uint32_t adjust_counter = 0;
    bool     control_armed  = false;

    LQR lqr;

    // 创建传感数据队列 (控制任务 -> 通信任务 / 显示任务)
    g_mavSensorQueue  = osMessageQueueNew(8, sizeof(MavSensorData_t), NULL);
    g_dispSensorQueue = osMessageQueueNew(4, sizeof(MavSensorData_t), NULL);
    // 模式切换信号量
    g_controlModeSem = osSemaphoreNew(1, 0, NULL);
    // 姿态互斥锁
    g_att_mutex = osMutexNew(NULL);
    // 校准信号量
    g_calibSem = osSemaphoreNew(1, 0, NULL);

    uint32_t next_wake = osKernelGetTickCount();

    while (1)
    {

        // 获取校准后的 IMU 数据
        imu->getAccelData(ax, ay, az);
        imu->getGyroData(gx, gy, gz);

        // 无磁力计, mag 传 0
        ahrs.update(gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD, ax, ay, az, 0, 0, 0);
		
        ahrs.getEulerRad(roll, pitch, yaw);

        // 发布共享姿态 + 传感数据入队
        if (g_att_mutex != NULL && osMutexAcquire(g_att_mutex, 0) == osOK)
        {
            g_attitude.roll  = roll;
            g_attitude.pitch = pitch;
            g_attitude.yaw   = yaw;
            osMutexRelease(g_att_mutex);

            static uint8_t sensor_send_counter = 0;
            if ((++sensor_send_counter % 4U) == 0U)
            {
                MavSensorData_t sensor_data;
                sensor_data.roll  = roll;
                sensor_data.pitch = pitch;
                sensor_data.yaw   = yaw;
                sensor_data.rollspeed  = gx * DEG_TO_RAD;
                sensor_data.pitchspeed = gy * DEG_TO_RAD;
                sensor_data.yawspeed   = gz * DEG_TO_RAD;
                sensor_data.voltage = ina226 ? ina226->readBusVoltage() : 0.0f;
                sensor_data.current = -1.0f;
                sensor_data.battery_remaining = 0;
                osMessageQueuePut(g_mavSensorQueue, &sensor_data, 0, 0);
                osMessageQueuePut(g_dispSensorQueue, &sensor_data, 0, 0);
            }
        }

        // CAN 收发
        esc->spin_once();
		
        // 获取电调状态
        esc->get_esc_status(ESC1_Index, esc_status[ESC1_Index]);
        esc->get_esc_status(ESC2_Index, esc_status[ESC2_Index]);
        esc->get_esc_status(ESC3_Index, esc_status[ESC3_Index]);

        // 反正切计算角度偏差 (弧度)
        const float theta[3] = {
            AngleDiffRad(roll,  g_params.mech_mid[0]),   // 单边X轴
            AngleDiffRad(roll,  g_params.mech_mid[1]),   // 单点X轴
            AngleDiffRad(pitch, g_params.mech_mid[2])    // 单点Y轴
        };

        // 角速度 (rad/s)
        const float theta_dot[3] = {
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD
        };

        // 轮速 (rpm), 按电调方向标定符号映射回控制律内部符号约定
        const int32_t wheel_speed[3] = {
            (int32_t)g_params.esc_dir_sign[0] * kEscOldSign[0] * esc_status[ESC1_Index].rpm,
            (int32_t)g_params.esc_dir_sign[1] * kEscOldSign[1] * esc_status[ESC2_Index].rpm,
            (int32_t)g_params.esc_dir_sign[2] * kEscOldSign[2] * esc_status[ESC3_Index].rpm
        };

        // 消费模式切换信号
        if (g_controlModeSem != NULL)
            osSemaphoreAcquire(g_controlModeSem, 0);

        // 模式切换时复位起跳状态机 (重新进入起跳模式可再次起跳)
        static uint8_t prev_selected_mode = 0xFF;
        if (g_selected_control_mode != prev_selected_mode)
        {
            prev_selected_mode = g_selected_control_mode;
            JumpStateReset();
        }

        // ===== 模式分发 =====
        // 模式 0: 单边平衡
        if (g_selected_control_mode == 0 || g_rc_control_mode == 0)
        {
            SingleSideBalanceControl(
                esc, esc_status, theta, theta_dot, wheel_speed,
                esc_current_cmd, calib_counter, arm_counter,
                adjust_counter, control_armed, lqr);
        }
        // 模式 1: 单点平衡
        else if (g_selected_control_mode == 1 || g_rc_control_mode == 1)
        {
            SinglePointBalanceControl(
                esc, esc_status, theta, theta_dot, wheel_speed,
                esc_current_cmd, yaw, calib_counter, arm_counter,
                adjust_counter, control_armed, lqr);
        }
        // 模式 2: 单边起跳
        else if (g_selected_control_mode == 2)
        {
            SingleSideJumpControl(
                esc, esc_status, theta, theta_dot, wheel_speed,
                esc_current_cmd, roll, pitch, calib_counter, arm_counter,
                adjust_counter, control_armed, lqr);
        }
        // 模式 3: 单点起跳
        else if (g_selected_control_mode == 3)
        {
            SinglePointJumpControl(
                esc, esc_status, theta, theta_dot, wheel_speed,
                esc_current_cmd, roll, pitch, yaw, calib_counter, arm_counter,
                adjust_counter, control_armed, lqr);
        }
        // 模式 4: 转速测试
        else if (g_selected_control_mode == 4 || g_rc_control_mode == 4)
        {
            WheelSpeedTestControl(
                esc, esc_status, esc_current_cmd,
                calib_counter, arm_counter, control_armed);
        }
        // 不在控制模式, 油门置0
        else
        {
            esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;

            // 电调调准页激活期间暂停广播:
            // 1. 电调未上电时无节点 ACK, 持续广播会灌满 canard 发送队列, 后续命令被静默丢弃
            // 2. 校准期间不应并发 arm=1 的 Iq 指令 (与控制任务的校准时序一致)
            if (!g_esc_calib_menu_active)
                send_esc_cmds(esc, esc_current_cmd, 3);

            control_armed = false;
            arm_counter = 0;
            adjust_counter = 0;
        }

        // 校准命令处理
        if (g_calibSem != NULL)
        {
            if (osSemaphoreAcquire(g_calibSem, 0) == osOK)
            {
                if (g_calib_command == 1)
                {
                    imu->calibrateAllStatic();
                    g_gyro_calibrated = true;
                }
                g_calib_command = 0;
            }
        }

        next_wake += 2U;   // 500Hz
        osDelayUntil(next_wake);
    }
}
