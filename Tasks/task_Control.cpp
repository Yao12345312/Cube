#include "task_Control.hpp"
#include "sensors/drv_BMI088.hpp"
#include "drv_ina226.hpp"
#include "drv_CubeFOC.hpp"
#include "drv_buzzer.hpp"
#include "MahonyAHRS.hpp"
#include "LESO.hpp"
#include "LQR.hpp"
#include "param.hpp"

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
} // namespace

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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
            static LESO eso_x;
            eso_x.update(theta[0], u, 0.002f);
            float u_comp = u - eso_x.z3() / g_params.eso_b0;

            esc_current_cmd[0] = (int32_t)(u_comp * 1000.0f);
        }
        // 发送电调油门指令
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        }
        esc_current_cmd[0] = 0;
		esc_current_cmd[1] = 0;
		esc_current_cmd[2] = 0;
        esc->send_esc_current_commands(esc_current_cmd, 3);
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

        esc->send_esc_current_commands(esc_current_cmd, 3);
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

    // 线性 ESO
    static LESO eso_x, eso_y;
    static LESO_Rate eso_z_rate;

    // ESO 状态更新
    eso_x.update(theta[1], out_x, 0.002f);
    eso_y.update(theta[2], out_y, 0.002f);

    // Z轴速率ESO: 以theta_dot[2]为观测量, 仅补偿yaw速率扰动
    eso_z_rate.update(theta_dot[2], out_z, 0.002f);

    // 扰动补偿项
    const float comp_x = eso_x.z3() / g_params.eso_b0;
    const float comp_y = eso_y.z3() / g_params.eso_b0;
    const float comp_z = eso_z_rate.z2() / g_params.eso_b0;

    // 解锁后接入控制
    if (control_armed)
    {
        esc_current_cmd[0] = (int32_t)((out_x + g_params.z_axis_comp - comp_x) * 1000.0f);
        esc_current_cmd[1] = (int32_t)((out_z + g_params.z_axis_comp - comp_z) * 1000.0f);
        esc_current_cmd[2] = (int32_t)((out_y + g_params.z_axis_comp - comp_y) * 1000.0f);
    }

    esc->send_esc_current_commands(esc_current_cmd, 3);
}

// =============================================================================
// 单边起跳平衡控制 (仅 ESC1, 起跳后切回普通平衡)
// =============================================================================
static void SingleSideJumpBalanceControl(
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

        if (fabsf(theta[0]) > g_params.max_balance_angle)
        {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else
        {
            float theta_corr = theta[0] - g_params.lqr_k[0][2] * wheel_speed[0];
            float u = lqr.compute(theta_corr, theta[0], theta_dot[0], wheel_speed[0], ESC1_Index);
            esc_current_cmd[0] = (int32_t)(u * 1000.0f);
        }

        esc->send_esc_current_commands(esc_current_cmd, 3);
    }
}

// =============================================================================
// 转速测试模式 (3路电调交替正反转)
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
        esc->send_esc_current_commands(esc_current_cmd, 3);
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
    esc->send_esc_current_commands(esc_current_cmd, 3);
}

// =============================================================================
// 控制任务主循环 (500Hz / 2ms)
// =============================================================================
void StartControlTask(void *argument)
{

    auto *imu    = drv_bmi088();
    auto *esc    = drv_cubefoc();
    auto *ina226 = drv_ina226();

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

        // 轮速 (rpm)
        const int32_t wheel_speed[3] = {
            esc_status[ESC1_Index].rpm,
            esc_status[ESC2_Index].rpm,
            esc_status[ESC3_Index].rpm
        };

        // 消费模式切换信号
        if (g_controlModeSem != NULL)
            osSemaphoreAcquire(g_controlModeSem, 0);

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
        // 模式 2: 单边起跳 (暂未实现, 油门置0)
        else if (g_selected_control_mode == 2)
        {
            esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
            esc->send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
            adjust_counter = 0;
        }
        // 模式 3: 单点起跳 (暂未实现, 油门置0)
        else if (g_selected_control_mode == 3)
        {
            esc_current_cmd[0] = 0; esc_current_cmd[1] = 0; esc_current_cmd[2] = 0;
            esc->send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
            adjust_counter = 0;
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
            esc->send_esc_current_commands(esc_current_cmd, 3);
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
