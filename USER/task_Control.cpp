#include "task_Control.hpp"
#include "esc_node.hpp"
#include "board.hpp"
#include "MahonyAHRS.hpp"
#include "LQR_control.hpp"
#include "attitute.hpp"

#include <cmath>
#include <cstdio>

// Task handle
osThreadId_t controlTaskHandle = NULL;

// Task attributes
const osThreadAttr_t controlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 4 * 1024,
    .priority = (osPriority_t)osPriorityHigh,
};

Attitude_t g_attitude = {0};
osMutexId_t g_att_mutex = osMutexNew(NULL);

osMessageQueueId_t g_mavSensorQueue = NULL;
osMessageQueueId_t g_dispSensorQueue = NULL;

osSemaphoreId_t g_controlModeSem = NULL;
uint8_t g_selected_control_mode = 0xFF;  // No default mode, requires user selection

namespace {
static inline float AngleDiffRad(float a, float b)
{
    // Wrap angle difference to [-pi, pi] using atan2
    return atan2f(sinf(a - b), cosf(a - b));
}
} // namespace


// Single side balance control (1-axis, ESC1 only)
static void SingleSideBalanceControl(
    ESCNode &esc_node,
    ESCNode::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    bool &control_armed)
{
    const float max_balance_angle = 0.15f;
    const float arm_angle = 0.10f;
    const float arm_rate = 0.5f;
    const uint32_t arm_count_need = 20U;

    const float bias_gain = 0.0001f;

    // Balance angle bias (adaptive)
    static float theta_bias = 0.0f;

    // ========================= Calibration & Arm Logic =========================
    // ESC not calibrated
    if (!esc_status[ESC1_Index].calib_flag) {
        esc_current_cmd[0] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U) {
            esc_node.calib_esc_command((ESC1_Index + 1));
        }
    } else {
        calib_counter = 0;

        // Not armed yet
        if (!control_armed) {
            if (fabsf(theta[0]) < arm_angle && fabsf(theta_dot[0]) < arm_rate) {
                arm_counter++;
            } else {
                arm_counter = 0;
            }

            if (arm_counter >= arm_count_need) {
                control_armed = true;
            }

            esc_current_cmd[0] = 0;
        }
        // Out of balance range -> disarm
        else if (fabsf(theta[0]) > max_balance_angle) {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else {
            // Adaptive bias estimation (near equilibrium, low speed)
            if (fabsf(theta[0]) < 0.05f &&
                fabsf(theta_dot[0]) < 0.1f &&
                fabsf(wheel_speed[0]) < 800.0f)
            {
                theta_bias += bias_gain * theta[0];
            }

            // Speed feedback: angle correction from wheel speed
            float theta_corr = theta[0] - K_lqr[0][2] * wheel_speed[0];

            float u = LQR_Compute(theta_corr, theta_dot[0], wheel_speed[0], ESC1_Index);

            esc_current_cmd[0] = (int32_t)(u * 1000.0f);
        }
        // Always send command
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
    }
}

// Three point balance control (3-axis, ESC1/2/3)
// theta[1]: X-axis angle (roll)
// theta[2]: Y-axis angle (pitch)
// theta_dot[0]: X-axis angular rate
// theta_dot[1]: Y-axis angular rate
static void SinglePointBalanceControl(
    ESCNode &esc_node,
    ESCNode::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
    bool &control_armed)
{
    const float max_balance_angle = 0.15f;
    const float arm_angle         = 0.07f;
    const float arm_rate          = 0.5f;
    const uint32_t arm_count_need = 20U;

    const float bias_k = 0.002f;     // Learning rate
    const float bias_leak = 0.001f;  // Leak term to prevent drift
    const float bias_limit = 0.1f;   // Integral clamp

    // X/Y axis adaptive bias
    static float theta_bias_x = 0.0f;
    static float theta_bias_y = 0.0f;

    // All three ESCs must be calibrated
    if (!(esc_status[ESC1_Index].calib_flag &&
          esc_status[ESC2_Index].calib_flag &&
          esc_status[ESC3_Index].calib_flag))
    {
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
        {
            const uint8_t esc_id = (uint8_t)((calib_counter / 20U) % 3U);
            esc_node.calib_esc_command(esc_id + 1U);
        }
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;

    // Not armed: wait for small angles to arm
    if (!control_armed)
    {
        if (fabsf(theta[1]) < arm_angle &&
            fabsf(theta[2]) < arm_angle)
        {
            arm_counter++;
        }
        else
        {
            arm_counter = 0;
        }
        if (arm_counter >= arm_count_need)
        {
            control_armed = true;
        }
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }

    // Out of balance range -> disarm
    if (fabsf(theta[1]) > max_balance_angle ||
        fabsf(theta[2]) > max_balance_angle)
    {
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }

    // Adaptive bias estimation: wheel-speed-driven learning
    if (fabsf(wheel_speed[0]) > 300 &&
        fabsf(wheel_speed[2]) > 300)
    {
        // Angle bias integral from wheel speed
        theta_bias_x += bias_k * wheel_speed[0] - bias_leak * theta_bias_x;
        theta_bias_y += bias_k * wheel_speed[2] - bias_leak * theta_bias_y;

        // Integral clamp
        theta_bias_x = fminf(fmaxf(theta_bias_x, -bias_limit), bias_limit);
        theta_bias_y = fminf(fmaxf(theta_bias_y, -bias_limit), bias_limit);
    }


    // Get XY angles (with adaptive bias)
    const float theta_x = theta[1];
    const float theta_y = theta[2];

    // Speed feedback mixed into angle error
    const float theta_corr_x =  theta_x - K_lqr[ESC1_Index][2] * wheel_speed[0] - K_lqr[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_y = -theta_y - K_lqr[ESC3_Index][2] * wheel_speed[2] - K_lqr[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_z = -K_lqr[ESC2_Index][2] * wheel_speed[1];

    // LQR control outputs
    const float out_x =  LQR_Compute(theta_corr_x, theta_dot[0], wheel_speed[0], ESC1_Index);
    const float out_y =  LQR_Compute(theta_corr_y, theta_dot[1], wheel_speed[2], ESC3_Index);
    const float out_z =  LQR_Compute(theta_corr_z, theta_dot[2], wheel_speed[1], ESC2_Index);


    // Map to ESCs
    esc_current_cmd[0] = (int32_t)(out_x * 1000.0f);
    esc_current_cmd[1] = (int32_t)(out_z * 1000.0f);
    esc_current_cmd[2] = (int32_t)(out_y * 1000.0f);

    esc_node.send_esc_current_commands(esc_current_cmd, 3);
}

void StartControlTask(void *argument)
{
    osDelay(100);
    printf("Control Task Started!\r\n");

    static MahonyAHRS ahrs(200.0f, 2.0f, 0.001f);

    // Get driver wrappers
    auto &imu = Board::getImu();
    auto &mag = Board::getQMC5883P();
    auto &esc_node = Board::getESCNode();
    auto &ina226 = Board::getINA226();

    // Sensor data variables
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    QMC5883P::MagData magData;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    // ESC status and commands
    ESCNode::ESCStatusCache esc_status[Max_ESC_Num] = {0};
    int32_t esc_current_cmd[3] = {0};

    uint32_t log_counter = 0;
    uint32_t calib_counter = 0;
    uint32_t arm_counter = 0;
    bool control_armed = false;

    // Mechanical equilibrium angles (rad)
    const float mechanics_medium[3] = {
        -2.35f,  // X-axis side 1
        -2.35f,  // X-axis side 2
        -0.64f   // Y-axis
    };

    // System tick
    uint32_t next_wake = osKernelGetTickCount();

    // IMU static bias calibration
    imu.calibrateAllStatic();


    g_mavSensorQueue = osMessageQueueNew(8, sizeof(MavSensorData_t), NULL);
    g_dispSensorQueue = osMessageQueueNew(4, sizeof(MavSensorData_t), NULL);
    if (g_mavSensorQueue == NULL || g_dispSensorQueue == NULL) {
        printf("Failed to create sensor data queues!\r\n");
        Error_Handler();
    }

    g_controlModeSem = osSemaphoreNew(1, 0, NULL);
    if (g_controlModeSem == NULL) {
        printf("Failed to create control mode semaphore!\r\n");
        Error_Handler();
    }

    while (1) {

        // AHRS attitude update
        imu.getAccelDataCalibrated(ax, ay, az);
        imu.getGyroDataCalibrated(gx, gy, gz);

        // Read magnetometer and transform to BMI088 coordinate frame
        // QMC5883P -> BMI088: 180 deg CW around Z-axis
        // X_acc = -X_mag, Y_acc = -Y_mag, Z_acc = Z_mag
        mag.readRaw(magData);
        mag.convertMagFrame(magData);

        // AHRS fusion (mag data must be in same frame as accel/gyro)
        ahrs.update(
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD,
            ax, ay, az,
            (float)magData.x,
            (float)magData.y,
            (float)magData.z);

        // Get attitude
        ahrs.getEulerRad(roll, pitch, yaw);

        if(osMutexAcquire(g_att_mutex, 0) == osOK)
        {
            g_attitude.roll = roll;
            g_attitude.pitch = pitch;
            g_attitude.yaw = yaw;
            osMutexRelease(g_att_mutex);


            // Send sensor data to MAVLink queue (50Hz = every 4th iteration)
            static uint8_t sensor_send_counter = 0;
            if ((++sensor_send_counter % 4U) == 0U) {
                MavSensorData_t sensor_data;
                sensor_data.roll = roll;
                sensor_data.pitch = pitch;
                sensor_data.yaw = yaw;
                sensor_data.rollspeed = gx * DEG_TO_RAD;
                sensor_data.pitchspeed = gy * DEG_TO_RAD;
                sensor_data.yawspeed = gz * DEG_TO_RAD;
                sensor_data.voltage = ina226.INA226_ReadBusVoltage();
                sensor_data.current = -1.0f;
                sensor_data.battery_remaining = -1;
                osMessageQueuePut(g_mavSensorQueue, &sensor_data, 0, 0);
                osMessageQueuePut(g_dispSensorQueue, &sensor_data, 0, 0);
            }}

        // CAN send/receive
        esc_node.spin_once();

        // Get ESC status
        esc_node.get_esc_status(ESC1_Index, esc_status[ESC1_Index]);
        esc_node.get_esc_status(ESC2_Index, esc_status[ESC2_Index]);
        esc_node.get_esc_status(ESC3_Index, esc_status[ESC3_Index]);

        // Compute angle errors from mechanical equilibrium
        // theta[0]: X-axis side 1, theta[1]: X-axis side 2, theta[2]: Y-axis
        const float theta[3] = {
            (float)AngleDiffRad(roll, mechanics_medium[0]),
            (float)AngleDiffRad(roll, mechanics_medium[1]),
            (float)AngleDiffRad(pitch, mechanics_medium[2])
        };

        // Angular rates (rad/s)
        const float theta_dot[3] = {
            (float)(gx * DEG_TO_RAD),
            (float)(gy * DEG_TO_RAD),
            (float)(gz * DEG_TO_RAD)
        };

        // Wheel speeds (rpm)
        const int32_t wheel_speed[3] = {
            esc_status[ESC1_Index].rpm,
            esc_status[ESC2_Index].rpm,
            esc_status[ESC3_Index].rpm
        };

        // Check for control mode change signal from menu
        if(g_controlModeSem != NULL)
            osSemaphoreAcquire(g_controlModeSem, 0);

        // Execute selected control mode
        if(g_selected_control_mode == 0)
        {
            // Single side balance control
            SingleSideBalanceControl(
                esc_node,
                esc_status,
                theta,
                theta_dot,
                wheel_speed,
                esc_current_cmd,
                calib_counter,
                arm_counter,
                control_armed);
        }
        else if(g_selected_control_mode == 1)
        {
            // Three point balance control
            SinglePointBalanceControl(
                esc_node,
                esc_status,
                theta,
                theta_dot,
                wheel_speed,
                esc_current_cmd,
                calib_counter,
                arm_counter,
                control_armed);
        }
        else
        {
            // No mode selected: idle, send zero current
            esc_current_cmd[0] = 0;
            esc_current_cmd[1] = 0;
            esc_current_cmd[2] = 0;
            esc_node.send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
        }

        // LOG (disabled)
//        if ((log_counter++ % 10U) == 0U) {
//            printf("%ld,%ld,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
//                   (long)esc_status[ESC1_Index].rpm,
//                   (long)esc_status[ESC2_Index].rpm,
//                   (long)esc_status[ESC3_Index].rpm,
//                   (float)gx,
//                   (float)gy,
//                   (float)gz,
//                   (float)theta[1],
//                   (float)theta[2],
//                   (float)yaw);
//        }

        next_wake += 5U;  // 200Hz
        osDelayUntil(next_wake);
    }
}
