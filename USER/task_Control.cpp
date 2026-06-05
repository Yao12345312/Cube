#include "task_Control.hpp"
#include "task_CPUMonitor.h"
#include "esc_node.hpp"
#include "board.hpp"
#include "MahonyAHRS.hpp"
#include "LQR_control.hpp"
#include "LESO.hpp"
#include "attitute.hpp"


#include <cmath>
#include <cstdio>

// 定义任务句柄
osThreadId_t controlTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t controlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 8 * 1024,
    .priority = (osPriority_t)osPriorityHigh,
};

Attitude_t g_attitude = {0};
osMutexId_t g_att_mutex = osMutexNew(NULL);

osMessageQueueId_t g_mavSensorQueue = NULL;
osMessageQueueId_t g_dispSensorQueue = NULL;

osSemaphoreId_t g_controlModeSem = NULL;
uint8_t g_selected_control_mode = 0xFF;  //模式切换标志位，无任务为0xFF

//校准状态全局变量
osSemaphoreId_t g_calibSem = NULL;
uint8_t g_calib_command = 0;              // 0=idle, 1=gyro, 2=mag
bool g_gyro_calibrated = false;
bool g_mag_calibrated = false;

static float z_axis_Compensation =0.25f;
	
static uint32_t cpu_usage = 0;

namespace {
	static inline float AngleDiffRad(float a, float b)
	{
		// Wrap angle difference to [-pi, pi] using atan2
		return atan2f(sinf(a - b), cosf(a - b));
	}
} // namespace


//单边控制（ESC1）
// theta[0]: X轴角度
// theta_dot[0]: X轴角速度
static void SingleSideBalanceControl(
    ESCNode &esc_node,
    ESCNode::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
	uint32_t &adjust_counter,
    bool &control_armed)
{	
	auto& buzzer = Board::getBuzzer();
	
    const float max_balance_angle = 0.20f;
    const float arm_angle = 0.10f;
    const float arm_rate = 0.5f;
    const uint32_t arm_count_need = 20U;

    const float bias_gain = 0.0001f;

    //自适应重心变量
    static float theta_bias = 0.0f;

    //如果电调没校准，则校准
    if (!esc_status[ESC1_Index].calib_flag)
	{
        esc_current_cmd[0] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
		{
            esc_node.calib_esc_command((ESC1_Index + 1));
        }
    }
	else 
	{
        calib_counter = 0;

        //未解锁
        if (!control_armed) {
            if (fabsf(theta[0]) < arm_angle ) {
                arm_counter++;
            } 
			else{
                arm_counter = 0;
            }

            if (arm_counter >= arm_count_need) {
				buzzer.beep(500,200);
                control_armed = true;
            }

            esc_current_cmd[0] = 0;
        }
        //角度超过可控制范围，上锁
        else if (fabsf(theta[0]) > max_balance_angle) {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else {
            //稳定状态下积分消除重心偏差
            if (fabsf(theta[0]) < 0.05f &&
                fabsf(theta_dot[0]) < 0.1f &&
                fabsf(wheel_speed[0]) < 800.0f)
            {
                theta_bias += bias_gain * theta[0];
            }

            //速度反馈
            float theta_corr = theta[0] - K_lqr[0][2] * wheel_speed[0];

            float u = LQR_Compute(theta_corr,theta[0], theta_dot[0], wheel_speed[0], ESC1_Index);

            // ESO状态更新与扰动补偿
            static ESO_t eso_x;
            static bool eso_x_init = false;
            if (!eso_x_init) {
                ESO_Init(&eso_x);
                eso_x_init = true;
            }
            ESO_Update(&eso_x, theta[0], u, 0.002f);
            float u_comp = u - eso_x.z3 / eso_x.b0;

            esc_current_cmd[0] = (int32_t)(u_comp * 1000.0f);
        }
        //发送电调油门指令
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
    }
}

//单点控制
// theta[1]: X轴角度
// theta[2]: Y轴角度
// theta_dot[0]: X轴角速度
// theta_dot[1]: Y轴角速度
static void SinglePointBalanceControl(
    ESCNode &esc_node,
    ESCNode::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    float yaw,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
	uint32_t &adjust_counter,
    bool &control_armed)
{	
	auto& buzzer = Board::getBuzzer();
	
    const float max_balance_angle = 0.20f;
    const float arm_angle         = 0.02f;
    const uint32_t arm_count_need = 20U;
	
	const float bias_k = 0.0002f;     //重心适应学习率
    const float bias_leak = 0.0001f;  
    const float bias_limit = 0.02f;   //积分限幅
	
	// X/Y轴角度偏差
    static float theta_bias_x = 0.0f;
    static float theta_bias_y = 0.0f;
	
    // 如果电机未校准，停止输出，进行校准
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

    // 到平衡点附近解锁控制
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
			buzzer.beep(500,200);
            control_armed = true;
        }
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }

    // 超过可控角度限制，上锁
    if (fabsf(theta[1]) > max_balance_angle ||
        fabsf(theta[2]) > max_balance_angle)
    {
        esc_current_cmd[0] = 0;
        esc_current_cmd[1] = 0;
        esc_current_cmd[2] = 0;
        control_armed = false;
        arm_counter = 0;
		
		adjust_counter = 0;
		theta_bias_x = 0.0f;
		theta_bias_y = 0.0f;
		
        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }
	
//	if (fabsf(wheel_speed[0]) > 500 &&
//        fabsf(wheel_speed[1]) > 500 )
//    {
//		adjust_counter++;
//		
//		if(adjust_counter> 1000)
//		{
//			
//		adjust_counter = 1000;
//		// Angle bias integral from wheel speed
//        theta_bias_x += bias_k * wheel_speed[0] - bias_leak * theta_bias_x;
//        theta_bias_y += bias_k * wheel_speed[2] - bias_leak * theta_bias_y;
//		
//        // Integral clamp
//        theta_bias_x = fminf(fmaxf(theta_bias_x, -bias_limit), bias_limit);
//        theta_bias_y = fminf(fmaxf(theta_bias_y, -bias_limit), bias_limit);
//		
//		}	
//    }
	
    // LQR + 前馈控制
    const float theta_x = theta[1];// - theta_bias_x;
    const float theta_y = theta[2];// - theta_bias_y;
	
    const float theta_corr_x =  theta_x - K_lqr[ESC1_Index][2] * wheel_speed[0] - K_lqr[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_y = -theta_y - K_lqr[ESC3_Index][2] * wheel_speed[2] - K_lqr[ESC2_Index][2] * wheel_speed[1];
    const float theta_corr_z = -K_lqr[ESC2_Index][2] * wheel_speed[1];
	
    float out_x = LQR_Compute(theta_corr_x, (theta[1]-2.35f),theta_dot[0], wheel_speed[0], ESC1_Index);
    float out_z = LQR_Compute(theta_corr_z, 0.0f, theta_dot[2], wheel_speed[1], ESC2_Index);
    float out_y = LQR_Compute(theta_corr_y, -(theta[2]-0.64f), theta_dot[1], wheel_speed[2], ESC3_Index);
    //线性ESO初始化
    static ESO_t eso_x, eso_y, eso_z;
    static bool eso_init = false;
    if (!eso_init) {
        ESO_Init(&eso_x);
        ESO_Init(&eso_y);
        ESO_Init(&eso_z);
        eso_init = true;
    }
	//线性ESO状态更新
    ESO_Update(&eso_x, theta[1], out_x, 0.002f);
    ESO_Update(&eso_y, theta[2], out_y, 0.002f);
    ESO_Update(&eso_z, yaw, out_z, 0.002f);

    // 扰动补偿项计算，补偿总扰动z3估计值
    const float comp_x = eso_x.z3 / eso_x.b0;
    const float comp_y = eso_y.z3 / eso_y.b0;
    const float comp_z = eso_z.z3 / eso_z.b0;
	
	//解锁后接入控制
	if(control_armed)
	{
	esc_current_cmd[0] = (int32_t)((out_x + z_axis_Compensation - comp_x) * 1000.0f);
	esc_current_cmd[1] = (int32_t)((out_z + z_axis_Compensation - comp_z) * 1000.0f);
    esc_current_cmd[2] = (int32_t)((out_y + z_axis_Compensation - comp_y) * 1000.0f);
	
	}
	
    esc_node.send_esc_current_commands(esc_current_cmd, 3);
}

static void SingleSideJumpBalanceControl(
    ESCNode &esc_node,
    ESCNode::ESCStatusCache esc_status[],
    const float *theta,
    const float *theta_dot,
    const int32_t *wheel_speed,
    int32_t *esc_current_cmd,
    uint32_t &calib_counter,
    uint32_t &arm_counter,
	uint32_t &adjust_counter,
    bool &control_armed)
{
    const float max_balance_angle = 0.20f;
	
    if (!esc_status[ESC1_Index].calib_flag)
	{
        esc_current_cmd[0] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U)
		{
            esc_node.calib_esc_command((ESC1_Index + 1));
        }
    }
	else 
	{
        calib_counter = 0;
		

        if (fabsf(theta[0]) > max_balance_angle) {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else {

            float theta_corr = theta[0] - K_lqr[0][2] * wheel_speed[0];

            float u = LQR_Compute(theta_corr,theta[0], theta_dot[0], wheel_speed[0], ESC1_Index);

            esc_current_cmd[0] = (int32_t)(u * 1000.0f);
        }

        esc_node.send_esc_current_commands(esc_current_cmd, 3);
    }
}

void StartControlTask(void *argument)
{
    osDelay(100);
    printf("Control Task Started!\r\n");

    static MahonyAHRS ahrs(500.0f, 2.0f, 0.001f);

    // 获取驱动接口
    auto &imu = Board::getImu();
    auto &mag = Board::getQMC5883P();
    auto &esc_node = Board::getESCNode();
    auto &ina226 = Board::getINA226();
	
    //传感器数据变量
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    QMC5883P::MagData magData;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

    //电调状态
    ESCNode::ESCStatusCache esc_status[Max_ESC_Num] = {0};
    int32_t esc_current_cmd[3] = {0};

    uint32_t log_counter = 0;
    uint32_t calib_counter = 0;
    uint32_t arm_counter = 0;
	uint32_t adjust_counter = 0;
	
    bool control_armed = false;

    //机械中点
    const float mechanics_medium[3] = {
        -2.35f,  // 单边X轴
        -2.35f,  // 单点X轴
        -0.64f   // 单点Y轴
    };

    //系统时间戳
    uint32_t next_wake = osKernelGetTickCount();

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

    g_calibSem = osSemaphoreNew(1, 0, NULL);
    if (g_calibSem == NULL) {
        printf("Failed to create calibration semaphore!\r\n");
        Error_Handler();
    }

    while (1) {
		
		//cpu_usage = GetCPUUsage();
		
        //获取校准后的IMU数据
        imu.getAccelDataCalibrated(ax, ay, az);
        imu.getGyroDataCalibrated(gx, gy, gz);
		
        // 获取磁力计数据
        // QMC5883P -> BMI088: 180 deg CW around Z-axis
        // X_acc = -X_mag, Y_acc = -Y_mag, Z_acc = Z_mag
        mag.readRaw(magData);
        mag.convertMagFrame(magData);
		
		
		//传感器校准状态判断
        {
            static bool calib_was_collecting = false;
            mag.updateCalibration(magData);
            if (calib_was_collecting && !mag.isCollecting())
            {
                //磁力计校准完成标志位
                g_mag_calibrated = true;
            }
            calib_was_collecting = mag.isCollecting();
        }

        //磁力计硬铁/软铁校准
        if(g_mag_calibrated)
        {
            float ox, oy, oz;
            mag.getOffset(ox, oy, oz);
            float m[3][3];
            mag.getMatrix(m);

            float x = magData.x - ox;
            float y = magData.y - oy;
            float z = magData.z - oz;

            magData.x = m[0][0]*x + m[0][1]*y + m[0][2]*z;
            magData.y = m[1][0]*x + m[1][1]*y + m[1][2]*z;
            magData.z = m[2][0]*x + m[2][1]*y + m[2][2]*z;
        }

        //更新互补滤波姿态解算
        ahrs.update(
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD,
            ax, ay, az,
            (float)magData.x,
            (float)magData.y,
            (float)magData.z);

        //获取姿态
        ahrs.getEulerRad(roll, pitch, yaw);

        if(osMutexAcquire(g_att_mutex, 0) == osOK)
        {
            g_attitude.roll = roll;
            g_attitude.pitch = pitch;
            g_attitude.yaw = yaw;
            osMutexRelease(g_att_mutex);

		
            //传感器数据通过队列发送到通信任务
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
                sensor_data.battery_remaining = 0;
                osMessageQueuePut(g_mavSensorQueue, &sensor_data, 0, 0);
                osMessageQueuePut(g_dispSensorQueue, &sensor_data, 0, 0);
            }
		}

        //CAN收发
        esc_node.spin_once();

        //获取电调状态
        esc_node.get_esc_status(ESC1_Index, esc_status[ESC1_Index]);
        esc_node.get_esc_status(ESC2_Index, esc_status[ESC2_Index]);
        esc_node.get_esc_status(ESC3_Index, esc_status[ESC3_Index]);

        //反正切计算角度偏差（弧度制）
        const float theta[3] = {
            (float)AngleDiffRad(roll, mechanics_medium[0]),
            (float)AngleDiffRad(roll, mechanics_medium[1]),
            (float)AngleDiffRad(pitch, mechanics_medium[2])
        };

        //角速度（rad/s）
        const float theta_dot[3] = {
            (float)(gx * DEG_TO_RAD),
            (float)(gy * DEG_TO_RAD),
            (float)(gz * DEG_TO_RAD)
        };

        //轮速（rpm）
        const int32_t wheel_speed[3] = {
            esc_status[ESC1_Index].rpm,
            esc_status[ESC2_Index].rpm,
            esc_status[ESC3_Index].rpm
        };

        // 获取显示任务信号量，通知进入控制模式
        if(g_controlModeSem != NULL)
            osSemaphoreAcquire(g_controlModeSem, 0);
		
		//模式1：单边
        if(g_selected_control_mode == 0)
        {
            
            SingleSideBalanceControl(
                esc_node,
                esc_status,
                theta,
                theta_dot,
                wheel_speed,
                esc_current_cmd,
                calib_counter,
                arm_counter,
				adjust_counter,
                control_armed);
        }
		//模式2：单点
        else if(g_selected_control_mode == 1)
        {
			
			
		    	SinglePointBalanceControl(
	                esc_node,
	                esc_status,
	                theta,
	                theta_dot,
	                wheel_speed,
	                esc_current_cmd,
	                yaw,
	                calib_counter,
	                arm_counter,
					adjust_counter,
	                control_armed);
             
					
		}
        //模式3：单边起跳
        else if(g_selected_control_mode == 2)
        {
//            static uint8_t  jump_state = 0;
//            static uint32_t jump_timer = 0;


//            esc_current_cmd[1] = 0;
//            esc_current_cmd[2] = 0;

//            if(jump_state == 0)
//            {
//                esc_current_cmd[0] = -25000;
//                if(++jump_timer >= 800)
//                {
//                    jump_timer = 0;
//                    jump_state = 1;
//                }
//            }
//			else if(jump_state == 1)
//			{
//				esc_current_cmd[0] += 2000;
//				if(esc_current_cmd[0] > 20000)esc_current_cmd[0] = 20000;
//				
//				if(fabsf(theta[0] < 0.2f))
//				{
//					jump_timer = 0;
//					jump_state = 2;
//				}
//			}				
//			
//            if(jump_state < 2)
//            {
//                esc_node.send_esc_current_commands(esc_current_cmd, 3);
//            }
//            else
//            {
//                SingleSideJumpBalanceControl(
//                    esc_node, esc_status, theta, theta_dot, wheel_speed,
//                    esc_current_cmd, calib_counter, arm_counter,
//                    adjust_counter, control_armed);
//            }

		    esc_current_cmd[0] = 0;
            esc_current_cmd[1] = 0;
            esc_current_cmd[2] = 0;
            esc_node.send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
            adjust_counter = 0;
        }
		//模式4：单点起跳
        else if(g_selected_control_mode == 3)
        {
			
            esc_current_cmd[0] = 0;
            esc_current_cmd[1] = 0;
            esc_current_cmd[2] = 0;
            esc_node.send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
            adjust_counter = 0;
        }
		//不在控制模式，油门置0
        else
        {
            
            esc_current_cmd[0] = 0;
            esc_current_cmd[1] = 0;
            esc_current_cmd[2] = 0;
            esc_node.send_esc_current_commands(esc_current_cmd, 3);
            control_armed = false;
            arm_counter = 0;
			adjust_counter=0;
        }

        //校准状态判断
        if(g_calibSem != NULL)
        {
            if(osSemaphoreAcquire(g_calibSem, 0) == osOK)
            {
                if(g_calib_command == 1)
                {
                    //IMU零偏校准
                    imu.calibrateAllStatic();
                    g_gyro_calibrated = true;
                }
                else if(g_calib_command == 2)
                {
                    //磁力计校准
                    mag.startCalibration();
                    g_mag_calibrated = false;  
                }
                g_calib_command = 0;
            }
        }

        // 串口3日志输出
//        if ((log_counter++ % 10U) == 0U) {
//            printf("%ld,%ld,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
//                   (long)esc_status[ESC1_Index].rpm,
//                   (long)esc_status[ESC2_Index].rpm,
//                   (long)esc_status[ESC3_Index].rpm,
//                   (float)ax,
//                   (float)ay,
//                   (float)az,
//                   (float)theta[1],
//                   (float)theta[2],
//                   (float)yaw);
//        }

        next_wake += 2U;  // 500Hz
        osDelayUntil(next_wake);
    }
}
