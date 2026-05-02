#include "task_Control.hpp"
#include "esc_node.hpp"
#include "board.hpp"
#include "MahonyAHRS.hpp"
#include "LQR_control.hpp"

#include <cmath>
#include <cstdio>
// 定义任务句柄
osThreadId_t controlTaskHandle = NULL;
// 定义任务属性
const osThreadAttr_t controlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 4 * 1024,
    .priority = (osPriority_t)osPriorityHigh,
};

Attitude_t g_attitude = {0};
osMutexId_t g_att_mutex = osMutexNew(NULL);

namespace {
static inline float AngleDiffRad(float a, float b)
{	
	//转成单位圆下再取arctan，求最短角度误差,使误差在[-Π,Π]内
    return atan2f(sinf(a - b), cosf(a - b));
}
} // namespace


//单边平衡控制函数
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

    // 平衡点偏置
    static float theta_bias = 0.0f;
	
    //=============================控制逻辑===============================
    // 电调未校准
    if (!esc_status[ESC1_Index].calib_flag) {
        esc_current_cmd[0] = 0;
        control_armed = false;
        arm_counter = 0;

        if ((calib_counter++ % 20U) == 0U) {
            esc_node.calib_esc_command((ESC1_Index + 1));
			
        }
    } else {
        calib_counter = 0;

        // 未解锁
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
        // 超范围保护
        else if (fabsf(theta[0]) > max_balance_angle) {
            esc_current_cmd[0] = 0;
            control_armed = false;
            arm_counter = 0;
        }
        else {
            // 自适应重心
            if (fabsf(theta[0]) < 0.05f &&
                fabsf(theta_dot[0]) < 0.1f &&
                fabsf(wheel_speed[0]) < 800.0f)
            {
                theta_bias += bias_gain * theta[0];
            }
			//轮速代入角度误差做负反馈
            float theta_corr = theta[0] - K_lqr[0][2] * wheel_speed[0];

            float u = LQR_Compute(theta_corr, theta_dot[0], wheel_speed[0], ESC1_Index);

            esc_current_cmd[0] = (int32_t)(u * 1000.0f);
        }
        // 始终发送
        esc_node.send_esc_current_commands(esc_current_cmd,3);
    }
}

// 单点平衡控制函数
// theta[1] : 单点X轴倾角（roll方向）
// theta[2] : 单点Y轴倾角（pitch方向）
// theta_dot[0] : X轴角速度
// theta_dot[1] : Y轴角速度

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
    const float arm_angle         = 0.10f;
    const float arm_rate          = 0.5f;
    const uint32_t arm_count_need = 20U;

    const float bias_gain = 0.0001f;

    // X/Y轴平衡点自适应偏置
    static float theta_bias_x = 0.0f;
    static float theta_bias_y = 0.0f;
	
	bool cal_flag = false;
	
    // 电调未校准
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
			
			cal_flag = true;
        }

        esc_node.send_esc_current_commands(esc_current_cmd, 3);
        return;
    }
    calib_counter = 0;
	
    // 未解锁
    if (!control_armed)
    {
        if (fabsf(theta[1]) < arm_angle &&
            fabsf(theta[2]) < arm_angle 
            )
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

    // 超范围保护
    if (fabsf(theta[1]) > max_balance_angle|| 
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

	
    if (fabsf(theta[1]) < 0.05f &&
        fabsf(theta[2]) < 0.05f &&
        fabsf(theta_dot[0]) < 0.1f &&
        fabsf(theta_dot[1]) < 0.1f &&
        fabsf(wheel_speed[0]) < 800.0f &&
        fabsf(wheel_speed[1]) < 800.0f &&
		fabsf(wheel_speed[2]) < 800.0f)
    {
        theta_bias_x += bias_gain * theta[1];
        theta_bias_y += bias_gain * theta[2];
    }
 
	//获取xy轴角度偏差
    const float theta_x = theta[1];
    const float theta_y = theta[2];

	//速度环反馈叠加到误差角
    const float theta_corr_x =  theta_x - K_lqr[ESC1_Index][2] * wheel_speed[0];
	const float theta_corr_y = -theta_y - K_lqr[ESC3_Index][2] * wheel_speed[2];
	const float theta_corr_z = -K_lqr[ESC2_Index][2] * wheel_speed[1];

	//获取原始三轴输出
	const float out_x =  LQR_Compute(theta_corr_x,theta_dot[0],wheel_speed[0],ESC1_Index);
	const float out_y =  LQR_Compute(theta_corr_y,theta_dot[1],wheel_speed[2],ESC3_Index);
	const float out_z =  LQR_Compute(theta_corr_z,theta_dot[2],wheel_speed[1],ESC2_Index);
	
//	printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
//			out_x,out_y,out_z,
//			cmd_m1,cmd_m2,cmd_m3);
	
	
	//获取电调电流环控制量
	esc_current_cmd[0] = (int32_t)((out_x) * 1000.0f);
	esc_current_cmd[1] = (int32_t)((out_z) * 1000.0f);
	esc_current_cmd[2] = (int32_t)((out_y) * 1000.0f);

	//发送电调电流控制量
    esc_node.send_esc_current_commands(esc_current_cmd, 3);
}
	
void StartControlTask(void *argument)
{
    osDelay(100);
    printf("Control Task Started!\r\n");

    static MahonyAHRS ahrs(200.0f, 2.0f, 0.001f);
	//获取wrapper接口
    auto &imu = Board::getImu();
    auto &mag = Board::getQMC5883P();
    auto &esc_node = Board::getESCNode();
	
	//平衡立方体姿态
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    QMC5883P::MagData magData;
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
	//电调状态变量
    ESCNode::ESCStatusCache esc_status[Max_ESC_Num] = {0};
	//初始化电调电流环控制量
    int32_t esc_current_cmd[3] = {0};
	
    uint32_t log_counter = 0;
    uint32_t calib_counter = 0;
    uint32_t arm_counter = 0;
    bool control_armed = false;
	//机械中值
	const float mechanics_medium[3] = {-2.35f,  //单边x轴
									   -2.35f,  //单点x轴
									   -0.64f}; //单点y轴
	
	//获取系统时间戳
    uint32_t next_wake = osKernelGetTickCount();
	
	//IMU零偏校准
	imu.calibrateAllStatic();
	
    while (1) {

        //AHRS姿态解算
        imu.getAccelDataCalibrated(ax, ay, az);
        imu.getGyroDataCalibrated(gx, gy, gz);

        mag.readRaw(magData);
        mag.convertMagFrame(magData);
		//角度更新
        ahrs.update(
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD,
            ax, ay, az,
            (float)magData.x,
            (float)magData.y,
            (float)magData.z);
		//转发姿态角
        ahrs.getEulerRad(roll, pitch, yaw);
	
		if(osMutexAcquire(g_att_mutex, 0) == osOK)
		{
		g_attitude.roll = roll;
        g_attitude.pitch = pitch;
        g_attitude.yaw = yaw;
        osMutexRelease(g_att_mutex);
		}
        
        //CAN收发
        esc_node.spin_once();

        //获取电调状态
        esc_node.get_esc_status(ESC1_Index, esc_status[ESC1_Index]);
		esc_node.get_esc_status(ESC2_Index, esc_status[ESC2_Index]);
		esc_node.get_esc_status(ESC3_Index, esc_status[ESC3_Index]);
		
		//获取偏差角度,[0]:边平衡x轴误差 ,[1]：点平衡x轴误差,[2]:点平衡y轴误差
        const float theta[3] = {(float)AngleDiffRad(roll, mechanics_medium[0]) , (float)AngleDiffRad(roll, mechanics_medium[1]) , (float)AngleDiffRad(pitch, mechanics_medium[2])};
		//获取角速度
        const float theta_dot[3] = {(float)(gx * DEG_TO_RAD) , (float)(gy * DEG_TO_RAD) , (float)(gz * DEG_TO_RAD)};
		//获取轮速(rpm)
        const int32_t wheel_speed[3] = {esc_status[ESC1_Index].rpm , esc_status[ESC2_Index].rpm ,esc_status[ESC3_Index].rpm };
		
		//单边平衡控制
//		SingleSideBalanceControl(
//			esc_node,
//			esc_status,
//			theta,
//			theta_dot,
//			wheel_speed,
//			esc_current_cmd,
//			calib_counter,
//			arm_counter,
//			control_armed);
		//单点平衡控制
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
		

        // LOG
        if ((log_counter++ % 10U) == 0U) {
            printf("%ld,%ld,%ld,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\r\n",
                   (long)esc_status[ESC1_Index].rpm,
				   (long)esc_status[ESC2_Index].rpm,
				   (long)esc_status[ESC3_Index].rpm,
				   (float)esc_status[ESC1_Index].calib_flag,
				   (float)esc_status[ESC2_Index].calib_flag,
				   (float)esc_status[ESC3_Index].calib_flag, 
				   (float)roll,					   
                   (float)pitch,
				   (float)yaw);
        }
		
        next_wake += 5U;  // 200Hz
        osDelayUntil(next_wake);
    }
}

