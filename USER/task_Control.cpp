#include "task_Control.hpp"
#include "esc_node.hpp"
#include "board.hpp"
#include "MahonyAHRS.hpp"
#include "LQR_control.hpp"
#include <cstdio>

// 定义任务句柄
osThreadId_t controlTaskHandle = NULL;

// 定义任务属性
const osThreadAttr_t controlTask_attributes = {
    .name = "ControlTask",
    .stack_size = 4*1024,      // 控制任务栈大小
    .priority = (osPriority_t) osPriorityHigh,  // 控制任务优先级较高
};

// 控制任务入口函数
void StartControlTask(void *argument)
{
    // 等待系统稳定
    osDelay(100);

    // 打印启动信息
    printf("Control Task Started!\r\n");
	
	//AHRS初始化
	static MahonyAHRS ahrs(250.0f, 2.0f, 0.001f);
	
    // 获取硬件访问接口
    auto& imu = Board::getImu();
    auto& mag = Board::getQMC5883P();
    auto& led = Board::getLedPwm();
	auto& oled = Board::getOled();
	auto& uavcan = Board::getCan();
	auto& esc_node = Board::getESCNode();
	auto& ina226 = Board::getINA226();
	
	//开启OLED显示
	OLED_Display_On(&oled);
	
    // 传感器数据变量
    float ax, ay, az;
    float gx, gy, gz;
    QMC5883P::MagData magData;
    float roll, pitch, yaw;	
	
	//获取角度误差
	float ex, ey, ez;
	
	//获取电调状态
	ESCNode::ESCStatusCache esc_status[3]={0};
	float esc_output = 0.0f;
	
	static uint64_t last_time = 0;
	
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);
	
	//电调初始化
	esc_node.set_esc_index_command(ESC2_Index);
	
	esc_node.calib_esc_command(ESC2_Index);
	
	esc_node.spin_once();
	
    while (1) {
        // 控制任务主循环

		//读取IMU数据
        imu.getAccelData(ax, ay, az);
        imu.getGyroData(gx, gy, gz);
        
        // 读取磁力计数据
        mag.readRaw(magData);
        mag.convertMagFrame(magData);
        
        // 姿态解算
        ahrs.update(
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD,
            ax, ay, az,
            (float)magData.x,
            (float)magData.y,
            (float)magData.z
        );
        
        ahrs.getEuler(roll,pitch,yaw);
		
		ahrs.getAttitudeError(ex, ey, ez);
		
		esc_node.send_node_status();	
			
		
			

		esc_node.spin_once();
				
		//获取电调转速
		if(esc_node.get_esc_status(ESC2_Index,esc_status[ESC2_Index]))
		{
			if(esc_status[ESC2_Index].calib_flag == 1)	
			{
			esc_node.send_esc_rpm_commmand(ESC2_Index,1000);
			}

		//原始roll需要转换到LQR建模的坐标系
		//esc_output= LQR_Compute(ex, gx * DEG_TO_RAD, esc_status[0].rpm);
		}
		
		OLED_ShowString(&oled, 0, 0,"vol:");
		OLED_ShowString(&oled, 1, 0,"cur:");
		OLED_ShowString(&oled, 2, 0,"tmp:");
		OLED_ShowString(&oled, 3, 0,"rpm:");
		
		OLED_ShowFloat(&oled,0,5,esc_status[ESC2_Index].voltage,2);	
		OLED_ShowFloat(&oled,1,5,esc_status[ESC2_Index].current,2);
		OLED_ShowFloat(&oled,2,5,esc_status[ESC2_Index].temperature,2);
		OLED_ShowInt32(&oled,3,5, esc_status[ESC2_Index].rpm);
		
        // 200HZ
        osDelay(20);
    }
}