#include "task_Control.hpp"
#include "board.hpp"
#include "MahonyAHRS.hpp"
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
    auto& key1 = Board::getKey1();
	auto& oled = Board::getOled();


	//开启OLED显示
	OLED_Display_On(&oled);
	

    // 传感器数据变量
    float ax, ay, az;
    float gx, gy, gz;
    QMC5883P::MagData magData;
    float roll, pitch, yaw;	
	
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
		printf("%.4f,%.4f,%.4f\n",roll,pitch,yaw);

		OLED_ShowFloat(&oled,0,0,roll,4);
		OLED_ShowFloat(&oled,1,0,pitch,4);	
		OLED_ShowFloat(&oled,2,0,yaw,4);
			
        //按键检测
        key1.update();
        Key::Event event = key1.getEvent();
        if (event == Key::Event::ShortPress) {
        // 测试按键短按
            led.setRGB(0, 100, 0);
        }
		else if(event == Key::Event::LongPress){
		// 测试按键长按
            led.setRGB(0, 0, 0);
		}
        
        // 200HZ
        osDelay(5);
    }
}