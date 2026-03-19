#include "board.hpp"  // Device header
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h" // 如果需要使用队列等功能
#include "cmsis_os2.h"
#include "BMI088.hpp"
#include "QMC5883P.hpp"
#include "uart3Driver.hpp"
#include "led.hpp"
#include "key.hpp"
#include "oled.hpp"
#include "MahonyAHRS.hpp"
#include "KT6368A.hpp"

#if 1 //如果没有这段，则需要在target选项中选择使用USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//注释本行, 方法1
	extern "C"
	{
		struct __FILE {
		int handle;
		};
		std::FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}
		
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}

	}
#endif

extern void MX_GPIO_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_SPI2_Init(void);
extern void MX_FDCAN1_Init(void);
//串口对象句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern Uart3Driver* uart3;
extern Uart1Driver* uart1;
	
//LED对象句柄
extern TIM_HandleTypeDef htim2;
extern LedPwm* g_ledPwm;

//按键对象
extern Key key1;

//oled结构体
//extern u8g2_t u8g2;
extern oled_dev_t oled_dev;

//BMI088
int8_t result;
extern Bmi088 imu;

//磁力计
extern QMC5883P g_qmc5883p;
QMC5883P::MagData mag;

//蓝牙
extern BluetoothDriver* g_bt;

MahonyAHRS ahrs(250.0f, 2.0f, 0.001f);

void SystemClock_Config(void);
void StartDefaultTask(void* argument);

osThreadId_t defaultTaskHandle;

const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3072,
  .priority = (osPriority_t) osPriorityNormal,
};

int main(void)
{

  //HAL库初始化
  HAL_Init();
  //系统时钟初始化
  SystemClock_Config();
  //外设初始化
  MX_GPIO_Init();	
  MX_I2C2_Init();	
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_FDCAN1_Init();

  static Uart3Driver uart3Instance(&huart3);
  uart3 = &uart3Instance;
  
  static Uart1Driver uart1Instance(&huart1);
  uart1 = &uart1Instance;
  
  static BluetoothDriver bt(uart1);
  g_bt = &bt;

  
  static LedPwm ledPwm(&htim2);
  g_ledPwm = &ledPwm;
  if (uart3->init() && uart1->init()) {
        printf("UARTDriver initialized successfully!\r\n");
    }
	
  if (ledPwm.init()) {
        printf("LED PWM initialized successfully!\r\n");
    }
  
  if (g_bt->init()) {
        printf("kt6368a initialized successfully!\r\n");
    }
  
  g_ledPwm->setRGB(100, 0, 0);
	
  osKernelInitialize();
  
  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
 
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();


  while(1){} // 调度器启动后，不会执行到这里

}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}


void StartDefaultTask(void * argument)
{	
	OLED_Hw_Init(&oled_dev);
	OLED_Init(&oled_dev);
	
	OLED_Clear(&oled_dev);
	
	float ax, ay, az;
    float gx, gy, gz;
  
	if(!g_bt->autoBaudScan())
	{
	Error_Handler();
	}
	
    if(imu.init()!=BMI08_OK)
	{
	Error_Handler();
	}
	
	if (!g_qmc5883p.init())
    {
	Error_Handler();
    }
	
	
	OLED_ShowString(&oled_dev, 0, 0,"mag calibrate");
	OLED_ShowString(&oled_dev, 1, 0,"push key1");
	
	key1.update();	
	Key::Event event = key1.getEvent();	
	//磁力计校准
	while(event != Key::Event::LongPress)
	{
	key1.update();	
	event = key1.getEvent();	
	}

	g_qmc5883p.startCalibration();
	
	for(int i=0;i<2500;i++)
	{
    g_qmc5883p.readRaw(mag);
    g_qmc5883p.convertMagFrame(mag);

    g_qmc5883p.updateCalibration(mag);

    osDelay(10);
	}
	
	g_qmc5883p.finishCalibration();

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   ;  
	OLED_ShowString(&oled_dev, 1, 0,"finished"); 
    while (1)
    {	
     
    // 读取加速度计数据
    imu.getAccelData(ax, ay, az);
    // 读取陀螺仪数据   
	imu.getGyroData(gx, gy, gz);
	
	g_qmc5883p.readRaw(mag);
	
	g_qmc5883p.convertMagFrame(mag);
		
        ahrs.update(
            gx * DEG_TO_RAD,
            gy * DEG_TO_RAD,
            gz * DEG_TO_RAD,
            ax, ay, az,
            (float)mag.x,
            (float)mag.y,
            (float)mag.z
        );

        float roll, pitch, yaw;
        ahrs.getEuler(roll, pitch, yaw);	
		
 	printf("%.4f,%.4f,%.4f\n",roll,pitch,yaw);
//		key1.update();

//        Key::Event event = key1.getEvent();

//        if (event == Key::Event::ShortPress)
//        {
//            g_ledPwm->setRGB(0, 100, 0);
//        }
//        else if (event == Key::Event::LongPress)
//        {
//            g_ledPwm->setRGB(0, 0, 100);
//        }
	
        osDelay(1);
    }
}


extern "C" void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

