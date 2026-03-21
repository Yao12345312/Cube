#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//BMI088引脚定义
#define BMI_ACS_Port GPIOC
#define BMI_ACS_Pin  GPIO_PIN_4
#define BMI_GCS_Port GPIOC
#define BMI_GCS_Pin  GPIO_PIN_5

//按键引脚定义
#define KEY1_Port GPIOE
#define KEY1_PIN  GPIO_PIN_3
#define KEY2_Port GPIOE
#define KEY2_PIN  GPIO_PIN_4
#define KEY3_Port GPIOE
#define KEY3_PIN  GPIO_PIN_5

//OLED模块引脚定义
#define OLED_DC_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_12
#define OLED_RST_Port GPIOE	
#define OLED_RST_Pin GPIO_PIN_15	

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */



#ifdef __cplusplus
}
#endif

//C++接口
#ifdef __cplusplus

#include "oled.hpp" 
#include "QMC5883P.hpp"
#include "uart1Driver.hpp"
#include "uart3Driver.hpp"
#include "led.hpp"
#include "BMI088.hpp"
#include "can.hpp"
#include "KT6368A.hpp"
#include "key.hpp"


/**
 * @brief Board命名空间 - 提供所有硬件模块的统一访问接口
 * 
 * 使用方法：
 *   auto& imu = Board::getImu();
 *   imu.getAccelData(ax, ay, az);
 *   
 *   Board::getLedPwm().setRGB(255, 0, 0);
 *   
 *   Board::init();  // 初始化所有硬件
 */
namespace Board {
    
    /**
     * @brief 初始化所有板载外设
     * @return true 初始化成功，false 初始化失败
     * @note 必须在RTOS启动后、创建任务前调用
     */
    bool init();
    
    /**
     * @brief 获取QMC5883P磁力计实例
     * @return QMC5883P对象的引用
     */
    QMC5883P& getQMC5883P();
    
    /**
     * @brief 获取UART3驱动实例
     * @return Uart3Driver对象的引用
     */
    Uart3Driver& getUart3();
    
    /**
     * @brief 获取UART1驱动实例
     * @return Uart1Driver对象的引用
     */
    Uart1Driver& getUart1();
    
    /**
     * @brief 获取LED PWM驱动实例
     * @return LedPwm对象的引用
     */
    LedPwm& getLedPwm();
    
    /**
     * @brief 获取BMI088 IMU实例
     * @return Bmi088对象的引用
     */
    Bmi088& getImu();
    
    /**
     * @brief 获取CAN驱动实例
     * @return CanDriver对象的引用
     */
    CanDriver& getCan();
    
    /**
     * @brief 获取OLED设备实例
     * @return oled_dev_t结构体的引用
     */
    oled_dev_t& getOled();
    
    /**
     * @brief 获取蓝牙驱动实例
     * @return BluetoothDriver对象的引用
     */
    BluetoothDriver& getBluetooth();
    
    /**
     * @brief 获取按键1实例
     * @return Key对象的引用
     */
    Key& getKey1();
    
    /**
     * @brief 获取按键2实例
     * @return Key对象的引用
     */
    Key& getKey2();
    
    /**
     * @brief 获取按键3实例
     * @return Key对象的引用
     */
    Key& getKey3();
    
} // namespace Board

#endif // __cplusplus
