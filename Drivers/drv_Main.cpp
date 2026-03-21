#include "drv_Main.hpp"
#include "task_Control.hpp"
#include "task_Communication.hpp"
#include "uart3Driver.hpp"
#include "board.hpp"

extern void MX_GPIO_Init(void);
extern void MX_I2C2_Init(void);
extern void MX_SPI1_Init(void);
extern void MX_USART1_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_DMA_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_SPI2_Init(void);
extern void MX_FDCAN1_Init(void);

//驱动初始化函数
void init_drv_Main(void)
{
  //HAL库外设初始化
  MX_GPIO_Init();	
	
  MX_I2C2_Init();	
	
  MX_SPI1_Init();
	
  MX_SPI2_Init();
	
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_DMA_Init();
	
  MX_TIM2_Init();
	
  MX_FDCAN1_Init();
	
  //驱动对象初始化
  if (!Board::init()) {
        Error_Handler();
    }
  
	
}

void create_application_tasks(void)
{
    printf("Creating application tasks...\r\n");
    
    //创建通信任务
    communicationTaskHandle = osThreadNew(
        StartCommunicationTask, 
        NULL, 
        &communicationTask_attributes
    );
    
    if (communicationTaskHandle == NULL) {
        printf("Failed to create Communication Task!\r\n");
        Error_Handler();
    }
    
    //创建控制任务
    controlTaskHandle = osThreadNew(
        StartControlTask, 
        NULL, 
        &controlTask_attributes
    );
    
    if (controlTaskHandle == NULL) {
        printf("Failed to create Control Task!\r\n");
        Error_Handler();
    }
    
    printf("All tasks created successfully!\r\n");
}
