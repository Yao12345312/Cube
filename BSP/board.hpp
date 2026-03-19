#pragma once

#ifdef __cplusplus
extern "C" {
#endif

//BMI088Òý½Å¶¨Òå
#define BMI_ACS_Port GPIOC
#define BMI_ACS_Pin  GPIO_PIN_4
#define BMI_GCS_Port GPIOC
#define BMI_GCS_Pin  GPIO_PIN_5

#define KEY1_Port GPIOE
#define KEY1_PIN  GPIO_PIN_3
#define KEY2_Port GPIOE
#define KEY2_PIN  GPIO_PIN_4
#define KEY3_Port GPIOE
#define KEY3_PIN  GPIO_PIN_5
	
	
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

