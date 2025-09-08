/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RL_INB_GPO_Pin GPIO_PIN_0
#define RL_INB_GPO_GPIO_Port GPIOH
#define RL_INA_GPO_Pin GPIO_PIN_1
#define RL_INA_GPO_GPIO_Port GPIOH
#define FL_INA_GPO_Pin GPIO_PIN_0
#define FL_INA_GPO_GPIO_Port GPIOC
#define FL_INB_GPO_Pin GPIO_PIN_1
#define FL_INB_GPO_GPIO_Port GPIOC
#define LR_INA_GPO_Pin GPIO_PIN_2
#define LR_INA_GPO_GPIO_Port GPIOC
#define LR_INB_GPO_Pin GPIO_PIN_3
#define LR_INB_GPO_GPIO_Port GPIOC
#define RL_PWM_T2C1_Pin GPIO_PIN_0
#define RL_PWM_T2C1_GPIO_Port GPIOA
#define FL_PWM_T2C2_Pin GPIO_PIN_1
#define FL_PWM_T2C2_GPIO_Port GPIOA
#define UR_INA_GPO_Pin GPIO_PIN_4
#define UR_INA_GPO_GPIO_Port GPIOA
#define UR_ENC_A_T3C1_Pin GPIO_PIN_6
#define UR_ENC_A_T3C1_GPIO_Port GPIOA
#define UR_ENC_B_T3C2_Pin GPIO_PIN_7
#define UR_ENC_B_T3C2_GPIO_Port GPIOA
#define RR_INB_GPO_Pin GPIO_PIN_4
#define RR_INB_GPO_GPIO_Port GPIOC
#define UR_INB_GPO_Pin GPIO_PIN_0
#define UR_INB_GPO_GPIO_Port GPIOB
#define RR_PWM_T2C4_Pin GPIO_PIN_2
#define RR_PWM_T2C4_GPIO_Port GPIOB
#define FR_PWM_T2C3_Pin GPIO_PIN_10
#define FR_PWM_T2C3_GPIO_Port GPIOB
#define RR_INA_GPO_Pin GPIO_PIN_13
#define RR_INA_GPO_GPIO_Port GPIOB
#define UL_INA_GPO_Pin GPIO_PIN_14
#define UL_INA_GPO_GPIO_Port GPIOB
#define UL_INB_GPO_Pin GPIO_PIN_15
#define UL_INB_GPO_GPIO_Port GPIOB
#define LR_ENC_A_T8C1_Pin GPIO_PIN_6
#define LR_ENC_A_T8C1_GPIO_Port GPIOC
#define LR_ENC_B_T8C2_Pin GPIO_PIN_7
#define LR_ENC_B_T8C2_GPIO_Port GPIOC
#define RR_VDD_GPO_Pin GPIO_PIN_8
#define RR_VDD_GPO_GPIO_Port GPIOC
#define FR_VDD_GPO_Pin GPIO_PIN_9
#define FR_VDD_GPO_GPIO_Port GPIOC
#define UL_ENC_A_T1C1_Pin GPIO_PIN_8
#define UL_ENC_A_T1C1_GPIO_Port GPIOA
#define UL_ENC_B_T1C2_Pin GPIO_PIN_9
#define UL_ENC_B_T1C2_GPIO_Port GPIOA
#define FR_INA_GPO_Pin GPIO_PIN_10
#define FR_INA_GPO_GPIO_Port GPIOA
#define RL_VDD_GPO_Pin GPIO_PIN_10
#define RL_VDD_GPO_GPIO_Port GPIOC
#define FL_VDD_GPO_Pin GPIO_PIN_11
#define FL_VDD_GPO_GPIO_Port GPIOC
#define FR_INB_GPO_Pin GPIO_PIN_5
#define FR_INB_GPO_GPIO_Port GPIOB
#define LL_ENC_A_T4C1_Pin GPIO_PIN_6
#define LL_ENC_A_T4C1_GPIO_Port GPIOB
#define LL_ENC_B_T4C2_Pin GPIO_PIN_7
#define LL_ENC_B_T4C2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
