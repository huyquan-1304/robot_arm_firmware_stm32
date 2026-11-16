/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define FL1_Pin GPIO_PIN_0
#define FL1_GPIO_Port GPIOA
#define FL2_Pin GPIO_PIN_1
#define FL2_GPIO_Port GPIOA
#define RR1_Pin GPIO_PIN_2
#define RR1_GPIO_Port GPIOB
#define RR_IN1_Pin GPIO_PIN_7
#define RR_IN1_GPIO_Port GPIOE
#define RR_IN2_Pin GPIO_PIN_8
#define RR_IN2_GPIO_Port GPIOE
#define FL_Pin GPIO_PIN_9
#define FL_GPIO_Port GPIOE
#define MOT_STBY_Pin GPIO_PIN_10
#define MOT_STBY_GPIO_Port GPIOE
#define FR_Pin GPIO_PIN_11
#define FR_GPIO_Port GPIOE
#define RL_Pin GPIO_PIN_13
#define RL_GPIO_Port GPIOE
#define RR_Pin GPIO_PIN_14
#define RR_GPIO_Port GPIOE
#define FL_IN1_Pin GPIO_PIN_8
#define FL_IN1_GPIO_Port GPIOD
#define FL_IN2_Pin GPIO_PIN_9
#define FL_IN2_GPIO_Port GPIOD
#define FR_IN1_Pin GPIO_PIN_10
#define FR_IN1_GPIO_Port GPIOD
#define FR_IN2_Pin GPIO_PIN_11
#define FR_IN2_GPIO_Port GPIOD
#define FR1_Pin GPIO_PIN_12
#define FR1_GPIO_Port GPIOD
#define FR2_Pin GPIO_PIN_13
#define FR2_GPIO_Port GPIOD
#define RL_IN1_Pin GPIO_PIN_14
#define RL_IN1_GPIO_Port GPIOD
#define RL_IN2_Pin GPIO_PIN_15
#define RL_IN2_GPIO_Port GPIOD
#define RL1_Pin GPIO_PIN_6
#define RL1_GPIO_Port GPIOC
#define RL2_Pin GPIO_PIN_7
#define RL2_GPIO_Port GPIOC
#define RR2_Pin GPIO_PIN_12
#define RR2_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
