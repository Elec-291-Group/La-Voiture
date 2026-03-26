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
#include "stm32l0xx_hal.h"

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
#define right_motor_hbridge1_Pin GPIO_PIN_2
#define right_motor_hbridge1_GPIO_Port GPIOA
#define right_motor_hbridge2_Pin GPIO_PIN_3
#define right_motor_hbridge2_GPIO_Port GPIOA
#define IR_Receiver_Pin GPIO_PIN_7
#define IR_Receiver_GPIO_Port GPIOA
#define IR_Receiver_EXTI_IRQn EXTI4_15_IRQn
#define left_motor_hbridge1_Pin GPIO_PIN_15
#define left_motor_hbridge1_GPIO_Port GPIOA
#define left_motor_hbridge2_Pin GPIO_PIN_3
#define left_motor_hbridge2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
enum path_tracking_states {
  Running,
  Intersection_encountered,
  Intersection_turning,
  Intersection_stop,
};

enum intersection_directions {
  Forward,
  Left,
  Right,
  Stop,
};

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
