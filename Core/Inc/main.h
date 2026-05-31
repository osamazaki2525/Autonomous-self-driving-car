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
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include "../../ECUAL/Inc/Buzzer.h"
#include "../../ECUAL/Inc/DC_MOTOR.h"
#include "../../ECUAL/Inc/HCSR04.h"
#include "../../ECUAL/Inc/LED.h"
#include "../../ECUAL/Inc/SERVO.h"
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
#define Echo4_Pin GPIO_PIN_0
#define Echo4_GPIO_Port GPIOA
#define L298_IN1_Pin GPIO_PIN_2
#define L298_IN1_GPIO_Port GPIOA
#define L298_IN2_Pin GPIO_PIN_3
#define L298_IN2_GPIO_Port GPIOA
#define LED_GREEN_Pin GPIO_PIN_4
#define LED_GREEN_GPIO_Port GPIOA
#define LED_YELLOW_Pin GPIO_PIN_5
#define LED_YELLOW_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOA
#define Buzzer_Pin GPIO_PIN_7
#define Buzzer_GPIO_Port GPIOA
#define Trigger6_Pin GPIO_PIN_0
#define Trigger6_GPIO_Port GPIOB
#define Trigger5_Pin GPIO_PIN_1
#define Trigger5_GPIO_Port GPIOB
#define Echo6_Pin GPIO_PIN_10
#define Echo6_GPIO_Port GPIOB
#define Trigger4_Pin GPIO_PIN_11
#define Trigger4_GPIO_Port GPIOB
#define Trigger2_Pin GPIO_PIN_13
#define Trigger2_GPIO_Port GPIOB
#define Trigger3_Pin GPIO_PIN_14
#define Trigger3_GPIO_Port GPIOB
#define Trigger1_Pin GPIO_PIN_15
#define Trigger1_GPIO_Port GPIOB
#define Servo_Pin GPIO_PIN_4
#define Servo_GPIO_Port GPIOB
#define L298_ENA_Pin GPIO_PIN_5
#define L298_ENA_GPIO_Port GPIOB
#define Echo5_Pin GPIO_PIN_6
#define Echo5_GPIO_Port GPIOB
#define Echo2_Pin GPIO_PIN_7
#define Echo2_GPIO_Port GPIOB
#define Echo3_Pin GPIO_PIN_8
#define Echo3_GPIO_Port GPIOB
#define Echo1_Pin GPIO_PIN_9
#define Echo1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
