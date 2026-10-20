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
#define motor_ph1_Pin GPIO_PIN_0
#define motor_ph1_GPIO_Port GPIOC
#define motor_ph2_Pin GPIO_PIN_1
#define motor_ph2_GPIO_Port GPIOC
#define motor_ph3_Pin GPIO_PIN_2
#define motor_ph3_GPIO_Port GPIOC
#define motor_ph4_Pin GPIO_PIN_3
#define motor_ph4_GPIO_Port GPIOC
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_5
#define NRF_CE_GPIO_Port GPIOC
#define NRF_IRQ_Pin GPIO_PIN_0
#define NRF_IRQ_GPIO_Port GPIOB
#define motor_en4_Pin GPIO_PIN_7
#define motor_en4_GPIO_Port GPIOE
#define motor_en3_Pin GPIO_PIN_9
#define motor_en3_GPIO_Port GPIOE
#define motor_en2_Pin GPIO_PIN_11
#define motor_en2_GPIO_Port GPIOE
#define motor_en1_Pin GPIO_PIN_13
#define motor_en1_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_0
#define LED_GPIO_Port GPIOD
#define key1_Pin GPIO_PIN_3
#define key1_GPIO_Port GPIOD
#define key4_Pin GPIO_PIN_4
#define key4_GPIO_Port GPIOD
#define key3_Pin GPIO_PIN_5
#define key3_GPIO_Port GPIOD
#define key2_Pin GPIO_PIN_6
#define key2_GPIO_Port GPIOD
#define OLED_SCL_Pin GPIO_PIN_6
#define OLED_SCL_GPIO_Port GPIOB
#define OLED_SDA_Pin GPIO_PIN_7
#define OLED_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
