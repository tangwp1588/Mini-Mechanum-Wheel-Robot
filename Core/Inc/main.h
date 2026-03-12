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
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
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
#define M2_DIR1_Pin GPIO_PIN_0
#define M2_DIR1_GPIO_Port GPIOC
#define M2_DIR2_Pin GPIO_PIN_1
#define M2_DIR2_GPIO_Port GPIOC
#define M3_DIR1_Pin GPIO_PIN_2
#define M3_DIR1_GPIO_Port GPIOC
#define M3_DIR2_Pin GPIO_PIN_3
#define M3_DIR2_GPIO_Port GPIOC
#define KEY_0_Pin GPIO_PIN_0
#define KEY_0_GPIO_Port GPIOA
#define M0_DIR1_Pin GPIO_PIN_0
#define M0_DIR1_GPIO_Port GPIOB
#define M0_DIR2_Pin GPIO_PIN_1
#define M0_DIR2_GPIO_Port GPIOB
#define M1_DIR1_Pin GPIO_PIN_2
#define M1_DIR1_GPIO_Port GPIOB
#define M1_DIR2_Pin GPIO_PIN_7
#define M1_DIR2_GPIO_Port GPIOE
#define KEY_3_Pin GPIO_PIN_8
#define KEY_3_GPIO_Port GPIOD
#define KEY_2_Pin GPIO_PIN_9
#define KEY_2_GPIO_Port GPIOD
#define KEY_1_Pin GPIO_PIN_10
#define KEY_1_GPIO_Port GPIOD
#define Oled_MOSI_Pin GPIO_PIN_4
#define Oled_MOSI_GPIO_Port GPIOB
#define Oled_RES_Pin GPIO_PIN_5
#define Oled_RES_GPIO_Port GPIOB
#define Oled_DC_Pin GPIO_PIN_6
#define Oled_DC_GPIO_Port GPIOB
#define Oled_BLK_Pin GPIO_PIN_7
#define Oled_BLK_GPIO_Port GPIOB
#define IMU_RST_Pin GPIO_PIN_0
#define IMU_RST_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
