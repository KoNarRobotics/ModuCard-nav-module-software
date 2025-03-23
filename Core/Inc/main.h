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
#define USER_LED_1_Pin GPIO_PIN_3
#define USER_LED_1_GPIO_Port GPIOE
#define USER_LED_2_Pin GPIO_PIN_4
#define USER_LED_2_GPIO_Port GPIOE
#define STATUS_LED_Pin GPIO_PIN_5
#define STATUS_LED_GPIO_Port GPIOE
#define USR_BUTTON_Pin GPIO_PIN_6
#define USR_BUTTON_GPIO_Port GPIOE
#define GPS_GEOFENCE_STAT_Pin GPIO_PIN_2
#define GPS_GEOFENCE_STAT_GPIO_Port GPIOD
#define GPS_RTK_STAT_Pin GPIO_PIN_3
#define GPS_RTK_STAT_GPIO_Port GPIOD
#define GPS_EXINTERUPT_Pin GPIO_PIN_4
#define GPS_EXINTERUPT_GPIO_Port GPIOD
#define GPS_MODE_SELECT_Pin GPIO_PIN_5
#define GPS_MODE_SELECT_GPIO_Port GPIOD
#define GPS_NRESET_Pin GPIO_PIN_6
#define GPS_NRESET_GPIO_Port GPIOD
#define BOOT_EN_Pin GPIO_PIN_7
#define BOOT_EN_GPIO_Port GPIOB
#define IMU_EXINTERUPT_Pin GPIO_PIN_0
#define IMU_EXINTERUPT_GPIO_Port GPIOE
#define IMU_NRESET_Pin GPIO_PIN_1
#define IMU_NRESET_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
