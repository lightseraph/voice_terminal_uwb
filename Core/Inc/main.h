/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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
#define LED2_Pin GPIO_PIN_15
#define LED2_GPIO_Port GPIOC
#define DW_RESET_Pin GPIO_PIN_0
#define DW_RESET_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOA
#define DW_IRQn_Pin GPIO_PIN_2
#define DW_IRQn_GPIO_Port GPIOA
#define DW_IRQn_EXTI_IRQn EXTI2_3_IRQn
#define DW_WUP_Pin GPIO_PIN_3
#define DW_WUP_GPIO_Port GPIOA
#define DW_SCK_Pin GPIO_PIN_5
#define DW_SCK_GPIO_Port GPIOA
#define DW_MISO_Pin GPIO_PIN_6
#define DW_MISO_GPIO_Port GPIOA
#define DW_MOSI_Pin GPIO_PIN_7
#define DW_MOSI_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_0
#define DW_NSS_GPIO_Port GPIOB
#define DW_SYNC_Pin GPIO_PIN_8
#define DW_SYNC_GPIO_Port GPIOA
#define I2C_SCL_Pin GPIO_PIN_11
#define I2C_SCL_GPIO_Port GPIOA
#define I2C_SDA_Pin GPIO_PIN_12
#define I2C_SDA_GPIO_Port GPIOA
#define KEY_Plus_Pin GPIO_PIN_3
#define KEY_Plus_GPIO_Port GPIOB
#define KEY_Plus_EXTI_IRQn EXTI2_3_IRQn
#define KEY_Minus_Pin GPIO_PIN_4
#define KEY_Minus_GPIO_Port GPIOB
#define KEY_Minus_EXTI_IRQn EXTI4_15_IRQn
#define RF_CE_Pin GPIO_PIN_8
#define RF_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
  typedef const int32_t sc32;
  typedef const int16_t sc16;
  typedef const int8_t sc8;

  typedef __IO int32_t vs32;
  typedef __IO int16_t vs16;
  typedef __IO int8_t vs8;

  typedef __I int32_t vsc32;
  typedef __I int16_t vsc16;
  typedef __I int8_t vsc8;

  typedef uint32_t u32;
  typedef uint16_t u16;
  typedef uint8_t u8;

  typedef const uint32_t uc32;
  typedef const uint16_t uc16;
  typedef const uint8_t uc8;

  typedef __IO uint32_t vu32;
  typedef __IO uint16_t vu16;
  typedef __IO uint8_t vu8;

  typedef __I uint32_t vuc32;
  typedef __I uint16_t vuc16;
  typedef __I uint8_t vuc8;

  extern vu32 time_delay;
  extern const uint8_t LOCAL_ID[16];
  extern uint8_t local_id;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
