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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define IRQ1_Pin GPIO_PIN_0
#define IRQ1_GPIO_Port GPIOC
#define IRQ1_EXTI_IRQn EXTI0_IRQn
#define IRQ2_Pin GPIO_PIN_1
#define IRQ2_GPIO_Port GPIOC
#define IRQ2_EXTI_IRQn EXTI1_IRQn
#define IRQ3_Pin GPIO_PIN_2
#define IRQ3_GPIO_Port GPIOC
#define IRQ3_EXTI_IRQn EXTI2_IRQn
#define IRQ4_Pin GPIO_PIN_3
#define IRQ4_GPIO_Port GPIOC
#define IRQ4_EXTI_IRQn EXTI3_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_1
#define CS1_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_10
#define DIR1_GPIO_Port GPIOB
#define CS4_Pin GPIO_PIN_13
#define CS4_GPIO_Port GPIOB
#define CS3_Pin GPIO_PIN_14
#define CS3_GPIO_Port GPIOB
#define CS2_Pin GPIO_PIN_15
#define CS2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_8
#define PWM1_GPIO_Port GPIOA
#define PWM2_Pin GPIO_PIN_9
#define PWM2_GPIO_Port GPIOA
#define PWM3_Pin GPIO_PIN_10
#define PWM3_GPIO_Port GPIOA
#define PWM4_Pin GPIO_PIN_11
#define PWM4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RST_Pin GPIO_PIN_15
#define RST_GPIO_Port GPIOA
#define DIR4_Pin GPIO_PIN_3
#define DIR4_GPIO_Port GPIOB
#define DIR2_Pin GPIO_PIN_4
#define DIR2_GPIO_Port GPIOB
#define DIR3_Pin GPIO_PIN_5
#define DIR3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
