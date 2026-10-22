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
#include "stm32f1xx_hal.h"

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
#define ADC15_Pin GPIO_PIN_0
#define ADC15_GPIO_Port GPIOC
#define ADC14_Pin GPIO_PIN_1
#define ADC14_GPIO_Port GPIOC
#define ADC13_Pin GPIO_PIN_2
#define ADC13_GPIO_Port GPIOC
#define ADC12_Pin GPIO_PIN_3
#define ADC12_GPIO_Port GPIOC
#define ADC11_Pin GPIO_PIN_0
#define ADC11_GPIO_Port GPIOA
#define ADC10_Pin GPIO_PIN_1
#define ADC10_GPIO_Port GPIOA
#define ADC9_Pin GPIO_PIN_2
#define ADC9_GPIO_Port GPIOA
#define ADC8_Pin GPIO_PIN_3
#define ADC8_GPIO_Port GPIOA
#define ADC7_Pin GPIO_PIN_4
#define ADC7_GPIO_Port GPIOA
#define ADC6_Pin GPIO_PIN_5
#define ADC6_GPIO_Port GPIOA
#define ADC5_Pin GPIO_PIN_6
#define ADC5_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_7
#define ADC4_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_4
#define ADC3_GPIO_Port GPIOC
#define ADC2_Pin GPIO_PIN_5
#define ADC2_GPIO_Port GPIOC
#define ADC1_Pin GPIO_PIN_0
#define ADC1_GPIO_Port GPIOB
#define ADC0_Pin GPIO_PIN_1
#define ADC0_GPIO_Port GPIOB
#define TX3EN_Pin GPIO_PIN_14
#define TX3EN_GPIO_Port GPIOE
#define RX3EN_Pin GPIO_PIN_15
#define RX3EN_GPIO_Port GPIOE
#define LCD_CS_Pin GPIO_PIN_8
#define LCD_CS_GPIO_Port GPIOD
#define LCD_RS_Pin GPIO_PIN_9
#define LCD_RS_GPIO_Port GPIOD
#define LCD_SDA_Pin GPIO_PIN_10
#define LCD_SDA_GPIO_Port GPIOD
#define LCD_CLK_Pin GPIO_PIN_11
#define LCD_CLK_GPIO_Port GPIOD
#define KEY3_Pin GPIO_PIN_12
#define KEY3_GPIO_Port GPIOD
#define KEY1_Pin GPIO_PIN_13
#define KEY1_GPIO_Port GPIOD
#define KEY2_Pin GPIO_PIN_14
#define KEY2_GPIO_Port GPIOD
#define KEY4_Pin GPIO_PIN_15
#define KEY4_GPIO_Port GPIOD
#define LCD_RST_Pin GPIO_PIN_6
#define LCD_RST_GPIO_Port GPIOC
#define LCD_BK_Pin GPIO_PIN_7
#define LCD_BK_GPIO_Port GPIOC
#define SEV2_Pin GPIO_PIN_0
#define SEV2_GPIO_Port GPIOD
#define SEV3_Pin GPIO_PIN_1
#define SEV3_GPIO_Port GPIOD
#define SEV4_Pin GPIO_PIN_3
#define SEV4_GPIO_Port GPIOD
#define SEV5_Pin GPIO_PIN_4
#define SEV5_GPIO_Port GPIOD
#define BT_AT_PIN_Pin GPIO_PIN_7
#define BT_AT_PIN_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
