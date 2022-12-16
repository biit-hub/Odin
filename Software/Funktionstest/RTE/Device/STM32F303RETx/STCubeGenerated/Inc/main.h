/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define Button_5_Pin GPIO_PIN_13
#define Button_5_GPIO_Port GPIOC
#define Button_6_Pin GPIO_PIN_14
#define Button_6_GPIO_Port GPIOC
#define Button_7_Pin GPIO_PIN_15
#define Button_7_GPIO_Port GPIOC
#define LED_0_Pin GPIO_PIN_0
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_1
#define LED_1_GPIO_Port GPIOC
#define LED_2_Pin GPIO_PIN_2
#define LED_2_GPIO_Port GPIOC
#define LED_3_Pin GPIO_PIN_3
#define LED_3_GPIO_Port GPIOC
#define ADC1_1_Pin GPIO_PIN_0
#define ADC1_1_GPIO_Port GPIOA
#define ADC1_2_Pin GPIO_PIN_1
#define ADC1_2_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define DAC1_1_Pin GPIO_PIN_4
#define DAC1_1_GPIO_Port GPIOA
#define IO_0_Pin GPIO_PIN_5
#define IO_0_GPIO_Port GPIOA
#define IO_1_Pin GPIO_PIN_6
#define IO_1_GPIO_Port GPIOA
#define IO_2_Pin GPIO_PIN_7
#define IO_2_GPIO_Port GPIOA
#define LED_4_Pin GPIO_PIN_4
#define LED_4_GPIO_Port GPIOC
#define LED_5_Pin GPIO_PIN_5
#define LED_5_GPIO_Port GPIOC
#define CS_DISPLAY_Pin GPIO_PIN_0
#define CS_DISPLAY_GPIO_Port GPIOB
#define ADC3_1_Pin GPIO_PIN_1
#define ADC3_1_GPIO_Port GPIOB
#define CD_SDCARD_Pin GPIO_PIN_2
#define CD_SDCARD_GPIO_Port GPIOB
#define RGB_GREEN_Pin GPIO_PIN_10
#define RGB_GREEN_GPIO_Port GPIOB
#define RGB_BLUE_Pin GPIO_PIN_11
#define RGB_BLUE_GPIO_Port GPIOB
#define SPI_NSS_Pin GPIO_PIN_12
#define SPI_NSS_GPIO_Port GPIOB
#define SPI_SCK_Pin GPIO_PIN_13
#define SPI_SCK_GPIO_Port GPIOB
#define SPI_MISO_Pin GPIO_PIN_14
#define SPI_MISO_GPIO_Port GPIOB
#define SPI_MOSI_Pin GPIO_PIN_15
#define SPI_MOSI_GPIO_Port GPIOB
#define LED_6_Pin GPIO_PIN_6
#define LED_6_GPIO_Port GPIOC
#define LED_7_Pin GPIO_PIN_7
#define LED_7_GPIO_Port GPIOC
#define Button_0_Pin GPIO_PIN_8
#define Button_0_GPIO_Port GPIOC
#define Button_1_Pin GPIO_PIN_9
#define Button_1_GPIO_Port GPIOC
#define IO_3_Pin GPIO_PIN_8
#define IO_3_GPIO_Port GPIOA
#define IO_4_Pin GPIO_PIN_9
#define IO_4_GPIO_Port GPIOA
#define IO_5_Pin GPIO_PIN_10
#define IO_5_GPIO_Port GPIOA
#define IO_6_Pin GPIO_PIN_11
#define IO_6_GPIO_Port GPIOA
#define IO_7_Pin GPIO_PIN_12
#define IO_7_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define RGB_RED_Pin GPIO_PIN_15
#define RGB_RED_GPIO_Port GPIOA
#define Button_2_Pin GPIO_PIN_10
#define Button_2_GPIO_Port GPIOC
#define Button_3_Pin GPIO_PIN_11
#define Button_3_GPIO_Port GPIOC
#define Button_4_Pin GPIO_PIN_12
#define Button_4_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define PWM_BACKLIGHT_Pin GPIO_PIN_4
#define PWM_BACKLIGHT_GPIO_Port GPIOB
#define RESET_DISPLAY_Pin GPIO_PIN_5
#define RESET_DISPLAY_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define UART_RX_Pin GPIO_PIN_8
#define UART_RX_GPIO_Port GPIOB
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
