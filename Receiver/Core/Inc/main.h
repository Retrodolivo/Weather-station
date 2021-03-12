/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <string.h>
#include "BME280.h"
#include "nrf24.h"
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
#define NRF24_CE_Pin GPIO_PIN_1
#define NRF24_CE_GPIO_Port GPIOA
#define NRF24_CSN_Pin GPIO_PIN_2
#define NRF24_CSN_GPIO_Port GPIOA
#define NRF24_IRQ_Pin GPIO_PIN_3
#define NRF24_IRQ_GPIO_Port GPIOA
#define NRF24_SCK_Pin GPIO_PIN_5
#define NRF24_SCK_GPIO_Port GPIOA
#define NRF24_MISO_Pin GPIO_PIN_6
#define NRF24_MISO_GPIO_Port GPIOA
#define NRF24_MOSI_Pin GPIO_PIN_7
#define NRF24_MOSI_GPIO_Port GPIOA
#define NEXTION_USART_TX_Pin GPIO_PIN_9
#define NEXTION_USART_TX_GPIO_Port GPIOA
#define NEXION_USART_RX_Pin GPIO_PIN_10
#define NEXION_USART_RX_GPIO_Port GPIOA
#define BME280_SCL_Pin GPIO_PIN_6
#define BME280_SCL_GPIO_Port GPIOB
#define BME280_SDA_Pin GPIO_PIN_7
#define BME280_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
