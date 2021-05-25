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
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define THIS_NODE_ADDRESS  NODE1_ADDRESS
#define THIS_NODE_LOCATION LOCATION_GIAI_PHONG_1
#define ADDR_RELAY_STATE   ADDR_FLASH_PAGE_55
#define ADDR_LOCATION      ADDR_FLASH_PAGE_55+4
#define ADDR_ERROR_CODE    ADDR_FLASH_PAGE_55+8
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
void updateDataToFlash(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LIGHT_SENSOR_OUTPUT_Pin GPIO_PIN_0
#define LIGHT_SENSOR_OUTPUT_GPIO_Port GPIOA
#define RELAY_OUTPUT_Pin GPIO_PIN_1
#define RELAY_OUTPUT_GPIO_Port GPIOA
#define BUTTON_INPUT_Pin GPIO_PIN_2
#define BUTTON_INPUT_GPIO_Port GPIOA
#define INTERRUPT_LORA_Pin GPIO_PIN_3
#define INTERRUPT_LORA_GPIO_Port GPIOA
#define INTERRUPT_LORA_EXTI_IRQn EXTI3_IRQn
#define SPI1_NSS_Pin GPIO_PIN_4
#define SPI1_NSS_GPIO_Port GPIOA
#define LED_OUTPUT_Pin GPIO_PIN_0
#define LED_OUTPUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define WHICH_ERROR(err) (err == HAL_ERROR) ? "HAL_ERROR" : ((err == HAL_BUSY) ? "HAL_BUSY" : "HAL_TIMEOUT")

#define WHICH_MSG_STS(msg_sts) (msg_sts == MSG_STS_OK) ? "MSG_STS_OK" : ((msg_sts == MSG_STS_NONE) ? "MSG_STS_NONE" : "MSG_STS_FAILED")
#define WHICH_MSG_TYPE(type) (type == MSG_TYPE_REQUEST) ? "REQUEST" : ((type == MSG_TYPE_RESPONSE) ? "RESPONSE" : "NOTIF")
#define WHICH_RELAY(state) (state == RELAY_STATE_OFF) ? "RELAY_OFF" : ((state == RELAY_STATE_ON) ? "RELAY_ON" : "RELAY_UNKNOWN")
#define WHICH_RELAY_ERR(err) (err == ERR_CODE_NONE) ? "ERR_NONE" : ((err == ERR_CODE_LIGHT_ON_FAILED) ? "ON_FAILED" : "OFF_FAILED")

#define WHICH_MODE(mode)           \
(mode == SLEEP_MODE) ? "SLEEP" :   \
((mode == STDBY_MODE) ? "STANDBY" :\
((mode == TX_MODE) ? "TX" :        \
((mode == RXCONTINUOUS_MODE) ? "RX_CONT" : "UNKNOWN")))


#define ERROR_CHECK(ret)                                           \
  do                                                               \
  {                                                                \
    if (ret != HAL_OK)                                             \
    {                                                              \
      STM_LOGE("ERROR_CHECK_TAG", "[Error] %s", WHICH_ERROR(ret)); \
    }                                                              \
  } while (0)
#define TOGGLE_LED()         (HAL_GPIO_TogglePin(LED_OUTPUT_GPIO_Port, LED_OUTPUT_Pin))
#define RELAY_CONTROL(state) (HAL_GPIO_WritePin(RELAY_OUTPUT_GPIO_Port, RELAY_OUTPUT_Pin, state))
#define RELAY_GET_STATE()    (HAL_GPIO_ReadPin(RELAY_OUTPUT_GPIO_Port, RELAY_OUTPUT_Pin))
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
