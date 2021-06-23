/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  /* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm_log.h"
#include "lora.h"
#include "data-format.h"
#include "flash.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAIN_TAG  "MAIN"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
NodeTypedef_t thisNode = {
  .nodeID = THIS_NODE_ADDRESS,
  .location = THIS_NODE_LOCATION,
  .relayState = RELAY_STATE_OFF,
  .errCode = ERR_CODE_NONE,
  .friendNodeID = FRIEND_NODE_ADDRESS,
};

IWDG_HandleTypeDef hiwdg;
LoraConf_t LoraInit = {
  .Access_Shared_Reg = ACCESS_LORA_REGISTERS,
  .Access_Frequence_Mode = ACCESS_LOW_FREQUENCY_MODE,
  .Rf_Frequency = RF_FREQUENCY,
  .Pa_Select = PA_BOOST,
  .Output_Power = OUTPUT_POWER,
  .Ocp_Strim = OCP_TRIM,
  .Fifo_Tx_Base_Addr = FIFO_TX_BASE_ADDR,
  .Fifo_Rx_Base_Addr = FIFO_RX_BASE_ADDR,
  .Coding_Rate = CODING_RATE_4_5,
  .Band_Width = BANDWIDTH_62K5,
  .Header_Mode = IMPLICIT_HEADER,
  .Spreading_Factor = SPREADING_FACTOR_12_4096,
  .Rx_Payload_Crc = CRC_ENABLE,
  .Preamble_Length = PREAMBLE_LENGTH,
  .Payload_Length = PAYLOAD_LENGTH,
  .Detection_Optimize = 0x03,
  .Detection_Threshold = 0x0A,
  .Crystal_Oscillator = XTAL_INPUT, /* TCXO_INPUT XTAL_INPUT */
  .Pa_Dac = PA_DAC
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
static void nodeInit();
static void pwdOnNotif();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
//  vLoraInit();
  LED_OFF();
  vLoraInit(&LoraInit);
  STM_LOGI(MAIN_TAG, "Reset cause:  {%s}", resetCauseGetName(resetCauseGet()));

  /* Retrieve old state from FLASH */
  nodeInit();

  STM_LOGI(MAIN_TAG, "NodeID:     {%d}", thisNode.nodeID);
  STM_LOGI(MAIN_TAG, "Relay:      {%s}", WHICH_RELAY(thisNode.relayState));
  STM_LOGI(MAIN_TAG, "Location:   {%d}", thisNode.location);
  STM_LOGI(MAIN_TAG, "Error:      {%s}", WHICH_RELAY_ERR(thisNode.errCode));
  STM_LOGI(MAIN_TAG, "FriendID:     {%d}", thisNode.friendNodeID);
  STM_LOGI(MAIN_TAG, "GatewayID:  {%d}", GATEWAY_ADDRESS);

  /* Send notif gw after power on */
  pwdOnNotif();

  /* Enable watchdog */
  STM_LOGI(MAIN_TAG, "Watchdog Init {%ums}", iwdgInit(&hiwdg, WATCHDOG_TIME));

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
  RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };
  RCC_PeriphCLKInitTypeDef PeriphClkInit = { 0 };

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
    | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static void nodeInit() {
  if (Flash_ReadAddress(FLASH_ADDR_RELAY_STATE) == FLASH_EMPTY)
  {
    STM_LOGV(MAIN_TAG, "data not found, write relay data to flash");
    ERROR_CHECK(Flash_ErasePage(FLASH_ADDR_RELAY_STATE, 1));
    ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_NODE_ID, (uint32_t)thisNode.nodeID));
    ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_FRIEND_NODE_ID, (uint32_t)thisNode.friendNodeID));
    ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_RELAY_STATE, (uint32_t)thisNode.relayState));
    ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_LOCATION, (uint32_t)thisNode.location));
    ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_ERROR_CODE, (uint32_t)thisNode.errCode));
  }
  else
  {
    STM_LOGV(MAIN_TAG, "detect flash data, restore old state ...");
    uint8_t flashNodeID = Flash_ReadAddress(FLASH_ADDR_NODE_ID);
    if (thisNode.nodeID != flashNodeID && flashNodeID != 0xFF)
    {
      STM_LOGI(MAIN_TAG, "NodeID update: %d ---> %d", thisNode.nodeID, flashNodeID);
      thisNode.nodeID = flashNodeID;
    }

    uint8_t flashFriendNodeID = Flash_ReadAddress(FLASH_ADDR_FRIEND_NODE_ID);
    if (thisNode.friendNodeID != flashFriendNodeID && flashFriendNodeID != 0xFF && flashFriendNodeID != thisNode.nodeID)
    {
      STM_LOGI(MAIN_TAG, "Friend NodeID update: %d ---> %d", thisNode.friendNodeID, flashFriendNodeID);
      thisNode.friendNodeID = flashFriendNodeID;
    }

    thisNode.relayState = Flash_ReadAddress(FLASH_ADDR_RELAY_STATE);
    thisNode.location = Flash_ReadAddress(FLASH_ADDR_LOCATION);
    thisNode.errCode = Flash_ReadAddress(FLASH_ADDR_ERROR_CODE);
    RELAY_CONTROL(thisNode.relayState);
  }
}

static void pwdOnNotif()
{
  uint8_t notifData[PAYLOAD_LENGTH];
  PACK_NOTIF_MSG(notifData, thisNode, TIME_TO_LIVE_DEFAULT);
  LoRaTransmit(notifData, PAYLOAD_LENGTH, LORA_DELAY);
}

void updateDataToFlash(void)
{
  taskENTER_CRITICAL();
  ERROR_CHECK(Flash_ErasePage(FLASH_ADDR_RELAY_STATE, 1));
  ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_FRIEND_NODE_ID, (uint32_t)thisNode.friendNodeID));
  ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_RELAY_STATE, (uint32_t)thisNode.relayState));
  ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_LOCATION, (uint32_t)thisNode.location));
  ERROR_CHECK(Flash_WriteWord(FLASH_ADDR_ERROR_CODE, (uint32_t)thisNode.errCode));
  taskEXIT_CRITICAL();
  STM_LOGD(MAIN_TAG, "update data to flash");
}

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
     /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
