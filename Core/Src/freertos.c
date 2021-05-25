/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stm_log.h"
#include "lora.h"
#include "queue.h"
#include "data-format.h"
#include "light-sensor.h"
#include <stdbool.h>
#include "flash.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern NodeTypedef_t thisNode;
extern uint32_t adcLightSensor;
extern IWDG_HandleTypeDef hiwdg;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskProducer */
osThreadId_t taskProducerHandle;
const osThreadAttr_t taskProducer_attributes = {
  .name = "taskProducer",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for taskConsumer */
osThreadId_t taskConsumerHandle;
const osThreadAttr_t taskConsumer_attributes = {
  .name = "taskConsumer",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for taskPeriodic */
osThreadId_t taskPeriodicHandle;
const osThreadAttr_t taskPeriodic_attributes = {
  .name = "taskPeriodic",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
};
/* Definitions for nodedataMutex */
osMutexId_t nodedataMutexHandle;
const osMutexAttr_t nodedataMutex_attributes = {
  .name = "nodedataMutex",
  .attr_bits = osMutexRecursive,
};
/* Definitions for rxDoneSemaphore */
osSemaphoreId_t rxDoneSemaphoreHandle;
const osSemaphoreAttr_t rxDoneSemaphore_attributes = {
  .name = "rxDoneSemaphore"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void opcodeInquiry(uint8_t seqID);
static void opcodeRelayControl(uint8_t newState, uint8_t seqID);
static void opcodeMcuReset(void);
static void opcodeLocationUpdate(uint8_t newLocation, uint8_t seqID);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void entryProducer(void *argument);
void entryConsumer(void *argument);
void entryPeriodic(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
  return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the recursive mutex(es) */
  /* creation of nodedataMutex */
  nodedataMutexHandle = osMutexNew(&nodedataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of rxDoneSemaphore */
  rxDoneSemaphoreHandle = osSemaphoreNew(5, 5, &rxDoneSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  if (rxDoneSemaphoreHandle == NULL)
    STM_LOGE("ERROR", "create rxDoneSemaphoreHandle failed");
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew (10, 10, &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if (myQueue01Handle == NULL)
    STM_LOGE("ERROR", "create myQueue01Handle failed");

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of taskProducer */
  taskProducerHandle = osThreadNew(entryProducer, NULL, &taskProducer_attributes);

  /* creation of taskConsumer */
  taskConsumerHandle = osThreadNew(entryConsumer, NULL, &taskConsumer_attributes);

  /* creation of taskPeriodic */
  taskPeriodicHandle = osThreadNew(entryPeriodic, NULL, &taskPeriodic_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  if (taskProducerHandle == NULL)
    STM_LOGE("ERROR", "create taskProducerHandle failed");
  if (taskConsumerHandle == NULL)
    STM_LOGE("ERROR", "create taskConsumerHandle failed");
  if (taskPeriodicHandle == NULL)
    STM_LOGE("ERROR", "create taskPeriodicHandle failed");
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  STM_LOGV("MileStone", "Kernel starts");
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_entryProducer */
/**
* @brief Function implementing the taskProducer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entryProducer */
void entryProducer(void *argument)
{
  /* USER CODE BEGIN entryProducer */
  osStatus_t err;
  uint8_t receivedMsg[10];
  uint32_t tickToWait = pdMS_TO_TICKS(5000);
  /* Infinite loop */
  for (;;)
  {
    err = osSemaphoreAcquire(rxDoneSemaphoreHandle, portMAX_DELAY);
    if (!err) {

      STM_LOGV("Producer", "Get semaphore ok");

      if (LoRaGetITFlag(PAYLOAD_CRC_ERROR_MskPos) == 1)
      {
        STM_LOGE("Producer", "Payload CRC failed");
      }
      else
      {
        LORA_SET_FIFO_CURRENT_MSG();
        for (uint8_t i = 0; i < PAYLOAD_LENGTH; i++) {
          receivedMsg[i] = ucSpi1Read(RegFifo);
          // STM_LOGI("Producer", "receivedData[%d]: %x", i, receivedMsg[i]);
        }

        STM_LOGI("Producer", "msg dest ID: %x - thisNodeID: %x", receivedMsg[INDEX_DEST_ID], thisNode.nodeID);
        if (receivedMsg[INDEX_DEST_ID] == thisNode.nodeID &&
          receivedMsg[INDEX_MSG_TYPE] == MSG_TYPE_REQUEST)
        {

          STM_LOGV("Producer", "put queue");
          err = osMessageQueuePut(myQueue01Handle, receivedMsg, 0, tickToWait);
          if (!err)
          {
            STM_LOGV("Producer", "put queue ok");
          }
          else
          {
            STM_LOGE("Producer", "put queue failed, err %d\n\r NbOfMsg in queue : % d\n\ravailable size : % d", \
              err, \
              osMessageQueueGetCount(myQueue01Handle), \
              osMessageQueueGetSpace(myQueue01Handle));
          }
        }
        else
        {
          STM_LOGV("Producer", "msg not matched --> dicarded");
        }
      }
      LoRaClearITFlag(RX_DONE_Msk | PAYLOAD_CRC_ERROR_Msk);
    }
  }
  /* USER CODE END entryProducer */
}

/* USER CODE BEGIN Header_entryConsumer */
/**
* @brief Function implementing the taskConsumer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entryConsumer */
void entryConsumer(void *argument)
{
  /* USER CODE BEGIN entryConsumer */
  static uint8_t receivedMsgFromQueue[PAYLOAD_LENGTH];
  osStatus_t err;
  /* Infinite loop */
  for (;;)
  {
    err = osMessageQueueGet(myQueue01Handle, receivedMsgFromQueue, 0, portMAX_DELAY);
    if (!err)
    {
      printf("\r\n");
      STM_LOGV("Consumer", "Get queue ok");
      STM_LOGI("Consumer", "-----> OPCODE: %d", receivedMsgFromQueue[INDEX_COMMAND_OPCODE]);
      vModeInit(STDBY_MODE);
      switch (receivedMsgFromQueue[INDEX_COMMAND_OPCODE])
      {
      case OPCODE_REQUEST_STATE:
        opcodeInquiry(receivedMsgFromQueue[INDEX_SEQUENCE_ID]);
        break;
      case OPCODE_REQUEST_RELAY_CONTROL:
        opcodeRelayControl(receivedMsgFromQueue[INDEX_DATA_RELAY_STATE], receivedMsgFromQueue[INDEX_SEQUENCE_ID]);
        break;
      case OPCODE_REQUEST_MCU_RESET:
        opcodeMcuReset();
        updateDataToFlash();
        break;
      case OPCODE_REQUEST_LOCATION_UPDATE:
        opcodeLocationUpdate(receivedMsgFromQueue[INDEX_DATA_LOCATION], receivedMsgFromQueue[INDEX_SEQUENCE_ID]);
        break;
      default:
        STM_LOGE("Consumer", "No service for opcode %d", receivedMsgFromQueue[INDEX_COMMAND_OPCODE]);
        break;
      }

      vModeInit(RXCONTINUOUS_MODE);
      updateDataToFlash();
    }
  }
  /* USER CODE END entryConsumer */
}

/* USER CODE BEGIN Header_entryPeriodic */
/**
* @brief Function implementing the taskPeriodic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entryPeriodic */
void entryPeriodic(void *argument)
{
  /* USER CODE BEGIN entryPeriodic */
  static const uint32_t tickToWait = pdMS_TO_TICKS(1000);
  static int count;
  /* Infinite loop */
  for (;;)
  {
    TOGGLE_LED();
    if (++count >= 10) {
      count = 0;
      // STM_LOGI("Periodic", "relay: %s", WHICH_RELAY(thisNode.relayState));
    }
    HAL_IWDG_Refresh(&hiwdg);
    osDelay(tickToWait);
  }
  /* USER CODE END entryPeriodic */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static void opcodeInquiry(uint8_t seqID)
{
  uint8_t msg[10];
  PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_STATE);
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void opcodeRelayControl(uint8_t newState, uint8_t seqID)
{
  uint8_t msg[10];
  uint8_t countCheck = 0;
  bool isAck = false;
  bool isChecking = true;
  if (thisNode.relayState != newState)
  {
    STM_LOGI("Consumer", "State changes: {%s} to {%s}", WHICH_RELAY(thisNode.relayState), WHICH_RELAY(newState));
    thisNode.relayState = newState;
    RELAY_CONTROL(thisNode.relayState);
    STM_LOGV("Consumer", "Checking relay ...");
    ADC_READ_LIGHTSENSOR();
    while (isChecking && ++countCheck <= 70) {
      STM_LOGD("Consumer", "adcLightSensor: %d - count: %d", adcLightSensor, countCheck);

      if (((adcLightSensor < LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_ON)) ||
        ((adcLightSensor >= LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_OFF)))
      {
        isChecking = false;
        isAck = true;
        thisNode.errCode = ERR_CODE_NONE;
        STM_LOGD("Consumer", "----> check ok ");
      }
      else if (countCheck == 70) {
        STM_LOGE("Consumer", "----> check failed");
        isAck = false;
        thisNode.errCode = (thisNode.relayState == RELAY_STATE_ON) ? ERR_CODE_LIGHT_ON_FAILED : ERR_CODE_LIGHT_OFF_FAILED;
        thisNode.relayState = !thisNode.relayState;
        RELAY_CONTROL(thisNode.relayState);
      }
      else {
        ADC_READ_LIGHTSENSOR();
        osDelay(10);
        STM_LOGD("Consumer", "----> check again");
      }
    }
  }
  else
  {
    isAck = true;
    thisNode.errCode = ERR_CODE_NONE;
    STM_LOGV("Consumer", "----> already %s", WHICH_RELAY(thisNode.relayState));
  }

  if (isAck) {
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_RELAY_CONTROL);
  }
  else {
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_FAILED, seqID, OPCODE_RESPOSNE_RELAY_CONTROL);
  }
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void opcodeMcuReset(void)
{
  STM_LOGV("Consumer", "Perform self reset");
  NVIC_SystemReset();
}

static void opcodeLocationUpdate(uint8_t newLocation, uint8_t seqID)
{
  uint8_t msg[10];
  if (newLocation != LOCATION_NONE)
  {
    thisNode.location = newLocation;
    STM_LOGV("Consumer", "----> Update location %d", newLocation);
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_LOCATION_UPDATE);
  }
  else
  {
    STM_LOGV("Consumer", "Invalid data %d", newLocation);
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_FAILED, seqID, OPCODE_RESPOSNE_LOCATION_UPDATE);
  }
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
