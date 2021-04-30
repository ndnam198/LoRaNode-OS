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

/* USER CODE END Variables */
/* Definitions for taskProducer */
osThreadId_t taskProducerHandle;
const osThreadAttr_t taskProducer_attributes = {
  .name = "taskProducer",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for taskConsumer */
osThreadId_t taskConsumerHandle;
const osThreadAttr_t taskConsumer_attributes = {
  .name = "taskConsumer",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for taskPeriodic */
osThreadId_t taskPeriodicHandle;
const osThreadAttr_t taskPeriodic_attributes = {
  .name = "taskPeriodic",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityLow4,
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
extern NodeTypedef_t thisNode;
extern uint32_t adcLightSensor;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);
void entryProducer(void* argument);
void entryConsumer(void* argument);
void entryPeriodic(void* argument);

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
  rxDoneSemaphoreHandle = osSemaphoreNew(5, 0, &rxDoneSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  if (rxDoneSemaphoreHandle == NULL)
    STM_LOGE("ERROR", "create rxDoneSemaphoreHandle failed");
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew(10, sizeof(uint8_t) * 10, &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  if (myQueue01Handle == NULL)
    STM_LOGE("ERROR", "create myQueue01Handle failed");

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

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
  STM_LOGI("MileStone", "Kernel starts");
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_entryProducer */
/**
* @brief Function implementing the taskProducer thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entryProducer */
void entryProducer(void* argument)
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

      STM_LOGD("Producer", "Get semaphore ok");

      if (LoRaGetITFlag(PAYLOAD_CRC_ERROR_MskPos) == 1)
      {
        STM_LOGW("Producer", "Payload CRC failed");
      }
      else
      {
        LORA_SET_FIFO_CURRENT_MSG();
        for (uint8_t i = 0; i < PAYLOAD_LENGTH; i++) {
          receivedMsg[i] = ucSpi1Read(RegFifo);
          // STM_LOGI("Producer", "receivedData[%d]: %x", i, receivedMsg[i]);
        }

        STM_LOGV("Producer", "receiveNodeID: %x - thisNodeID: %x", receivedMsg[INDEX_DEST_ID], thisNode.nodeID);
        if (receivedMsg[INDEX_DEST_ID] == thisNode.nodeID)
        {

          STM_LOGD("Producer", "put queue");
          err = osMessageQueuePut(myQueue01Handle, receivedMsg, 0, tickToWait);
          if (!err)
          {
            STM_LOGD("Producer", "put queue ok");
          }
          else
          {
            STM_LOGW("Producer", "put queue failed, err %d\n\r NbOfMsg in queue : % d\n\ravailable size : % d", \
              err, \
              osMessageQueueGetCount(myQueue01Handle), \
              osMessageQueueGetSpace(myQueue01Handle));
          }
        }
        else
        {
          STM_LOGW("Producer", "msg not matched --> dicarded");
        }
      }
      /* CLEAR RX_DONE FLAG */
      // vModeInit(STDBY_MODE);
      STM_LOGD("Producer", "Clear flag");
      vSpi1Write(RegIrqFlags, RX_DONE_Msk | PAYLOAD_CRC_ERROR_Msk);
      // STM_LOGD("Producer", "Reset RxPointer");
      // vModeInit(RXCONTINUOUS_MODE);
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
void entryConsumer(void* argument)
{
  /* USER CODE BEGIN entryConsumer */
  osStatus_t err;
  static bool isAck;
  // static bool isChecking = false;
  // static uint32_t countCheck;
  static uint8_t receivedMsgFromQueue[10];

  /* Infinite loop */
  for (;;)
  {
    err = osMessageQueueGet(myQueue01Handle, receivedMsgFromQueue, 0, portMAX_DELAY);
    if (!err)
    {

      printf("\r\n");
      STM_LOGD("Consumer", "Get queue ok");
      for (size_t i = 0; i < PAYLOAD_LENGTH; i++)
      {
        STM_LOGV("Consumer", "msg from queue: receivedMsgFromQueue[%d]: %x", i, receivedMsgFromQueue[i]);
      }

      STM_LOGV("Consumer", "current : location: %d - relaySts: %s", thisNode.location, WHICH_RELAY(thisNode.relayState));
      STM_LOGV("Consumer", "msg info: location: %d - relaySts: %s", receivedMsgFromQueue[INDEX_DATA_LOCATION], WHICH_RELAY(receivedMsgFromQueue[INDEX_DATA_RELAY_STATE]));

      if (receivedMsgFromQueue[INDEX_DATA_LOCATION] != LOCATION_UNKNOWN)
      {
        thisNode.location = receivedMsgFromQueue[INDEX_DATA_LOCATION];
        STM_LOGI("Consumer", "----> Update location %d", receivedMsgFromQueue[INDEX_DATA_LOCATION]);
      }

      if (thisNode.relayState != receivedMsgFromQueue[INDEX_DATA_RELAY_STATE] && thisNode.errCode == ERR_CODE_NONE)
      {
        bool isChecking = true;
        uint32_t countCheck = 0;

        taskENTER_CRITICAL();
        thisNode.relayState = receivedMsgFromQueue[INDEX_DATA_RELAY_STATE];
        taskEXIT_CRITICAL();
        STM_LOGI("Consumer", "----> Update relay state to %s", WHICH_RELAY(receivedMsgFromQueue[INDEX_DATA_RELAY_STATE]));
        RELAY_CONTROL(thisNode.relayState);

        vModeInit(STDBY_MODE);
        STM_LOGD("Consumer", "Test relay ...");
        STM_LOGD("Consumer", "Invoke ADC");
        ADC_READ_LIGHTSENSOR();

        while (isChecking && ++countCheck <= 70) {
          STM_LOGD("Consumer", "adcLightSensor: %d - relayState: %d - count: %d - isChecking: %d", adcLightSensor, thisNode.relayState, countCheck, isChecking);

          if (((adcLightSensor < LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_ON)) ||
            ((adcLightSensor >= LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_OFF)))
          {
            countCheck = 0;
            isChecking = false;
            isAck = true;
            STM_LOGW("Consumer", "----> check ok ");
          }
          else if (countCheck == 70) {
            STM_LOGW("Consumer", "----> check failed");
            isAck = false;
            receivedMsgFromQueue[INDEX_DATA_ERR_CODE] = (thisNode.relayState == RELAY_STATE_ON) ? ERR_CODE_LIGHT_ON_FAILED : ERR_CODE_LIGHT_OFF_FAILED;
          }
          else {
            ADC_READ_LIGHTSENSOR();
            osDelay(10);
            STM_LOGW("Consumer", "----> check again");
          }
        }
      }
      else {
        isAck = true;
        STM_LOGI("Consumer", "Relay already satisfies");
      }

      receivedMsgFromQueue[INDEX_SOURCE_ID] = thisNode.nodeID;
      receivedMsgFromQueue[INDEX_DEST_ID] = GATEWAY_ADDRESS;
      receivedMsgFromQueue[INDEX_MSG_TYPE] = MSG_TYPE_RESPONSE;
      receivedMsgFromQueue[INDEX_DATA_LOCATION] = thisNode.location;
      receivedMsgFromQueue[INDEX_DATA_RELAY_STATE] = thisNode.relayState;
      if (isAck) {
        /* TODO: Send ACK */
        receivedMsgFromQueue[INDEX_MSG_STATUS] = MSG_STS_OK;
        receivedMsgFromQueue[INDEX_DATA_ERR_CODE] = ERR_CODE_NONE;
      }
      else
      {
        /* TODO: Send NACK + ERR_CODE */
        receivedMsgFromQueue[INDEX_MSG_STATUS] = MSG_STS_FAILED;

      }
      LoRaTransmit(receivedMsgFromQueue, PAYLOAD_LENGTH, 5000);
      vModeInit(RXCONTINUOUS_MODE);
    }
  }
}
/* USER CODE END entryConsumer */

/* USER CODE BEGIN Header_entryPeriodic */
/**
* @brief Function implementing the taskPeriodic thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_entryPeriodic */
void entryPeriodic(void* argument)
{
  /* USER CODE BEGIN entryPeriodic */
  static uint32_t tickToWait = pdMS_TO_TICKS(500);
  /* Infinite loop */
  for (;;)
  {
    TOGGLE_LED();
    osDelay(tickToWait);
  }
  /* USER CODE END entryPeriodic */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
