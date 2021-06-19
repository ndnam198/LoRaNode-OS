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
#include "led_mngr.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PRODUCER_TAG  "PRODUCER"
#define CONSUMER_TAG  "CONSUMER"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern NodeTypedef_t thisNode;
extern uint32_t adcLightSensor;
extern IWDG_HandleTypeDef hiwdg;

static StateMachineTypeDef_t stateMachine = STATE_WAIT_FOR_MSG;

/* USER CODE END Variables */
/* Definitions for taskProducer */
osThreadId_t taskProducerHandle;
const osThreadAttr_t taskProducer_attributes = {
  .name = "taskProducer",
  .stack_size = 1200 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for taskConsumer */
osThreadId_t taskConsumerHandle;
const osThreadAttr_t taskConsumer_attributes = {
  .name = "taskConsumer",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t)osPriorityNormal3,
};
/* Definitions for taskPeriodic */
osThreadId_t taskPeriodicHandle;
const osThreadAttr_t taskPeriodic_attributes = {
  .name = "taskPeriodic",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};
/* Definitions for myQueue01 */
osMessageQueueId_t myQueue01Handle;
const osMessageQueueAttr_t myQueue01_attributes = {
  .name = "myQueue01"
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
static void opcodeFriendNodeIdUpdate(uint8_t newFriendNodeID, uint8_t seqID);

static void stateChange(StateMachineTypeDef_t newState, int caller);
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

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of rxDoneSemaphore */
  rxDoneSemaphoreHandle = osSemaphoreNew(5, 0, &rxDoneSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  STM_ERROR_CHECK(rxDoneSemaphoreHandle == NULL, "create rxDoneSemaphoreHandle failed");
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of myQueue01 */
  myQueue01Handle = osMessageQueueNew(10, 10, &myQueue01_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  STM_ERROR_CHECK(myQueue01Handle == NULL, "create myQueue01Handle failed");

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of taskProducer */
  taskProducerHandle = osThreadNew(entryProducer, NULL, &taskProducer_attributes);

  /* creation of taskConsumer */
  taskConsumerHandle = osThreadNew(entryConsumer, NULL, &taskConsumer_attributes);

  /* creation of taskPeriodic */
  taskPeriodicHandle = osThreadNew(entryPeriodic, NULL, &taskPeriodic_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  STM_ERROR_CHECK(taskProducerHandle == NULL, "create taskProducerHandle failed");
  STM_ERROR_CHECK(taskConsumerHandle == NULL, "create taskConsumerHandle failed");
  STM_ERROR_CHECK(taskPeriodicHandle == NULL, "create taskPeriodicHandle failed");
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
void StartDefaultTask(void* argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;)
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
void entryProducer(void* argument)
{
  /* USER CODE BEGIN entryProducer */
  osStatus_t err;
  uint8_t receivedMsg[PAYLOAD_LENGTH];
  uint32_t tickToWait = pdMS_TO_TICKS(5000);
  /* Infinite loop */
  for (;;)
  {
    err = osSemaphoreAcquire(rxDoneSemaphoreHandle, portMAX_DELAY);
    if (!err)
    {
      STM_LOGV(PRODUCER_TAG, "Get semaphore ok");
      if (LoRaGetITFlag(PAYLOAD_CRC_ERROR_MskPos) == 1)
      {
        STM_LOGW(PRODUCER_TAG, "Payload CRC failed");
        stateChange(STATE_WAIT_FOR_MSG, __LINE__);
      }
      else
      {
        stateChange(STATE_MESSAGE_PROCESSING, __LINE__);
        vGetLoRaData(receivedMsg, PAYLOAD_LENGTH);

        STM_LOGI(PRODUCER_TAG, "---> Got message: source: %d - dest: %d - this: %d",
          receivedMsg[INDEX_SOURCE_ID],
          receivedMsg[INDEX_DEST_ID],
          thisNode.nodeID);

        if (receivedMsg[INDEX_DEST_ID] == BROADCAST_ADDRESS || receivedMsg[INDEX_DEST_ID] == thisNode.nodeID)
        {
          if (receivedMsg[INDEX_MSG_TYPE] == MSG_TYPE_REQUEST
            && receivedMsg[INDEX_SOURCE_ID] == GATEWAY_ADDRESS)
          {
            STM_ERROR_CHECK((err = osMessageQueuePut(myQueue01Handle, receivedMsg, 0, tickToWait)) != osOK, "enqueue failed, err %d\n\r NbOfMsg in queue : % d\n\ravailable size : % d", \
              err, \
              osMessageQueueGetCount(myQueue01Handle), \
              osMessageQueueGetSpace(myQueue01Handle));

            if (err == osOK)
            {
              STM_LOGV(PRODUCER_TAG, "enqueue ok");
            }
            else
            {
              stateChange(STATE_WAIT_FOR_MSG, __LINE__);
            }
          }
          else
          {
            STM_LOGW(PRODUCER_TAG, "Msg type not valid: {%s}", WHICH_MSG_TYPE(receivedMsg[INDEX_MSG_TYPE]));
          }
        }
        else if (thisNode.friendNodeID != UNUSED_ADDRESS
          && (receivedMsg[INDEX_DEST_ID] == thisNode.friendNodeID || receivedMsg[INDEX_SOURCE_ID] == thisNode.friendNodeID))
        {
          STM_LOGD(PRODUCER_TAG, "Forward friend node msg");
          receivedMsg[INDEX_TIME_TO_LIVE] = TIME_TO_LIVE_NONE;
          LoRaTransmit(receivedMsg, PAYLOAD_LENGTH, LORA_DELAY);
          stateChange(STATE_WAIT_FOR_MSG, __LINE__);
        }
        else if (receivedMsg[INDEX_DEST_ID] != thisNode.nodeID && receivedMsg[INDEX_TIME_TO_LIVE] > TIME_TO_LIVE_NONE && receivedMsg[INDEX_TIME_TO_LIVE] <= TIME_TO_LIVE_MAX)
        {
          STM_LOGD(PRODUCER_TAG, "Relay msg, TTL: %d", receivedMsg[INDEX_TIME_TO_LIVE]);
          receivedMsg[INDEX_TIME_TO_LIVE]--;
          LoRaTransmit(receivedMsg, PAYLOAD_LENGTH, LORA_DELAY);
          stateChange(STATE_WAIT_FOR_MSG, __LINE__);
        }
        else
        {
          STM_LOGV(PRODUCER_TAG, "not matched ---> ignore");
          stateChange(STATE_WAIT_FOR_MSG, __LINE__);
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
void entryConsumer(void* argument)
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
      STM_LOGV(CONSUMER_TAG, "Dequeue ok");
      STM_LOGI(CONSUMER_TAG, "Request opcode: %d", receivedMsgFromQueue[INDEX_COMMAND_OPCODE]);
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
        break;
      case OPCODE_REQUEST_LOCATION_UPDATE:
        opcodeLocationUpdate(receivedMsgFromQueue[INDEX_DATA_LOCATION], receivedMsgFromQueue[INDEX_SEQUENCE_ID]);
        break;
      case OPCODE_REQUEST_FRIEND_NODE_ID_UPDATE:
        opcodeFriendNodeIdUpdate(receivedMsgFromQueue[INDEX_DATA_FRIEND_NODE_ID], receivedMsgFromQueue[INDEX_SEQUENCE_ID]);
        break;
      default:
        STM_LOGW(CONSUMER_TAG, "Opcode not found %d", receivedMsgFromQueue[INDEX_COMMAND_OPCODE]);
        break;
      }

      updateDataToFlash();
      stateChange(STATE_WAIT_FOR_MSG, __LINE__);
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
void entryPeriodic(void* argument)
{
  /* USER CODE BEGIN entryPeriodic */
  static const uint32_t DELAY_MS = 500;
  static const uint32_t tickToWait = pdMS_TO_TICKS(DELAY_MS);
  /* Infinite loop */
  for (;;)
  {
    LedControl(thisNode.errCode, thisNode.relayState, 3 * DELAY_MS, DELAY_MS);
    if (stateMachine == STATE_WAIT_FOR_MSG && ucGetLoraWorkingMode() != RXCONTINUOUS_MODE)
    {
      vModeInit(RXCONTINUOUS_MODE);
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
  uint8_t msg[PAYLOAD_LENGTH];
  PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_STATE, TIME_TO_LIVE_DEFAULT);
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void opcodeRelayControl(uint8_t newState, uint8_t seqID)
{
  uint8_t msg[PAYLOAD_LENGTH];
  uint8_t countCheck = 0;
  bool isAck = false;
  bool isChecking = true;
  if (thisNode.relayState != newState)
  {
    STM_LOGI(CONSUMER_TAG, "State changes: {%s} to {%s}", WHICH_RELAY(thisNode.relayState), WHICH_RELAY(newState));
    thisNode.relayState = newState;
    RELAY_CONTROL(thisNode.relayState);
    STM_LOGV(CONSUMER_TAG, "Checking relay ...");
    ADC_READ_LIGHTSENSOR();
    while (isChecking && ++countCheck <= NB_OF_RELAY_CHECK) {
      STM_LOGD(CONSUMER_TAG, "adcLightSensor: %d - count: %d", adcLightSensor, countCheck);

      if (((adcLightSensor < LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_ON)) ||
        ((adcLightSensor >= LIGHTSENSOR_THRESHOLD) && (thisNode.relayState == RELAY_STATE_OFF)))
      {
        isChecking = false;
        isAck = true;
        thisNode.errCode = ERR_CODE_NONE;
        STM_LOGD(CONSUMER_TAG, "----> check ok ");
      }
      else if (countCheck == NB_OF_RELAY_CHECK) {
        STM_LOGW(CONSUMER_TAG, "----> check failed");
        isAck = false;
        thisNode.errCode = (thisNode.relayState == RELAY_STATE_ON) ? ERR_CODE_LIGHT_ON_FAILED : ERR_CODE_LIGHT_OFF_FAILED;
        thisNode.relayState = !thisNode.relayState;
        RELAY_CONTROL(thisNode.relayState);
      }
      else {
        ADC_READ_LIGHTSENSOR();
        osDelay(10);
        STM_LOGD(CONSUMER_TAG, "----> check again");
      }
    }
  }
  else
  {
    isAck = true;
    thisNode.errCode = ERR_CODE_NONE;
    STM_LOGV(CONSUMER_TAG, "----> already %s", WHICH_RELAY(thisNode.relayState));
  }

  if (isAck) {
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_RELAY_CONTROL, TIME_TO_LIVE_DEFAULT);
  }
  else {
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_FAILED, seqID, OPCODE_RESPOSNE_RELAY_CONTROL, TIME_TO_LIVE_DEFAULT);
  }
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void opcodeMcuReset(void)
{
  STM_LOGV(CONSUMER_TAG, "Perform self reset");
  NVIC_SystemReset();
}

static void opcodeLocationUpdate(uint8_t newLocation, uint8_t seqID)
{
  uint8_t msg[PAYLOAD_LENGTH];
  if (newLocation != LOCATION_NONE)
  {
    thisNode.location = newLocation;
    STM_LOGI(CONSUMER_TAG, "----> Update location %d", newLocation);
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_LOCATION_UPDATE, TIME_TO_LIVE_DEFAULT);
  }
  else
  {
    STM_LOGW(CONSUMER_TAG, "Invalid data %d", newLocation);
    PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_FAILED, seqID, OPCODE_RESPOSNE_LOCATION_UPDATE, TIME_TO_LIVE_DEFAULT);
  }
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void opcodeFriendNodeIdUpdate(uint8_t newFriendNodeID, uint8_t seqID)
{
  uint8_t msg[PAYLOAD_LENGTH];
  if (thisNode.friendNodeID != newFriendNodeID && thisNode.nodeID != newFriendNodeID)
  {
    thisNode.friendNodeID = newFriendNodeID;
  }
  PACK_RESPONSE_MSG(msg, thisNode, MSG_STS_OK, seqID, OPCODE_RESPOSNE_FRIEND_NODE_ID_UPDATE, TIME_TO_LIVE_DEFAULT);
  LoRaTransmit(msg, PAYLOAD_LENGTH, LORA_DELAY);
}

static void stateChange(StateMachineTypeDef_t newState, int caller)
{
  STM_LOGI("STATE MACHINE", "CALLER: %d - {%s} to {%s}",
    caller,
    WHICH_STATE(stateMachine),
    WHICH_STATE(newState));

  if (newState == STATE_WAIT_FOR_MSG)
  {
    vModeInit(RXCONTINUOUS_MODE);
  }
  else if (newState == STATE_MESSAGE_PROCESSING)
  {
    vModeInit(STDBY_MODE);
  }
  stateMachine = newState;
}


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
