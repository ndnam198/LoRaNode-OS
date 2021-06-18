#include "led_mngr.h"
#include <stdbool.h>

void LedControl(NodeErrCodeTypeDef_t err, NodeStsTypedef_t relayState, uint32_t idleTimeMs, uint32_t parenDelayMs)
{
    static int8_t cacheErr = -1;
    static int8_t cacheRelayState = -1;
    static uint8_t blinkTime;
    static uint8_t countBlinkTime;
    static int8_t idleTime = -1;
    static uint8_t countIdleTime;
    static bool isLedOn;

    // printf("countBlinkTime: %d - blinkTime: %d - countIdleTime: %d - idleTime: %d - isLedOn: %d\r\n",
    //     countBlinkTime,
    //     blinkTime,
    //     countIdleTime,
    //     idleTime,
    //     isLedOn);

    if (cacheErr != err || cacheRelayState != relayState)
    {
        cacheErr = err;
        cacheRelayState = relayState;
        LED_OFF();
        isLedOn = false;
        switch (cacheErr)
        {
        case ERR_CODE_NONE:
            if (cacheRelayState == RELAY_STATE_OFF)
            {
                blinkTime = BLINK_ERR_NONE_LIGHT_OFF * 2;
            }
            else if (cacheRelayState == RELAY_STATE_ON)
            {
                blinkTime = BLINK_ERR_NONE_LIGHT_ON * 2;
            }
            break;
        case ERR_CODE_LIGHT_ON_FAILED:
            blinkTime = BLINK_ERR_LIGHT_ON_FAILED * 2;
            break;
        case ERR_CODE_LIGHT_OFF_FAILED:
            blinkTime = BLINK_ERR_LIGHT_OFF_FAILED * 2;
            break;
        }
        STM_LOGV("LED", "Update Led blinkType %d", blinkTime);
    }

    if (idleTime == -1 && parenDelayMs != 0)
    {
        idleTime = (int)(idleTimeMs / parenDelayMs);
    }

    if (countBlinkTime < blinkTime)
    {
        if (isLedOn)
        {
            LED_OFF();
        }
        else
        {
            LED_ON();
        }
        countBlinkTime++;
        isLedOn = !isLedOn;
    }
    else
    {
        if (countIdleTime < idleTime)
        {
            if (countIdleTime == 0)
            {
                LED_OFF();
                isLedOn = false;
            }
            countIdleTime++;
        }
        else
        {
            countIdleTime = 0;
            countBlinkTime = 0;
        }
    }
}
