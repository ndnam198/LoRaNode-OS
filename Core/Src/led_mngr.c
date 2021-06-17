#include "led_mngr.h"
#include <stdbool.h>

void LedControl(NodeErrCodeTypeDef_t err, uint32_t idleTimeMs, uint32_t parenDelayMs)
{
    static int8_t cacheErr = -1;
    static uint8_t blinkTime;
    static uint8_t countBlinkTime;
    static int8_t idleTime = -1;
    static uint8_t countIdleTime;
    static bool isLedOn;

    // printf("countBlinkTime: %d - countIdleTime: %d - isLedOn: %d\r\n",
    //     countBlinkTime,
    //     countIdleTime,
    //     isLedOn);

    if (cacheErr != err)
    {
        cacheErr = err;
        LED_OFF();
        isLedOn = false;
        switch (cacheErr)
        {
        case ERR_CODE_NONE:
            blinkTime = BLINK_ERR_NONE * 2;
            break;
        case ERR_CODE_LIGHT_ON_FAILED:
            blinkTime = BLINK_ERR_LIGHT_ON_FAILED * 2;
            break;
        case ERR_CODE_LIGHT_OFF_FAILED:
            blinkTime = BLINK_ERR_LIGHT_OFF_FAILED * 2;
            break;
        }
    }

    if (idleTime == -1 && parenDelayMs != 0)
    {
        idleTime = (int)(idleTimeMs / parenDelayMs);
    }

    if (countBlinkTime != blinkTime)
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
        if (countIdleTime != idleTime)
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
