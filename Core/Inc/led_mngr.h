#ifndef __LED_MNGR_H
#define __LED_MNGR_H

#include "stm32f1xx_hal.h"
#include "misc.h"
#include "main.h"
#include "data-format.h"

enum {
    BLINK_ERR_NONE = 1,
    BLINK_ERR_LIGHT_ON_FAILED = 4,
    BLINK_ERR_LIGHT_OFF_FAILED = 5,
};


void LedControl(NodeErrCodeTypeDef_t err, uint32_t idleTimeMs, uint32_t parenDelayMs);

#endif
