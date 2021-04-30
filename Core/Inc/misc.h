#ifndef __MISC_H
#define __MISC_H

#include "main.h"
#include "stm_log.h"

#define WATCHDOG_TIME (10000u)

typedef enum reset_cause
{
    eRESET_CAUSE_UNKNOWN = 0,
    eRESET_CAUSE_LOW_POWER_RESET,            /*  */
    eRESET_CAUSE_WINDOW_WATCHDOG_RESET,      /*  */
    eRESET_CAUSE_INDEPENDENT_WATCHDOG_RESET, /* IWDG Timeout */
    eRESET_CAUSE_SOFTWARE_RESET,             /* Reset caused by NVIC_SystemReset() */
    eRESET_CAUSE_POWER_ON_POWER_DOWN_RESET,  /*  */
    eRESET_CAUSE_EXTERNAL_RESET_PIN_RESET,   /* Low signal on NRST pin | Reset pin pushed */
    eRESET_CAUSE_BROWNOUT_RESET,             /*  */
} reset_cause_t;


/* Independent watchdog constant  */
#define PRESCALER_128_UPPER_LIMIT (13107u)
#define PRESCALER_256_UPPER_LIMIT (26214u)
#define IWDG_RESOLUTION (4095u)

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)       \
    (byte & 0x80 ? '1' : '0'),     \
        (byte & 0x40 ? '1' : '0'), \
        (byte & 0x20 ? '1' : '0'), \
        (byte & 0x10 ? '1' : '0'), \
        (byte & 0x08 ? '1' : '0'), \
        (byte & 0x04 ? '1' : '0'), \
        (byte & 0x02 ? '1' : '0'), \
        (byte & 0x01 ? '1' : '0')

reset_cause_t resetCauseGet(void);
const char *resetCauseGetName(reset_cause_t reset_cause);
uint32_t iwdgInit(IWDG_HandleTypeDef *hiwdg, uint32_t millis);

#endif // !