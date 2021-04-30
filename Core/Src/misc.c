#include "misc.h"
#include "stm_log.h"

reset_cause_t resetCauseGet(void)
{
    /*
		RCC_FLAG_BORRST: BOR reset flag
		RCC_FLAG_PINRST: NRST pin reset flag
		RCC_FLAG_PORRST: POR/PDR reset flag
		RCC_FLAG_SFTRST: Software reset flag
		RCC_FLAG_IWDGRST: Independent watchdog reset flag
		RCC_FLAG_WWDGRST: Window watchdog reset flag
		RCC_FLAG_LPWRRST: Low power reset flag 
	*/
    reset_cause_t reset_cause;

    if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST))
    {
        reset_cause = eRESET_CAUSE_LOW_POWER_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST))
    {
        reset_cause = eRESET_CAUSE_WINDOW_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST))
    {
        reset_cause = eRESET_CAUSE_INDEPENDENT_WATCHDOG_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))
    {
        reset_cause = eRESET_CAUSE_SOFTWARE_RESET; // This reset is induced by calling the ARM CMSIS `NVIC_SystemReset()` function!
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PORRST))
    {
        reset_cause = eRESET_CAUSE_POWER_ON_POWER_DOWN_RESET;
    }
    else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))
    {
        reset_cause = eRESET_CAUSE_EXTERNAL_RESET_PIN_RESET;
    }
    else
    {
        reset_cause = eRESET_CAUSE_UNKNOWN;
    }

    __HAL_RCC_CLEAR_RESET_FLAGS();

    return reset_cause;
}

const char *resetCauseGetName(reset_cause_t reset_cause)
{
    const char *reset_cause_name = "";

    switch (reset_cause)
    {
    case eRESET_CAUSE_UNKNOWN:
        reset_cause_name = "UNKNOWN";
        break;
    case eRESET_CAUSE_LOW_POWER_RESET:
        reset_cause_name = "LOW_POWER_RESET";
        break;
    case eRESET_CAUSE_WINDOW_WATCHDOG_RESET:
        reset_cause_name = "WINDOW_WATCHDOG_RESET";
        break;
    case eRESET_CAUSE_INDEPENDENT_WATCHDOG_RESET:
        reset_cause_name = "INDEPENDENT_WATCHDOG_RESET";
        break;
    case eRESET_CAUSE_SOFTWARE_RESET:
        reset_cause_name = "SOFTWARE_RESET";
        break;
    case eRESET_CAUSE_POWER_ON_POWER_DOWN_RESET:
        reset_cause_name = "POWER-ON_RESET (POR) / POWER-DOWN_RESET (PDR)";
        break;
    case eRESET_CAUSE_EXTERNAL_RESET_PIN_RESET:
        reset_cause_name = "EXTERNAL_RESET_PIN_RESET";
        break;
    case eRESET_CAUSE_BROWNOUT_RESET:
        reset_cause_name = "BROWNOUT_RESET (BOR)";
        break;
    }
    return reset_cause_name;
}

uint32_t iwdgInit(IWDG_HandleTypeDef *hiwdg, uint32_t millis)
{
    uint32_t configTime = millis;
    if (configTime > PRESCALER_256_UPPER_LIMIT)
    {
        configTime = PRESCALER_256_UPPER_LIMIT;
    }
    /* Select INDEPENDENT_WATCHDOG */
    hiwdg->Instance = IWDG;
    /* Use prescaler LSI/128 */
    hiwdg->Init.Prescaler = IWDG_PRESCALER_128;
    hiwdg->Init.Reload = (int)(IWDG_RESOLUTION * ((float)configTime / PRESCALER_128_UPPER_LIMIT));
    if (HAL_IWDG_Init(hiwdg) != HAL_OK)
    {
        STM_LOGE("IWDG", "Watchdog init failed");
    }
    return configTime;
}
