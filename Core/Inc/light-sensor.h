#ifndef __LIGHT_SENSOR_H
#define __LIGHT_SENSOR_H
#include "main.h"
#include "adc.h"

#define WHICH_LIGHTSENSOR(adcVal) ((adcVal < LIGHTSENSOR_THRESHOLD) ? "ON" : "OFF")
#define IS_LIGHT_ON(adcVal) ((adcVal < LIGHTSENSOR_THRESHOLD) ? true : false)
#define LIGHTSENSOR_THRESHOLD (1000u)

#define ADC_READ_LIGHTSENSOR() ERROR_CHECK(HAL_ADC_Start_IT(&hadc1))

#endif /* !__LIGHT_SENSOR_H */
