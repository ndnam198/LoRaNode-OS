#ifndef __STM_LOG_H
#define __STM_LOG_H

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include <sys/stat.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    STM_LOG_NONE = 0,       /*!< No log output */
    STM_LOG_ERROR = 1,      /*!< Critical errors, software module can not recover on its own */
    STM_LOG_WARN = 2,       /*!< Error conditions from which recovery measures have been taken */
    STM_LOG_INFO = 3,       /*!< Information messages which describe normal flow of events */
    STM_LOG_DEBUG = 4,      /*!< Extra information which is not necessary for normal use (values, pointers, sizes, etc). */
    STM_LOG_VERBOSE = 5     /*!< Bigger chunks of debugging information, or frequent messages which can potentially flood the output. */
} stm_log_level_t;

#define LOG_LOCAL_LEVEL   STM_LOG_VERBOSE

#define LOG_COLOR_BLACK   "30"
#define LOG_COLOR_RED     "31"
#define LOG_COLOR_GREEN   "32"
#define LOG_COLOR_BROWN   "33"
#define LOG_COLOR_BLUE    "34"
#define LOG_COLOR_PURPLE  "35"
#define LOG_COLOR_CYAN    "36"
#define LOG_COLOR(COLOR)  "\033[0;" COLOR "m"
#define LOG_BOLD(COLOR)   "\033[1;" COLOR "m"
#define LOG_RESET_COLOR   "\033[0m"
#define LOG_COLOR_E       LOG_COLOR(LOG_COLOR_RED)
#define LOG_COLOR_W       LOG_COLOR(LOG_COLOR_BROWN)
#define LOG_COLOR_I       LOG_COLOR(LOG_COLOR_GREEN)
#define LOG_COLOR_D       LOG_COLOR(LOG_COLOR_CYAN)
#define LOG_COLOR_V       

void stm_log_write(stm_log_level_t level, const char *tag, const char *format, ...);

#define LOG_FORMAT(letter, format)  LOG_COLOR_ ## letter " " #letter " (%d) %s - %s {%d}: " format LOG_RESET_COLOR "\r\n"

#define STM_LOGE( tag, format, ... )  if (LOG_LOCAL_LEVEL >= STM_LOG_ERROR)   { stm_log_write(STM_LOG_ERROR,   tag, LOG_FORMAT(E, format), HAL_GetTick(), tag, __func__, __LINE__, ##__VA_ARGS__); }

#define STM_LOGW( tag, format, ... )  if (LOG_LOCAL_LEVEL >= STM_LOG_WARN)    { stm_log_write(STM_LOG_WARN,    tag, LOG_FORMAT(W, format), HAL_GetTick(), tag, __func__, __LINE__, ##__VA_ARGS__); }

#define STM_LOGI( tag, format, ... )  if (LOG_LOCAL_LEVEL >= STM_LOG_INFO)    { stm_log_write(STM_LOG_INFO,    tag, LOG_FORMAT(I, format), HAL_GetTick(), tag, __func__, __LINE__, ##__VA_ARGS__); }

#define STM_LOGD( tag, format, ... )  if (LOG_LOCAL_LEVEL >= STM_LOG_DEBUG)   { stm_log_write(STM_LOG_DEBUG,   tag, LOG_FORMAT(D, format), HAL_GetTick(), tag, __func__, __LINE__, ##__VA_ARGS__); }

#define STM_LOGV( tag, format, ... )  if (LOG_LOCAL_LEVEL >= STM_LOG_VERBOSE) { stm_log_write(STM_LOG_VERBOSE, tag, LOG_FORMAT(V, format), HAL_GetTick(), tag, __func__, __LINE__, ##__VA_ARGS__); }

#define STM_ERROR_CHECK(expression, format, ...)                     \
do                                                                   \
{                                                                    \
    if (expression)                                                  \
    {                                                                \
        STM_LOGE("ERROR", format, ##__VA_ARGS__);                    \
    }                                                                \
} while (0)

#ifdef __cplusplus
}
#endif
#endif /* __STM_LOG_H */ 
