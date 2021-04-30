#include "stm_log.h"
#include "retarget.h"
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

void stm_log_write(stm_log_level_t level,
                   const char *tag,
                   const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    vprintf(format, arg);
    va_end(arg);
}

