#include <stdarg.h>
#include <stdio.h>
#include "selfUART.h"
#include "main.h"
#include "usart.h"
void UART_Printf(const char *fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        if (len > (int)sizeof(buf)) len = sizeof(buf);  // ∑¿÷π‘ΩΩÁ
        HAL_UART_Transmit(&huart6, (uint8_t *)buf, len, 100);
    }
}