#include "main.h"
#include "spi.h"
#include "gpio.h"
#include "SystemClock_Config.h"
#include "icm20602.h"
#include <math.h>
#include "tim.h"
#include "MG513.h"
#include "selfUART.h"
#include "MG513_Encoder.h"
#include "usart.h"
#include "pid.h"
#include "Balance.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI2_Init();
    MX_TIM1_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM6_Init();
    MX_USART6_UART_Init();
    MG513_Encoder_Init();

    if (ICM20602_Init() == 1) {
        UART_Printf("ICM20602 init failed\n");
        while (1);
    } else {
        UART_Printf("ICM20602 init success\n");
        ICM20602_Calibrate();
        HAL_Delay(1000);
    }

    MG513_Init();
    Balance_Init();
    HAL_Delay(500);

    HAL_TIM_Base_Start_IT(&htim6);

    while (1)
{
	       UART_Printf("v=%.3f corr=%.2f tgt=%.2f  L=%.3f R=%.3f\n",
                bal_avg_speed, bal_correction, bal_angle_target,
                speed_L, speed_R);
         HAL_Delay(300);
}
}

/* 定时器中断（副循环） */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6)
    {
        /* ==== 1ms 执行一次 ==== */
        ICM20602_Update();
        Encoder_Update();
        Balance_Update();
    }
}