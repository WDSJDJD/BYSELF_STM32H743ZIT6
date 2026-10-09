#include "MG513.h"
#include "main.h"
#include "stdlib.h"
#include "tim.h"

void MG513_Init(void)
{
    // 初始化电机控制引脚
    // ...
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_MOE_ENABLE(&htim1);
}
void MG513_SetSpeed(uint8_t motor_id, int16_t Speed)
{
    if (Speed > MAX_PWM)  Speed = MAX_PWM;
    if (Speed < -MAX_PWM) Speed = -MAX_PWM;

    /* ★ 死区补偿：PWM 绝对值在 (0, DEADZONE] 之间时，
     *   提升到 DEADZONE；为 0 时保持 0 */
    const int16_t DEADZONE = 260;   /* ★ 实测这个值：从 200 开始试 */
    if (Speed > 0 && Speed < DEADZONE) {
        Speed = DEADZONE;
    } else if (Speed < 0 && Speed > -DEADZONE) {
        Speed = -DEADZONE;
    }

    uint16_t pwm = MAX_PWM - (uint16_t)(abs(Speed));
    if(motor_id == LEFT_MOTOR_ID)
    {
        if(Speed > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, MAX_PWM);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, pwm);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, MAX_PWM);
        }
    }
    else if(motor_id == RIGHT_MOTOR_ID)
    {
        if(Speed > 0)
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pwm);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, MAX_PWM);
        }
        else
        {
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, MAX_PWM);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, pwm);
        }
    }   
}
       
