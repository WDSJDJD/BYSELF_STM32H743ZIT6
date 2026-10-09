#include "MG513_Encoder.h"
#include "tim.h"
#include "MG513.h"
#include "main.h"

volatile float encoder_count_L = 0, encoder_count_R = 0;
volatile float RPM_L = 0, RPM_R = 0;
volatile float speed_L = 0, speed_R = 0;

/**
 * @brief 初始化编码器
 */
void MG513_Encoder_Init(void)
{
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim4, 0);
}

/**
 * @brief 计算编码器计数变化（处理 16 位溢出）
 */
static int16_t CalcDelta(uint16_t curr, uint16_t last)
{
    int32_t delta = (int32_t)curr - (int32_t)last;

    if (delta > 32767)   delta -= 65536;   /* 下溢 */
    if (delta < -32768)  delta += 65536;   /* 上溢 */

    return (int16_t)delta;
}

/**
 * @brief 读取单个电机的计数增量
 */
int16_t Encoder_GetSpeed(uint8_t motor_id)
{
    TIM_HandleTypeDef *htim;
    static uint16_t last_L = 0, last_R = 0;
    uint16_t curr;
    int16_t delta;

    if (motor_id == 0) {
        htim  = &htim3;
        curr  = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
        delta = CalcDelta(curr, last_L);
        last_L = curr;
    } else if (motor_id == 1) {
        htim  = &htim4;
        curr  = (uint16_t)__HAL_TIM_GET_COUNTER(htim);
        delta = CalcDelta(curr, last_R);
        last_R = curr;
    } else {
        delta = 0;
    }
    return delta;
}

/**
 * @brief 计算 RPM
 */
static void rpm_of_motor(void)
{
    int16_t left_speed  = Encoder_GetSpeed(LEFT_MOTOR_ID);
    int16_t right_speed = Encoder_GetSpeed(RIGHT_MOTOR_ID);

    float rps_l = (float)left_speed  / (PULSES_PER_REV * SAMPLE_PERIOD_S);
    float rps_r = (float)right_speed / (PULSES_PER_REV * SAMPLE_PERIOD_S);

    RPM_L = rps_l * 60.0f;
    RPM_R = rps_r * 60.0f;
}

/**
 * @brief 计算线速度（m/s）
 * 注意：× 2.0f 是因为左右轮电机安装方向相反，需要反向补偿
 *       如果你确认不需要 × 2，请自行去掉（会影响速度环增益量级）
 */
static void speed_of_motor(void)
{
    speed_L = - RPM_L * WHEEL_CIRCUMFERENCE * 2.0f / 60.0f;
    speed_R =   RPM_R * WHEEL_CIRCUMFERENCE * 2.0f / 60.0f;
}

/**
 * @brief 编码器更新（在 TIM6 1ms 中断里调用）
 * 内部每 20ms 采样一次，并做一阶低通滤波
 */
void Encoder_Update(void)
{
    static uint8_t div = 0;
    static float spd_L_f = 0.0f, spd_R_f = 0.0f;

    /* ★ 每 20ms 采样一次（1ms 中断 × 20 次） */
    if (++div < ENCODER_DIV) return;
    div = 0;

    rpm_of_motor();
    speed_of_motor();

    /* ★ 一阶低通滤波，alpha = 0.5，抑制残余噪声 */
    spd_L_f = 0.5f * spd_L_f + 0.5f * speed_L;
    spd_R_f = 0.5f * spd_R_f + 0.5f * speed_R;

    speed_L = spd_L_f;
    speed_R = spd_R_f;
}