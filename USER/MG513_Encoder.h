#ifndef __MG513_ENCODER_H__
#define __MG513_ENCODER_H__

#include "main.h"

/* ==================== 编码器硬件参数 ==================== */
#define ENCODER_PPR          13.0f      /* 编码器线数 */
#define GEAR_RATIO           30.0f      /* 减速比 */
#define ENCODER_MULTIPLIER   4.0f       /* TI12 4 倍频 */
#define WHEEL_CIRCUMFERENCE  (3.14159265f * 0.085f)  /* 轮子周长 m */

/* 每圈总脉冲数 */
#define PULSES_PER_REV  (ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLIER)
/* = 13 × 30 × 4 = 1560 脉冲/圈 */

/* ★ 采样周期 [s]，和 Encoder_Update 里的分频数一致
 * 改动历史：0.001f → 0.02f，减少量化噪声
 */
#define SAMPLE_PERIOD_S     0.02f

/* 采样分频系数（Encoder_Update 里使用） */
#define ENCODER_DIV         20          /* 20ms 采一次 */

void MG513_Encoder_Init(void);
int16_t Encoder_GetSpeed(uint8_t motor_id);
void Encoder_Update(void);

extern volatile float encoder_count_L, encoder_count_R;
extern volatile float RPM_L, RPM_R;
extern volatile float speed_L, speed_R;

#endif