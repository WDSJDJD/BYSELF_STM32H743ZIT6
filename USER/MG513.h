#ifndef __MG513_H__
#define __MG513_H__
#include "main.h"

#define MAX_PWM 3999
#define ZERO_PWM 0
#define LEFT_MOTOR_ID 0
#define RIGHT_MOTOR_ID 1

void MG513_Init(void);
void MG513_SetSpeed(uint8_t motor_id, int16_t Speed);


#endif
