#ifndef __PID_H__
#define __PID_H__

#include "main.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float target;      // 目标值
    float actual;      // 实际值
    float err;         // 当前误差
    float last_err;    // 上次误差
    float prev_err;    // 上上次误差，增量式用

    float integral;    // 积分项
    float output;      // PID 输出

    float out_min;     // 输出限幅
    float out_max;

    float integral_min;
    float integral_max;

    float deadband;    // 误差死区
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max);
void PID_SetTarget(PID_t *pid, float target);
void PID_Reset(PID_t *pid);

/* 位置式 PID，推荐用于速度环 */
float PID_Calc(PID_t *pid, float actual, float dt);

/* 增量式 PID，可选 */
float PID_CalcInc(PID_t *pid, float actual, float dt);

#endif