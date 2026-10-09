#include "pid.h"
#include <math.h>

void PID_Init(PID_t *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->out_min = out_min;
    pid->out_max = out_max;

    /* 积分限幅，可按需修改 */
    pid->integral_min = -10000.0f;
    pid->integral_max =  10000.0f;

    pid->deadband = 0.0f;

    PID_Reset(pid);
}

void PID_SetTarget(PID_t *pid, float target)
{
    pid->target = target;
}

void PID_Reset(PID_t *pid)
{
    pid->target = 0.0f;
    pid->actual = 0.0f;
    pid->err = 0.0f;
    pid->last_err = 0.0f;
    pid->prev_err = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

float PID_Calc(PID_t *pid, float actual, float dt)
{
    if (dt <= 0.0f) dt = 0.001f;

    pid->actual = actual;

    float err = pid->target - actual;

    if (fabsf(err) < pid->deadband)
    {
        err = 0.0f;
    }

    pid->err = err;

    /* 积分 */
    pid->integral += err * dt;

    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;

    if (pid->integral < pid->integral_min)
        pid->integral = pid->integral_min;

    /* 微分 */
    float derivative = (err - pid->last_err) / dt;

    /* 位置式 PID */
    float out = pid->Kp * err
              + pid->Ki * pid->integral
              + pid->Kd * derivative;

    /* 输出限幅 */
    if (out > pid->out_max) out = pid->out_max;
    if (out < pid->out_min) out = pid->out_min;

    pid->last_err = err;
    pid->output = out;

    return out;
}

float PID_CalcInc(PID_t *pid, float actual, float dt)
{
    if (dt <= 0.0f) dt = 0.001f;

    pid->actual = actual;

    float err = pid->target - actual;

    float delta = pid->Kp * (err - pid->last_err)
                + pid->Ki * err * dt
                + pid->Kd * (err - 2.0f * pid->last_err + pid->prev_err) / dt;

    pid->output += delta;

    if (pid->output > pid->out_max) pid->output = pid->out_max;
    if (pid->output < pid->out_min) pid->output = pid->out_min;

    pid->prev_err = pid->last_err;
    pid->last_err = err;

    return pid->output;
}