#include "Balance.h"
#include "icm20602.h"
#include "MG513.h"
#include "MG513_Encoder.h"
#include <math.h>

/* ================= 三环 PID ================= */
PID_t angle_pid;
PID_t speed_pid;
PID_t pos_pid;

/* ================= 调试观察量 ================= */
volatile float bal_pwm_L        = 0.0f;
volatile float bal_pwm_R        = 0.0f;
volatile float bal_angle_target = 0.0f;
volatile float bal_v_target     = 0.0f;
volatile float bal_pos          = 0.0f;

/* ================= ICM20602 角速度 (extern) ================= */
extern volatile float gx, gy, gz;

/* ================= 内部状态 ================= */
static uint16_t bal_tick_cnt = 0;       /* ms 计数，分频用 */
static uint8_t  bal_enabled  = 0;       /* 1 = 正在平衡 */
static uint16_t bal_idle_cnt = 0;       /* 待机稳定计数（ms） */

/* 累积位置（编码器积分）单位：m */
static float pos_L = 0.0f;
static float pos_R = 0.0f;

volatile float bal_avg_speed    = 0.0f;    /* 平均速度 m/s */
volatile float bal_correction   = 0.0f;    /* 速度环输出的倾角修正量 */

/* =============================================================
 * 初始化：只调一次
 * ============================================================= */
void Balance_Init(void)
{
    /* 角度环：输出直接给 PWM */
    PID_Init(&angle_pid,
             BAL_ANGLE_KP, BAL_ANGLE_KI, 0.0f,
             BAL_OUT_MIN, BAL_OUT_MAX);

    /* 速度环：输出是"倾角修正量" */
    PID_Init(&speed_pid,
             BAL_SPEED_KP, BAL_SPEED_KI, BAL_SPEED_KD,
             BAL_ANGLE_TARGET_MIN, BAL_ANGLE_TARGET_MAX);

    /* 位置环：输出是"速度目标" */
    PID_Init(&pos_pid,
             BAL_POS_KP, BAL_POS_KI, BAL_POS_KD,
             BAL_POS_OUT_MIN, BAL_POS_OUT_MAX);

    /* 位置累积清零：把当前位置当作新原点 */
    pos_L   = 0.0f;
    pos_R   = 0.0f;
    bal_pos = 0.0f;

    bal_tick_cnt = 0;
    bal_idle_cnt = 0;
    bal_enabled  = 1;
}

/* =============================================================
 * 停机
 * ============================================================= */
void Balance_Stop(void)
{
    bal_enabled  = 0;
    bal_idle_cnt = 0;

    MG513_SetSpeed(LEFT_MOTOR_ID,  0);
    MG513_SetSpeed(RIGHT_MOTOR_ID, 0);

    bal_pwm_L        = 0.0f;
    bal_pwm_R        = 0.0f;
    bal_v_target     = 0.0f;
    bal_angle_target = 0.0f;

    pos_L = 0.0f;
    pos_R = 0.0f;

    PID_Reset(&angle_pid);
    PID_Reset(&speed_pid);
    PID_Reset(&pos_pid);
}

/* =============================================================
 * 位置环（外环）—— 每 20ms
 *
 * 1. 累积位置（带死区，抑制静止噪声）
 * 2. 位置 PID：目标是回到原点
 * 3. 抗积分饱和：输出饱和时清积分
 * 4. 输出作为速度环的"速度目标"
 * ============================================================= */
static void Balance_PosLoop(float dt)
{
    static uint8_t in_deadband = 0;
    float vL = speed_L;
    float vR = speed_R;

    /* ★ 死区迟滞：进入用 0.02，出来用 0.025 */
    const float dz_in  = BAL_SPD_DEADBAND;         /* 0.02 */
    const float dz_out = BAL_SPD_DEADBAND * 1.25f; /* 0.025 */

    float vmax = fmaxf(fabsf(vL), fabsf(vR));
    if (in_deadband) {
        if (vmax > dz_out) in_deadband = 0;
    } else {
        if (vmax < dz_in) in_deadband = 1;
    }

    if (in_deadband) {
        vL = 0.0f;
        vR = 0.0f;
    }

    pos_L += vL * dt;
    pos_R += vR * dt;

    float avg_pos = (pos_L + pos_R) / 2.0f;
    bal_pos = avg_pos;

    PID_SetTarget(&pos_pid, 0.0f);
    float v_target = PID_Calc(&pos_pid, avg_pos, dt);

    if (v_target >= BAL_POS_OUT_MAX || v_target <= BAL_POS_OUT_MIN) {
        pos_pid.integral = 0.0f;
    }
    if (v_target > BAL_POS_OUT_MAX) v_target = BAL_POS_OUT_MAX;
    if (v_target < BAL_POS_OUT_MIN) v_target = BAL_POS_OUT_MIN;

    bal_v_target = v_target;
}

/* =============================================================
 * 速度环（中环）—— 每 20ms
 *
 * 目标速度来自位置环输出，反馈实际平均速度
 * 输出是"倾角修正量"，叠加机械中值后给角度环
 * ============================================================= */
static void Balance_SpeedLoop(float dt)
{
    float avg_speed = (speed_L + speed_R) / 2.0f;
    bal_avg_speed = avg_speed;   /* ★ 输出到全局 */

    PID_SetTarget(&speed_pid, bal_v_target);
    float correction = - PID_Calc(&speed_pid, avg_speed, dt);
    bal_correction = correction;  /* ★ 输出到全局 */

    if (correction > BAL_ANGLE_TARGET_MAX) correction = BAL_ANGLE_TARGET_MAX;
    if (correction < BAL_ANGLE_TARGET_MIN) correction = BAL_ANGLE_TARGET_MIN;

    if (correction >= BAL_ANGLE_TARGET_MAX || correction <= BAL_ANGLE_TARGET_MIN) {
        speed_pid.integral = 0.0f;
    }

    float angle_target = BAL_PITCH_OFFSET + correction;
    bal_angle_target = angle_target;
    PID_SetTarget(&angle_pid, angle_target);
}

/* =============================================================
 * 角度环（内环）—— 每 5ms
 *
 * output = Kp * err + Ki * ∫err + Kd_gain * gy
 * ============================================================= */
static void Balance_AngleLoop(float dt)
{
    float theta   = pitch;
    float theta_d = gy;

    float err = angle_pid.target - theta;

    /* 积分累加 */
    angle_pid.integral += err * dt;
    if (angle_pid.integral > angle_pid.integral_max)
        angle_pid.integral = angle_pid.integral_max;
    if (angle_pid.integral < angle_pid.integral_min)
        angle_pid.integral = angle_pid.integral_min;

    /* 角度环输出 */
    float out = angle_pid.Kp * err
              + angle_pid.Ki * angle_pid.integral
              - BAL_ANGLE_KD_GAIN * theta_d;

    /* 输出限幅 */
    if (out > angle_pid.out_max) out = angle_pid.out_max;
    if (out < angle_pid.out_min) out = angle_pid.out_min;

    angle_pid.err      = err;
    angle_pid.last_err = err;
    angle_pid.output   = out;

    bal_pwm_L = out;
    bal_pwm_R = out;

    MG513_SetSpeed(LEFT_MOTOR_ID,  (int16_t)-out);
    MG513_SetSpeed(RIGHT_MOTOR_ID, (int16_t)-out);
}

/* =============================================================
 * 主入口：TIM6 1ms 中断里调用
 * 分频出三个环的周期
 * ============================================================= */
void Balance_Update(void)
{
    /* ---------- 未使能：等待被扶稳后自动重启 ---------- */
    if (!bal_enabled)
    {
        if (fabsf(pitch) < 15.0f && fabsf(gy) < 30.0f)
        {
            bal_idle_cnt++;
            if (bal_idle_cnt >= 500)   /* 稳定 500ms */
            {
                Balance_Init();
            }
        }
        else
        {
            bal_idle_cnt = 0;
        }
        return;
    }

    /* ---------- 已使能：正常控制 ---------- */
    bal_tick_cnt++;

    /* 失稳保护 */
    if (fabsf(pitch) > BAL_FALL_ANGLE)
    {
        Balance_Stop();
        return;
    }

    /* 位置环：每 20ms */
    if ((bal_tick_cnt % BAL_POS_LOOP_MS) == 0)
    {
        Balance_PosLoop((float)BAL_POS_LOOP_MS * 0.001f);
    }

    /* 速度环：每 20ms */
    if ((bal_tick_cnt % BAL_SPEED_LOOP_MS) == 0)
    {
        Balance_SpeedLoop((float)BAL_SPEED_LOOP_MS * 0.001f);
    }

    /* 角度环：每 5ms */
    if ((bal_tick_cnt % BAL_ANGLE_LOOP_MS) == 0)
    {
        Balance_AngleLoop((float)BAL_ANGLE_LOOP_MS * 0.001f);
    }
}