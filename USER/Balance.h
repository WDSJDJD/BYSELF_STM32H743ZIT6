#ifndef __BALANCE_H__
#define __BALANCE_H__

#include "main.h"
#include "pid.h"

/* ================= 可修改的硬件/算法参数 ================= */

/* ================= 角度环（内环，200Hz） ================= */
#define BAL_ANGLE_KP       220.0f      /* 角度环比例：主参数 */
#define BAL_ANGLE_KI       0.0f        /* 角度环积分：保持 0 */
#define BAL_ANGLE_KD_GAIN  10.0f        /* ★ 关键修改：gy 已实时，从 35 降到 2.0
                                        *   调试建议：从 0 开始逐次 +0.5
                                        *   直到摇摆消失（一般 1.5 ~ 3.0） */

/* ================= 速度环（中环，50Hz） ================= */
#define BAL_SPEED_KP       20.0f       /* 速度环比例：让车跟踪速度目标 */
#define BAL_SPEED_KI       0.0f        /* 速度环积分：与位置环 Ki 冲突，保持 0 */
#define BAL_SPEED_KD       0.0f        /* 速度环微分：0 避免符号坑 */

/* ================= 位置环（外环，50Hz） ================= */
#define BAL_POS_KP         2.0f        /* ★ 1.5 → 0.8，降低刚度，减少漂移 */
#define BAL_POS_KI         0.00f        /* ★ 0.05 → 0，先关掉积分，避免低频漂移 */
#define BAL_POS_KD         0.0f        /* ★ 0.1 → 0，位置信号含噪声，微分无益 */
#define BAL_POS_OUT_MAX    0.8f        /* 速度目标上限 m/s（防猛冲） */
#define BAL_POS_OUT_MIN   -0.8f

/* ================= 输出限幅 ================= */
#define BAL_OUT_MAX        3999.0f     /* 对应 MG513 MAX_PWM */
#define BAL_OUT_MIN       -3999.0f

/* ================= 机械中值 ================= */
/* 车架平衡时 pitch 的实际值（需实车标定）
 * ★ 调试方法：手扶车至平衡，读 pitch 值填入此处 */
#define BAL_PITCH_OFFSET   -2.84f

/* ================= 速度环输出（倾角修正量）限幅 ================= */
/* 最终角度目标范围 = [OFFSET-MAX, OFFSET+MAX] */
#define BAL_ANGLE_TARGET_MAX   18.0f   /* 度 */
#define BAL_ANGLE_TARGET_MIN  -18.0f

/* ================= 调用周期（TIM6 每 1ms 进一次） ================= */
#define BAL_ANGLE_LOOP_MS   2          /* 角度环 5ms → 200Hz */
#define BAL_SPEED_LOOP_MS   10         /* 速度环 20ms → 50Hz */
#define BAL_POS_LOOP_MS     10         /* 位置环 20ms → 50Hz */

/* ================= 失稳保护 ================= */
#define BAL_FALL_ANGLE      45.0f      /* 度 */

/* ================= 位置累积死区（m/s） ================= */
/* ★ 0.002 → 0.01：编码器噪声在 20ms 采样后仍残留，
 *   死区过小会让位置环一直"追噪声"导致漂移 */
#define BAL_SPD_DEADBAND    0.01f

/* ================= PID 实例 ================= */
extern PID_t angle_pid;     /* 角度环（内环） */
extern PID_t speed_pid;     /* 速度环（中环） */
extern PID_t pos_pid;       /* 位置环（外环） */

/* ================= 调试观察量 ================= */
extern volatile float bal_pwm_L;         /* 左轮 PWM */
extern volatile float bal_pwm_R;         /* 右轮 PWM */
extern volatile float bal_angle_target;  /* 最终倾角目标（含中值） */
extern volatile float bal_v_target;      /* 位置环输出的速度目标 m/s */
extern volatile float bal_pos;           /* 累积位置 m */
extern volatile float bal_avg_speed;
extern volatile float bal_correction;

/* ================= 对外接口 ================= */
void Balance_Init(void);
void Balance_Update(void);
void Balance_Stop(void);

#endif