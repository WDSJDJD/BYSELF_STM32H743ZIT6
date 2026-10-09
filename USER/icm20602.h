#ifndef __ICM20602_H__
#define __ICM20602_H__

#include "main.h"
#include "spi.h"

/* 寄存器地址 */
#define ICM20602_WHO_AM_I       0x75
#define ICM20602_PWR_MGMT_1     0x6B
#define ICM20602_GYRO_CONFIG    0x1B
#define ICM20602_ACCEL_CONFIG   0x1C
#define ICM20602_CONFIG         0x1A
#define ICM20602_SMPLRT_DIV     0x19
#define ICM20602_ACCEL_XOUT_H   0x3B
#define ICM20602_TEMP_OUT_H     0x41
#define ICM20602_GYRO_XOUT_H    0x43

/* 片选控制：根据你的实际接线修改 GPIO */
#define ICM_CS_LOW()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)
#define ICM_CS_HIGH() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)

/* 量程灵敏度 */
#define ICM_ACC_SENS    2048.0f    /* ±16g    -> 2048 LSB/g */
#define ICM_GYR_SENS    16.4f      /* ±2000dps -> 16.4 LSB/(deg/s) */

/* 安装方向（后面姿态解算用） */
#define ICM_MOUNT_MODE  0

/* 弧度转角度因子 */
#define RAD2DEG  (57.29577951308232f)

extern volatile float roll, pitch, yaw;   /* 姿态角（deg） */
extern volatile float temp;               /* 温度（℃） */

/* ★ 补充：加速度（g）与角速度（deg/s） */
extern volatile float ax, ay, az;
extern volatile float gx, gy, gz;

void    ICM20602_WriteReg(uint8_t reg, uint8_t data);
uint8_t ICM20602_ReadReg(uint8_t reg);
void    ICM20602_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t ICM20602_Init(void);
void    ICM20602_ReadRaw(int16_t *accel, int16_t *gyro, int16_t *temp);
void    ICM20602_Calibrate(void);
void    ICM20602_ReadScaled(float *acc_g, float *gyro_dps, float *temp_c);
void    ICM20602_Update(void);

#endif