#include "icm20602.h"

float icm_gyro_bias[3] = {0};   /* 陀螺仪偏移量 */

volatile float roll = 0, pitch = 0, yaw = 0;   /* 姿态角（deg） */
volatile float temp = 0;                       /* 温度（℃） */

volatile float ax = 0, ay = 0, az = 0;         /* 加速度（g） */
volatile float gx = 0, gy = 0, gz = 0;         /* 角速度（deg/s） */

/**
 * @brief 写入 ICM20602 寄存器
 * @param reg  寄存器地址
 * @param data 要写入的数据
 */
void ICM20602_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   /* 最高位清零表示写 */
    tx[1] = data;

    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi2, tx, 2, 100);
    ICM_CS_HIGH();
}

/**
 * @brief 读取 ICM20602 寄存器
 * @param reg 寄存器地址
 * @return uint8_t 寄存器值
 */
uint8_t ICM20602_ReadReg(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2] = {0, 0};

    tx[0] = reg | 0x80;   /* 最高位置 1 表示读 */
    tx[1] = 0x00;         /* dummy */

    ICM_CS_LOW();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 2, 100);
    ICM_CS_HIGH();

    return rx[1];         /* 第二个字节才是寄存器值 */
}

/**
 * @brief 连续读取 ICM20602 寄存器数据
 * @param reg 寄存器地址
 * @param buf 存储数据的缓冲区
 * @param len 要读取的字节数
 */
void ICM20602_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t addr = reg | 0x80;

    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi2, &addr, 1, 100);
    HAL_SPI_Receive(&hspi2, buf, len, 100);
    ICM_CS_HIGH();
}

/**
 * @brief 初始化 ICM20602
 * @return uint8_t 0 成功，1 失败
 */
uint8_t ICM20602_Init(void)
{
    ICM20602_WriteReg(ICM20602_PWR_MGMT_1, 0x80);  /* 复位 */
    HAL_Delay(100);

    ICM20602_WriteReg(ICM20602_PWR_MGMT_1, 0x01);  /* 唤醒，自动选择时钟 */
    HAL_Delay(50);

    ICM20602_WriteReg(ICM20602_GYRO_CONFIG,  0x18); /* ±2000 dps */
    ICM20602_WriteReg(ICM20602_ACCEL_CONFIG, 0x18); /* ±16 g */
    ICM20602_WriteReg(ICM20602_CONFIG,       0x03); /* DLPF 42Hz */
    ICM20602_WriteReg(ICM20602_SMPLRT_DIV,   0x00); /* 1kHz */

    /* 验证 WHO_AM_I */
    uint8_t who = ICM20602_ReadReg(ICM20602_WHO_AM_I);
    if (who != 0x2E) {
        return 1;   /* 初始化失败 */
    }
    return 0;       /* 成功 */
}

/**
 * @brief 读取 ICM20602 原始数据
 * @param accel 加速度数据
 * @param gyro  陀螺仪数据
 * @param temp  温度数据
 */
void ICM20602_ReadRaw(int16_t *accel, int16_t *gyro, int16_t *temp)
{
    uint8_t buf[14];
    ICM20602_ReadRegs(ICM20602_ACCEL_XOUT_H, buf, 14);

    accel[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accel[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accel[2] = (int16_t)((buf[4] << 8) | buf[5]);
    *temp    = (int16_t)((buf[6] << 8) | buf[7]);
    gyro[0]  = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro[1]  = (int16_t)((buf[10] << 8) | buf[11]);
    gyro[2]  = (int16_t)((buf[12] << 8) | buf[13]);
}

/**
 * @brief 校准 ICM20602（仅统计陀螺仪零偏）
 */
void ICM20602_Calibrate(void)
{
    int16_t accel[3], gyro[3], temp;
    long sum_g[3] = {0};
    const int N = 1000;

    /* 丢弃前 100 个样本 */
    for (int i = 0; i < 100; i++) {
        ICM20602_ReadRaw(accel, gyro, &temp);
        HAL_Delay(1);
    }

    /* 累加陀螺仪数据 */
    for (int i = 0; i < N; i++) {
        ICM20602_ReadRaw(accel, gyro, &temp);
        for (int j = 0; j < 3; j++) {
            sum_g[j] += gyro[j];
        }
        HAL_Delay(1);
    }

    for (int j = 0; j < 3; j++) {
        icm_gyro_bias[j] = (float)sum_g[j] / (float)N;
    }
}

/**
 * @brief 读取 ICM20602 标准化数据
 * @param acc_g     加速度（g）
 * @param gyro_dps  陀螺仪（deg/s）
 * @param temp_c    温度（℃）
 */
void ICM20602_ReadScaled(float *acc_g, float *gyro_dps, float *temp_c)
{
    int16_t accel[3], gyro[3], temp;
    ICM20602_ReadRaw(accel, gyro, &temp);

    acc_g[0] = (float)accel[0] / ICM_ACC_SENS;
    acc_g[1] = (float)accel[1] / ICM_ACC_SENS;
    acc_g[2] = (float)accel[2] / ICM_ACC_SENS;

    gyro_dps[0] = ((float)gyro[0] - icm_gyro_bias[0]) / ICM_GYR_SENS;
    gyro_dps[1] = ((float)gyro[1] - icm_gyro_bias[1]) / ICM_GYR_SENS;
    gyro_dps[2] = ((float)gyro[2] - icm_gyro_bias[2]) / ICM_GYR_SENS;

    if (temp_c) *temp_c = ((float)temp / 326.8f) + 25.0f;
}

/**
 * @brief 姿态更新（1ms 调用一次，与 TIM6 一致）
 */
void ICM20602_Update(void)
{
    float accle[3], gyro[3];
    float temp_t;
    ICM20602_ReadScaled(accle, gyro, &temp_t);

    /* 1. 加速度算角度（观测值，长期准） */
    float roll_acc  = atan2f(accle[1], accle[2]) * RAD2DEG;
    float pitch_acc = atan2f(-accle[0],
                             sqrtf(accle[1]*accle[1] + accle[2]*accle[2])) * RAD2DEG;

    /* 2. 姿态积分量 */
    static float roll_gyro  = 0.0f;
    static float pitch_gyro = 0.0f;

    const float dt    = 0.001f;    /* 必须和调用周期一致 */
    const float alpha = 0.99f;    /* 互补滤波：0.5s 时间常数，信任陀螺仪短期 */

    /* 首次调用：直接用加速度计角度作为初值 */
    static uint8_t first_run = 1;
    if (first_run) {
        roll_gyro  = roll_acc;
        pitch_gyro = pitch_acc;
        first_run  = 0;
    }

    /* 陀螺仪积分（用实时角速度，不要用滤波后的） */
    roll_gyro  += gyro[0] * dt;    /* 绕 X 轴 → roll  */
    pitch_gyro += gyro[1] * dt;    /* 绕 Y 轴 → pitch */

    /* 3. 互补融合 */
    roll_gyro  = alpha * roll_gyro  + (1.0f - alpha) * roll_acc;
    pitch_gyro = alpha * pitch_gyro + (1.0f - alpha) * pitch_acc;

    /* 4. 加速度、角速度低通滤波
     *    ★ 加速度计：重滤波（用于显示/判断，不影响姿态解算）
     *    ★ 陀螺仪：  轻滤波（参与控制回路，必须实时）
     */
    static float ax_f = 0.0f, ay_f = 0.0f, az_f = 0.0f;
    static float gx_f = 0.0f, gy_f = 0.0f, gz_f = 0.0f;

    static uint8_t first_filter = 1;
    if (first_filter) {
        ax_f = accle[0]; ay_f = accle[1]; az_f = accle[2];
        gx_f = gyro[0];  gy_f = gyro[1];  gz_f = gyro[2];
        first_filter = 0;
    } else {
        /* 加速度计：α = 0.998 → τ ≈ 0.5s，可以很平滑 */
        ax_f = alpha * ax_f + (1.0f - alpha) * accle[0];
        ay_f = alpha * ay_f + (1.0f - alpha) * accle[1];
        az_f = alpha * az_f + (1.0f - alpha) * accle[2];

        /* ★ 陀螺仪：α = 0.2 → τ ≈ 1ms，截止 ~150Hz
         *   保证角度环微分项(gy)拥有真实相位，
         *   不会因为相位滞后造成摇摆。 */
        const float alpha_g = 0.05f;
        gx_f = alpha_g * gx_f + (1.0f - alpha_g) * gyro[0];
        gy_f = alpha_g * gy_f + (1.0f - alpha_g) * gyro[1];
        gz_f = alpha_g * gz_f + (1.0f - alpha_g) * gyro[2];
    }

    /* 5. 输出 */
    roll  = roll_gyro;
    pitch = pitch_gyro;
    temp  = temp_t;

    ax = ax_f;
    ay = ay_f;
    az = az_f;
    gx = gx_f;
    gy = gy_f;
    gz = gz_f;

    /* yaw 不算，加速度计无法感知 */
}