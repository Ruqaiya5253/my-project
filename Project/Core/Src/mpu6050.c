#include "mpu6050.h"

/* Registers */
#define REG_PWR_MGMT_1   0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG  0x1B
#define REG_ACCEL_XOUT_H 0x3B

/* PRIVATE buffer */
static uint8_t rx_buffer[14];

/* ✅ DEFINE shared variables HERE */
volatile uint8_t mpu_data_ready = 0;
volatile uint8_t i2c_error_flag = 0;
MPU6050_RawData mpu_latest_raw;

/* INIT */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;

    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT_1, 1, &data, 1, I2C_INIT_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    HAL_Delay(10);

    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_ACCEL_CONFIG, 1, &data, 1, I2C_INIT_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_GYRO_CONFIG, 1, &data, 1, I2C_INIT_TIMEOUT) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

/* START ASYNC READ */
void MPU6050_ReadAsync_Start(I2C_HandleTypeDef *hi2c) {
    if (hi2c->State == HAL_I2C_STATE_READY) {
        HAL_I2C_Mem_Read_IT(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, rx_buffer, 14);
    }
}

/* PARSE */
void MPU6050_ParseBuffer(const uint8_t buffer[14], MPU6050_RawData *raw) {
    raw->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
    raw->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
    raw->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);

    raw->gyro_x  = (int16_t)((buffer[8] << 8) | buffer[9]);
    raw->gyro_y  = (int16_t)((buffer[10] << 8) | buffer[11]);
    raw->gyro_z  = (int16_t)((buffer[12] << 8) | buffer[13]);
}

/* SCALE */
void MPU6050_ConvertToScaled(const MPU6050_RawData *raw, MPU6050_ScaledData *scaled) {
    scaled->ax = raw->accel_x / ACCEL_FS;
    scaled->ay = raw->accel_y / ACCEL_FS;
    scaled->az = raw->accel_z / ACCEL_FS;

    scaled->gx = raw->gyro_x / GYRO_FS;
    scaled->gy = raw->gyro_y / GYRO_FS;
    scaled->gz = raw->gyro_z / GYRO_FS;
}

/* CALIBRATION (unchanged) */
void MPU6050_CalibrateBlocking(I2C_HandleTypeDef *hi2c, MPU6050_Offsets *offsets, uint16_t samples) {
    MPU6050_RawData raw;
    MPU6050_ScaledData scaled;

    float acc_sum[3] = {0}, gyro_sum[3] = {0};

    for (uint16_t i = 0; i < samples; i++) {
        uint8_t buf[14];

        if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, 1, buf, 14, 10) == HAL_OK) {
            MPU6050_ParseBuffer(buf, &raw);
            MPU6050_ConvertToScaled(&raw, &scaled);

            acc_sum[0] += scaled.ax;
            acc_sum[1] += scaled.ay;
            acc_sum[2] += scaled.az;

            gyro_sum[0] += scaled.gx;
            gyro_sum[1] += scaled.gy;
            gyro_sum[2] += scaled.gz;
        }

        HAL_Delay(2);
    }

    offsets->accel_offset[0] = acc_sum[0] / samples;
    offsets->accel_offset[1] = acc_sum[1] / samples;
    offsets->accel_offset[2] = acc_sum[2] / samples;

    offsets->gyro_offset[0] = gyro_sum[0] / samples;
    offsets->gyro_offset[1] = gyro_sum[1] / samples;
    offsets->gyro_offset[2] = gyro_sum[2] / samples;
}

/* CALLBACKS */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {
        MPU6050_ParseBuffer(rx_buffer, &mpu_latest_raw);
        mpu_data_ready = 1;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C2) {
        i2c_error_flag = 1;
    }
}