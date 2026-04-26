#include "mpu6050.h"

#define REG_PWR_MGMT_1 0x6B
#define REG_ACCEL_CONFIG 0x1C
#define REG_GYRO_CONFIG 0x1B
#define REG_ACCEL_XOUT_H 0x3B
#define REG_GYRO_XOUT_H 0x43

#define MPU6050_I2C_TIMEOUT_MS 5U

static mpu6050_offsets_t g_offsets = {0};

static int16_t to_i16(uint8_t msb, uint8_t lsb) {
    return (int16_t)((uint16_t)msb << 8U | (uint16_t)lsb);
}

HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c) {
    uint8_t value = 0x00U;

    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &value, 1U,
                          MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &value, 1U,
                          MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, REG_GYRO_CONFIG, I2C_MEMADD_SIZE_8BIT, &value, 1U,
                          MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef mpu6050_read_accel(I2C_HandleTypeDef *hi2c, mpu6050_axes_t *accel_out) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                         MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    accel_out->x = (float)to_i16(buf[0], buf[1]) / MPU6050_ACCEL_SCALE;
    accel_out->y = (float)to_i16(buf[2], buf[3]) / MPU6050_ACCEL_SCALE;
    accel_out->z = (float)to_i16(buf[4], buf[5]) / MPU6050_ACCEL_SCALE;

    accel_out->x -= g_offsets.accel.x;
    accel_out->y -= g_offsets.accel.y;
    accel_out->z -= g_offsets.accel.z;
    return HAL_OK;
}

HAL_StatusTypeDef mpu6050_read_gyro(I2C_HandleTypeDef *hi2c, mpu6050_axes_t *gyro_out) {
    uint8_t buf[6];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, REG_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, buf, sizeof(buf),
                         MPU6050_I2C_TIMEOUT_MS) != HAL_OK) {
        return HAL_ERROR;
    }

    gyro_out->x = (float)to_i16(buf[0], buf[1]) / MPU6050_GYRO_SCALE;
    gyro_out->y = (float)to_i16(buf[2], buf[3]) / MPU6050_GYRO_SCALE;
    gyro_out->z = (float)to_i16(buf[4], buf[5]) / MPU6050_GYRO_SCALE;

    gyro_out->x -= g_offsets.gyro.x;
    gyro_out->y -= g_offsets.gyro.y;
    gyro_out->z -= g_offsets.gyro.z;
    return HAL_OK;
}

HAL_StatusTypeDef mpu6050_calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples) {
    mpu6050_axes_t accel = {0};
    mpu6050_axes_t gyro = {0};
    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;

    if (samples == 0U) {
        return HAL_ERROR;
    }

    for (uint16_t i = 0U; i < samples; i++) {
        if (mpu6050_read_accel(hi2c, &accel) != HAL_OK || mpu6050_read_gyro(hi2c, &gyro) != HAL_OK) {
            return HAL_ERROR;
        }

        ax += accel.x;
        ay += accel.y;
        az += accel.z;
        gx += gyro.x;
        gy += gyro.y;
        gz += gyro.z;
    }

    g_offsets.accel.x = ax / (float)samples;
    g_offsets.accel.y = ay / (float)samples;
    g_offsets.accel.z = (az / (float)samples) - 1.0f;
    g_offsets.gyro.x = gx / (float)samples;
    g_offsets.gyro.y = gy / (float)samples;
    g_offsets.gyro.z = gz / (float)samples;
    return HAL_OK;
}

void mpu6050_get_offsets(mpu6050_offsets_t *offsets_out) {
    if (offsets_out != NULL) {
        *offsets_out = g_offsets;
    }
}