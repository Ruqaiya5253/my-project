#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f3xx_hal.h"

#define MPU6050_ADDR (0x68U << 1)
#define MPU6050_ACCEL_SCALE 16384.0f
#define MPU6050_GYRO_SCALE 131.0f

typedef struct {
    float x;
    float y;
    float z;
} mpu6050_axes_t;

typedef struct {
    mpu6050_axes_t accel;
    mpu6050_axes_t gyro;
} mpu6050_offsets_t;

HAL_StatusTypeDef mpu6050_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef mpu6050_calibrate(I2C_HandleTypeDef *hi2c, uint16_t samples);
HAL_StatusTypeDef mpu6050_read_accel(I2C_HandleTypeDef *hi2c, mpu6050_axes_t *accel_out);
HAL_StatusTypeDef mpu6050_read_gyro(I2C_HandleTypeDef *hi2c, mpu6050_axes_t *gyro_out);
void mpu6050_get_offsets(mpu6050_offsets_t *offsets_out);

#endif