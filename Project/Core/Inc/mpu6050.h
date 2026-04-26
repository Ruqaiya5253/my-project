#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f3xx_hal.h"

/* Device address */
#define MPU6050_ADDR (0x68 << 1)

/* Scaling */
#define ACCEL_FS 16384.0f
#define GYRO_FS 131.0f

#define I2C_INIT_TIMEOUT 100

typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
} MPU6050_RawData;

typedef struct {
    float ax, ay, az;
    float gx, gy, gz;
} MPU6050_ScaledData;

typedef struct {
    float accel_offset[3];
    float gyro_offset[3];
} MPU6050_Offsets;

/*  SHARED VARIABLES */
extern volatile uint8_t mpu_data_ready;
extern volatile uint8_t i2c_error_flag;
extern MPU6050_RawData mpu_latest_raw;

/* FUNCTIONS */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_ReadAsync_Start(I2C_HandleTypeDef *hi2c);
void MPU6050_ParseBuffer(const uint8_t buffer[14], MPU6050_RawData *raw);
void MPU6050_ConvertToScaled(const MPU6050_RawData *raw, MPU6050_ScaledData *scaled);
void MPU6050_CalibrateBlocking(I2C_HandleTypeDef *hi2c, MPU6050_Offsets *offsets, uint16_t samples);

#endif