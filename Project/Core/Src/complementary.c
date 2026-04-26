#include "complementary.h"
#include "mpu6050.h"

#include <math.h>

#define RAD2DEG 57.2957795f

static I2C_HandleTypeDef *s_i2c = NULL;
static float s_angle_deg = 0.0f;
static float s_gyro_rate_dps = 0.0f;

HAL_StatusTypeDef angle_init(I2C_HandleTypeDef *hi2c) {
    s_i2c = hi2c;
    s_angle_deg = 0.0f;
    s_gyro_rate_dps = 0.0f;
    return HAL_OK;
}

HAL_StatusTypeDef angle_update(float dt) {
    mpu6050_axes_t accel;
    mpu6050_axes_t gyro;
    float accel_angle;

    if (s_i2c == NULL) {
        return HAL_ERROR;
    }

    if (mpu6050_read_accel(s_i2c, &accel) != HAL_OK) {
        return HAL_ERROR;
    }

    if (mpu6050_read_gyro(s_i2c, &gyro) != HAL_OK) {
        return HAL_ERROR;
    }

    s_gyro_rate_dps = gyro.x;
    accel_angle = atan2f(accel.y, accel.z) * RAD2DEG;
    s_angle_deg = 0.98f * (s_angle_deg + (s_gyro_rate_dps * dt)) + 0.02f * accel_angle;

    return HAL_OK;
}

float angle_get_deg(void) { return s_angle_deg; }

float angle_get_gyro_rate_dps(void) { return s_gyro_rate_dps; }
