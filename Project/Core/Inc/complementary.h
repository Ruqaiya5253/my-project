#ifndef COMPLEMENTARY_H
#define COMPLEMENTARY_H

#include "stm32f3xx_hal.h"

HAL_StatusTypeDef angle_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef angle_update(float dt);
float angle_get_deg(void);
float angle_get_gyro_rate_dps(void);

#endif
