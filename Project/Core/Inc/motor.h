#ifndef MOTOR_H
#define MOTOR_H

#include "stm32f3xx_hal.h"

typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} motor_id_t;

#define MOTOR_PWM_MAX 1599

void motor_init(TIM_HandleTypeDef *htim);
void motor_set_speed(motor_id_t motor, int16_t speed);

#endif
