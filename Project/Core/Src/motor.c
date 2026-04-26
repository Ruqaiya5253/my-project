#include "motor.h"
#include "main.h"

static TIM_HandleTypeDef *s_pwm_timer = NULL;

void motor_init(TIM_HandleTypeDef *htim) {
    s_pwm_timer = htim;
    HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(s_pwm_timer, TIM_CHANNEL_4);
}

void motor_set_speed(motor_id_t motor, int16_t speed) {
    uint32_t channel = (motor == MOTOR_LEFT) ? TIM_CHANNEL_3 : TIM_CHANNEL_4;
    uint16_t magnitude = 0U;

    if (speed > MOTOR_PWM_MAX) {
        speed = MOTOR_PWM_MAX;
    } else if (speed < -MOTOR_PWM_MAX) {
        speed = -MOTOR_PWM_MAX;
    }

    if (speed < 0) {
        magnitude = (uint16_t)(-speed);
    } else {
        magnitude = (uint16_t)speed;
    }

    if (motor == MOTOR_RIGHT) {
        if (speed > 0) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        } else if (speed < 0) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);
        }
    } else {
        if (speed > 0) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
        } else if (speed < 0) {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET);
        }
    }

    if (s_pwm_timer != NULL) {
        __HAL_TIM_SET_COMPARE(s_pwm_timer, channel, magnitude);
    }
}
