#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f3xx_hal.h"

typedef struct {
    uint32_t period_ticks;
    float frequency_hz;
    uint32_t pulse_count;
} encoder_data_t;

void encoder_init(TIM_HandleTypeDef *htim);
void encoder_handle_ic_callback(TIM_HandleTypeDef *htim);
encoder_data_t encoder_get_left_data(void);
encoder_data_t encoder_get_right_data(void);

#endif
