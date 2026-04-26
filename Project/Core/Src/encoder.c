#include "encoder.h"

static TIM_HandleTypeDef *s_encoder_timer = NULL;

static volatile uint32_t s_left_last_capture = 0U;
static volatile uint32_t s_right_last_capture = 0U;

static volatile encoder_data_t s_left = {0};
static volatile encoder_data_t s_right = {0};

void encoder_init(TIM_HandleTypeDef *htim) {
    s_encoder_timer = htim;
    HAL_TIM_IC_Start_IT(s_encoder_timer, TIM_CHANNEL_1);
    HAL_TIM_IC_Start_IT(s_encoder_timer, TIM_CHANNEL_2);
}

void encoder_handle_ic_callback(TIM_HandleTypeDef *htim) {
    uint32_t now = 0U;
    uint32_t period = 0U;
    uint32_t timer_clk_hz;
    RCC_ClkInitTypeDef clk_cfg;
    uint32_t flash_latency;

    if (htim->Instance != TIM2) {
        return;
    }

    HAL_RCC_GetClockConfig(&clk_cfg, &flash_latency);
    timer_clk_hz = HAL_RCC_GetPCLK1Freq();
    if (clk_cfg.APB1CLKDivider != RCC_HCLK_DIV1) {
        timer_clk_hz *= 2U;
    }
    timer_clk_hz /= (uint32_t)(htim->Init.Prescaler + 1U);

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
        now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        period = (now >= s_left_last_capture) ? (now - s_left_last_capture)
                                              : ((htim->Init.Period - s_left_last_capture) + now + 1U);
        s_left_last_capture = now;
        s_left.period_ticks = period;
        s_left.pulse_count++;
        s_left.frequency_hz = (period != 0U) ? ((float)timer_clk_hz / (float)period) : 0.0f;
    } else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        period = (now >= s_right_last_capture) ? (now - s_right_last_capture)
                                               : ((htim->Init.Period - s_right_last_capture) + now + 1U);
        s_right_last_capture = now;
        s_right.period_ticks = period;
        s_right.pulse_count++;
        s_right.frequency_hz = (period != 0U) ? ((float)timer_clk_hz / (float)period) : 0.0f;
    }
}

encoder_data_t encoder_get_left_data(void) {
    encoder_data_t data;
    __disable_irq();
    data.period_ticks = s_left.period_ticks;
    data.frequency_hz = s_left.frequency_hz;
    data.pulse_count = s_left.pulse_count;
    __enable_irq();
    return data;
}

encoder_data_t encoder_get_right_data(void) {
    encoder_data_t data;
    __disable_irq();
    data.period_ticks = s_right.period_ticks;
    data.frequency_hz = s_right.frequency_hz;
    data.pulse_count = s_right.pulse_count;
    __enable_irq();
    return data;
}
