#include "pid.h"

static pid_gains_t s_gains = {0};
static float s_integral = 0.0f;
static float s_prev_error = 0.0f;

static float clampf(float value, float min_value, float max_value) {
    if (value > max_value) {
        return max_value;
    }
    if (value < min_value) {
        return min_value;
    }
    return value;
}

void pid_init(pid_gains_t gains) {
    s_gains = gains;
    s_integral = 0.0f;
    s_prev_error = 0.0f;
}

float pid_compute(float setpoint, float measurement, float dt) {
    float error = setpoint - measurement;
    float derivative;
    float output;

    if (dt <= 0.0f) {
        return 0.0f;
    }

    s_integral += error * dt;
    s_integral = clampf(s_integral, -s_gains.integral_limit, s_gains.integral_limit);
    derivative = (error - s_prev_error) / dt;
    s_prev_error = error;

    output = (s_gains.kp * error) + (s_gains.ki * s_integral) + (s_gains.kd * derivative);
    return clampf(output, -s_gains.output_limit, s_gains.output_limit);
}
