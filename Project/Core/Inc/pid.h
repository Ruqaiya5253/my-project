#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral_limit;
    float output_limit;
} pid_gains_t;

void pid_init(pid_gains_t gains);
float pid_compute(float setpoint, float measurement, float dt);

#endif
