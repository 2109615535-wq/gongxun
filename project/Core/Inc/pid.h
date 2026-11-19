#ifndef PID_H
#define PID_H

#include <stddef.h>

typedef struct {
    float target;
    float current;
    float output;

    float kp;
    float ki;
    float kd;

    float error_now;
    float error_last;
    float error_int;

    float output_max;
    float error_int_max;
} PID_t;

void PID_Init(PID_t *pid);
void PID_Update(PID_t *pid);
#endif
