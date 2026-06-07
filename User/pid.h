#ifndef __PID_H
#define __PID_H

#include <stdint.h>

typedef struct {
    float kp, ki, kd;
    float target;
    float out;
    float kpout, kiout, kdout;
    float err0, err1;
    float err_int;
    float out_max, ki_max;
} PID_t;

float PID_Calc(PID_t *pid, int16_t error);

#endif
