#include "pid.h"

float PID_Calc(PID_t *pid, int16_t error)
{
    pid->err1 = pid->err0;
    pid->err0 = (float)error - pid->target;

    // 积分抗饱和：输出未饱和时才积分
    if (pid->out < pid->out_max && pid->out > -pid->out_max)
        pid->err_int += pid->err0;

    // 积分限幅
    if (pid->err_int > pid->ki_max)  pid->err_int = pid->ki_max;
    if (pid->err_int < -pid->ki_max) pid->err_int = -pid->ki_max;

    pid->kpout = pid->kp * pid->err0;
    pid->kiout = pid->ki * pid->err_int;
    pid->kdout = pid->kd * (pid->err0 - pid->err1);

    float temp = pid->kpout + pid->kiout + pid->kdout;

    if (temp > pid->out_max)  temp = pid->out_max;
    if (temp < -pid->out_max) temp = -pid->out_max;

    pid->out = temp;
    return temp;
}
