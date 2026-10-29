#include "pid.h"

void PID_Init(PID_t *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->output = 0.0f;

    pid->error_now = 0.0f;
    pid->error_last = 0.0f;
    pid->error_int = 0.0f;
    
    pid->error_int_max = 0.0f;
}

void PID_Update(PID_t *pid)
{

    //误差计算
    pid->error_last = pid->error_now;//上次误差
    pid->error_now = pid->target - pid->current;//本次误差

    if (pid->ki != 0.0f)
    {
        pid->error_int += pid->error_now;

        // 积分限幅
        if(pid->error_int > pid->error_int_max) pid->error_int = pid->error_int_max;
        else if (pid->error_int < -pid->error_int_max) pid->error_int = -pid->error_int_max;
    }
    else
    {
        pid->error_int = 0.0f;
    }


    //pid计算
    pid->output = pid->kp * pid->error_now
                + pid->kd * (pid->error_now - pid->error_last)
                + pid->ki * pid->error_int;


    // 限幅
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < -pid->output_max) pid->output = -pid->output_max;

}

