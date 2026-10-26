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
    
}

void PID_Update(PID_t *pid)
{
    float pd_output;
    float integral_candidate;
    float integral_limit;
    float integral_step;
    float output_candidate;

    if (pid == NULL)
    {
        return;
    }

    //Îó²î¼ÆËã
    pid->error_last = pid->error_now;
    pid->error_now = pid->target - pid->current;
    
    pd_output = pid->kp * pid->error_now
              + pid->kd * (pid->error_now - pid->error_last);

    if (pid->ki != 0.0f)
    {
        /* Bound the integral contribution by the available output range. */
        integral_limit = pid->output_max / (pid->ki > 0.0f ? pid->ki : -pid->ki);
        integral_candidate = pid->error_int + pid->error_now;
        if (integral_candidate > integral_limit)
            integral_candidate = integral_limit;
        else if (integral_candidate < -integral_limit)
            integral_candidate = -integral_limit;

        integral_step = pid->ki * pid->error_now;
        output_candidate = pd_output + pid->ki * integral_candidate;

        /* Freeze integration that pushes further into saturation.
         * Opposite-sign error can still unwind the stored integral. */
        if (!((output_candidate > pid->output_max && integral_step > 0.0f) ||
              (output_candidate < -pid->output_max && integral_step < 0.0f)))
        {
            pid->error_int = integral_candidate;
        }
    }
    else
    {
        pid->error_int = 0.0f;
    }

    pid->output = pd_output + pid->ki * pid->error_int;


    // ÏŞ·ù
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < -pid->output_max) pid->output = -pid->output_max;

}

