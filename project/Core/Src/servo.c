#include "servo.h"

#include "main.h"
#include "scheduler.h"

extern TIM_HandleTypeDef htim9;

static float current_angle;
static float target_angle;
static float speed_deg_per_second;
static uint32_t last_update_tick;
static uint8_t moving;

static void Servo_WriteAngle(float angle)
{
    float pulse = (float)SERVO_PULSE_MIN + angle / SERVO_PULSE_ANGLE_MAX *
                  (float)(SERVO_PULSE_MAX - SERVO_PULSE_MIN);
    __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, (uint16_t)(pulse + 0.5f));
}

void Servo_Init(void)
{
    current_angle = SERVO_ANGLE_MIN;
    target_angle = SERVO_ANGLE_MIN;
    speed_deg_per_second = 0.0f;
    last_update_tick = scheduler_get_tick();
    moving = 0U;

    Servo_WriteAngle(current_angle);
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

void Servo_SetAngle(float angle, float speed)
{
    angle += SERVO_OFFSET;
    if (angle < SERVO_ANGLE_MIN) angle = SERVO_ANGLE_MIN;
    if (angle > SERVO_ANGLE_MAX) angle = SERVO_ANGLE_MAX;

    target_angle = angle;
    if (speed <= 0.0f)
    {
        current_angle = target_angle;
        moving = 0U;
        Servo_WriteAngle(current_angle);
        return;
    }

    speed_deg_per_second = speed;
    last_update_tick = scheduler_get_tick();
    moving = (current_angle != target_angle);
}

void Servo_Update(void)
{
    uint32_t now;
    uint32_t elapsed_ms;
    float step;
    float diff;

    if (!moving) return;

    now = scheduler_get_tick();
    elapsed_ms = now - last_update_tick;
    if (elapsed_ms == 0U) return;
    last_update_tick = now;

    step = speed_deg_per_second * (float)elapsed_ms / 1000.0f;
    diff = target_angle - current_angle;
    if (diff <= step && diff >= -step)
    {
        current_angle = target_angle;
        moving = 0U;
    }
    else if (diff > 0.0f)
    {
        current_angle += step;
    }
    else
    {
        current_angle -= step;
    }

    Servo_WriteAngle(current_angle);
}

float Servo_GetAngle(void)
{
    return current_angle;
}

