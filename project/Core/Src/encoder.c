#include "encoder.h"
#include "main.h"
#include "scheduler.h"
#include <stddef.h>

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

Encoder_t encoder1 = {&htim4, ENCODER1_DIRECTION, 0, 0, 0U, 0U};
Encoder_t encoder2 = {&htim2, ENCODER2_DIRECTION, 0, 0, 0U, 0U};
Encoder_t encoder3 = {&htim3, ENCODER3_DIRECTION, 0, 0, 0U, 0U};
Encoder_t encoder4 = {&htim5, ENCODER4_DIRECTION, 0, 0, 0U, 0U};

static Encoder_t *const encoders[] =
{
    &encoder1, &encoder2, &encoder3, &encoder4
};

void encoder_reset(Encoder_t *enc)
{
    if (enc == NULL)
    {
        return;
    }

    enc->last_counter = __HAL_TIM_GET_COUNTER(enc->timer);
    enc->last_tick = scheduler_get_tick();
    enc->count = 0;
    enc->speed = 0;
}

void encoder_init(void)
{
    uint32_t i;

    for (i = 0U; i < sizeof(encoders) / sizeof(encoders[0]); i++)
    {
        __HAL_TIM_SET_COUNTER(encoders[i]->timer, 0U);
        encoder_reset(encoders[i]);
        if (HAL_TIM_Encoder_Start(encoders[i]->timer, TIM_CHANNEL_ALL) != HAL_OK)
        {
            Error_Handler();
            return;
        }
    }
}

static void encoder_update(Encoder_t *enc, uint32_t now)
{
    uint32_t elapsed = now - enc->last_tick;
    uint32_t current;
    int64_t period;
    int64_t delta;
    int64_t speed;

    /* No time has elapsed: keep the previous sample to avoid dividing by zero. */
    if (elapsed == 0U)
    {
        return;
    }

    current = __HAL_TIM_GET_COUNTER(enc->timer);
    period = (int64_t)__HAL_TIM_GET_AUTORELOAD(enc->timer) + 1;
    delta = (int64_t)current - enc->last_counter;

    /* Correct forward/reverse rollover for both 16-bit and 32-bit timers.
     * Never clear CNT here: an edge arriving during a read cannot be lost.
     */
    if (delta > period / 2)
    {
        delta -= period;
    }
    else if (delta < -period / 2)
    {
        delta += period;
    }

    delta *= enc->direction;
    enc->count += delta;
    speed = delta * 1000 / elapsed;

    if (speed > INT32_MAX)
    {
        speed = INT32_MAX;
    }
    else if (speed < INT32_MIN)
    {
        speed = INT32_MIN;
    }

    enc->speed = (int32_t)speed;
    enc->last_counter = current;
    enc->last_tick = now;
}

void encoder_process(void)
{
    uint32_t i;
    uint32_t now = scheduler_get_tick();

    for (i = 0U; i < sizeof(encoders) / sizeof(encoders[0]); i++)
    {
        encoder_update(encoders[i], now);
    }
}

int64_t encoder_get_count(const Encoder_t *enc)
{
    return enc != NULL ? enc->count : 0;
}

int32_t encoder_get_speed(const Encoder_t *enc)
{
    int64_t speed;

    if (enc == NULL || ENCODER_SPEED_AT_PWM1000 <= 0L)
    {
        return 0;
    }

    /* Convert raw counts/s to the same -1000..1000 scale used by PWM. */
    speed = (int64_t)enc->speed * 1000L / ENCODER_SPEED_AT_PWM1000;

    if (speed > 1000L)
    {
        speed = 1000L;
    }
    else if (speed < -1000L)
    {
        speed = -1000L;
    }

    return (int32_t)speed;
}
