#include "key.h"
#include "servo.h"
#include "decode_move.h"

static uint8_t key_sample;
static uint8_t key_state;
static uint8_t key_down;
static uint8_t key_up;
int16_t servo_angle = (int16_t)SERVO_ANGLE_MIN;

static void key_run(void);

uint8_t Key_Read(void)
{
    uint8_t state = 0U;

    if (HAL_GPIO_ReadPin(key1_Port, key1_Pin) == GPIO_PIN_RESET) state |= KEY_1;
    if (HAL_GPIO_ReadPin(key2_Port, key2_Pin) == GPIO_PIN_RESET) state |= KEY_2;
    if (HAL_GPIO_ReadPin(key3_Port, key3_Pin) == GPIO_PIN_RESET) state |= KEY_3;
    if (HAL_GPIO_ReadPin(key4_Port, key4_Pin) == GPIO_PIN_RESET) state |= KEY_4;

    return state;
}

void Key_Update(void)
{
    uint8_t sample = Key_Read();
    uint8_t changed;

    if (sample == key_sample)
    {
        changed = sample ^ key_state;
        key_down |= changed & sample;
        key_up |= changed & key_state;
        key_state = sample;
    }

    key_run();
    key_sample = sample;
}

uint8_t Key_Press(uint8_t keys)
{
    if (keys & KEY_4) { task_emergency_stop(); return 0; }
    if (Key_Read() & KEY_4) return 2;
    if (keys & KEY_1) { task_start(); return 0; }
    return 1; /* KEY2/3 are intentionally unbound on the hardware too. */
}
static void key_run(void)
{
    /* Stop wins if both buttons are pressed. KEY2/3 cannot override the task. */
    if (key_state & KEY_4) Key_Press(KEY_4);
    else if (key_down) Key_Press(key_down);
    key_down = 0U;
}

uint8_t Key_GetState(void)
{
    return key_state;
}

uint8_t Key_GetUp(void)
{
    uint8_t events = key_up;
    key_up = 0U;
    return events;
}
