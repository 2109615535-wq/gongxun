#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;

#define MOTOR_SPEED_MAX 1000

static void motor_set_speed(uint32_t channel, GPIO_TypeDef *port,
                            uint16_t pin, int32_t speed, int8_t direction)
{
    uint32_t magnitude;
    uint32_t pulse;

    if (speed > MOTOR_SPEED_MAX)
    {
        speed = MOTOR_SPEED_MAX;
    }
    else if (speed < -MOTOR_SPEED_MAX)
    {
        speed = -MOTOR_SPEED_MAX;
    }

    /* Apply mounting direction after clamping so negation cannot overflow. */
    speed *= direction;

    if (speed != 0)
    {
        HAL_GPIO_WritePin(port, pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    magnitude = (uint32_t)(speed < 0 ? -speed : speed);
    /* PWM1 counts 0..ARR, so ARR + 1 gives a true 100% duty cycle. */
    pulse = ((__HAL_TIM_GET_AUTORELOAD(&htim1) + 1U) * magnitude)
            / MOTOR_SPEED_MAX;
    __HAL_TIM_SET_COMPARE(&htim1, channel, pulse);
}

void Motor_Init(void)
{
    HAL_GPIO_WritePin(motor1_En_Port, motor1_En_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor2_En_Port, motor2_En_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor3_En_Port, motor3_En_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(motor4_En_Port, motor4_En_Pin, GPIO_PIN_RESET);

    MotorSetSpeed1(0);
    MotorSetSpeed2(0);
    MotorSetSpeed3(0);
    MotorSetSpeed4(0);

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    HAL_GPIO_WritePin(motor1_En_Port, motor1_En_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_En_Port, motor2_En_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_En_Port, motor3_En_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor4_En_Port, motor4_En_Pin, GPIO_PIN_SET);
}

void MotorSetSpeed1(int32_t Speed)
{
    motor_set_speed(TIM_CHANNEL_4, motor1_Port, motor1_Pin, Speed, -1);
}

void MotorSetSpeed2(int32_t Speed)
{
    motor_set_speed(TIM_CHANNEL_3, motor2_Port, motor2_Pin, Speed, 1);
}

void MotorSetSpeed3(int32_t Speed)
{
    motor_set_speed(TIM_CHANNEL_2, motor3_Port, motor3_Pin, Speed, 1);
}

void MotorSetSpeed4(int32_t Speed)
{
    motor_set_speed(TIM_CHANNEL_1, motor4_Port, motor4_Pin, Speed, -1);
}

