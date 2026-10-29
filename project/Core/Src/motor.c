#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;



PID_t pid_motor1;
PID_t pid_motor2;
PID_t pid_motor3;
PID_t pid_motor4;


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

void MotorEnable1(uint8_t Enable)
{
    HAL_GPIO_WritePin(motor1_En_Port, motor1_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable2(uint8_t Enable)
{
    HAL_GPIO_WritePin(motor2_En_Port, motor2_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable3(uint8_t Enable)
{
    HAL_GPIO_WritePin(motor3_En_Port, motor3_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable4(uint8_t Enable)
{
    HAL_GPIO_WritePin(motor4_En_Port, motor4_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Motor_Init(void) 
{
    MotorEnable1(0U);
    MotorEnable2(0U);
    MotorEnable3(0U);
    MotorEnable4(0U);

    MotorSetSpeed1(0);
    MotorSetSpeed2(0);
    MotorSetSpeed3(0);
    MotorSetSpeed4(0);

    motor_set_speed(TIM_CHANNEL_4, motor1_Port, motor1_Pin, 0, 1);
    motor_set_speed(TIM_CHANNEL_3, motor2_Port, motor2_Pin, 0,  1);
    motor_set_speed(TIM_CHANNEL_2, motor3_Port, motor3_Pin, 0,  1);
    motor_set_speed(TIM_CHANNEL_1, motor4_Port, motor4_Pin, 0, -1);

    PID_Init(&pid_motor1);//清空pid
    PID_Init(&pid_motor2);
    PID_Init(&pid_motor3);
    PID_Init(&pid_motor4);

    pid_motor1.kp = kp_motor1;//赋值pid
    pid_motor1.ki = ki_motor1;
    pid_motor1.kd = kd_motor1;
    pid_motor1.error_int_max = ERROR_INT_MAX;

    pid_motor2.kp = kp_motor2;
    pid_motor2.ki = ki_motor2;
    pid_motor2.kd = kd_motor2;
    pid_motor2.error_int_max = ERROR_INT_MAX;

    pid_motor3.kp = kp_motor3;
    pid_motor3.ki = ki_motor3;
    pid_motor3.kd = kd_motor3;
    pid_motor3.error_int_max = ERROR_INT_MAX;

    pid_motor4.kp = kp_motor4;
    pid_motor4.ki = ki_motor4;
    pid_motor4.kd = kd_motor4;
    pid_motor4.error_int_max = ERROR_INT_MAX;

    pid_motor1.output_max = MOTOR_SPEED_MAX;
    pid_motor2.output_max = MOTOR_SPEED_MAX;
    pid_motor3.output_max = MOTOR_SPEED_MAX;
    pid_motor4.output_max = MOTOR_SPEED_MAX;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    MotorEnable1(1U);
    MotorEnable2(1U);
    MotorEnable3(1U);
    MotorEnable4(1U);
}

void motor_process(void)
{
    encoder_process();

    pid_motor1.current = (float)encoder_get_speed(&encoder1);
    pid_motor2.current = (float)encoder_get_speed(&encoder2);
    pid_motor3.current = (float)encoder_get_speed(&encoder3);
    pid_motor4.current = (float)encoder_get_speed(&encoder4);//  计数/秒

    PID_Update(&pid_motor1);
    PID_Update(&pid_motor2);
    PID_Update(&pid_motor3);
    PID_Update(&pid_motor4);

    motor_set_speed(TIM_CHANNEL_4, motor1_Port, motor1_Pin,
                    (int32_t)pid_motor1.output, 1);
    motor_set_speed(TIM_CHANNEL_3, motor2_Port, motor2_Pin,
                    (int32_t)pid_motor2.output, 1);
    motor_set_speed(TIM_CHANNEL_2, motor3_Port, motor3_Pin,
                    (int32_t)pid_motor3.output, 1);
    motor_set_speed(TIM_CHANNEL_1, motor4_Port, motor4_Pin,
                    (int32_t)pid_motor4.output, -1);
}


void MotorSetSpeed1(int32_t Speed)
{
    pid_motor1.target = (float)Speed;
    
}

void MotorSetSpeed2(int32_t Speed)
{
    pid_motor2.target = (float)Speed;
}

void MotorSetSpeed3(int32_t Speed)
{
    pid_motor3.target = (float)Speed;
}

void MotorSetSpeed4(int32_t Speed)
{
    pid_motor4.target = (float)Speed;
}

