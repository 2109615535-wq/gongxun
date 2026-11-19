#include "motor.h"
#include "vision.h"
#include "main.h"

extern TIM_HandleTypeDef htim1;



PID_t pid_motor1;
PID_t pid_motor2;
PID_t pid_motor3;
PID_t pid_motor4;

static uint8_t motor_pwm_mode = 0U;
static uint8_t motor_drive_enabled;


static void motor_set_speed(uint32_t channel, GPIO_TypeDef *port,
                            uint16_t pin, int32_t speed, int8_t direction)
{
    uint32_t magnitude;
    uint32_t pulse;

    if (!motor_drive_enabled) speed = 0;
    if (speed > MOTOR_WHEEL_LIMIT) speed = MOTOR_WHEEL_LIMIT;
    if (speed < -MOTOR_WHEEL_LIMIT) speed = -MOTOR_WHEEL_LIMIT;

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
    if (!motor_drive_enabled) Enable = 0;
    HAL_GPIO_WritePin(motor1_En_Port, motor1_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable2(uint8_t Enable)
{
    if (!motor_drive_enabled) Enable = 0;
    HAL_GPIO_WritePin(motor2_En_Port, motor2_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable3(uint8_t Enable)
{
    if (!motor_drive_enabled) Enable = 0;
    HAL_GPIO_WritePin(motor3_En_Port, motor3_En_Pin,
                      Enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void MotorEnable4(uint8_t Enable)
{
    if (!motor_drive_enabled) Enable = 0;
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

    motor_set_speed(TIM_CHANNEL_4, motor1_Port, motor1_Pin, 0, motor1_direction);
    motor_set_speed(TIM_CHANNEL_3, motor2_Port, motor2_Pin, 0, motor2_direction);
    motor_set_speed(TIM_CHANNEL_2, motor3_Port, motor3_Pin, 0, motor3_direction);
    motor_set_speed(TIM_CHANNEL_1, motor4_Port, motor4_Pin, 0, motor4_direction);

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

    pid_motor1.output_max = MOTOR_WHEEL_LIMIT;
    pid_motor2.output_max = MOTOR_WHEEL_LIMIT;
    pid_motor3.output_max = MOTOR_WHEEL_LIMIT;
    pid_motor4.output_max = MOTOR_WHEEL_LIMIT;

    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK ||
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4) != HAL_OK)
    {
        Error_Handler();
        return;
    }

    /* Remain disabled until an explicitly started task commands motion. */
}

void motor_process(void)
{
    encoder_process();

    pid_motor1.current = (float)encoder_get_speed(&encoder1);
    pid_motor2.current = (float)encoder_get_speed(&encoder2);
    pid_motor3.current = (float)encoder_get_speed(&encoder3);
    pid_motor4.current = (float)encoder_get_speed(&encoder4);//  计数/秒

    /* Encoder feedback stays active in both modes. */
    if (!motor_pwm_mode)
    {
        PID_Update(&pid_motor1);
        PID_Update(&pid_motor2);
        PID_Update(&pid_motor3);
        PID_Update(&pid_motor4);
    }

    motor_set_speed(TIM_CHANNEL_4, motor1_Port, motor1_Pin,
                    (int32_t)pid_motor1.output, motor1_direction);
    motor_set_speed(TIM_CHANNEL_3, motor2_Port, motor2_Pin,
                    (int32_t)pid_motor2.output, motor2_direction);
    motor_set_speed(TIM_CHANNEL_2, motor3_Port, motor3_Pin,
                    (int32_t)pid_motor3.output, motor3_direction);
    motor_set_speed(TIM_CHANNEL_1, motor4_Port, motor4_Pin,
                    (int32_t)pid_motor4.output, motor4_direction);
}


void MotorSetSpeed1(int32_t Speed)
{
    motor_pwm_mode = 0U;
    pid_motor1.target = (float)(Speed > MOTOR_WHEEL_LIMIT ? MOTOR_WHEEL_LIMIT :
                               Speed < -MOTOR_WHEEL_LIMIT ? -MOTOR_WHEEL_LIMIT : Speed);
    
}

void MotorSetSpeed2(int32_t Speed)
{
    motor_pwm_mode = 0U;
    pid_motor2.target = (float)(Speed > MOTOR_WHEEL_LIMIT ? MOTOR_WHEEL_LIMIT :
                               Speed < -MOTOR_WHEEL_LIMIT ? -MOTOR_WHEEL_LIMIT : Speed);
}

void MotorSetSpeed3(int32_t Speed)
{
    motor_pwm_mode = 0U;
    pid_motor3.target = (float)(Speed > MOTOR_WHEEL_LIMIT ? MOTOR_WHEEL_LIMIT :
                               Speed < -MOTOR_WHEEL_LIMIT ? -MOTOR_WHEEL_LIMIT : Speed);
}

void MotorSetSpeed4(int32_t Speed)
{
    motor_pwm_mode = 0U;
    pid_motor4.target = (float)(Speed > MOTOR_WHEEL_LIMIT ? MOTOR_WHEEL_LIMIT :
                               Speed < -MOTOR_WHEEL_LIMIT ? -MOTOR_WHEEL_LIMIT : Speed);
}


/* All four motors use fixed PWM until a speed command restores PID control. */
void MotorSetPWM(int32_t pwm1, int32_t pwm2, int32_t pwm3, int32_t pwm4)
{
    float p1 = (float)pwm1, p2 = (float)pwm2, p3 = (float)pwm3, p4 = (float)pwm4;
    float peak, value, scale;
    if (!motor_pwm_mode)
    {
        /* Discard old speed targets and PID history when entering open loop. */
        pid_motor1.target = pid_motor2.target = pid_motor3.target = pid_motor4.target = 0.0f;
        pid_motor1.error_int = pid_motor2.error_int = pid_motor3.error_int = pid_motor4.error_int = 0.0f;
        pid_motor1.error_now = pid_motor2.error_now = pid_motor3.error_now = pid_motor4.error_now = 0.0f;
        pid_motor1.error_last = pid_motor2.error_last = pid_motor3.error_last = pid_motor4.error_last = 0.0f;
    }

    motor_pwm_mode = 1U;
    peak = p1 < 0 ? -p1 : p1;
    value = p2 < 0 ? -p2 : p2; if (value > peak) peak = value;
    value = p3 < 0 ? -p3 : p3; if (value > peak) peak = value;
    value = p4 < 0 ? -p4 : p4; if (value > peak) peak = value;
    if (peak > MOTOR_WHEEL_LIMIT) {
        scale = (float)MOTOR_WHEEL_LIMIT / peak;
        p1 *= scale; p2 *= scale; p3 *= scale; p4 *= scale;
    }
    pid_motor1.output = p1; pid_motor2.output = p2;
    pid_motor3.output = p3; pid_motor4.output = p4;
}

/* Single task-owned drive gate. Disable clears PWM immediately, not next tick. */
void Motor_SetDriveEnabled(uint8_t enable)
{
    if (enable && motor_drive_enabled) return;
    motor_drive_enabled = 0;
    MotorEnable1(0); MotorEnable2(0); MotorEnable3(0); MotorEnable4(0);
    MotorSetPWM(0, 0, 0, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0);
    if (enable) {
        motor_drive_enabled = 1;
        MotorEnable1(1); MotorEnable2(1); MotorEnable3(1); MotorEnable4(1);
    }
}
