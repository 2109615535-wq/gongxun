#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f4xx_hal.h"

#define motor1_Pin      GPIO_PIN_0
#define motor1_Port     GPIOC
#define motor2_Pin      GPIO_PIN_1
#define motor2_Port     GPIOC
#define motor3_Pin      GPIO_PIN_2
#define motor3_Port     GPIOC
#define motor4_Pin      GPIO_PIN_3
#define motor4_Port     GPIOC
#define motor4_En_Pin   GPIO_PIN_7
#define motor4_En_Port  GPIOE
#define motor3_En_Pin   GPIO_PIN_9
#define motor3_En_Port  GPIOE
#define motor2_En_Pin   GPIO_PIN_11
#define motor2_En_Port  GPIOE
#define motor1_En_Pin   GPIO_PIN_13
#define motor1_En_Port  GPIOE

/* Call after GPIO and TIM1 initialization. All motors start at zero duty. */
void Motor_Init(void);

/* Speed: -1000..1000 (PWM duty command, not measured RPM).
 * Positive: forward; negative: reverse; zero: PWM off.
 * Forward PH: low for motors 1/4, high for motors 2/3.
 * Motors 1..4 use TIM1 CH4, CH3, CH2, CH1 respectively; EN is active high.
 * PH/EN: PC0/PE13, PC1/PE11, PC2/PE9, PC3/PE7 respectively.
 */
void MotorSetSpeed1(int32_t Speed);
void MotorSetSpeed2(int32_t Speed);
void MotorSetSpeed3(int32_t Speed);
void MotorSetSpeed4(int32_t Speed);

#endif
