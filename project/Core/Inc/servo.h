#ifndef SERVO_H
#define SERVO_H

/* TIM9_CH1 / PE5: 1 us per count, 20 ms PWM period. */
#define SERVO_ANGLE_MIN  50.0f
#define SERVO_ANGLE_MAX  140.0f
#define SERVO_PULSE_ANGLE_MAX 180.0f
#define SERVO_PULSE_MIN  500U
#define SERVO_PULSE_MAX  2500U
#define SERVO_OFFSET     0.0f
#define SERVO_UPDATE_PERIOD_MS 20U

void Servo_Init(void);
/* speed is degrees/second; speed <= 0 sets the angle immediately. */
void Servo_SetAngle(float angle, float speed);
void Servo_Update(void);
float Servo_GetAngle(void);

#endif
