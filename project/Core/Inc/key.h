#ifndef KEY_H
#define KEY_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define key1_Pin    GPIO_PIN_3
#define key1_Port   GPIOD
#define key4_Pin    GPIO_PIN_4
#define key4_Port   GPIOD
#define key3_Pin    GPIO_PIN_5
#define key3_Port   GPIOD
#define key2_Pin    GPIO_PIN_6
#define key2_Port   GPIOD

#define KEY_1 (1U << 0)
#define KEY_2 (1U << 1)
#define KEY_3 (1U << 2)
#define KEY_4 (1U << 3)

/* Active-low key bitmap; bit values are KEY_1..KEY_4. */
uint8_t Key_Read(void);
/* Call every 10 ms. A change is accepted after two equal samples. */
void Key_Update(void);
uint8_t Key_GetState(void);
/* Release events accumulate until read. */
uint8_t Key_GetUp(void);
/* Angle selected with KEY_1/KEY_2; KEY_3 sends it to the servo. */
extern int16_t servo_angle;

#endif
