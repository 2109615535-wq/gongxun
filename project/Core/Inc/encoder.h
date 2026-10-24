#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f4xx_hal.h"

/* Change a channel to -1 if forward motor motion gives negative counts. */
#define ENCODER1_DIRECTION (1)
#define ENCODER2_DIRECTION (-1)
#define ENCODER3_DIRECTION (-1)
#define ENCODER4_DIRECTION (1)

/* Numbering follows the current motor PH/EN pins:
 * Motor 1: PC0/PE13, A=PD12, B=PD13, TIM4.
 * Motor 2: PC1/PE11, A=PA15, B=PB3,  TIM2.
 * Motor 3: PC2/PE9,  A=PB4,  B=PB5,  TIM3.
 * Motor 4: PC3/PE7,  A=PA0,  B=PA1,  TIM5.
 * TIM encoder mode TI12 counts all four AB edges without GPIO interrupts.
 */
typedef struct
{
    TIM_HandleTypeDef *timer;
    int8_t direction;
    int64_t count;          /* Accumulated signed count since init/reset. */
    int32_t speed;          /* Signed counts per second, not RPM. */
    uint32_t last_counter;
    uint32_t last_tick;
} Encoder_t;

extern Encoder_t encoder1;
extern Encoder_t encoder2;
extern Encoder_t encoder3;
extern Encoder_t encoder4;

/* Call once after MX_TIM2..5_Init() and scheduler_init(). */
void encoder_init(void);
/* The scheduler controls the sampling period; counters keep running on reads.
 * Sample before any counter moves half its range (32768 for TIM3/TIM4).
 * Call process/get/reset from the main loop, not interrupt handlers.
 */
void encoder_process(void);
int64_t encoder_get_count(const Encoder_t *enc);
int32_t encoder_get_speed(const Encoder_t *enc);
/* Clears software count/speed and snapshots the running hardware counter. */
void encoder_reset(Encoder_t *enc);

#endif
