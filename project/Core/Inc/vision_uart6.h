#ifndef VISION_UART6_H
#define VISION_UART6_H

#include "stm32f4xx_hal.h"

/* IRQ callbacks enqueue bytes. Parsing and OLED access run in the scheduler. */
void vision_uart6_init(void);
void vision_uart6_process(void);
void vision_uart6_oled_task(void);
void vision_uart6_rx_complete(UART_HandleTypeDef *huart);
void vision_uart6_error(UART_HandleTypeDef *huart);

extern volatile uint32_t g_vision_uart6_rx_bytes;
extern volatile uint32_t g_vision_uart6_overflows;
extern volatile uint32_t g_vision_uart6_errors;

#endif
