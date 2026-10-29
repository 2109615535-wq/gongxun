#ifndef FARAWAY_H
#define FARAWAY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* UART5: 115200, 8N1; PC12 TX, PD2 RX. Connect TX/RX crossed and share GND.
 * ASCII protocol, one command per line (LF, CRLF or CR), case sensitive:
 *   TARGET,500       Motor 2 target speed, integer -1000..1000.
 *   PID,1.2,0.3,0    Motor 2 Kp, Ki, Kd; each finite and within 0..100.
 *   ENABLE,0         Disable motor 2 immediately; 1 enables it again.
 *   RUN,1            Enable all four motors; 0 disables all four.
 *   GET              Request an immediate status report.
 * Optional spaces around comma-separated fields are accepted.
 * Maximum command length: 95 bytes, excluding the line ending.
 * Replies: OK,TARGET / OK,PID / OK,ENABLE / OK,RUN, or ERR,<reason>.
 * No command is applied until its complete line has passed validation.
 * An overflow or UART error discards input through the next line ending.
 *
 * Status is one JSON object per line, reported every 50 ms by main.c:
 * {"target":500,"output":480,"speed":499,"kp":1.2,"ki":0.3,"kd":0,"enable":1}
 * Speed is normalized encoder speed, not PWM duty or RPM.
 * Ki/Kd use the existing PID convention, per control update (currently 20 ms).
 * PID changes clear error history. TARGET changes preserve the integral.
 * ENABLE keeps the target and clears error history on both transitions.
 * Remote settings are held in RAM and are lost after reset.
 * SerialWeb: text mode, UTF-8, 115200/8N1; enable LF/CRLF on send.
 * Select text parsing and clear the rule: JSON is parsed automatically.
 * Plot target, output and speed together. Other motors are not controlled.
 */

extern volatile uint32_t g_faraway_rx_overflow_count;
extern volatile uint32_t g_faraway_uart_error_count;
extern volatile uint32_t g_faraway_tx_drop_count;

/* Call after MX_UART5_Init() and Motor_Init(); does not change motor state. */
bool faraway_init(void);
/* Scheduler tasks: process every 5 ms; report every 50 ms. Never call in ISR. */
void faraway_process(void);
void faraway_report(void);
/* Forward UART5 RX/error callbacks here from the shared HAL callbacks. */
void faraway_uart_rx_complete(UART_HandleTypeDef *huart);
void faraway_uart_error(UART_HandleTypeDef *huart);

#endif
