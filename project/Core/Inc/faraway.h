#ifndef FARAWAY_H
#define FARAWAY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* UART5: 115200, 8N1; PC12 TX, PD2 RX. Connect TX/RX crossed and share GND.
 * ASCII protocol, one command per line (LF, CRLF or CR), case sensitive:
 *   TARGET,500       Set all four motor target speeds, integer -1000..1000.
 *   PID,2,0.3,0      Motor 1 Kp, Ki, Kd; each finite and within 0..100.
 *   ENABLE,0         Disable motor 1 immediately; 1 enables it again.
 *   RUN,1            Enable motor 1; 0 disables it.
 *   GET              Request an immediate status report.
 * Optional spaces around comma-separated fields are accepted.
 * Maximum command length: 95 bytes, excluding the line ending.
 * Replies: OK,TARGET / OK,PID / OK,ENABLE / OK,RUN, or ERR,<reason>.
 * No command is applied until its complete line has passed validation.
 * An overflow or UART error discards input through the next line ending.
 *
 * Status is one JSON object per line, reported every 50 ms by main.c:
 * {"speed1":499,"target1":500,"speed2":498,"target2":500,
 *  "speed3":501,"target3":500,"speed4":500,"target4":500,"y_body":12.34}
 * y_body is forward displacement from the odometry zero, in cm (2 decimals).
 * Current odometry assumes a fixed chassis heading.
 * Speed is normalized encoder speed, not PWM duty or RPM.
 * Ki/Kd use the existing PID convention, per control update (currently 20 ms).
 * PID changes clear error history. TARGET changes preserve the integral.
 * ENABLE keeps the target and clears error history on both transitions.
 * Remote settings are held in RAM and are lost after reset.
 * SerialWeb: text mode, UTF-8, 115200/8N1; enable LF/CRLF on send.
 * Select text parsing and clear the rule: JSON is parsed automatically.
 * Plot speedN and targetN together for the desired motor (N=1..4).
 * TARGET addresses all four motors; PID, ENABLE and RUN address motor 1.
 */

extern volatile uint32_t g_faraway_rx_overflow_count;
extern volatile uint32_t g_faraway_uart_error_count;
extern volatile uint32_t g_faraway_tx_drop_count;

/* Call after MX_UART5_Init() and Motor_Init(); does not change motor state. */
bool faraway_init(void);
/* Scheduler tasks: process/report every 50 ms in main.c. Never call in ISR. */
void faraway_process(void);
void faraway_report(void);
/* Forward UART5 RX/error callbacks here from the shared HAL callbacks. */
void faraway_uart_rx_complete(UART_HandleTypeDef *huart);
void faraway_uart_error(UART_HandleTypeDef *huart);

#endif
