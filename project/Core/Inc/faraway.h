#ifndef FARAWAY_H
#define FARAWAY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* UART5 PC12 TX / PD2 RX, 115200 8N1, crossed TX/RX and common GND.
 * V6 telemetry = V5 + [last_command_id|null,status,x_mm|null,y_mm|null,speed_cm_s|null].
 * Commands: ASCII body + '*' + CRC16-CCITT-FALSE (4 hex) + LF.
 * C1,id,K,key_number              key=1..4, same as hardware actions
 * C1,id,G,x_mm,y_mm,speed_mm_s     field x/y 0..3000, speed >0
 * C1,id,P,kp_1e4,ki_1e4,kd_1e4    shared angle gains, nonnegative, RAM only
 * C1,id,T,0_or_1                 optional S1 stream; refresh within 6 seconds
 * S1=[tag,tick,source,age|null,target,current,error,out,P,I,D,kp,ki,kd]*CRC
 * A1,id,status,step*CRC            reply; status 0 accepted,1 unbound,
 *                                2 physical KEY4 held,3 bad args,4 old sequence.
 * IDs increase modulo uint32; duplicate last ID+CRC does not reapply motion.
 * KEY4 remains effective even with an old ID. No automatic command retry.
 * GET requests telemetry. Legacy motor/PID commands remain rejected.
 * Task stops disable drive; KEY4 also releases the claw.
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
