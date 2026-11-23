#ifndef FARAWAY_H
#define FARAWAY_H

#include <stdbool.h>
#include <stdint.h>
#include "stm32f4xx_hal.h"

/* UART5 PC12 TX / PD2 RX, 115200 8N1, crossed TX/RX and common GND.
 * Upstream uses the COBS-framed binary format defined in faraway.c.
 * Checksummed commands: ASCII body + '*' + CRC16-CCITT-FALSE (4 hex) + CR/LF.
 * C1,id,K,key_number              key=1..4, same as hardware actions
 * C1,id,G,x_mm,y_mm,speed_mm_s     field x/y 0..3000, speed >0
 * C1,id,P,kp_1e4,ki_1e4,kd_1e4    shared angle gains, nonnegative, RAM only
 * C1,id,T,0_or_1                 optional STEER stream; refresh within 6 seconds
 * HIST,epoch,id                  request one confirmed object's HISTORY
 * GET returns INFO and CONTROL. MAP requests a full MAP_SYNC.
 * KRUN?,nonce adopts the first nonzero browser nonce after boot; KRUN? polls.
 * KREAD,boot,session,seq requests one CRC-protected diagnostic chunk.
 * ACK status: 0 accepted, 1 unbound, 2 KEY4 held, 3 bad args, 4 old sequence.
 * IDs increase modulo uint32; duplicate last ID+CRC does not reapply motion.
 * KEY4 remains effective even with an old ID. No automatic command retry.
 * Legacy motor/PID commands remain rejected.
 * Task stops disable drive; KEY4 also releases the claw.
 */

extern volatile uint32_t g_faraway_rx_overflow_count;
extern volatile uint32_t g_faraway_uart_error_count;
extern volatile uint32_t g_faraway_tx_drop_count;

/* Call after MX_UART5_Init() and Motor_Init(); does not change motor state. */
bool faraway_init(void);
/* Scheduler tasks. Never call in ISR. */
void faraway_process(void);
void faraway_report(void);
/* Record the final step/reason immediately after task or key dispatch. */
void faraway_task_observe(void);
/* KEY2 diagnostics are collected in main-loop context before any UART work.
 * Include START (kind 1) first and END (kind 7) before calling hui_log_end().
 * While active a repeated begin returns 0 and preserves the current run. */
uint8_t hui_log_begin(uint32_t tick);
uint8_t hui_log_active(void);
void hui_log_record(uint8_t kind, uint32_t tick, const void *data, uint8_t size);
void hui_log_end(void);
/* Forward UART5 RX/error callbacks here from the shared HAL callbacks. */
void faraway_uart_rx_complete(UART_HandleTypeDef *huart);
void faraway_uart_error(UART_HandleTypeDef *huart);

#endif
