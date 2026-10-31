#ifndef RESCUE_VISION_H
#define RESCUE_VISION_H

#include <stdint.h>

#define RV_RING_SIZE 512u
#define RV_FRAME_MAX 192u
#define RV_MAX_TARGETS 12u
#define RV_TIMEOUT_MS 300u

typedef enum { RV_WAIT, RV_ALIGN, RV_APPROACH, RV_STOPPED } RV_State;
typedef enum { RV_NO_FRAME, RV_TRACKING, RV_INVALID, RV_HAZARD,
               RV_TIMEOUT, RV_TOO_CLOSE, RV_ARRIVED, RV_OVERFLOW } RV_Reason;

typedef struct {
    uint8_t type, confidence;
    int16_t angle_cdeg;
    uint16_t distance_mm, width_px, height_px;
    uint8_t posture, flags;
} RV_Target;

typedef struct {
    uint32_t source_tick_ms, received_ms, generation;
    uint8_t flags, status, count, sequence;
    RV_Target targets[RV_MAX_TARGETS];
} RV_Observation;

typedef struct {
    float bumper_forward_mm, collection_center_left_mm;
    /* Copy these offsets from the validated camera calibration, in chassis axes. */
    uint8_t allow_forward; /* 0: turn-only trial; 1: approach at <=100mm/s */
} RV_Config;

typedef struct {
    RV_State state;
    RV_Reason reason;
    float forward_mm_s;
    float clockwise_rad_s; /* right target -> positive; adapt chassis sign explicitly */
    float gap_mm, lateral_mm;
} RV_Command;

typedef struct {
    uint32_t received_ms;
    uint8_t byte;
} RV_RxByte;

typedef struct {
    RV_Config config;
    volatile uint16_t head, tail;
    volatile uint8_t overflow;
    volatile RV_RxByte ring[RV_RING_SIZE];
    uint8_t frame[RV_FRAME_MAX];
    uint16_t used;
    uint32_t partial_ms, source_rx_ms, last_source_tick;
    uint8_t have_source, arrival_count;
    uint32_t generation, consumed_generation, crc_errors, format_errors, duplicates;
    RV_Observation observation;
    RV_Command command;
} RV_Receiver;

/* Init before enabling UART IRQ. The module uses no HAL or motor interfaces. */
void RV_Init(RV_Receiver *receiver, RV_Config config);
/* One UART ISR producer only. received_ms is local HAL_GetTick(), not sender time. */
void RV_OnByte(RV_Receiver *receiver, uint8_t byte, uint32_t received_ms);
/* Single control-context consumer, call every 10ms. Always apply returned zeroes too. */
RV_Command RV_Step(RV_Receiver *receiver, uint32_t now_ms);
/* Explicitly start a new trial after STOPPED; call in the control context only. */
void RV_ResetTrial(RV_Receiver *receiver);
uint16_t RV_Crc16(const uint8_t *data, uint16_t length);

#endif
