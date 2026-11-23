#ifndef VISION_H
#define VISION_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/* Simple vision observations (RV1-RV4), USART6: PC6 TX / PC7 RX, 115200 8N1.
 * Application APIs and result reads belong to the main loop, never an ISR.
 * Results are updated/expired by vision_process() every 5 ms. */
#define VISION_MAX_OBJECTS 12U
#define VISION_GREEN_MASK  1U
#define VISION_BLACK_MASK  2U
#define VISION_ORANGE_MASK 4U

typedef enum {
    VISION_NONE, VISION_GREEN, VISION_BLACK, VISION_ORANGE, VISION_BLUE,
    VISION_RED_CASUALTY_ZONE, VISION_RED_MATERIAL_ZONE,
    VISION_BLUE_CASUALTY_ZONE, VISION_BLUE_MATERIAL_ZONE, VISION_UNKNOWN
} VisionKind;

typedef enum {
    VISION_IDLE, VISION_SCAN, VISION_SEARCH, VISION_ALIGN, VISION_APPROACH,
    VISION_VERIFY_BEFORE_TRANSFER, VISION_VERIFY_AFTER_TRANSFER,
    VISION_NEAR_DOCK, VISION_DELIVER, VISION_RELEASE, VISION_RETREAT,
    VISION_DELIVERY_CHECK
} VisionPhase;

typedef enum { VISION_RED, VISION_TEAM_BLUE } VisionTeam;
typedef enum {
    VISION_MECHANISM_UNKNOWN, VISION_OPEN, VISION_CLOSED, VISION_OPENING,
    VISION_CLOSING, VISION_AT_POSITION, VISION_MECHANISM_FAULT
} VisionMechanism;

typedef struct {
    uint32_t task_id;             //
    uint8_t state_id;             /* Competition state 0..3, NOT local step. */
    VisionPhase phase;
    VisionTeam team;
    uint8_t allowed_kinds;         /* OR the three *_MASK values. */
    VisionKind expected_kind;     /* VISION_NONE sends null. Count is always 1. */
    uint32_t locked_target_id;    /* 0 = unlocked; upper/lower IDs differ. */
    VisionMechanism mechanism;
} VisionTask;

typedef struct {
    uint32_t target_id;
    VisionKind kind;
    int32_t distance_mm;
    int32_t x_mm, y_mm;           /* RV3/RV4 ground coordinates in mm, +X right and +Y forward. */
    float angle;
    uint8_t distance_valid, angle_valid, in_roi, in_safety_zone;
    uint8_t position_present;    /* Both coordinates supplied, including unverified estimates. */
    uint8_t angle_present;        /* Numeric angle, including 0; false for NA. */
} VisionObject;

typedef enum {
    VISION_OBS_OK, VISION_OBS_EMPTY, VISION_OBS_STALE,
    VISION_OBS_FAULT, VISION_OBS_OVERFLOW
} VisionObservationStatus;

typedef struct {
    uint8_t available;            /* Normal, current frame, total age <=400 ms. */
    uint8_t fresh;                /* Diagnostic freshness, independent of ready. */
    uint8_t danger_flag;
    VisionObservationStatus status;
    uint32_t frame_id, age_ms;
    uint32_t updated_ms;          /* Local tick when age_ms was calculated. */
    uint8_t count;
    VisionObject objects[VISION_MAX_OBJECTS];
    float yaw_deg;
    uint8_t yaw_valid;
} VisionObservation;

typedef struct {
    uint8_t result_valid, danger_flag;
    uint32_t frame_id, age_ms, roi_count;
    uint8_t kinds_count;
    VisionKind roi_kinds[VISION_MAX_OBJECTS];
    char danger_reason[64];
} VisionVerification;

/* The current Python implementation only supplies invalid dock/delivery
 * placeholders. Positive validity/success flags are deliberately rejected. */
typedef struct {
    uint8_t received, dock_valid, result_valid, danger_flag;
    uint8_t entry_allowed, object_inside, delivery_confirmed;
    char zone_kind[32], danger_reason[64];
} VisionPlaceholder;

typedef struct {
    uint8_t connected, acknowledged, vision_ready, danger_flag;
    uint32_t ack_seq;
    char fault_code[64];
    VisionObservation upper, lower;
    VisionVerification verification;
    VisionPlaceholder dock, delivery;
    uint32_t rx_packets, bad_packets, ignored_packets, uart_errors, tx_packets;
} VisionResult;

extern VisionResult vision;       /* Read directly; application must not write. */

bool vision_init(void);           /* Start observation reception on USART6. */
bool vision_set_task(const VisionTask *task); /* Simple protocols send observations only. */
void vision_set_yaw(float degrees, bool valid); /* Unknown until explicitly set. */
void vision_process(void);        /* RX and expiry. */

/* Routed from the project's existing HAL callbacks. */
void vision_uart_rx_complete(UART_HandleTypeDef *huart);
void vision_uart_tx_complete(UART_HandleTypeDef *huart);
void vision_uart_error(UART_HandleTypeDef *huart);

#endif
