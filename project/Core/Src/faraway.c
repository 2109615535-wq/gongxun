#include "faraway.h"
#include "vision.h"
#include "key.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vision_simple.h"
#include "motor.h"
#include "decode_move.h"
#include "field_map.h"
#include "imu.h"
#include <math.h>

/* UART5 upstream binary wire format and bounded payload encoder. */
#define U5_VERSION 1U
#define U5_MAX_PAYLOAD 448U
#define U5_MAX_RAW (U5_MAX_PAYLOAD + 12U)
#define U5_MAX_WIRE (U5_MAX_RAW + U5_MAX_RAW / 254U + 2U)
#define U5_MAX_OBJECTS 12U

typedef enum {
    U5_LIVE = 1, U5_EVENT, U5_TRACK, U5_HISTORY, U5_INFO, U5_STATS,
    U5_STEER, U5_ACK, U5_CONTROL, U5_ERROR, U5_MAP_SYNC,
    U5_KEY2_META, U5_KEY2_CHUNK
} U5_Type;

typedef enum {
    U5_ERR_TASK_CONTROL = 1, U5_ERR_CONTROL_CRC, U5_ERR_CONTROL_FORMAT,
    U5_ERR_RX_LOST, U5_ERR_LINE, U5_ERR_HIST_UNAVAILABLE
} U5_Error;

enum {
    U5_LINK = 1, U5_IMU_READY = 2, U5_HEADING = 4,
    U5_HAS_UPPER = 8, U5_HAS_LOWER = 16,
    U5_FRESH = 1, U5_AVAILABLE = 2, U5_ALIGNED = 4,
    U5_DISTANCE = 1, U5_ANGLE = 2, U5_RAW_XY = 4, U5_VALID = 8, U5_FIELD_XY = 16,
    U5_IN_ROI = 1, U5_HAS_ZONE = 2, U5_INSIDE = 4
};

typedef struct {
    int32_t x_mm, y_mm, yaw_cdeg;
} U5_Pose;

typedef struct {
    uint32_t frame, age;
    uint8_t status, flags, count;
} U5_Observation;

typedef struct {
    uint8_t kind, flags;
    uint16_t distance;
    int16_t angle;
    int32_t raw_x, raw_y, field_x, field_y;
    uint32_t track_id;
    uint16_t radius;
} U5_UpperObject;

typedef struct {
    uint8_t kind, flags, zone;
} U5_LowerObject;

typedef struct {
    uint8_t step, reason, held_kind, roi_counts;
    int16_t servo;
    uint8_t flags;
    U5_Pose robot;
    uint32_t imu_age;
    uint16_t imu_yaw;
    U5_Observation upper, lower;
    U5_Pose observed;
    U5_UpperObject upper_objects[U5_MAX_OBJECTS];
    U5_LowerObject lower_objects[U5_MAX_OBJECTS];
} U5_Live;

typedef struct {
    uint32_t epoch, map_rev, unmapped, id, revision;
    uint8_t count, kind, flags;
    int32_t x_mm, y_mm;
    uint16_t radius;
    uint32_t seen, position, observations;
} U5_Track;

typedef struct {
    uint16_t length;
    uint8_t bytes[U5_MAX_PAYLOAD];
} U5_Payload;

static void U5_U8(U5_Payload *p, uint8_t value)
{
    if (p->length < U5_MAX_PAYLOAD) {
        p->bytes[p->length++] = value;
    } else {
        p->length = U5_MAX_PAYLOAD + 1U;
    }
}

static void U5_U16(U5_Payload *p, uint16_t value)
{
    U5_U8(p, (uint8_t)value);
    U5_U8(p, (uint8_t)(value >> 8));
}

static void U5_U32(U5_Payload *p, uint32_t value)
{
    U5_U16(p, (uint16_t)value);
    U5_U16(p, (uint16_t)(value >> 16));
}

static void U5_F32(U5_Payload *p, float value)
{
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    U5_U32(p, bits);
}

static void U5_PutPose(U5_Payload *p, const U5_Pose *pose)
{
    U5_U32(p, (uint32_t)pose->x_mm);
    U5_U32(p, (uint32_t)pose->y_mm);
    U5_U32(p, (uint32_t)pose->yaw_cdeg);
}

static void put_observation(U5_Payload *p, const U5_Observation *obs)
{
    U5_U32(p, obs->frame);
    U5_U32(p, obs->age);
    U5_U8(p, obs->status);
    U5_U8(p, obs->flags);
    U5_U8(p, obs->count);
}

static void U5_LivePayload(U5_Payload *p, const U5_Live *live)
{
    unsigned i;
    if (((live->flags & U5_HAS_UPPER) && live->upper.count > U5_MAX_OBJECTS) ||
        ((live->flags & U5_HAS_LOWER) && live->lower.count > U5_MAX_OBJECTS)) {
        p->length = U5_MAX_PAYLOAD + 1U;
        return;
    }
    U5_U8(p, live->step);
    U5_U8(p, live->reason);
    U5_U8(p, live->held_kind);
    U5_U8(p, live->roi_counts);
    U5_U16(p, (uint16_t)live->servo);
    U5_U8(p, live->flags);
    U5_PutPose(p, &live->robot);
    U5_U32(p, live->imu_age);
    U5_U16(p, live->imu_yaw);
    if (live->flags & U5_HAS_UPPER) {
        put_observation(p, &live->upper);
        if (live->upper.flags & U5_ALIGNED) U5_PutPose(p, &live->observed);
        for (i = 0; i < live->upper.count; ++i) {
            const U5_UpperObject *o = &live->upper_objects[i];
            U5_U8(p, o->kind);
            U5_U8(p, o->flags);
            if (o->flags & U5_DISTANCE) U5_U16(p, o->distance);
            if (o->flags & U5_ANGLE) U5_U16(p, (uint16_t)o->angle);
            if (o->flags & U5_RAW_XY) {
                U5_U32(p, (uint32_t)o->raw_x);
                U5_U32(p, (uint32_t)o->raw_y);
            }
            if (o->flags & U5_FIELD_XY) {
                U5_U32(p, (uint32_t)o->field_x);
                U5_U32(p, (uint32_t)o->field_y);
                U5_U32(p, o->track_id);
                U5_U16(p, o->radius);
            }
        }
    }
    if (live->flags & U5_HAS_LOWER) {
        put_observation(p, &live->lower);
        for (i = 0; i < live->lower.count; ++i) {
            const U5_LowerObject *o = &live->lower_objects[i];
            U5_U8(p, o->kind);
            U5_U8(p, o->flags);
            if (o->flags & U5_HAS_ZONE) U5_U8(p, o->zone);
        }
    }
}

static void U5_TrackPayload(U5_Payload *p, const U5_Track *t)
{
    U5_U32(p, t->epoch);
    U5_U32(p, t->map_rev);
    U5_U8(p, t->count);
    U5_U32(p, t->unmapped);
    U5_U32(p, t->id);
    U5_U32(p, t->revision);
    U5_U8(p, t->kind);
    U5_U8(p, t->flags);
    U5_U32(p, (uint32_t)t->x_mm);
    U5_U32(p, (uint32_t)t->y_mm);
    U5_U16(p, t->radius);
    U5_U32(p, t->seen);
    U5_U32(p, t->position);
    U5_U32(p, t->observations);
}

static uint16_t U5_Frame(uint8_t *out, U5_Type type, uint16_t seq, uint32_t tick,
                  const U5_Payload *payload)
{
    uint8_t raw[U5_MAX_RAW], block_size = 1, byte;
    uint16_t i, len, crc, write = 1, code_index = 0;
    if (payload->length > U5_MAX_PAYLOAD) return 0;
    raw[0] = U5_VERSION;
    raw[1] = (uint8_t)type;
    raw[2] = (uint8_t)seq;
    raw[3] = (uint8_t)(seq >> 8);
    raw[4] = (uint8_t)payload->length;
    raw[5] = (uint8_t)(payload->length >> 8);
    for (i = 0; i < 4U; ++i) raw[i + 6U] = (uint8_t)(tick >> (i * 8U));
    memcpy(raw + 10U, payload->bytes, payload->length);
    len = (uint16_t)(10U + payload->length);
    crc = VSP_Crc16(raw, len);
    raw[len++] = (uint8_t)crc;
    raw[len++] = (uint8_t)(crc >> 8);
    for (i = 0; i < len; ++i) {
        byte = raw[i];
        if (byte) {
            out[write++] = byte;
            if (++block_size != 255U) continue;
        }
        out[code_index] = block_size;
        code_index = write++;
        block_size = 1;
    }
    out[code_index] = block_size;
    out[write++] = 0;
    return write;
}

#define FARAWAY_RX_SIZE       512U
#define FARAWAY_RX_MASK       (FARAWAY_RX_SIZE - 1U)
#define FARAWAY_TX_SIZE       4096U
#define FARAWAY_TX_CHUNK 64U
#define FARAWAY_TX_GAP_MS 5U
#define FARAWAY_TX_MASK       (FARAWAY_TX_SIZE - 1U)
#define FARAWAY_LINE_SIZE     96U
#define FARAWAY_PROCESS_LIMIT 128U
#define LIVE_RESERVE          39U
#define ACK_CAPACITY         16U
#define EVENT_CAPACITY       16U
#define ERROR_CAPACITY        8U

extern UART_HandleTypeDef huart5;

static uint8_t rx_byte;
static uint8_t rx_buffer[FARAWAY_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_fault;
static uint8_t tx_buffer[FARAWAY_TX_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_length;
static volatile bool opened;
static volatile uint32_t tx_next_ms;
static char line[FARAWAY_LINE_SIZE];
static uint16_t line_length;
static bool discard_line;
static uint8_t command_seen, command_status;
static uint32_t command_id;
static uint16_t command_crc;
static uint8_t steer_stream;
static uint32_t steer_stream_tick, steer_last;
static uint8_t packet[U5_MAX_WIRE];
static U5_Payload payload;
static uint16_t tx_seq, tx_budget, tx_high_water, camera_wire_cost;
static volatile uint32_t tx_completed_bytes;
static uint32_t budget_tick, next_live, next_camera, camera_deadline, next_stats, next_track;
static bool live_pending, info_pending, control_pending, history_pending;
static uint32_t merged_snapshots, event_overflow, history_epoch, history_id;
static GotoTarget sent_target;

typedef struct { uint32_t tick, id; uint8_t status, step; } Ack;
typedef struct { uint32_t tick; uint8_t step, reason; } Event;
typedef struct { uint32_t tick; U5_Error reason; } ErrorReply;
static Ack acks[ACK_CAPACITY];
static Event events[EVENT_CAPACITY];
static ErrorReply errors[ERROR_CAPACITY];
static unsigned ack_head, ack_count, event_head, event_count, error_head, error_count;
static uint8_t observed_step, observed_reason;

static uint32_t map_epoch, map_sent[FIELD_TRACK_CAPACITY];
static uint8_t map_delivered[FIELD_TRACK_CAPACITY];
static unsigned map_cursor;
static uint16_t sync_id;
static uint8_t sync_index, sync_total;
static bool sync_active, sync_started, sync_requested;

/* KEY2 records use ordinary SRAM and CPU-only CCM. No UART transfer refers
 * to either bank directly. One sparse index entry covers 16 records.
 * Samples leave room for state changes and a dedicated END slot. Even a
 * discarded sample consumes a sequence number so the page can show the gap. */
#define HUI_LOG_SRAM_BYTES (58U * 1024U)
#define HUI_LOG_CCM_BYTES (64U * 1024U)
#define HUI_LOG_BYTES (HUI_LOG_SRAM_BYTES + HUI_LOG_CCM_BYTES)
#define HUI_LOG_GROUP 16U
#define HUI_LOG_GROUP_COUNT 256U
#define HUI_LOG_RECORDS (HUI_LOG_GROUP * HUI_LOG_GROUP_COUNT)
#define HUI_LOG_HEADER 10U
#define HUI_LOG_CRITICAL_BYTES 3072U
#define HUI_LOG_END_BYTES (HUI_LOG_HEADER + UINT8_MAX)
#define HUI_CHUNK_HEADER 17U
#define HUI_CHUNK_TARGET 365U
/* Optional traffic during driving aims for 1500 B/s total, counting all packets.
 * After KEY4 the normal 2000 B/s UART budget governs recovery. */
#define HUI_MOVING_CREDIT_MAX 300000L
#define HUI_MOVING_CREDIT_MIN (-1500000L)
#define HUI_SCHEMA_VERSION 1U

/* START, STEP and END use reserved capacity. Record kinds match hui_ku.h. */
enum { HUI_KEEP_START = 1, HUI_KEEP_STEP = 2, HUI_KEEP_END = 7 };

static uint8_t hui_log_sram[HUI_LOG_SRAM_BYTES];
#if defined(__CC_ARM)
__attribute__((section("KEY2_CCM"), zero_init))
#elif (defined(__ARMCC_VERSION) || defined(__arm__)) && !defined(_WIN32)
__attribute__((section("KEY2_CCM")))
#endif
static uint8_t hui_log_ccm[HUI_LOG_CCM_BYTES];
static uint32_t hui_groups[HUI_LOG_GROUP_COUNT];
static struct {
    uint32_t boot, session, seq, dropped, start_tick, end_tick, read_seq;
    uint32_t used, sram_used;
    int32_t moving_credit;
    uint16_t records;
    bool active, sealed, meta_pending, read_pending;
} hui_log;

static uint8_t *hui_record_at(uint32_t offset)
{
    return offset < HUI_LOG_SRAM_BYTES ? hui_log_sram + offset :
           hui_log_ccm + (offset - HUI_LOG_SRAM_BYTES);
}

static uint32_t hui_next_offset(uint32_t offset, unsigned length)
{
    uint32_t next = offset + length;
    if (offset < HUI_LOG_SRAM_BYTES && next == hui_log.sram_used &&
        hui_log.used > HUI_LOG_SRAM_BYTES) return HUI_LOG_SRAM_BYTES;
    return next;
}

static void hui_write_u32(uint8_t *p, uint32_t value)
{
    unsigned i;
    for (i = 0; i < 4U; ++i) p[i] = (uint8_t)(value >> (8U * i));
}

static uint32_t hui_read_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

uint8_t hui_log_begin(uint32_t tick)
{
    if (hui_log.active) return 0;
    if (++hui_log.session == 0) ++hui_log.session;
    hui_log.seq = hui_log.dropped = 0;
    hui_log.used = hui_log.sram_used = hui_log.records = 0;
    hui_log.start_tick = tick;
    hui_log.end_tick = 0;
    hui_log.sealed = hui_log.read_pending = false;
    hui_log.active = hui_log.meta_pending = true;
    return 1;
}

uint8_t hui_log_active(void)
{
    return hui_log.active;
}

void hui_log_record(uint8_t kind, uint32_t tick, const void *data, uint8_t size)
{
    unsigned count_limit = HUI_LOG_RECORDS - 1U;
    uint32_t byte_limit = HUI_LOG_BYTES - HUI_LOG_END_BYTES;
    unsigned length = HUI_LOG_HEADER + size;
    uint32_t offset = hui_log.used;
    uint8_t *record;

    if (!hui_log.active || hui_log.sealed) return;
    if (kind == HUI_KEEP_END) {
        byte_limit = HUI_LOG_BYTES;
        count_limit = HUI_LOG_RECORDS;
    } else if (kind != HUI_KEEP_START && kind != HUI_KEEP_STEP) {
        byte_limit -= HUI_LOG_CRITICAL_BYTES;
        count_limit -= 64U;
    }
    if (offset < HUI_LOG_SRAM_BYTES && offset + length > HUI_LOG_SRAM_BYTES)
        offset = HUI_LOG_SRAM_BYTES; /* A record always fits wholly in one bank. */
    if (offset + length > byte_limit || hui_log.records >= count_limit) {
        ++hui_log.seq;
        ++hui_log.dropped;
        return;
    }

    if (hui_log.records % HUI_LOG_GROUP == 0)
        hui_groups[hui_log.records / HUI_LOG_GROUP] = offset;
    record = hui_record_at(offset);
    hui_write_u32(record, hui_log.seq++);
    hui_write_u32(record + 4U, tick);
    record[8] = kind;
    record[9] = size;
    if (size) memcpy(record + HUI_LOG_HEADER, data, size);
    hui_log.used = offset + length;
    if (offset < HUI_LOG_SRAM_BYTES) hui_log.sram_used = hui_log.used;
    ++hui_log.records;

    if (kind == HUI_KEEP_END) {
        hui_log.end_tick = tick;
        hui_log.sealed = true;
    }
}

void hui_log_end(void)
{
    if (!hui_log.active) return;
    hui_log.active = false;
    hui_log.meta_pending = true;
}

volatile uint32_t g_faraway_rx_overflow_count;
volatile uint32_t g_faraway_uart_error_count;
volatile uint32_t g_faraway_tx_drop_count;

/* Called with interrupts masked, or from the UART ISR. The queued storage
 * remains valid until HAL reports that this contiguous part has completed. */
static void faraway_tx_start(void)
{
    uint16_t length;

    if (!opened || tx_length != 0U || tx_tail == tx_head) {
        return;
    }
    length = tx_head > tx_tail ? (uint16_t)(tx_head - tx_tail)
                              : (uint16_t)(FARAWAY_TX_SIZE - tx_tail);
    if ((int32_t)(HAL_GetTick() - tx_next_ms) < 0) return;
    if (length > FARAWAY_TX_CHUNK) length = FARAWAY_TX_CHUNK;
    tx_length = length;
    if (HAL_UART_Transmit_IT(&huart5, &tx_buffer[tx_tail], length) != HAL_OK) {
        tx_length = 0U;
        g_faraway_uart_error_count++;
    }
}

static bool faraway_send(const uint8_t *bytes, uint16_t length)
{
    uint32_t irq;
    uint16_t free_bytes;
    uint16_t i;
    uint16_t next_head;

    if (!opened || !length) return false;
    irq = __get_PRIMASK();
    __disable_irq();
    free_bytes = (uint16_t)((tx_tail + FARAWAY_TX_SIZE - tx_head - 1U)
                            & FARAWAY_TX_MASK);
    if (length > free_bytes) {
        g_faraway_tx_drop_count++;
        __set_PRIMASK(irq);
        return false;
    }
    next_head = tx_head;
    __set_PRIMASK(irq);
    /* Sole producer is the main loop. Keep RX interrupts enabled while copying
     * a long report; ISR sees the old head until the complete frame is ready. */
    for (i = 0U; i < length; i++) {
        tx_buffer[next_head] = bytes[i];
        next_head = (uint16_t)((next_head + 1U) & FARAWAY_TX_MASK);
    }
    irq = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tx_head = next_head;
    i = (uint16_t)((tx_head + FARAWAY_TX_SIZE - tx_tail) & FARAWAY_TX_MASK);
    if (i > tx_high_water) tx_high_water = i;
    faraway_tx_start();
    __set_PRIMASK(irq);
    return true;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        vision_uart_tx_complete(huart);
        return;
    }
    if (huart->Instance == UART5 && opened) {
        tx_completed_bytes += tx_length;
        tx_tail = (uint16_t)((tx_tail + tx_length) & FARAWAY_TX_MASK);
        tx_length = 0U;
        tx_next_ms = HAL_GetTick() + FARAWAY_TX_GAP_MS;
    }
}

bool faraway_init(void)
{
    if (opened) {
        return true;
    }
    if (huart5.Instance != UART5) {
        return false;
    }
#ifdef RCC_AHB1ENR_CCMDATARAMEN
    RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
    (void)RCC->AHB1ENR;
#endif
    rx_head = 0U;
    rx_tail = 0U;
    rx_fault = false;
    tx_head = 0U;
    tx_tail = 0U;
    tx_length = 0U;
    line_length = 0U;
    discard_line = false;
    command_seen = command_status = 0; command_id = 0;
    steer_stream = 0; steer_stream_tick = steer_last = 0;
    g_faraway_rx_overflow_count = 0U;
    g_faraway_uart_error_count = 0U;
    g_faraway_tx_drop_count = 0U;
    tx_next_ms = HAL_GetTick();
    tx_seq = tx_high_water = camera_wire_cost = 0;
    tx_budget = U5_MAX_WIRE;
    hui_log.moving_credit = HUI_MOVING_CREDIT_MAX;
    tx_completed_bytes = 0;
    budget_tick = next_live = next_camera = camera_deadline = next_track = tx_next_ms;
    next_stats = tx_next_ms + 1000U;
    live_pending = history_pending = false;
    control_pending = info_pending = sync_requested = true;
    hui_log.meta_pending = true;
    merged_snapshots = event_overflow = 0;
    ack_head = ack_count = event_head = event_count = error_head = error_count = 0;
    observed_step = step;
    observed_reason = task_info.reason;
    map_epoch = field_map.epoch;
    memset(map_delivered, 0, sizeof(map_delivered));
    map_cursor = sync_id = 0;
    sync_active = false;
    opened = true;
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    if (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) != HAL_OK) {
        opened = false;
        g_faraway_uart_error_count++;
        return false;
    }
    return true;
}

void faraway_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != UART5 || !opened) {
        return;
    }
    next = (uint16_t)((rx_head + 1U) & FARAWAY_RX_MASK);
    if (next == rx_tail) {
        rx_fault = true;
        g_faraway_rx_overflow_count++;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        rx_fault = true;
        g_faraway_uart_error_count++;
    }
}

void faraway_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5 || !opened) {
        return;
    }
    g_faraway_uart_error_count++;
    rx_fault = true;
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        g_faraway_uart_error_count++;
    }
}

static char *faraway_trim(char *text)
{
    char *end;

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }
    return text;
}

static unsigned telemetry_fault(void)
{
    if (!strcmp(vision.fault_code, "waiting_rv3")) return 1;
    if (!strcmp(vision.fault_code, "invalid_packet")) return 2;
    if (!strcmp(vision.fault_code, "uart_error")) return 3;
    if (!strcmp(vision.fault_code, "vision_timeout")) return 4;
    if (!strcmp(vision.fault_code, "upper_unavailable")) return 5;
    return 0;
}

static U5_Pose wire_pose(const FieldPose *pose)
{
    U5_Pose result;
    result.x_mm = (int32_t)(pose->x * 10.0f);
    result.y_mm = (int32_t)(pose->y * 10.0f);
    result.yaw_cdeg = (int32_t)fmodf(pose->yaw * 18000.0f / 3.14159265358979323846f, 36000.0f);
    return result;
}

static U5_Observation wire_observation(const VisionObservation *obs, uint32_t now)
{
    U5_Observation out;
    uint32_t elapsed = now - obs->updated_ms;
    out.frame = obs->frame_id;
    out.age = obs->age_ms;
    out.status = (uint8_t)obs->status;
    out.flags = 0;
    out.count = 0;
    if (out.age != UINT32_MAX && elapsed >= UINT32_MAX - out.age)
        out.age = UINT32_MAX - 1U;
    else if (out.age != UINT32_MAX)
        out.age += elapsed;
    if (obs->fresh && out.age <= VSP_TIMEOUT_MS) {
        out.flags = U5_FRESH | (obs->available ? U5_AVAILABLE : 0);
        out.count = obs->count;
        if (out.count > U5_MAX_OBJECTS) {
            out.status = VISION_OBS_OVERFLOW;
            out.flags = out.count = 0;
        }
    } else if (obs->fresh) {
        out.status = VISION_OBS_STALE;
    }
    return out;
}

static void fill_live_cameras(U5_Live *live, uint32_t now)
{
    unsigned i;
    live->flags |= U5_HAS_UPPER | U5_HAS_LOWER;
    live->upper = wire_observation(&vision.upper, now);
    live->lower = wire_observation(&vision.lower, now);
    if ((live->upper.flags & U5_FRESH) && field_map.aligned && field_map.frame == live->upper.frame) {
        live->upper.flags |= U5_ALIGNED;
        live->observed = wire_pose(&field_map.observed);
    }
    for (i = 0; i < live->upper.count; ++i) {
        const VisionObject *o = &vision.upper.objects[i];
        const FieldPoint *p = &field_map.points[i];
        U5_UpperObject *out = &live->upper_objects[i];
        out->kind = (uint8_t)o->kind;
        out->flags = 0;
        if (o->distance_mm >= 0) {
            out->flags |= U5_DISTANCE;
            out->distance = (uint16_t)o->distance_mm;
        }
        if (o->angle_present) {
            out->flags |= U5_ANGLE;
            out->angle = (int16_t)(o->angle * 100.0f + (o->angle < 0 ? -0.5f : 0.5f));
        }
        if (o->position_present) {
            out->flags |= U5_RAW_XY;
            out->raw_x = o->x_mm;
            out->raw_y = o->y_mm;
            if ((live->upper.flags & U5_AVAILABLE) && live->upper.age < 150U &&
                (out->flags & U5_DISTANCE) && (out->flags & U5_ANGLE) &&
                o->distance_valid && o->angle_valid)
                out->flags |= U5_VALID;
        }
        if ((live->upper.flags & U5_ALIGNED) && i < field_map.count && p->present) {
            out->flags |= U5_FIELD_XY;
            out->field_x = (int32_t)(p->x * 10.0f);
            out->field_y = (int32_t)(p->y * 10.0f);
            out->track_id = p->track_id;
            out->radius = (uint16_t)(p->radius * 10.0f);
        }
    }
    for (i = 0; i < live->lower.count; ++i) {
        const VisionObject *o = &vision.lower.objects[i];
        U5_LowerObject *out = &live->lower_objects[i];
        out->kind = (uint8_t)o->kind;
        out->flags = o->in_roi ? U5_IN_ROI : 0;
        if (o->inside_valid && live->lower.age < 150U) {
            out->flags |= U5_HAS_ZONE | (o->object_inside ? U5_INSIDE : 0);
            out->zone = (uint8_t)o->zone_kind;
        }
    }
}

static uint16_t encode_live(uint32_t now, bool cameras)
{
    static U5_Live live;
    uint8_t inside = task_info.inside_frames > 15U ? 15U : task_info.inside_frames;
    uint8_t outside = task_info.outside_frames > 15U ? 15U : task_info.outside_frames;
    live.step = step;
    live.reason = task_info.reason;
    live.held_kind = task_info.held_kind;
    live.roi_counts = (uint8_t)(inside | (outside << 4));
    live.servo = servo_angle;
    live.flags = (uint8_t)((telemetry_fault() << 5) |
                 (vision.connected ? U5_LINK : 0) |
                 (imu.ready ? U5_IMU_READY : 0) |
                 (field_map.heading_valid ? U5_HEADING : 0));
    live.robot = wire_pose(&field_map.robot);
    live.imu_age = imu.ready ? now - imu.tick : 0;
    live.imu_yaw = imu.ready ? (uint16_t)(imu.raw_yaw * 100.0f + 0.5f) : 0;
    if (cameras) fill_live_cameras(&live, now);
    payload.length = 0;
    U5_LivePayload(&payload, &live);
    return U5_Frame(packet, U5_LIVE, tx_seq, now, &payload);
}

static void put_track(const FieldTrack *t)
{
    U5_Track track;
    track.epoch = field_map.epoch;
    track.map_rev = field_map.revision;
    track.count = field_map.track_count;
    track.unmapped = field_map.unmapped_count;
    track.id = t->id;
    track.revision = t->revision;
    track.kind = t->kind;
    track.flags = (uint8_t)((t->visible ? 1 : 0) | (t->review ? 2 : 0));
    track.x_mm = (int32_t)(t->x * 10.0f);
    track.y_mm = (int32_t)(t->y * 10.0f);
    track.radius = (uint16_t)((FIELD_MATCH_BASE_CM + FIELD_MATCH_DISTANCE_SCALE * t->distance) * 10.0f);
    track.seen = t->seen_ms;
    track.position = t->position_ms;
    track.observations = t->observations;
    U5_TrackPayload(&payload, &track);
}

static void put_history(const FieldTrack *track)
{
    unsigned i;
    put_track(track);
    U5_U8(&payload, track->history_count);
    for (i = 0; i < track->history_count; ++i) {
        const FieldSample *h = &track->history[(track->history_next + FIELD_TRACK_HISTORY -
                                                track->history_count + i) % FIELD_TRACK_HISTORY];
        U5_U32(&payload, h->tick);
        U5_U32(&payload, (uint32_t)(int32_t)(h->x * 10.0f));
        U5_U32(&payload, (uint32_t)(int32_t)(h->y * 10.0f));
        U5_U8(&payload, h->accepted);
    }
}

/* Keep 39 B available for the next state message before starting optional
 * traffic. Only ACK and EVENT bypass the bucket. */
static bool send_encoded(uint16_t length, unsigned reserve, bool urgent)
{
    if (!length) {
        ++g_faraway_tx_drop_count;
        return false;
    }
    if (!urgent && tx_budget < (uint16_t)(length + reserve)) return false;
    if (!faraway_send(packet, length)) return false;
    ++tx_seq;
    hui_log.moving_credit -= (int32_t)length * 1000;
    if (hui_log.moving_credit < HUI_MOVING_CREDIT_MIN)
        hui_log.moving_credit = HUI_MOVING_CREDIT_MIN;
    if (!urgent) tx_budget = (uint16_t)(tx_budget - length);
    return true;
}

static bool send_payload(U5_Type type, uint32_t tick, unsigned reserve, bool urgent)
{
    return send_encoded(U5_Frame(packet, type, tx_seq, tick, &payload), reserve, urgent);
}

static void refill_budget(uint32_t now)
{
    uint32_t elapsed = now - budget_tick;
    uint32_t replenished = elapsed >= U5_MAX_WIRE ? U5_MAX_WIRE : tx_budget + elapsed * 2U;
    tx_budget = (uint16_t)(replenished > U5_MAX_WIRE ? U5_MAX_WIRE : replenished);
    if (elapsed >= 2000U) {
        hui_log.moving_credit = HUI_MOVING_CREDIT_MAX;
    } else {
        hui_log.moving_credit += (int32_t)elapsed * 1500;
        if (hui_log.moving_credit > HUI_MOVING_CREDIT_MAX)
            hui_log.moving_credit = HUI_MOVING_CREDIT_MAX;
    }
    budget_tick = now;
}

/* Background traffic may use the credits the observed camera packet and the
 * intervening heartbeat slots will not need by the 400 ms camera deadline. */
static unsigned background_reserve(uint32_t now)
{
    int32_t until_camera = (int32_t)(camera_deadline - now);
    int32_t heartbeat_window = (int32_t)(camera_deadline - next_live);
    unsigned occupied = heartbeat_window > 0 ? ((unsigned)heartbeat_window + 99U) / 100U : 0;
    unsigned needed = camera_wire_cost + occupied * LIVE_RESERVE;
    unsigned earned = until_camera > 0 ? (unsigned)until_camera * 2U : 0;
    return needed > earned + LIVE_RESERVE ? needed - earned : LIVE_RESERVE;
}

static bool send_hui_meta(uint32_t now)
{
    U5_U32(&payload, hui_log.boot);
    U5_U32(&payload, hui_log.session);
    U5_U32(&payload, hui_log.seq);
    U5_U32(&payload, hui_log.dropped);
    U5_U32(&payload, hui_log.start_tick);
    U5_U32(&payload, hui_log.end_tick);
    U5_U16(&payload, hui_log.records);
    U5_U8(&payload, hui_log.active ? 1U : hui_log.sealed ? 2U : 0U);
    U5_U8(&payload, HUI_SCHEMA_VERSION);
    if (!send_payload(U5_KEY2_META, now, background_reserve(now), false)) return false;
    hui_log.meta_pending = false;
    return true;
}

static uint32_t hui_find_record(uint32_t seq)
{
    unsigned lo = 0, hi = (hui_log.records + HUI_LOG_GROUP - 1U) / HUI_LOG_GROUP;
    uint32_t offset;

    if (!hi) return hui_log.used;
    /* Locate the nearest group, then walk at most one group of records. */
    while (hi - lo > 1U) {
        unsigned mid = (lo + hi) / 2U;
        if (hui_read_u32(hui_record_at(hui_groups[mid])) <= seq) lo = mid;
        else hi = mid;
    }
    offset = hui_groups[lo];
    while (offset < hui_log.used) {
        const uint8_t *record = hui_record_at(offset);
        if (hui_read_u32(record) >= seq) break;
        offset = hui_next_offset(offset, HUI_LOG_HEADER + record[9]);
    }
    return offset;
}

static bool send_hui_chunk(uint32_t now)
{
    unsigned reserve = background_reserve(now);
    /* COBS framing and CRC add at most 15 bytes to these <=365B payloads. */
    unsigned overhead = reserve + 15U;
    int32_t available = (int32_t)tx_budget - (int32_t)overhead;
    unsigned max_payload, count = 0;
    uint32_t offset, scan_next;

    if (hui_log.active) {
        int32_t moving = hui_log.moving_credit / 1000 - 15;
        if (moving < available) available = moving;
    }
    if (available < (int32_t)HUI_CHUNK_HEADER) return false;
    max_payload = (unsigned)available;
    if (max_payload > HUI_CHUNK_TARGET) max_payload = HUI_CHUNK_TARGET;
    offset = hui_find_record(hui_log.read_seq);
    payload.length = HUI_CHUNK_HEADER;
    while (offset < hui_log.used) {
        const uint8_t *record = hui_record_at(offset);
        unsigned length = HUI_LOG_HEADER + record[9];
        if (payload.length + length > max_payload) break;
        memcpy(payload.bytes + payload.length, record, length);
        payload.length += length;
        offset = hui_next_offset(offset, length);
        ++count;
    }
    scan_next = offset < hui_log.used ? hui_read_u32(hui_record_at(offset)) : hui_log.seq;
    /* Wait for credit rather than sending an empty chunk at a present record. */
    if (!count && offset < hui_log.used && scan_next == hui_log.read_seq) return false;

    hui_write_u32(payload.bytes, hui_log.boot);
    hui_write_u32(payload.bytes + 4U, hui_log.session);
    hui_write_u32(payload.bytes + 8U, hui_log.read_seq);
    hui_write_u32(payload.bytes + 12U, scan_next);
    payload.bytes[16] = (uint8_t)count;
    if (!send_payload(U5_KEY2_CHUNK, now, reserve, false)) return false;
    hui_log.read_pending = false;
    return true;
}

static bool send_live(uint32_t now)
{
    bool cameras = (int32_t)(now - next_camera) >= 0;
    uint16_t length = encode_live(now, cameras);
    if (cameras && length) camera_wire_cost = length;
    if (cameras && (!length || tx_budget < length)) {
        if (!send_encoded(encode_live(now, false), 0, false)) return false;
        next_camera = (int32_t)(now - camera_deadline) >= 0 ? next_live : next_live + 100U;
        ++merged_snapshots;
    } else {
        if (!send_encoded(length, 0, false)) return false;
        if (cameras) {
            next_camera = next_live + 100U;
            camera_deadline = next_live + 300U;
        }
    }
    live_pending = false;
    return true;
}

void faraway_task_observe(void)
{
    if (!opened || (step == observed_step && task_info.reason == observed_reason)) return;
    observed_step = step;
    observed_reason = task_info.reason;
    if (event_count >= EVENT_CAPACITY) {
        ++event_overflow;
    } else {
        unsigned slot = (event_head + event_count) % EVENT_CAPACITY;
        events[slot].tick = HAL_GetTick();
        events[slot].step = observed_step;
        events[slot].reason = observed_reason;
        ++event_count;
    }
}

static void command_error(U5_Error reason)
{
    unsigned slot;
    if (error_count >= ERROR_CAPACITY) {
        ++g_faraway_tx_drop_count;
        return;
    }
    slot = (error_head + error_count) % ERROR_CAPACITY;
    errors[slot].tick = HAL_GetTick();
    errors[slot].reason = reason;
    ++error_count;
}

static bool send_ack(void)
{
    const Ack *ack = &acks[ack_head];
    U5_U32(&payload, ack->id);
    U5_U8(&payload, ack->status);
    U5_U8(&payload, ack->step);
    if (!send_payload(U5_ACK, ack->tick, 0, true)) return false;
    ack_head = (ack_head + 1U) % ACK_CAPACITY;
    --ack_count;
    return true;
}

static bool send_event(void)
{
    const Event *event = &events[event_head];
    U5_U8(&payload, event->step);
    U5_U8(&payload, event->reason);
    if (!send_payload(U5_EVENT, event->tick, 0, true)) return false;
    event_head = (event_head + 1U) % EVENT_CAPACITY;
    --event_count;
    return true;
}

static bool send_error(void)
{
    const ErrorReply *reply = &errors[error_head];
    U5_U8(&payload, (uint8_t)reply->reason);
    if (!send_payload(U5_ERROR, reply->tick, LIVE_RESERVE, false)) return false;
    error_head = (error_head + 1U) % ERROR_CAPACITY;
    --error_count;
    return true;
}

static void send_info(uint32_t now)
{
    const U5_Pose origin = {(int32_t)(FIELD_START_X_CM * 10.0f),
                            (int32_t)(FIELD_START_Y_CM * 10.0f),
                            (int32_t)(FIELD_START_YAW_DEG * 100.0f)};
    const U5_Pose extrinsic = {(int32_t)(FIELD_VISION_X_CM * 10.0f),
                               (int32_t)(FIELD_VISION_Y_CM * 10.0f),
                               (int32_t)(FIELD_VISION_YAW_DEG * 100.0f)};
    U5_U32(&payload, field_map.epoch);
    U5_PutPose(&payload, &origin);
    U5_PutPose(&payload, &extrinsic);
    U5_U16(&payload, 0x002FU); /* History, map sync, steering, control, KEY2 log. */
    if (send_payload(U5_INFO, now, LIVE_RESERVE, false)) info_pending = false;
}

static void send_control(uint32_t now)
{
    U5_U8(&payload, command_seen);
    if (command_seen) {
        U5_U32(&payload, command_id);
        U5_U8(&payload, command_status);
    }
    U5_U8(&payload, goto_target.valid);
    if (goto_target.valid) {
        U5_U16(&payload, (uint16_t)(goto_target.x * 10.0f));
        U5_U16(&payload, (uint16_t)(goto_target.y * 10.0f));
        U5_F32(&payload, goto_target.speed);
    }
    if (send_payload(U5_CONTROL, now, LIVE_RESERVE, false)) {
        control_pending = false;
        sent_target = goto_target;
    }
}

static void send_stats(uint32_t now)
{
    U5_U32(&payload, field_map.epoch);
    U5_U32(&payload, field_map.revision);
    U5_U32(&payload, field_map.unmapped_count);
    U5_U8(&payload, field_map.track_count);
    U5_U32(&payload, vision.rx_packets);
    U5_U32(&payload, vision.bad_packets);
    U5_U32(&payload, vision.uart_errors);
    U5_U32(&payload, g_faraway_tx_drop_count);
    U5_U32(&payload, tx_completed_bytes);
    U5_U32(&payload, merged_snapshots);
    U5_U32(&payload, event_overflow);
    U5_U32(&payload, g_faraway_uart_error_count);
    U5_U16(&payload, tx_high_water);
    if (send_payload(U5_STATS, now, background_reserve(now), false))
        next_stats += ((now - next_stats) / 1000U + 1U) * 1000U;
}

static bool send_steer(uint32_t now)
{
    U5_U8(&payload, steer_state.source);
    U5_U32(&payload, steer_state.source ? now - steer_state.tick : 0);
    U5_F32(&payload, pid_steer.target);
    U5_F32(&payload, pid_steer.current);
    U5_F32(&payload, pid_steer.error_now);
    U5_F32(&payload, pid_steer.output);
    U5_F32(&payload, steer_state.p);
    U5_F32(&payload, steer_state.i);
    U5_F32(&payload, steer_state.d);
    U5_F32(&payload, pid_steer.kp);
    U5_F32(&payload, pid_steer.ki);
    U5_F32(&payload, pid_steer.kd);
    if (!send_payload(U5_STEER, now, background_reserve(now), false)) return false;
    steer_last = now;
    return true;
}

static const FieldTrack *find_track(uint32_t epoch, uint32_t id)
{
    unsigned i;
    if (!id || epoch != field_map.epoch) return NULL;
    for (i = 0; i < field_map.track_count; ++i)
        if (field_tracks[i].id == id) return &field_tracks[i];
    return NULL;
}

static bool send_history(uint32_t now)
{
    const FieldTrack *track = find_track(history_epoch, history_id);
    if (!track) {
        history_pending = false;
        command_error(U5_ERR_HIST_UNAVAILABLE);
        return false;
    }
    put_history(track);
    if (!send_payload(U5_HISTORY, now, background_reserve(now), false)) return false;
    history_pending = false;
    return true;
}

static int find_dirty_track(bool new_only)
{
    unsigned i;
    for (i = 0; i < field_map.track_count; ++i) {
        unsigned slot = (map_cursor + i) % field_map.track_count;
        /* Unsent baseline items already carry the complete current track. */
        if (sync_requested || (sync_active && slot >= sync_index && slot < sync_total)) continue;
        if (!map_delivered[slot] ||
            (!new_only && map_sent[slot] != field_tracks[slot].revision)) return (int)slot;
    }
    return -1;
}

static void note_track_sent(unsigned slot)
{
    map_sent[slot] = field_tracks[slot].revision;
    map_delivered[slot] = 1;
}

static bool send_track(uint32_t now)
{
    int slot = find_dirty_track(true);
    if (slot < 0) slot = find_dirty_track(false);
    if (slot < 0) return false;
    put_track(&field_tracks[slot]);
    if (!send_payload(U5_TRACK, now, background_reserve(now), false)) return false;
    note_track_sent((unsigned)slot);
    map_cursor = ((unsigned)slot + 1U) % field_map.track_count;
    next_track = now + 334U;
    return true;
}

static bool send_map_sync(uint32_t now)
{
    uint8_t phase, index;
    if (sync_requested) {
        sync_requested = false;
        sync_active = true;
        sync_started = false;
        sync_index = 0;
        sync_total = field_map.track_count;
        ++sync_id;
    }
    if (!sync_active) return false;
    phase = !sync_started ? 0 : (sync_index < sync_total ? 1 : 2);
    index = sync_index;
    U5_U16(&payload, sync_id);
    U5_U8(&payload, phase);
    U5_U8(&payload, index);
    U5_U8(&payload, sync_total);
    U5_U32(&payload, map_epoch);
    if (phase == 1U) put_track(&field_tracks[index]);
    if (!send_payload(U5_MAP_SYNC, now, background_reserve(now), false)) return false;
    if (phase == 0U) sync_started = true;
    else if (phase == 1U) {
        note_track_sent(index);
        ++sync_index;
    } else {
        sync_active = false;
    }
    return true;
}

void faraway_report(void)
{
    uint32_t now = HAL_GetTick();
    if (!opened) return;
    refill_budget(now);
    if ((int32_t)(now - next_live) >= 0) {
        const uint32_t slots = (now - next_live) / 100U + 1U;
        merged_snapshots += live_pending ? slots : slots - 1U;
        next_live += slots * 100U;
        live_pending = true;
    }
    if (steer_stream && now - steer_stream_tick > 6000U) steer_stream = 0;
    if (map_epoch != field_map.epoch) {
        map_epoch = field_map.epoch;
        memset(map_delivered, 0, sizeof(map_delivered));
        map_cursor = 0;
        info_pending = sync_requested = true;
    }
    if (tx_head != tx_tail || (int32_t)(now - tx_next_ms) < 0) return;

    payload.length = 0;
    if (ack_count) { (void)send_ack(); return; }
    if (event_count) { (void)send_event(); return; }
    if (live_pending) { (void)send_live(now); return; }
    if (error_count && send_error()) return;
    payload.length = 0;
    if (info_pending) { send_info(now); return; }
    if (control_pending || goto_target.valid != sent_target.valid ||
        (goto_target.valid && (goto_target.x != sent_target.x ||
         goto_target.y != sent_target.y || goto_target.speed != sent_target.speed))) {
        send_control(now);
        return;
    }
    if ((int32_t)(now - next_stats) >= 0) { send_stats(now); return; }
    if (history_pending) { (void)send_history(now); return; }
    payload.length = 0;
    if (hui_log.meta_pending && send_hui_meta(now)) return;
    payload.length = 0;
    if (hui_log.read_pending && send_hui_chunk(now)) return;
    payload.length = 0;
    if ((int32_t)(now - next_track) >= 0 && send_track(now)) return;
    payload.length = 0;
    if (steer_stream && now - steer_last >= 200U && send_steer(now)) return;
    payload.length = 0;
    (void)send_map_sync(now);
}

/* Checksummed, sequenced control lines. No implicit retries or legacy motion commands. */
static uint8_t command_uint(const char *s, uint32_t *value)
{
    uint32_t n = 0, digit;
    if (!*s) return 0;
    while (*s) {
        if (*s < '0' || *s > '9') return 0;
        digit = (uint32_t)(*s++ - '0');
        if (n > (UINT32_MAX-digit)/10U) return 0;
        n = n*10U+digit;
    }
    *value = n; return 1;
}
static void command_ack(uint32_t id, unsigned status)
{
    unsigned slot;
    if (ack_count >= ACK_CAPACITY) {
        ++g_faraway_tx_drop_count;
        return;
    }
    slot = (ack_head + ack_count) % ACK_CAPACITY;
    acks[slot].tick = HAL_GetTick();
    acks[slot].id = id;
    acks[slot].status = (uint8_t)status;
    acks[slot].step = step;
    ++ack_count;
}

static char *command_checksum(char *command, uint16_t *value)
{
    char *star = strrchr(command, '*');
    unsigned i, crc = 0, digit;
    if (!star || strlen(star + 1) != 4U) return NULL;
    for (i = 1; i <= 4U; ++i) {
        if (star[i] >= '0' && star[i] <= '9') digit = (unsigned)(star[i] - '0');
        else if (star[i] >= 'A' && star[i] <= 'F') digit = (unsigned)(star[i] - 'A' + 10);
        else if (star[i] >= 'a' && star[i] <= 'f') digit = (unsigned)(star[i] - 'a' + 10);
        else return NULL;
        crc = crc * 16U + digit;
    }
    *value = VSP_Crc16((const uint8_t *)command, (uint16_t)(star - command));
    return *value == crc ? star : NULL;
}

static void history_command(char *command)
{
    uint16_t checksum;
    char *star = command_checksum(command, &checksum);
    char *comma = strchr(command + 5, ',');
    uint32_t epoch, id;
    if (!star) {
        command_error(U5_ERR_CONTROL_CRC);
        return;
    }
    if (!comma || comma > star) {
        command_error(U5_ERR_CONTROL_FORMAT);
        return;
    }
    *star = *comma = 0;
    if (!command_uint(command + 5, &epoch) || !command_uint(comma + 1, &id)) {
        command_error(U5_ERR_CONTROL_FORMAT);
        return;
    }
    if (!find_track(epoch, id) ||
        (history_pending && (epoch != history_epoch || id != history_id))) {
        command_error(U5_ERR_HIST_UNAVAILABLE);
        return;
    }
    history_epoch = epoch;
    history_id = id;
    history_pending = true;
}
static void hui_read_command(char *command)
{
    uint32_t boot, session, first;
    char *separator = strchr(command, ',');
    char *next;

    if (!separator) return;
    *separator = 0;
    next = strchr(separator + 1, ',');
    if (!next) return;
    *next = 0;
    if (!command_uint(command, &boot) || !command_uint(separator + 1, &session) ||
        !command_uint(next + 1, &first)) return;
    if (!hui_log.boot || boot != hui_log.boot || !session || session != hui_log.session ||
        first > hui_log.seq) {
        hui_log.meta_pending = true;
        return;
    }
    hui_log.read_seq = first;
    hui_log.read_pending = true;
}

static void faraway_command(char *command)
{
    char *star, *parts[6], *p;
    uint32_t id, args[3];
    unsigned count=1, i, status=3;
    uint16_t actual;
    uint8_t stop=0;
    command=faraway_trim(command);
    if (!strcmp(command,"MAP")) { sync_requested=true; return; }
    if (!strcmp(command,"GET")) {
        info_pending = control_pending = hui_log.meta_pending = true;
        return;
    }
    if (!strcmp(command, "KRUN?")) { hui_log.meta_pending = true; return; }
    if (!strncmp(command, "KRUN?,", 6U)) {
        uint32_t nonce;
        if (command_uint(command + 6, &nonce) && nonce && !hui_log.boot) hui_log.boot = nonce;
        hui_log.meta_pending = true;
        return;
    }
    if (!strncmp(command, "KREAD,", 6U)) { hui_read_command(command + 6U); return; }
    if (!strncmp(command,"HIST,",5)) { history_command(command); return; }
    if (strncmp(command,"C1,",3)) { command_error(U5_ERR_TASK_CONTROL); return; }
    star=command_checksum(command,&actual);
    if (!star) { command_error(U5_ERR_CONTROL_CRC); return; }
    *star=0;parts[0]=command;
    for(p=command;*p;p++) if(*p==',') {
        *p=0;if(count>=6){command_error(U5_ERR_CONTROL_FORMAT);return;}
        parts[count++]=p+1;
    }
    if(count<4 || !command_uint(parts[1],&id)) {command_error(U5_ERR_CONTROL_FORMAT);return;}
    for(i=3;i<count;i++) if(!command_uint(parts[i],&args[i-3])){command_ack(id,3);return;}
    stop=count==4 && !strcmp(parts[2],"K") && args[0]==4;
    /* KEY4 is never blocked by sequence ordering. Older movement is rejected. */
    if(command_seen && (int32_t)(id-command_id)<=0) {
        if(stop) {Key_Press(KEY_4);control_pending=true;command_ack(id,0);return;}
        command_ack(id,id==command_id && actual==command_crc ? command_status : 4);return;
    }
    if(count==4 && !strcmp(parts[2],"K") && args[0]>=1 && args[0]<=4)
        status=Key_Press((uint8_t)(1U<<(args[0]-1U)));
    else if(count==6 && !strcmp(parts[2],"G") && args[0]<=3000 && args[1]<=3000 && args[2]>0) {
        if(Key_Read()&KEY_4)status=2;
        else status=task_goto_start(args[0]*0.1f,args[1]*0.1f,args[2]*0.1f)?0:3;
    }
    else if(count==6 && !strcmp(parts[2],"P"))
        status=steering_set_pid(args[0]*0.0001f,args[1]*0.0001f,args[2]*0.0001f)?0:3;
    else if(count==4 && !strcmp(parts[2],"T") && args[0]<=1) {
        if(args[0] && !steer_stream)steer_last=HAL_GetTick()-200U;
        steer_stream=(uint8_t)args[0];steer_stream_tick=HAL_GetTick();status=0;
    }
    command_seen=1;command_id=id;command_crc=actual;command_status=(uint8_t)status;
    control_pending=true;
    command_ack(id,status);
}

void faraway_process(void)
{
    uint16_t i;
    uint8_t byte;
    uint32_t irq;

    if (!opened) {
        return;
    }
    for (i = 0U; i < FARAWAY_PROCESS_LIMIT; i++) {
        irq = __get_PRIMASK();
        __disable_irq();
        if (rx_fault) {
            rx_tail = rx_head;
            rx_fault = false;
            line_length = 0U;
            discard_line = true;
            __set_PRIMASK(irq);
            command_error(U5_ERR_RX_LOST);
            break;
        }
        if (rx_tail == rx_head) {
            faraway_tx_start();
            __set_PRIMASK(irq);
            break;
        }
        byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & FARAWAY_RX_MASK);
        __set_PRIMASK(irq);

        if (byte == '\r' || byte == '\n') {
            if (!discard_line && line_length != 0U) {
                line[line_length] = '\0';
                faraway_command(line);
                faraway_task_observe();
            }
            line_length = 0U;
            discard_line = false;
        } else if (!discard_line) {
            if (line_length >= FARAWAY_LINE_SIZE - 1U ||
                (byte < 0x20U && byte != '\t') || byte > 0x7EU) {
                line_length = 0U;
                discard_line = true;
                command_error(U5_ERR_LINE);
            } else {
                line[line_length++] = (char)byte;
            }
        }
    }
    /* Give command replies first selection of the newly available frame slot. */
    if (ack_count || error_count) faraway_report();
}
