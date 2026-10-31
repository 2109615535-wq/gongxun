#include "rescue_vision.h"
#include <math.h>
#include <string.h>

#define RV_PI 3.14159265358979323846f

static uint16_t read16(const uint8_t *p) {
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}
static uint32_t read32(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static float limit(float value, float bound) {
    if (value > bound) return bound;
    if (value < -bound) return -bound;
    return value;
}
uint16_t RV_Crc16(const uint8_t *data, uint16_t length) {
    uint16_t crc = 0xffffu, i;
    uint8_t bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc & 0x8000u) ? ((uint32_t)crc << 1) ^ 0x1021u : (uint32_t)crc << 1);
    }
    return crc;
}
void RV_Init(RV_Receiver *r, RV_Config config) {
    memset(r, 0, sizeof(*r));
    r->config = config;
    r->command.reason = RV_NO_FRAME;
}
void RV_OnByte(RV_Receiver *r, uint8_t byte, uint32_t received_ms) {
    uint16_t next = (uint16_t)((r->head + 1u) % RV_RING_SIZE);
    if (next == r->tail) {
        r->overflow = 1;
        return;
    }
    r->ring[r->head].byte = byte;
    r->ring[r->head].received_ms = received_ms;
    /* Volatile publication occurs after the complete slot write (single-core MCU). */
    r->head = next;
}
void RV_ResetTrial(RV_Receiver *r) {
    memset(&r->command, 0, sizeof(r->command));
    memset(&r->observation, 0, sizeof(r->observation));
    r->command.reason = RV_NO_FRAME;
    r->arrival_count = 0;
    r->consumed_generation = r->generation;
    /* Keep anti-replay timestamps; a new trial still requires a newer camera frame. */
}
static void discard(RV_Receiver *r, uint16_t n) {
    r->used = (uint16_t)(r->used - n);
    memmove(r->frame, r->frame + n, r->used);
}
static uint8_t publish(RV_Receiver *r, uint32_t stamp) {
    const uint8_t *p = r->frame + 6;
    RV_Observation obs;
    uint16_t payload = read16(r->frame+4), i;
    uint8_t count = p[5], zones = p[6];
    if (count > RV_MAX_TARGETS || zones > 4 || p[7] > 7 ||
        payload != (uint16_t)(8u + count*12u + zones*8u)) return 0;
    memset(&obs, 0, sizeof(obs));
    obs.source_tick_ms = read32(p);
    obs.flags = p[4]; obs.count = count; obs.status = p[7];
    obs.sequence = r->frame[3]; obs.received_ms = stamp;
    for (i = 0; i < count; ++i) {
        const uint8_t *t = p + 8u + i*12u;
        RV_Target *out = &obs.targets[i];
        out->type = t[0]; out->confidence = t[1];
        out->angle_cdeg = (int16_t)read16(t+2);
        out->distance_mm = read16(t+4);
        out->width_px = read16(t+6); out->height_px = read16(t+8);
        out->posture = t[10]; out->flags = t[11];
        if ((out->type != 1 && out->type != 2 && out->type != 3 &&
             out->type != 4 && out->type != 255) || out->confidence > 100 ||
            out->angle_cdeg < -18000 || out->angle_cdeg > 18000) return 0;
    }
    /* Fault heartbeats invalidate immediately, but must not advance source time:
       they can be emitted while a frame with an earlier capture tick is in flight. */
    if ((obs.flags & 1u) && obs.status == 0) {
        uint32_t delta = obs.source_tick_ms-r->last_source_tick;
        if (r->have_source && (delta == 0 ||
            ((uint32_t)(stamp-r->source_rx_ms) < RV_TIMEOUT_MS && delta >= 0x80000000u))) {
            r->duplicates++;
            return 1;
        }
        if (r->have_source && (uint32_t)(stamp-r->source_rx_ms) >= RV_TIMEOUT_MS)
            r->arrival_count = 0;
        r->have_source = 1;
        r->last_source_tick = obs.source_tick_ms;
        r->source_rx_ms = stamp;
    }
    obs.generation = ++r->generation;
    r->observation = obs; /* only written/read in RV_Step, never from UART ISR */
    return 1;
}
static void parse_byte(RV_Receiver *r, uint8_t byte, uint32_t stamp) {
    uint16_t length, total;
    if (r->used && (uint32_t)(stamp-r->partial_ms) > 30u) r->used = 0;
    r->partial_ms = stamp;
    if (r->used == RV_FRAME_MAX) discard(r, 1);
    r->frame[r->used++] = byte;
    while (r->used >= 2) {
        if (r->frame[0] != 0xaau || r->frame[1] != 0x55u) {
            discard(r, 1);
            continue;
        }
        if (r->used < 6) break;
        length = read16(r->frame+4);
        if (r->frame[2] != 1 || length < 8 || length > 184) {
            r->format_errors++;
            discard(r, 1);
            continue;
        }
        total = (uint16_t)(length+8);
        if (r->used < total) break;
        if (RV_Crc16(r->frame+2, (uint16_t)(total-4)) != read16(r->frame+total-2)) {
            r->crc_errors++;
            discard(r, 1);
        } else {
            if (!publish(r, stamp)) r->format_errors++;
            discard(r, total);
        }
    }
}
static RV_Command halt(RV_Receiver *r, RV_Reason reason) {
    r->command.forward_mm_s = 0;
    r->command.clockwise_rad_s = 0;
    r->command.reason = reason;
    if (r->command.state != RV_STOPPED) r->command.state = RV_WAIT;
    r->arrival_count = 0;
    return r->command;
}
RV_Command RV_Step(RV_Receiver *r, uint32_t now_ms) {
    const RV_Target *target = 0;
    const RV_Observation *obs;
    uint16_t i;
    uint8_t normal_count = 0, fresh;
    float angle, x, y, gap, lateral;
    if (r->overflow) {
        r->tail = r->head; r->overflow = 0; r->used = 0;
        memset(&r->observation, 0, sizeof(r->observation));
        return halt(r, RV_OVERFLOW);
    }
    while (r->tail != r->head) {
        RV_RxByte item = r->ring[r->tail];
        r->tail = (uint16_t)((r->tail+1u) % RV_RING_SIZE);
        parse_byte(r, item.byte, item.received_ms);
    }
    if (r->command.state == RV_STOPPED) return r->command;
    obs = &r->observation;
    if (!obs->generation) return halt(r, RV_NO_FRAME);
    if ((uint32_t)(now_ms-obs->received_ms) >= RV_TIMEOUT_MS) return halt(r, RV_TIMEOUT);
    if (!isfinite(r->config.bumper_forward_mm) || r->config.bumper_forward_mm < 0 ||
        !isfinite(r->config.collection_center_left_mm)) return halt(r, RV_INVALID);
    if (obs->flags & 16u) return halt(r, RV_HAZARD);
    if ((obs->flags & 3u) != 3u || obs->status != 0) return halt(r, RV_INVALID);
    for (i = 0; i < obs->count; ++i) {
        const RV_Target *t = &obs->targets[i];
        if (t->type == 255) return halt(r, RV_HAZARD);
        if (t->type == 4 || (t->flags & 1u)) {
            if (!(t->flags & 32u) || !t->distance_mm) return halt(r, RV_HAZARD);
            angle = t->angle_cdeg * (RV_PI/18000.f);
            x = t->distance_mm*cosf(angle)-r->config.bumper_forward_mm;
            y = -t->distance_mm*sinf(angle)-r->config.collection_center_left_mm;
            if (x > 0 && x <= 1400 && fabsf(y) <= 180) return halt(r, RV_HAZARD);
        }
        if (t->type == 1) {
            ++normal_count;
            target = t;
        }
    }
    if (normal_count != 1 || !target || target->confidence < 35 ||
        (target->flags & 48u) != 48u || (target->flags & 5u) || !target->distance_mm)
        return halt(r, RV_INVALID);
    angle = target->angle_cdeg * (RV_PI/18000.f);
    x = target->distance_mm*cosf(angle);
    y = -target->distance_mm*sinf(angle);
    gap = x-r->config.bumper_forward_mm;
    lateral = y-r->config.collection_center_left_mm;
    r->command.gap_mm = gap; r->command.lateral_mm = lateral;
    if (gap < 170) return halt(r, RV_TOO_CLOSE);
    fresh = obs->generation != r->consumed_generation;
    r->consumed_generation = obs->generation;
    r->command.forward_mm_s = 0;
    r->command.clockwise_rad_s = 0;
    r->command.reason = RV_TRACKING;
    if (gap <= 230 && fabsf(lateral) <= 20) {
        if (fresh && r->arrival_count < 3) ++r->arrival_count;
        if (r->arrival_count >= 3) {
            r->command.state = RV_STOPPED;
            r->command.reason = RV_ARRIVED;
        } else r->command.state = RV_ALIGN;
        return r->command;
    }
    r->arrival_count = 0;
    angle = atan2f(-lateral, x);
    r->command.clockwise_rad_s = limit(1.5f*angle, .4f);
    if (fabsf(angle) > 5.f*(RV_PI/180.f) || gap <= 230 || !r->config.allow_forward) {
        r->command.state = RV_ALIGN;
    } else {
        r->command.state = RV_APPROACH;
        r->command.forward_mm_s = limit(.6f*(gap-200), 100.f);
    }
    return r->command;
}
