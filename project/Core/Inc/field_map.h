#ifndef FIELD_MAP_H
#define FIELD_MAP_H

#include "vision.h"

/* cm, degrees clockwise from the top of the field. Red circle is approximate.
 * Place the wheel-centre reference here BEFORE resetting/powering the MCU. */
#define FIELD_START_X_CM 285.0f
#define FIELD_START_Y_CM 80.0f
#define FIELD_START_YAW_DEG 0.0f

/* Vision calibration origin relative to wheel centre: right, forward, clockwise.
 * Zero assumes calibration already uses that origin. Do not apply offsets twice. */
#define FIELD_VISION_X_CM 0.0f
#define FIELD_VISION_Y_CM 0.0f
#define FIELD_VISION_YAW_DEG 0.0f

typedef struct { uint32_t tick; float x, y, yaw; } FieldPose; /* yaw radians */
#define FIELD_MATCH_BASE_CM 4.0f
#define FIELD_MATCH_DISTANCE_SCALE 0.05f
#define FIELD_TRACK_CAPACITY 48U
#define FIELD_TRACK_HISTORY 8U
#define FIELD_JUMP_CONFIRM_FRAMES 3U
typedef struct {
    uint32_t tick;
    float x, y;
    uint8_t accepted; /* Raw observation; zero means a pending position jump. */
} FieldSample;
typedef struct {
    uint32_t id, seen_ms, position_ms, observations, revision;
    float x, y, distance;
    float pending_x, pending_y;
    uint8_t kind, visible, review, pending_count, history_count, history_next;
    FieldSample history[FIELD_TRACK_HISTORY];
} FieldTrack;
typedef struct {
    float x, y, radius;
    uint32_t track_id; /* Stable map-only ID; never modifies vision/task IDs. */
    uint8_t present;
} FieldPoint;
typedef struct {
    FieldPose robot, observed;
    uint32_t frame;
    uint8_t aligned, count, heading_valid;
    FieldPoint points[12];
    uint32_t epoch, revision, unmapped_count;
    uint8_t track_count;
} FieldMap;
extern FieldMap field_map;
extern FieldTrack field_tracks[FIELD_TRACK_CAPACITY]; /* Whole-round RAM map. */
void field_map_reset(uint32_t now);
void field_map_gap(void); /* Clear interpolation across an unavailable heading. */
void field_map_record(uint32_t sample_tick, float odom_x, float odom_y, float odom_yaw);
void field_map_update(const VisionObservation *observation, uint32_t now);
void field_map_process(void); /* Independent scheduler task; no motion/UART writes. */

#endif
