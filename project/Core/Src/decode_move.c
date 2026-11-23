#include "decode_move.h"
#include "vision.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <float.h>
#include <string.h>
#include "pid.h"
#include "servo.h"
#include "key.h"
#include "field_map.h"
#include "imu.h"
#include "hui_ku.h"

float x_body;
float y_body;
float angle_body;  /* rad, clockwise positive, relative to the last reset. */

float motordistance1;
float motordistance2;
float motordistance3;
float motordistance4;

static int64_t start_encoder_count1;
static int64_t start_encoder_count2;
static int64_t start_encoder_count3;
static int64_t start_encoder_count4;


static float last_dis1;
static float last_dis2;
static float last_dis3;
static float last_dis4;
static uint8_t return_push_active;

void position_encoder_init(void)
{
    start_encoder_count1 = encoder_get_count(&encoder1);
    start_encoder_count2 = encoder_get_count(&encoder2);
    start_encoder_count3 = encoder_get_count(&encoder3);
    start_encoder_count4 = encoder_get_count(&encoder4);

    motordistance1 = 0.0f;
    motordistance2 = 0.0f;
    motordistance3 = 0.0f;
    motordistance4 = 0.0f;

    last_dis1 = 0.0f;
    last_dis2 = 0.0f;
    last_dis3 = 0.0f;
    last_dis4 = 0.0f;

    x_body = 0.0f;
    y_body = 0.0f;
    angle_body = 0.0f;
    (void)imu_init();
    field_map_reset(HAL_GetTick());
    if (!imu.ready) field_map_gap();
}

void position_encoder(void)
{
    float dis1,dis2,dis3,dis4;
    float dx,dy,yaw;
    float mid_angle;

    motordistance1 = (float)(encoder_get_count(&encoder1) - start_encoder_count1) 
                    * wheel_diameter * 3.1415926f / 40810.0f;
    motordistance2 = (float)(encoder_get_count(&encoder2) - start_encoder_count2) 
                    * wheel_diameter * 3.1415926f / 40810.0f;
    motordistance3 = (float)(encoder_get_count(&encoder3) - start_encoder_count3) 
                    * wheel_diameter * 3.1415926f / 40810.0f;
    motordistance4 = (float)(encoder_get_count(&encoder4) - start_encoder_count4) 
                    * wheel_diameter * 3.1415926f / 40810.0f;

    //此次周期和上次周期每个轮子距离的差值，用来计算角度朝向。
    dis1 = motordistance1 - last_dis1;    last_dis1 = motordistance1;
    dis2 = motordistance2 - last_dis2;    last_dis2 = motordistance2;
    dis3 = motordistance3 - last_dis3;    last_dis3 = motordistance3;
    dis4 = motordistance4 - last_dis4;    last_dis4 = motordistance4;

    dx = (-dis1 + dis2 - dis3 + dis4) / 4.0f;   dx*= 0.952092f;
    dy = ( dis1 + dis2 + dis3 + dis4) / 4.0f;   dy*= 1.028666f;

    if (!imu_at(encoder1.last_tick, &yaw)) {
        /* No wheel-yaw fallback. Missing increments cannot be located reliably. */
        field_map_gap();
        return;
    }
    if (return_push_active) {
        /* Keep current IMU yaw and consume encoder counts, without integrating slip. */
        angle_body = yaw;
        return;
    }
    mid_angle = field_map.heading_valid ? (angle_body + yaw) / 2.0f : yaw;

    //y_body = ( motordistance1 + motordistance2 + motordistance3 + motordistance4) / 4.0f * 1.028666f;
    //x_body = (-motordistance1 + motordistance2 - motordistance3 + motordistance4) / 4.0f * 0.952092f;

    x_body += dx * cosf(mid_angle) + dy * sinf(mid_angle);
    y_body += -dx * sinf(mid_angle) + dy * cosf(mid_angle);

    angle_body = yaw;
    /* Counts belong to the encoder sample, not this scheduler invocation. */
    field_map_record(encoder1.last_tick, x_body, y_body, angle_body);
}




static void body_move_raw(float vy, float vx, float vtheta)
{
    float rotation = (wheel_X_length + wheel_Y_length) * 0.5f * vtheta * 3.1415926f / 180.0f;
    vx /= 0.952092f;
    vy /= 1.028666f;
    /* Physical wheel speeds; motor layer preserves the common 90% scale. */
    Motor_SetWheelSpeedCmS(vy-vx-rotation, vy+vx+rotation,
                          vy-vx+rotation, vy+vx-rotation);
}

BodyMotion body_motion;
static uint8_t body_applying;

void body_move_cancel(void)
{
    if (body_applying) return;
    body_motion.target_forward = body_motion.target_right = body_motion.target_turn = 0;
    body_motion.active = 0;
    /* Cancelling the body target must not erase the active wheel trajectory. */
}
void body_wheel_applied(float w1, float w2, float w3, float w4)
{
    /* Inverse kinematics of APPLIED commands, not measured odometry. */
    body_motion.forward = (w1+w2+w3+w4) * 0.25f * 1.028666f;
    body_motion.right = (-w1+w2-w3+w4) * 0.25f * 0.952092f;
    body_motion.turn = (-w1+w2+w3-w4) / (2.0f*(wheel_X_length+wheel_Y_length)) * 180.0f/3.1415926f;
    body_motion.updated_ms = HAL_GetTick();
}
void body_move(float vy, float vx, float vtheta)
{
    body_motion.target_forward = vy;
    body_motion.target_right = vx;
    body_motion.target_turn = vtheta;
    body_motion.active = 1;
}
void body_move_process(void)
{
    if (!body_motion.active) return;
    body_applying = 1;
    body_move_raw(body_motion.target_forward, body_motion.target_right, body_motion.target_turn);
    body_applying = 0;
}
void body_stop(void)
{
    /* Zero command, applied next motor cycle without a software ramp. */
    body_move(0, 0, 0);
}
float body_brake_distance(void)
{
    float speed = sqrtf(body_motion.forward*body_motion.forward + body_motion.right*body_motion.right);
    return speed * Motor_StopTime() * 0.5f;
}

#define TASK_FORWARD_CM_S 50.0f
#define TASK_STEER_KP 3.5f
#define TASK_STEER_KI 0.0f
#define TASK_STEER_KD 5.0f
#define TASK_ANGLE_DEAD_DEG 1.0f

/* Test setup: field cm, wheel centre as the position reference. */
/* Wheel-centre Y when the front bumper is flush with the wall. Test estimates. */
#define RETURN_RED_CONTACT_Y_CM 267.0f
#define RETURN_BLUE_CONTACT_Y_CM 33.0f
#define RETURN_CONTACT_START_CM  13.0f
#define RETURN_CONTACT_SPEED     (MOTOR_SPEED_MAX * 30 / 100) /* Normalized wheel speed target. */
#define RETURN_CONTACT_MS      1000U
#define RETURN_X_SETTLE_MS      200U
#define RETURN_X_SAMPLES         10U
#define RETURN_X_WINDOW          13U
#define RETURN_X_TIMEOUT_MS     3000U
#define RETURN_X_SPAN_CM          4.0f
#define RETURN_RETREAT_CM        50.0f
#define RETURN_REVERSE_CM_S      18.0f
#define RETURN_STRAFE_CM_S       20.0f
#define RETURN_APPROACH_CM_S     30.0f
#define RETURN_CENTER_KP          3.0f
#define RETURN_YAW_KP             2.5f
#define RETURN_CENTER_TOL_CM      2.0f
#define RETURN_ZONE_YAW_TOL_DEG   2.0f
#define RETURN_CENTER_YAW_TOL_DEG 5.0f
#define RETURN_RELEASE_MS       500U
#define RETURN_FIELD_CENTER_CM  150.0f
#define RETURN_CENTER_CM_S       30.0f
#define RETURN_SCAN_DEG_S        45.0f
#define RETURN_PI                 3.14159265358979323846f

enum {
    RETURN_BACK = 60, RETURN_FACE, RETURN_STRAFE, RETURN_APPROACH,
    RETURN_RELEASE, RETURN_RETREAT,
    RETURN_CONTACT_ABORT = 67, RETURN_CONTACT, RETURN_CALIBRATE_X,
    RETURN_CENTER_FACE = 71, RETURN_CENTER_DRIVE, RETURN_SCAN, RETURN_DONE
};

typedef struct {
    VisionKind kind;
    float centre_x, contact_y, yaw;
} ReturnZone;

static const ReturnZone return_zones[] = {
    { VISION_RED_MATERIAL_ZONE, 135.0f, RETURN_RED_CONTACT_Y_CM, 0.0f },
    { VISION_BLUE_MATERIAL_ZONE, 165.0f, RETURN_BLUE_CONTACT_Y_CM, RETURN_PI }
};
static const ReturnZone *const return_zone = &return_zones[RETURN_TEAM];
static FieldPose return_reverse_origin;
static uint8_t return_reverse_started;
static uint32_t return_release_tick, return_push_tick, return_sample_after, return_sample_frame;
static uint8_t return_sample_count, return_sample_next, return_sample_seen, return_x_skipped;
static float return_x_samples[RETURN_X_WINDOW];
static float return_scan_start_yaw;

/* Simple low-speed mission: nearest green -> forward + steer -> grip and hold.
 * Nonblocking tasks; missing observations stop motion and wait for recovery. */

uint8_t step = 0;
TaskInfo task_info;
GotoTarget goto_target;
#define GRIP_WINDOW 3U
#define GRIP_EMPTY_VOTES 2U
#define GRIP_OBSERVATION_MAX_AGE_MS 400U

typedef struct {
    uint32_t observed_ms, frame;
    uint8_t inside;
} GripSample;

static GripSample grip_samples[GRIP_WINDOW];
static unsigned grip_sample_count;
static uint32_t lower_frame, uart_at_start, grip_since, grip_reverse_tick;
static uint8_t lower_seen, lower_fault, grip_commanded, pending_kind;
static float target_angle, target_distance_cm;
static uint8_t near_align_state; /* 0 not checked, 1 turning, 2 done for this target. */
PID_t pid_steer = { .kp = TASK_STEER_KP, .ki = TASK_STEER_KI,
                    .kd = TASK_STEER_KD, .output_max = FLT_MAX,
                    .error_int_max = FLT_MAX };
SteerState steer_state;
static uint32_t steer_frame;
static uint8_t steer_seen;
static float goto_x, goto_y;
static uint8_t goto_active, goto_arrived;

typedef struct {
    uint32_t evidence_frame, evidence_tick;
    uint8_t state, reason, protected_zone, duplicate_id;
} TargetEntry;

/* Map slots are append-only and their IDs start at 1 in each epoch. */
static TargetEntry task_targets[FIELD_TRACK_CAPACITY];
static uint32_t task_map_epoch, task_upper_frame, delivery_lower_frame, delivery_released_at;
static uint8_t task_target_count, task_upper_seen, active_target, delivery_target;

/* Live gains are shared by both target sources and survive task resets. */
uint8_t steering_set_pid(float kp, float ki, float kd)
{
    if (!(kp >= 0 && kp <= FLT_MAX && ki >= 0 && ki <= FLT_MAX &&
          kd >= 0 && kd <= FLT_MAX)) return 0;
    pid_steer.kp = kp; pid_steer.ki = ki; pid_steer.kd = kd;
    PID_Init(&pid_steer);
    steer_seen = 0;
    steer_state.source = 0;
    steer_state.p = steer_state.i = steer_state.d = 0;
    return 1;
}
static void goto_reset(void)
{
    goto_active = goto_arrived = 0;
    near_align_state = 0;
}


static uint32_t observation_age(const VisionObservation *o)
{
    uint32_t wait = HAL_GetTick() - o->updated_ms;
    if (o->age_ms == UINT32_MAX || wait > UINT32_MAX - 1U - o->age_ms) return UINT32_MAX;
    return o->age_ms + wait;
}
static uint8_t observation_fresh(const VisionObservation *o)
{
    return o->fresh && observation_age(o) <= GRIP_OBSERVATION_MAX_AGE_MS &&
           (o->status == VISION_OBS_OK || o->status == VISION_OBS_EMPTY);
}
static TargetEntry *target_entry(uint32_t id)
{
    if (task_map_epoch != field_map.epoch || !id || id > task_target_count ||
        id > field_map.track_count || field_tracks[id-1U].id != id)
        return NULL;
    return &task_targets[id-1U];
}

static uint8_t aligned_upper(void)
{
    const VisionObservation *o = &vision.upper;
    return observation_fresh(o) && field_map.aligned && field_map.frame == o->frame_id &&
           field_map.observed.tick == o->updated_ms - o->age_ms;
}

uint8_t task_target_get(uint32_t epoch, uint32_t track_id, TaskTarget *out)
{
    const TargetEntry *entry = target_entry(track_id);
    const FieldTrack *track;
    if (!entry || !out || epoch != task_map_epoch) return 0;
    track = &field_tracks[track_id-1U];
    if (track->kind != VISION_GREEN) return 0;
    *out = (TaskTarget){epoch, track_id, entry->duplicate_id,
                        entry->evidence_frame, entry->evidence_tick, track->x, track->y,
                        (TaskTargetState)entry->state, (TaskTargetReason)entry->reason,
                        track->visible && aligned_upper(), entry->protected_zone,
                        entry->state == TASK_TARGET_AVAILABLE && !entry->protected_zone};
    return 1;
}

static void target_transition(TargetEntry *entry, TaskTargetState state, TaskTargetReason reason,
                              uint32_t frame, uint32_t tick)
{
    entry->state = (uint8_t)state;
    entry->reason = (uint8_t)reason;
    entry->evidence_frame = frame;
    entry->evidence_tick = tick;
}

/* Proximity alone can flag a new ID for review. Never suppress the older
 * track or a distinct track seen alongside the new one in this frame. */
static uint8_t possible_duplicate(unsigned index)
{
    const FieldTrack *track = &field_tracks[index];
    float best = 2.0f * (FIELD_MATCH_BASE_CM + FIELD_MATCH_DISTANCE_SCALE * track->distance);
    uint8_t related = 0;
    unsigned i;
    if (!track->visible || !aligned_upper()) return 0;
    for (i = 0; i < index; ++i) {
        const FieldTrack *other = &field_tracks[i];
        float separation;
        if (other->kind != VISION_GREEN || other->visible ||
            task_targets[i].reason == TARGET_DUPLICATE)
            continue;
        separation = hypotf(track->x-other->x, track->y-other->y);
        if (separation <= best) {
            best = separation;
            related = (uint8_t)other->id;
        }
    }
    return related;
}

static void task_observe_targets(void)
{
    const VisionObservation *o = &vision.upper;
    unsigned i;
    if (task_map_epoch != field_map.epoch) {
        task_map_epoch = field_map.epoch;
        task_target_count = task_upper_seen = active_target = delivery_target = 0;
    }

    while (task_target_count < field_map.track_count) {
        const unsigned index = task_target_count++;
        const FieldTrack *track = &field_tracks[index];
        TargetEntry *entry = &task_targets[index];
        memset(entry,0,sizeof(*entry));
        entry->evidence_tick = track->seen_ms;
        if (track->kind != VISION_GREEN) continue;
        entry->duplicate_id = possible_duplicate(index);
        if (entry->duplicate_id) {
            entry->state = TASK_TARGET_REVIEW;
            entry->reason = TARGET_DUPLICATE;
        }
    }

    if (!aligned_upper() || (task_upper_seen && task_upper_frame == o->frame_id)) return;
    task_upper_seen = 1;
    task_upper_frame = o->frame_id;
    for (i = 0; i < field_map.count && i < o->count; ++i) {
        const uint32_t id = field_map.points[i].track_id;
        TargetEntry *entry = target_entry(id);
        const VisionObject *object = &o->objects[i];
        const uint32_t captured = field_map.observed.tick;
        if (!entry || !field_map.points[i].present || object->kind != VISION_GREEN) continue;
        entry->protected_zone = object->in_safety_zone;
        if (entry->state == TASK_TARGET_REVIEW && !entry->protected_zone) {
            /* For duplicates, only a frame with two separate associations clears the review. */
            const uint8_t rechecked = entry->duplicate_id ?
                field_tracks[entry->duplicate_id-1U].visible :
                delivery_target != id && (int32_t)(captured - entry->evidence_tick) > 0;
            if (rechecked) {
                entry->duplicate_id = 0;
                target_transition(entry,TASK_TARGET_AVAILABLE,TARGET_RECHECKED,o->frame_id,captured);
            }
        }
        if (entry->state == TASK_TARGET_AVAILABLE || entry->state == TASK_TARGET_LOCKED ||
            entry->reason == TARGET_DUPLICATE) {
            entry->evidence_frame = o->frame_id;
            entry->evidence_tick = captured;
        }
    }
}

static void task_select_target(uint32_t id)
{
    TargetEntry *entry;
    if (id == active_target) return;
    entry = target_entry(active_target);
    if (entry && entry->state == TASK_TARGET_LOCKED) {
        entry->state = TASK_TARGET_AVAILABLE;
        entry->reason = TARGET_REPLACED;
    }
    active_target = (uint8_t)id;
    entry = target_entry(id);
    if (entry) {
        entry->state = TASK_TARGET_LOCKED;
        entry->reason = TARGET_SELECTED;
    }
}

static void task_cancel_target(void)
{
    TargetEntry *entry = target_entry(active_target);
    if (entry) {
        const uint8_t grip_started = step == 40 || step == RETURN_BACK ||
                                     entry->state == TASK_TARGET_CARRYING;
        entry->state = grip_started ? TASK_TARGET_REVIEW : TASK_TARGET_AVAILABLE;
        entry->reason = TARGET_INTERRUPTED;
        if (grip_started) {
            entry->evidence_frame = vision.lower.frame_id;
            entry->evidence_tick = HAL_GetTick();
        }
    }
    active_target = 0;
    entry = target_entry(delivery_target);
    if (entry) entry->reason = TARGET_RELEASE_UNVERIFIED;
    delivery_target = 0;
}

static void task_record_release(uint32_t now)
{
    const VisionObservation *o = &vision.lower;
    TargetEntry *entry = target_entry(active_target);
    unsigned i, inside = 0;
    if (!entry || entry->state != TASK_TARGET_CARRYING) return;

    /* Use the unique carried object at release, never another ball already in the zone. */
    if (vision.connected && observation_fresh(o) && observation_age(o) < 150U) {
        for (i = 0; i < o->count; ++i) {
            const VisionObject *v = &o->objects[i];
            if (!v->in_roi) continue;
            if (v->kind != VISION_GREEN || !v->inside_valid || !v->object_inside ||
                v->zone_kind != return_zone->kind) {
                inside = 0;
                break;
            }
            ++inside;
        }
    }
    target_transition(entry,TASK_TARGET_REVIEW,
                      inside == 1 ? TARGET_RELEASE_PENDING : TARGET_RELEASE_UNVERIFIED,
                      o->frame_id,inside == 1 ? o->updated_ms - o->age_ms : now);
    delivery_target = inside == 1 ? active_target : 0;
    delivery_lower_frame = o->frame_id;
    delivery_released_at = now;
    active_target = 0;
}

static void task_observe_delivery(void)
{
    const VisionObservation *o = &vision.lower;
    TargetEntry *entry = target_entry(delivery_target);
    uint32_t captured;
    unsigned i;
    if (!entry) return;
    if (step != RETURN_RELEASE && step != RETURN_RETREAT) {
        entry->reason = TARGET_RELEASE_UNVERIFIED;
        delivery_target = 0;
        return;
    }
    if (!vision.connected || !observation_fresh(o) || o->frame_id == delivery_lower_frame) return;
    captured = o->updated_ms - o->age_ms;
    if ((int32_t)(captured - delivery_released_at) <= 0) return;
    delivery_lower_frame = o->frame_id;
    for (i = 0; i < o->count; ++i)
        if (o->objects[i].in_roi) return;
    target_transition(entry,TASK_TARGET_DELIVERED,TARGET_DELIVERY_VERIFIED,o->frame_id,captured);
    delivery_target = 0;
}

/* Restore yesterday's pursuit stop: remove output and clear steering history. */
static void drive_stop(void)
{
    Motor_SetDriveEnabled(0);
    PID_Init(&pid_steer);
    steer_seen = 0;
    steer_state.source = 0;
    steer_state.p = steer_state.i = steer_state.d = 0;
}
static void drive(float forward, float turn)
{
    Motor_SetDriveEnabled(1);
    body_move(forward, 0, turn);
}
static void clear_grip_samples(void)
{
    grip_sample_count = 0;
    task_info.inside_frames = task_info.outside_frames = 0;
}

static uint8_t checking_grip(void)
{
    return step == RETURN_BACK ||
           (task_info.held_kind != VISION_NONE &&
            ((step >= RETURN_FACE && step <= RETURN_RELEASE) || step == RETURN_CONTACT));
}

uint8_t task_goto_start(float x_cm, float y_cm, float speed_cm_s)
{
    if (!(x_cm >= 0 && x_cm <= 300 && y_cm >= 0 && y_cm <= 300 &&
          speed_cm_s > 0 && speed_cm_s <= FLT_MAX)) return 0;
    if ((Key_Read() & KEY_4) || hui_ku_controls()) return 0;
    task_cancel_target();
    goto_reset();
    goto_target.x = x_cm; goto_target.y = y_cm; goto_target.speed = speed_cm_s;
    goto_target.valid = 1;
    drive_stop(); pending_kind = VISION_NONE; return_push_active = 0;
    clear_grip_samples();
    task_info.reason = TASK_OK; step = 50;
    return 1;
}
static void claw(int16_t angle)
{
    if (servo_angle != angle) { servo_angle = angle; Servo_SetAngle((float)angle, 0); }
    if (angle == 50) {
        task_info.held_kind = VISION_NONE;
        pending_kind = VISION_NONE;
        clear_grip_samples();
    }
}
static void enter_step(uint8_t next)
{
    drive_stop(); return_push_active = 0; step = next;
    if (next == 40) grip_commanded = 0;
    if (next == RETURN_BACK) {
        return_reverse_started = 0;
        lower_fault = 0;
        clear_grip_samples();
    }
    if (next == RETURN_CALIBRATE_X) {
        return_sample_count = return_sample_next = return_sample_seen = return_x_skipped = 0;
        return_sample_after = HAL_GetTick() + RETURN_X_SETTLE_MS;
    }
}
void task_start(void)
{
    if (task_info.held_kind != VISION_NONE) return; /* KEY1 must not drop cargo. */
    if (step != 0 && step != 90 && step != 99 && step != 50 && step != 51 && step != RETURN_DONE) return;
    goto_target.valid = 0; goto_reset();
    drive_stop(); claw(50); task_info.reason = TASK_OK; enter_step(1);
}
uint8_t task_start_hui_ku(uint8_t source, uint8_t blocked)
{
    if (blocked == HUI_START_OK &&
        (task_info.held_kind != VISION_NONE ||
         (step != 0 && step != 51 && step != 90 && step != 99 && step != RETURN_DONE)))
        blocked = HUI_START_BUSY;
    return hui_ku_start(source, blocked);
}
void task_stop(void)
{
    task_cancel_target();
    drive_stop(); return_push_active = 0; claw(50); step = 0; goto_target.valid = 0; goto_reset();
    task_info.reason = TASK_STOPPED;
    lower_fault = 0;
}
void task_emergency_stop(void)
{
    task_stop();
    Motor_SetDriveEnabled(0); /* KEY4 removes PWM immediately, including between motor cycles. */
}
void task_prepare(void)
{
    drive_stop(); claw(50);
    lower_fault = grip_commanded = 0;
    near_align_state = 0;
    lower_seen = 1; lower_frame = vision.lower.frame_id;
    task_info.reason = TASK_OK;
    uart_at_start = vision.uart_errors;
}
/* Keep the newest fresh samples in capture order and expose the two vote counts. */
static void count_grip_samples(uint32_t now)
{
    unsigned i, kept = 0, inside = 0;
    for (i = 0; i < grip_sample_count; ++i) {
        if (now - grip_samples[i].observed_ms > GRIP_OBSERVATION_MAX_AGE_MS) continue;
        inside += grip_samples[i].inside;
        grip_samples[kept++] = grip_samples[i];
    }
    grip_sample_count = kept;
    task_info.inside_frames = (uint8_t)inside;
    task_info.outside_frames = (uint8_t)(kept - inside);
}

/* Called for each received frame as well as from move(). */
void task_observe_lower(void)
{
    const VisionObservation *o = &vision.lower;
    uint8_t picking_up = step == 10 || step == 30;
    uint8_t expected_kind;
    unsigned i, matches = 0, other = 0;
    uint32_t now, observed;
    if (!picking_up && !checking_grip()) return;
    if (step == RETURN_BACK && !return_reverse_started) return;
    if (!observation_fresh(o)) {
        lower_fault = TASK_LOWER_WAIT;
        clear_grip_samples();
        return;
    }
    now = HAL_GetTick();
    if (lower_seen && lower_frame == o->frame_id) {
        count_grip_samples(now);
        return;
    }
    lower_seen = 1;
    lower_frame = o->frame_id;
    observed = o->updated_ms - o->age_ms;
    if (step == RETURN_BACK && (int32_t)(observed - grip_reverse_tick) <= 0) return;

    expected_kind = picking_up ? VISION_GREEN :
                    (step == RETURN_BACK ? pending_kind : task_info.held_kind);
    lower_fault = 0;
    for (i = 0; i < o->count; ++i) {
        const VisionObject *v = &o->objects[i];
        if (!v->in_roi) continue;
        if (v->kind == VISION_BLUE) lower_fault = TASK_HAZARD;
        if (v->kind == expected_kind && !(picking_up && v->in_safety_zone)) ++matches;
        else ++other;
    }
    if (!lower_fault && (matches > 1 || other)) lower_fault = TASK_AMBIGUOUS;
    if (lower_fault) {
        clear_grip_samples();
        return;
    }

    if (grip_sample_count == GRIP_WINDOW) {
        for (i = 1; i < grip_sample_count; ++i) grip_samples[i-1U] = grip_samples[i];
        --grip_sample_count;
    }
    grip_samples[grip_sample_count].observed_ms = observed;
    grip_samples[grip_sample_count].frame = o->frame_id;
    grip_samples[grip_sample_count++].inside = matches == 1;
    count_grip_samples(now);
}
static uint8_t hazard_visible(void)
{
    unsigned camera, i;
    for (camera = 0; camera < 2; ++camera) {
        const VisionObservation *o = camera ? &vision.lower : &vision.upper;
        if (!observation_fresh(o)) continue;
        for (i = 0; i < o->count; ++i) {
            const VisionObject *v = &o->objects[i];
            if (v->kind == VISION_UNKNOWN ||
                (camera == 1 && v->kind == VISION_BLUE && v->in_roi)) return 1;
        }
    }
    return 0;
}
/* Keep the existing visual nearest selection and steering. The task ledger
 * excludes delivered/review IDs and records the identity feeding the chase. */
static uint32_t mapped_upper_target(unsigned i)
{
    return aligned_upper() && i < field_map.count && field_map.points[i].present ?
           field_map.points[i].track_id : 0;
}
static uint8_t select_nearest(void)
{
    const VisionObservation *o = &vision.upper;
    unsigned i;
    int best = -1;
    uint8_t green_seen = 0;
    if (!observation_fresh(o)) { task_info.reason = TASK_UPPER_WAIT; return 0; }
    for (i = 0; i < o->count; ++i) {
        const VisionObject *v = &o->objects[i];
        const TargetEntry *entry = target_entry(mapped_upper_target(i));
        if (v->kind != VISION_GREEN || v->in_safety_zone ||
            (entry && entry->state != TASK_TARGET_AVAILABLE &&
             entry->state != TASK_TARGET_LOCKED)) continue;
        green_seen = 1;
        /* Ordered bounds reject NaN/infinity and targets behind the car. */
        if (!v->angle_present || v->distance_mm <= 0 ||
            !(v->angle > -90.0f && v->angle < 90.0f)) continue;
        if (best < 0 || v->distance_mm < o->objects[best].distance_mm) best = (int)i;
    }
    if (best < 0) {
        task_info.reason = green_seen ? TASK_POSITION_INVALID : TASK_TARGET_LOST;
        return 0;
    }
    {
        const uint32_t id = mapped_upper_target((unsigned)best);
        const TargetEntry *active = target_entry(active_target);
        /* Keep the last mapped identity across the 12 cm map cutoff only
         * while its evidence is fresh and there is no other near green block. */
        if (id || o->objects[best].distance_mm > 120 || !active || active->protected_zone ||
            HAL_GetTick() - active->evidence_tick > GRIP_OBSERVATION_MAX_AGE_MS) {
            task_select_target(id);
        } else {
            for (i = 0; i < o->count; ++i) {
                const VisionObject *v = &o->objects[i];
                if (i != (unsigned)best && v->kind == VISION_GREEN && !v->in_safety_zone &&
                    v->distance_mm > 0 && v->distance_mm <= 120) {
                    task_select_target(0);
                    break;
                }
            }
        }
    }
    target_angle = o->objects[best].angle;
    target_distance_cm = o->objects[best].distance_mm * 0.1f;
    task_info.reason = TASK_OK;
    return 1;
}
TaskResult task_search(void)
{
    drive_stop();
    return select_nearest() ? TASK_DONE : TASK_RUNNING;
}
/* Reuse the motor PID implementation, once per fresh visual frame.
 * current=-bearing makes a target of zero produce clockwise output for a right target. */
static float steering_pid(float angle, uint32_t frame)
{
    if (!steer_seen || steer_frame != frame) {
        float previous_error = pid_steer.error_now;
        pid_steer.target = 0;
        pid_steer.current = -angle;
        if (fabsf(angle) <= TASK_ANGLE_DEAD_DEG) {
            /* Keep the measured error visible, but stop correction and clear I. */
            pid_steer.error_last = pid_steer.error_now = angle;
            pid_steer.error_int = pid_steer.output = 0;
            steer_state.p = steer_state.i = steer_state.d = 0;
        } else {
            /* Start outside the deadband without a derivative kick. */
            if (!steer_seen || fabsf(previous_error) <= TASK_ANGLE_DEAD_DEG)
                pid_steer.error_now = angle;
            PID_Update(&pid_steer);
            steer_state.p = pid_steer.kp * pid_steer.error_now;
            steer_state.i = pid_steer.ki * pid_steer.error_int;
            steer_state.d = pid_steer.kd * (pid_steer.error_now - pid_steer.error_last);
        }
        steer_state.tick = HAL_GetTick();
        steer_frame = frame; steer_seen = 1;
    }
    return pid_steer.output;
}
/* Shared follow controller. Check alignment once on entering the near zone.
 * Temporary loss/stop keeps this state; a new task/point resets it. */
static void follow_target(float angle, uint32_t frame, float distance_cm,
                          float speed_cm_s, uint8_t source)
{
    if (near_align_state == 0 && distance_cm <= 25.0f) near_align_state = 1;
    if (near_align_state == 1) {
        if (fabsf(angle) <= 10.0f) near_align_state = 2;
        else speed_cm_s = 0; /* Same angle PID, rotate without moving forward. */
    }
    steer_state.source = source;
    drive(speed_cm_s, steering_pid(angle, frame));
}
/* A map point becomes a virtual target in the body frame. No vision data is
 * overwritten. Both sources use exactly the same forward + bearing PID. */
TaskResult task_goto(float x_cm, float y_cm, float speed_cm_s)
{
    float dx, dy, right, forward, yaw, angle, distance_cm;
    if (!(fabsf(x_cm) <= FLT_MAX && fabsf(y_cm) <= FLT_MAX &&
          speed_cm_s > 0 && speed_cm_s <= FLT_MAX)) {
        drive_stop(); return TASK_FAILED;
    }
    if (!goto_active || x_cm != goto_x || y_cm != goto_y) {
        goto_reset(); drive_stop();
        goto_active = 1; goto_x = x_cm; goto_y = y_cm;
    }
    if (goto_arrived) { drive_stop(); return TASK_DONE; }
    if (!field_map.heading_valid) {
        drive_stop(); return TASK_RUNNING;
    }
    dx = x_cm - field_map.robot.x;
    dy = y_cm - field_map.robot.y;
    yaw = field_map.robot.yaw;
    if (!(fabsf(dx) <= FLT_MAX && fabsf(dy) <= FLT_MAX && fabsf(yaw) <= FLT_MAX)) {
        drive_stop(); return TASK_FAILED;
    }
    distance_cm = hypotf(dx, dy);
    if (distance_cm <= GOTO_POSITION_TOLERANCE_CM) {
        goto_arrived = 1; drive_stop(); return TASK_DONE;
    }
    right = dx*cosf(yaw) - dy*sinf(yaw);
    forward = dx*sinf(yaw) + dy*cosf(yaw);
    angle = atan2f(right, forward) * 180.0f / 3.1415926f;
    follow_target(angle, field_map.robot.tick, distance_cm, speed_cm_s, 2);
    return TASK_RUNNING;
}
TaskResult task_approach(void)
{
    if (!select_nearest()) { drive_stop(); return TASK_RUNNING; }
    follow_target(target_angle, vision.upper.frame_id, target_distance_cm, TASK_FORWARD_CM_S, 1);
    return TASK_RUNNING;
}
TaskResult task_grip(void)
{
    uint32_t now = HAL_GetTick();
    drive_stop();
    if (!grip_commanded) {
        if (pending_kind == VISION_NONE) return TASK_RUNNING;
        claw(120);
        grip_commanded = 1; grip_since = now;
    }
    return now-grip_since >= 500U ? TASK_DONE : TASK_RUNNING;
}
/* Return tasks share the wheel mixer, without using the pursuit PID state. */
static float return_heading_error(float target_yaw)
{
    float delta = target_yaw - field_map.robot.yaw;
    return atan2f(sinf(delta), cosf(delta)) * 180.0f / RETURN_PI;
}

static void return_drive_field(float vx, float vy, float turn_deg_s)
{
    float s = sinf(field_map.robot.yaw), c = cosf(field_map.robot.yaw);
    Motor_SetDriveEnabled(1);
    body_move(vx * s + vy * c, vx * c - vy * s, turn_deg_s);
}

/* Capture a separate origin for each straight reverse leg. */
static TaskResult task_return_reverse(float distance_cm)
{
    float forward_x, forward_y, travelled;
    if (!return_reverse_started) {
        return_reverse_origin = field_map.robot;
        return_reverse_started = 1;
    }
    forward_x = sinf(return_reverse_origin.yaw);
    forward_y = cosf(return_reverse_origin.yaw);
    travelled = (return_reverse_origin.x - field_map.robot.x) * forward_x +
                (return_reverse_origin.y - field_map.robot.y) * forward_y;
    if (travelled >= distance_cm) return TASK_DONE;
    return_drive_field(-RETURN_REVERSE_CM_S * forward_x,
                       -RETURN_REVERSE_CM_S * forward_y,
                       RETURN_YAW_KP * return_heading_error(return_reverse_origin.yaw));
    return TASK_RUNNING;
}

static void task_reverse_and_check_grip(void)
{
    if (!return_reverse_started) grip_reverse_tick = HAL_GetTick();
    if (lower_fault && lower_fault != TASK_LOWER_WAIT) {
        drive_stop();
        task_info.reason = lower_fault;
        return;
    }
    if (task_return_reverse(10.0f) != TASK_DONE) return;

    if (task_info.inside_frames) {
        TargetEntry *entry = target_entry(active_target);
        unsigned i = grip_sample_count;
        if (entry) {
            while (i != 0U) {
                const GripSample *sample = &grip_samples[--i];
                if (!sample->inside) continue;
                target_transition(entry,TASK_TARGET_CARRYING,TARGET_GRIP_VERIFIED,
                                  sample->frame,sample->observed_ms);
                break;
            }
        }
        task_info.held_kind = pending_kind;
        pending_kind = VISION_NONE;
        enter_step(RETURN_FACE);
    } else {
        drive_stop();
        task_info.reason = TASK_LOWER_WAIT;
    }
}

static TaskResult task_turn_to(float yaw, float tolerance_deg)
{
    float error = return_heading_error(yaw);
    if (fabsf(error) <= tolerance_deg) return TASK_DONE;
    drive(0, RETURN_YAW_KP * error);
    return TASK_RUNNING;
}

static TaskResult task_return_strafe(void)
{
    float error = return_zone->centre_x - field_map.robot.x;
    if (fabsf(error) <= RETURN_CENTER_TOL_CM) return TASK_DONE;
    return_drive_field(error > 0 ? RETURN_STRAFE_CM_S : -RETURN_STRAFE_CM_S, 0, 0);
    return TASK_RUNNING;
}

static uint8_t return_observed_zone_x(float *x)
{
    const VisionObservation *o = &vision.upper;
    unsigned i, matches = 0;
    if (!observation_fresh(o) || !field_map.aligned || field_map.frame != o->frame_id)
        return 0;
    for (i = 0; i < o->count && i < field_map.count; ++i) {
        const FieldPoint *p = &field_map.points[i];
        if (o->objects[i].kind != return_zone->kind || !o->objects[i].position_present || !p->present)
            continue;
        *x = p->x;
        ++matches;
    }
    return matches == 1;
}

static void task_return_approach(void)
{
    float direction = return_zone->kind == VISION_RED_MATERIAL_ZONE ? 1.0f : -1.0f;
    float remaining = (return_zone->contact_y - field_map.robot.y) * direction;
    float error, x;
    if (remaining <= RETURN_CONTACT_START_CM) {
        enter_step(RETURN_CONTACT);
        return;
    }
    if (!return_observed_zone_x(&x) || fabsf(x - return_zone->centre_x) > 15.0f)
        x = return_zone->centre_x;
    error = x - field_map.robot.x;
    if (fabsf(error) <= RETURN_CENTER_TOL_CM) error = 0;
    /* The preceding turn stage has ended; correct position by translation only. */
    return_drive_field(RETURN_CENTER_KP * error, direction * RETURN_APPROACH_CM_S, 0);
}

static void task_return_contact(void)
{
    const uint32_t now = HAL_GetTick();
    if (!return_push_active) {
        return_push_tick = now;
        return_push_active = 1;
        Motor_SetDriveEnabled(1);
        MotorEnable1(0);
        MotorEnable2(0);
        MotorSetSpeed3(RETURN_CONTACT_SPEED);
        MotorSetSpeed4(RETURN_CONTACT_SPEED);
        return;
    }
    if (now - return_push_tick < RETURN_CONTACT_MS) return;

    drive_stop();
    /* Incorporate the last IMU heading without counting motion during the push. */
    field_map_record(encoder1.last_tick, x_body, y_body, angle_body);
    field_map_correct_pose(field_map.robot.x, return_zone->contact_y, return_zone->yaw);
    enter_step(RETURN_RELEASE);
}

/* Choose the tightest group of X observations in the rolling window. */
static TaskReason return_x_mean(float *mean)
{
    float sorted[RETURN_X_WINDOW], span, best_span = RETURN_X_SPAN_CM, sum = 0;
    unsigned i, j;
    int best = -1;
    if (return_sample_count < RETURN_X_SAMPLES) return TASK_RETURN_X_WAIT;
    for (i = 0; i < return_sample_count; ++i) {
        float sample = return_x_samples[i];
        for (j = i; j > 0 && sorted[j - 1] > sample; --j)
            sorted[j] = sorted[j - 1];
        sorted[j] = sample;
    }
    for (i = 0; i + RETURN_X_SAMPLES <= return_sample_count; ++i) {
        span = sorted[i + RETURN_X_SAMPLES - 1] - sorted[i];
        if (span <= best_span) {
            best_span = span;
            best = (int)i;
        }
    }
    if (best < 0) return TASK_RETURN_X_SPREAD;
    for (i = 0; i < RETURN_X_SAMPLES; ++i) sum += sorted[(unsigned)best + i];
    *mean = sum / RETURN_X_SAMPLES;
    return TASK_OK;
}

static void task_return_calibrate_x(void)
{
    const VisionObservation *o = &vision.upper;
    const uint32_t now = HAL_GetTick();
    float x, mean;
    drive_stop();
    task_info.reason = TASK_RETURN_X_WAIT;
    if ((int32_t)(now - return_sample_after) < 0) return;
    if (now - return_sample_after >= RETURN_X_TIMEOUT_MS) {
        return_x_skipped = 1;
        task_info.reason = TASK_RETURN_X_SKIPPED;
        enter_step(RETURN_CENTER_FACE);
        return;
    }
    if (!observation_fresh(o)) {
        task_info.reason = TASK_UPPER_WAIT;
        return;
    }
    if (o->status == VISION_OBS_EMPTY) {
        task_info.reason = TASK_RETURN_X_NO_ZONE;
        return;
    }
    if (!field_map.aligned || field_map.frame != o->frame_id ||
        (int32_t)(field_map.observed.tick - return_sample_after) < 0) {
        task_info.reason = TASK_RETURN_X_ALIGN;
        return;
    }
    if (!return_observed_zone_x(&x)) {
        task_info.reason = TASK_RETURN_X_NO_ZONE;
        return;
    }
    if (!return_sample_seen || return_sample_frame != o->frame_id) {
        return_sample_seen = 1;
        return_sample_frame = o->frame_id;
        return_x_samples[return_sample_next] = x;
        return_sample_next = (return_sample_next + 1U) % RETURN_X_WINDOW;
        if (return_sample_count < RETURN_X_WINDOW) ++return_sample_count;
    }
    task_info.reason = return_x_mean(&mean);
    if (task_info.reason == TASK_OK) {
        float dx = return_zone->centre_x - mean;
        field_map_correct_pose(field_map.robot.x + dx, field_map.robot.y, field_map.robot.yaw);
        enter_step(RETURN_CENTER_FACE);
    }
}

static void task_return_release(void)
{
    const uint32_t now = HAL_GetTick();
    drive_stop();
    if (task_info.held_kind == VISION_GREEN) {
        task_record_release(now);
        claw(50);
        return_release_tick = now;
    }
    if (now - return_release_tick >= RETURN_RELEASE_MS) {
        return_reverse_started = 0;
        enter_step(RETURN_RETREAT);
    }
}

static TaskResult task_return_center_face(void)
{
    float dx = RETURN_FIELD_CENTER_CM - field_map.robot.x;
    float dy = RETURN_FIELD_CENTER_CM - field_map.robot.y;
    if (hypotf(dx, dy) <= RETURN_CENTER_TOL_CM) return TASK_DONE;
    return task_turn_to(atan2f(dx, dy), RETURN_CENTER_YAW_TOL_DEG);
}

static TaskResult task_return_center_drive(void)
{
    float dx = RETURN_FIELD_CENTER_CM - field_map.robot.x;
    float dy = RETURN_FIELD_CENTER_CM - field_map.robot.y;
    float distance = hypotf(dx, dy);
    if (distance <= RETURN_CENTER_TOL_CM) return TASK_DONE;
    return_drive_field(RETURN_CENTER_CM_S * dx / distance,
                       RETURN_CENTER_CM_S * dy / distance, 0);
    return TASK_RUNNING;
}

static void task_return_scan(void)
{
    if (!vision.connected || !observation_fresh(&vision.upper)) {
        drive_stop();
        task_info.reason = TASK_UPPER_WAIT;
        return;
    }
    if (select_nearest()) {
        task_prepare();
        enter_step(30);
        return;
    }
    /* The field yaw is continuous across the IMU's 0/360 degree boundary. */
    if (field_map.robot.yaw - return_scan_start_yaw >= 2.0f * RETURN_PI) {
        task_info.reason = TASK_OK;
        enter_step(RETURN_DONE);
        return;
    }
    task_info.reason = TASK_OK;
    drive(0, RETURN_SCAN_DEG_S);
}

static uint8_t return_can_run(void)
{
    if (!field_map.heading_valid) {
        drive_stop();
        task_info.reason = TASK_RETURN_POSE_WAIT;
        return 0;
    }
    if (hazard_visible()) {
        drive_stop();
        task_info.reason = TASK_HAZARD;
        return 0;
    }
    task_info.reason = TASK_OK;
    return 1;
}

void move(void)
{
    task_observe_targets();
    task_observe_delivery();
    task_observe_lower();
    hui_ku_update(hui_ku_controls() && hazard_visible());
    if (hui_ku_controls()) return;
    if (step == 10 || step == 20 || step == 30) {
        if (vision.uart_errors != uart_at_start || !vision.connected) {
            task_info.reason = vision.uart_errors != uart_at_start ? TASK_UART_ERROR : TASK_VISION_LOST;
            uart_at_start = vision.uart_errors;
            drive_stop();
            return;
        }
        if (hazard_visible()) { drive_stop(); task_info.reason = TASK_HAZARD; return; }
        if ((step == 10 || step == 30) && task_info.inside_frames) {
            const TargetEntry *entry = target_entry(active_target);
            if (entry && entry->protected_zone) {
                task_select_target(0);
                drive_stop();
                task_info.reason = TASK_AMBIGUOUS;
                return;
            }
            if (entry && HAL_GetTick() - entry->evidence_tick > GRIP_OBSERVATION_MAX_AGE_MS)
                task_select_target(0);
            pending_kind = VISION_GREEN;
            task_info.reason = TASK_OK;
            enter_step(40);
        } else if (step == 30 && lower_fault) {
            drive_stop(); task_info.reason = lower_fault;
            return;
        }
    }
    if (((step >= RETURN_BACK && step <= RETURN_RETREAT) ||
         step == RETURN_CONTACT || step == RETURN_CALIBRATE_X ||
         (step >= RETURN_CENTER_FACE && step <= RETURN_SCAN)) && !return_can_run()) {
        if (step == RETURN_CONTACT) enter_step(RETURN_CONTACT_ABORT);
        if (step == RETURN_CALIBRATE_X) return_sample_count = return_sample_next = 0;
        return;
    }
    if (checking_grip()) {
        if (!task_info.inside_frames && task_info.outside_frames >= GRIP_EMPTY_VOTES) {
            TargetEntry *entry = target_entry(active_target);
            if (entry)
                target_transition(entry,TASK_TARGET_REVIEW,TARGET_GRIP_FAILED,
                                  vision.lower.frame_id,vision.lower.updated_ms - vision.lower.age_ms);
            active_target = 0;
            task_prepare();
            enter_step(10);
            task_info.reason = TASK_GRIP_LOST;
            return;
        }
        if (step != RETURN_BACK && !task_info.inside_frames) {
            drive_stop();
            return_push_active = 0;
            task_info.reason = lower_fault ? lower_fault : TASK_LOWER_WAIT;
            return;
        }
    }
    if (return_x_skipped && (step == RETURN_CENTER_FACE || step == RETURN_CENTER_DRIVE))
        task_info.reason = TASK_RETURN_X_SKIPPED;
    switch (step) {
        case 0: drive_stop(); break;
        case 1: task_prepare(); enter_step(10); break;
        case 10: if (task_search() == TASK_DONE) enter_step(30); break;
        case 20: enter_step(30); break; /* Retired standalone alignment step. */
        case 30: task_approach(); break;
        case 40:
            if (task_grip() == TASK_DONE) {
                task_info.reason = TASK_OK;
                enter_step(RETURN_BACK);
            }
            break;
        case 50:
            switch (task_goto(goto_target.x, goto_target.y, goto_target.speed)) {
                case TASK_DONE: step = 51; break;
                case TASK_FAILED: task_info.reason = TASK_BAD_STEP; step = 90; break;
                default: break;
            }
            break;
        case 51: drive_stop(); break; /* Coordinate reached; retain target for display. */
        case RETURN_BACK:
            task_reverse_and_check_grip();
            break;
        case RETURN_FACE:
            if (task_turn_to(return_zone->yaw, RETURN_ZONE_YAW_TOL_DEG) == TASK_DONE) enter_step(RETURN_STRAFE);
            break;
        case RETURN_STRAFE:
            if (task_return_strafe() == TASK_DONE) enter_step(RETURN_APPROACH);
            break;
        case RETURN_APPROACH: task_return_approach(); break;
        case RETURN_RELEASE: task_return_release(); break;
        case RETURN_RETREAT:
            if (task_return_reverse(RETURN_RETREAT_CM) == TASK_DONE) enter_step(RETURN_CALIBRATE_X);
            break;
        case RETURN_CONTACT_ABORT: drive_stop(); break;
        case RETURN_CONTACT: task_return_contact(); break;
        case RETURN_CALIBRATE_X: task_return_calibrate_x(); break;
        case RETURN_CENTER_FACE:
            if (task_return_center_face() == TASK_DONE) enter_step(RETURN_CENTER_DRIVE);
            break;
        case RETURN_CENTER_DRIVE:
            if (task_return_center_drive() == TASK_DONE) {
                return_scan_start_yaw = field_map.robot.yaw;
                enter_step(RETURN_SCAN);
            }
            break;
        case RETURN_SCAN: task_return_scan(); break;
        case RETURN_DONE: drive_stop(); break;
        case 90: drive_stop(); break;
        case 99: drive_stop(); break; /* Hold until an explicit release (KEY4). */
        default: task_stop(); task_info.reason = TASK_BAD_STEP; break;
    }
}
