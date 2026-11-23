#include "hui_ku.h"

#include "decode_move.h"
#include "faraway.h"
#include "field_map.h"
#include "imu.h"
#include "motor.h"
#include <limits.h>
#include <math.h>
#include <string.h>

#define DEG_RAD (3.14159265358979323846f / 180.0f)
#define HUI_OBS_MAX_MS 200U
#define HUI_LOSS_MS 800U
#define HUI_LOSS_CM 15.0f
#define HUI_POS_TOL_CM 2.0f
#define HUI_ANGLE_TOL_DEG 2.0f
#define HUI_STOP_CM_S 2.0f
#define HUI_SETTLE_MS 200U
#define HUI_MOVE_CM_S 30.0f
#define HUI_REOBSERVE_DEG 15.0f
#define HUI_PENDING_FRAMES 8U
#define HUI_FIRMWARE_VERSION 2026112304U
#define COUNTS_TO_CM (3.14159265358979323846f * wheel_diameter / 40810.0f)

enum {
    LOG_START = 1, LOG_STEP, LOG_FIX, LOG_OBS, LOG_NAV, LOG_WHEEL, LOG_END
};
enum {
    REASON_NONE, REASON_BEGIN, REASON_FIX, REASON_EMPTY, REASON_OTHER_ZONE,
    REASON_AMBIGUOUS, REASON_INVALID, REASON_STALE, REASON_ALIGN,
    REASON_OUTLIER, REASON_SILENCE, REASON_LOSS_TIME, REASON_LOSS_TRAVEL,
    REASON_AT_POINT, REASON_SETTLED, REASON_IMU, REASON_HAZARD, REASON_KEY4,
    REASON_BUSY, REASON_NO_POSE, REASON_TURN, REASON_RESET, REASON_OLD,
    REASON_KEY4_HELD
};
enum { ACTION_STOP, ACTION_PREDICTED, ACTION_CAMERA, ACTION_SEARCH };
enum { OUTCOME_INTERRUPTED, OUTCOME_COMPLETE, OUTCOME_REJECTED, OUTCOME_FAILED };

/* Log payloads are fixed little-endian layouts over the versioned UART5
 * channel. Keep the compact samples aligned and in sync with vision_xy.html. */
typedef struct {
    uint32_t source, result, epoch, zone, valid, obs_max_ms, loss_ms, settle_ms;
    float zx, zy, gx, gy, x, y, yaw, odom_x, odom_y, odom_yaw, imu_yaw;
    float loss_cm, pos_tol, angle_tol, stop_speed, max_fix, ex_x, ex_y, ex_yaw; /* max_fix: 0 means no cap */
    uint32_t firmware_version;
} StartLog;
typedef struct {
    uint32_t from, to, reason, action, frame, age_ms;
    float dx, dy, bearing, loss_cm;
} StepLog;
typedef struct {
    uint32_t frame, obs_tick;
    int16_t x_mm, y_mm, angle, yaw, pred_x, pred_y, dx, dy, before_x, before_y, reserved;
    uint8_t flags, matches;
} FixLog;
typedef struct {
    uint32_t reason, first_tick, last_tick, first_frame, last_frame, count, last_good_frame, flags;
    float loss_cm;
} ObsLog;
typedef struct {
    uint32_t frame, applied_tick;
    uint16_t imu_age, obs_age;
    uint8_t stage, action, valid, drive;
    int16_t x, y, yaw, odom_x, odom_y, odom_yaw;
    uint16_t imu_yaw;
    int16_t bearing;
    int16_t forward, right, turn, reserved, applied_forward, applied_right, applied_turn, padding;
} NavLog;
typedef struct {
    uint32_t sample_tick, dt_ms;
    int16_t targets[4];
    int32_t delta[4];
} WheelLog;
typedef struct {
    uint32_t source, stage, reason, outcome, drive, started_tick, stopped_tick, encoder_tick;
    uint32_t frame, seen, fixes, rejected, longest_loss_ms, outputs_zero;
    int32_t speed[4];
    float x, y, yaw, dx, dy, bearing, longest_loss_cm;
} EndLog;
typedef char HuiLogSizes[(sizeof(StartLog) == 112 && sizeof(StepLog) == 40 &&
                         sizeof(FixLog) == 32 && sizeof(ObsLog) == 36 &&
                         sizeof(NavLog) == 48 && sizeof(WheelLog) == 32 &&
                         sizeof(EndLog) == 100) ? 1 : -1];

typedef struct {
    float zx, zy, gx, gy;
    VisionKind kind;
} Zone;

typedef struct {
    HuiKuStage stage;
    uint8_t action, reason;
    float forward, right, turn;
} Plan;

typedef struct {
    uint32_t frame, tick;
    int32_t x_mm, y_mm;
    float angle;
    uint8_t reason, flags;
} UpperFrame;

static const Encoder_t *const encoders[4] = { &encoder1, &encoder2, &encoder3, &encoder4 };
static struct {
    Zone zone;
    HuiKuStage stage;
    uint32_t epoch, started, settling_at, frame, obs_tick, last_good_tick, good_frame, applied_tick;
    uint32_t seen, fixes, rejected, longest_loss_ms, nav_tick, wheel_tick;
    uint32_t observed_before, input_frame;
    int64_t wheel_count[4];
    float good_travel, longest_loss_cm, bearing, good_heading, good_angle;
    uint8_t owns, action, reason, obs_reason, frame_seen, settling, rejected_start, failed;
    uint8_t input_seen, pending_start, pending_count, terminal_wait;
    UpperFrame pending[HUI_PENDING_FRAMES];
    ObsLog missed;
} run;

static float angle_deg(float angle_rad)
{
    return atan2f(sinf(angle_rad), cosf(angle_rad)) / DEG_RAD;
}

static int16_t scaled(float value, float scale)
{
    value *= scale;
    if (value >= INT16_MAX) return INT16_MAX;
    if (value <= INT16_MIN) return INT16_MIN;
    return (int16_t)lroundf(value);
}

static uint8_t pose_flags(uint32_t now)
{
    return (field_map.heading_valid ? 1U : 0U) | (imu_fresh(now) ? 2U : 0U);
}

static uint16_t age16(uint32_t age)
{
    return age < UINT16_MAX ? (uint16_t)age : UINT16_MAX;
}

static float goal_x(void) { return run.zone.gx - field_map.robot.x; }
static float goal_y(void) { return run.zone.gy - field_map.robot.y; }

static float lost_travel(void)
{
    return fmaxf(0.0f, field_map_travel_cm() - run.good_travel);
}

static void remember_loss(uint32_t now)
{
    uint32_t elapsed = now - run.last_good_tick;
    if (elapsed > run.longest_loss_ms) run.longest_loss_ms = elapsed;
    run.longest_loss_cm = fmaxf(run.longest_loss_cm, lost_travel());
}

static void flush_misses(uint32_t now)
{
    if (!run.missed.reason) return;
    hui_log_record(LOG_OBS, now, &run.missed, sizeof(run.missed));
    memset(&run.missed, 0, sizeof(run.missed));
}

static void observe_miss(uint8_t reason, uint32_t now, uint8_t flags, uint8_t new_frame)
{
    ObsLog *o = &run.missed;
    if (o->reason != reason) {
        flush_misses(now);
        o->reason = reason;
        o->first_tick = now;
        o->first_frame = run.frame;
    }
    o->last_tick = now;
    o->last_frame = run.frame;
    o->count += new_frame;
    o->last_good_frame = run.good_frame;
    o->flags = flags;
    o->loss_cm = lost_travel();
    if (now - o->first_tick >= 500U) flush_misses(now);
    run.obs_reason = reason;
    run.rejected += new_frame;
    remember_loss(now);
}

static void record_nav(uint32_t now)
{
    NavLog r;
    memset(&r, 0, sizeof(r));
    r.frame = run.frame;
    r.applied_tick = body_motion.updated_ms;
    r.imu_age = imu.ready ? age16(now - imu.tick) : UINT16_MAX;
    r.obs_age = run.frame_seen ? age16(now - run.obs_tick) : UINT16_MAX;
    r.stage = (uint8_t)run.stage;
    r.action = run.action;
    r.valid = pose_flags(now);
    r.drive = Motor_DriveEnabled();
    r.x = scaled(field_map.robot.x, 10.0f);
    r.y = scaled(field_map.robot.y, 10.0f);
    r.yaw = scaled(angle_deg(field_map.robot.yaw), 100.0f);
    r.odom_x = scaled(x_body, 10.0f);
    r.odom_y = scaled(y_body, 10.0f);
    r.odom_yaw = scaled(angle_deg(angle_body), 100.0f);
    r.imu_yaw = imu.ready ? (uint16_t)lroundf(imu.raw_yaw * 100.0f) : 0U;
    r.bearing = scaled(run.bearing, 100.0f);
    r.forward = scaled(body_motion.target_forward, 10.0f);
    r.right = scaled(body_motion.target_right, 10.0f);
    r.turn = scaled(body_motion.target_turn, 10.0f);
    r.applied_forward = scaled(body_motion.forward, 10.0f);
    r.applied_right = scaled(body_motion.right, 10.0f);
    r.applied_turn = scaled(body_motion.turn, 10.0f);
    hui_log_record(LOG_NAV, now, &r, sizeof(r));
    run.nav_tick = now;
}

static void record_wheels(uint32_t now)
{
    unsigned i;
    WheelLog r;
    r.sample_tick = encoder1.last_tick;
    r.dt_ms = r.sample_tick - run.wheel_tick;
    Motor_GetWheelTargets(r.targets);
    for (i = 0; i < 4; ++i) {
        int64_t count = encoder_get_count(encoders[i]);
        int64_t delta = count - run.wheel_count[i];
        r.delta[i] = delta > INT32_MAX ? INT32_MAX : delta < INT32_MIN ? INT32_MIN : (int32_t)delta;
        run.wheel_count[i] = count;
    }
    hui_log_record(LOG_WHEEL, now, &r, sizeof(r));
    run.wheel_tick = r.sample_tick;
}

static uint8_t change_stage(HuiKuStage stage, uint8_t action, uint8_t reason, uint32_t now)
{
    StepLog r;
    if (run.stage == stage && run.action == action && run.reason == reason) return 0;
    r.from = run.stage;
    r.to = stage;
    r.reason = reason;
    r.action = action;
    r.frame = run.frame;
    r.age_ms = run.frame_seen ? now - run.obs_tick : UINT32_MAX;
    r.dx = goal_x();
    r.dy = goal_y();
    r.bearing = run.bearing;
    r.loss_cm = lost_travel();
    hui_log_record(LOG_STEP, now, &r, sizeof(r));
    run.stage = stage;
    run.action = action;
    run.reason = reason;
    return 1;
}

static void record_start(uint8_t source, uint8_t blocked, uint32_t now)
{
    StartLog r = {
        source, blocked, run.epoch, run.zone.kind, pose_flags(now),
        HUI_OBS_MAX_MS, HUI_LOSS_MS, HUI_SETTLE_MS,
        run.zone.zx, run.zone.zy, run.zone.gx, run.zone.gy,
        field_map.robot.x, field_map.robot.y, angle_deg(field_map.robot.yaw),
        x_body, y_body, angle_deg(angle_body), imu.raw_yaw,
        HUI_LOSS_CM, HUI_POS_TOL_CM, HUI_ANGLE_TOL_DEG, HUI_STOP_CM_S, 0.0f,
        0.0f, 0.0f, 0.0f, HUI_FIRMWARE_VERSION
    };
    hui_log_record(LOG_START, now, &r, sizeof(r));
}

static void fix_position(const UpperFrame *zone, const FieldPose *pose, uint32_t now)
{
    float u = zone->x_mm * 0.1f, v = zone->y_mm * 0.1f;
    float c = cosf(pose->yaw), s = sinf(pose->yaw);
    float x = run.zone.zx - (u * c + v * s);
    float y = run.zone.zy - (-u * s + v * c);
    float dx = x - pose->x, dy = y - pose->y;
    FixLog r;

    if (!(x >= 0.0f && x <= 300.0f && y >= 0.0f && y <= 300.0f)) {
        observe_miss(REASON_OUTLIER, now, zone->flags, 1);
        return;
    }
    if (run.obs_reason != REASON_FIX || now - run.applied_tick > HUI_OBS_MAX_MS)
        remember_loss(now);
    flush_misses(now);
    r.frame = run.frame;
    r.obs_tick = pose->tick;
    r.x_mm = scaled((float)zone->x_mm, 1.0f);
    r.y_mm = scaled((float)zone->y_mm, 1.0f);
    r.angle = scaled(zone->angle, 100.0f);
    r.yaw = scaled(angle_deg(pose->yaw), 100.0f);
    r.pred_x = scaled(pose->x, 10.0f);
    r.pred_y = scaled(pose->y, 10.0f);
    r.dx = scaled(dx, 10.0f);
    r.dy = scaled(dy, 10.0f);
    r.before_x = scaled(field_map.robot.x, 10.0f);
    r.before_y = scaled(field_map.robot.y, 10.0f);
    r.reserved = 0;
    r.flags = zone->flags;
    r.matches = 1U;
    field_map_translate(dx, dy);
    hui_log_record(LOG_FIX, now, &r, sizeof(r));
    ++run.fixes;
    run.good_frame = run.frame;
    run.last_good_tick = pose->tick;
    run.good_travel = field_map_travel_cm();
    run.good_angle = zone->angle;
    run.good_heading = pose->yaw;
    run.terminal_wait = 0;
    run.applied_tick = now;
    run.obs_reason = REASON_FIX;
}

static void process_pending(uint32_t now)
{
    while (run.pending_count) {
        const UpperFrame *frame = &run.pending[run.pending_start];
        uint8_t reason = frame->reason;
        FieldPose pose;
        if (reason == REASON_NONE && now - frame->tick >= HUI_OBS_MAX_MS)
            reason = REASON_STALE;
        if (reason == REASON_NONE && run.fixes && (int32_t)(frame->tick - run.last_good_tick) < 0)
            reason = REASON_OLD;
        if (reason == REASON_NONE) {
            if (pose_flags(now) != 3U) {
                reason = REASON_ALIGN;
            } else if (!field_map_pose_at(frame->tick, &pose)) {
                if ((int32_t)(frame->tick - field_map.robot.tick) > 0)
                    return; /* Wait for the next wheel sample to bracket it. */
                reason = REASON_ALIGN;
            }
        }
        run.frame = frame->frame;
        run.obs_tick = frame->tick;
        run.frame_seen = 1;
        ++run.seen;
        if (reason != REASON_NONE) observe_miss(reason, now, frame->flags, 1);
        else fix_position(frame, &pose, now);
        run.pending_start = (uint8_t)((run.pending_start + 1U) % HUI_PENDING_FRAMES);
        --run.pending_count;
    }
}

static void enqueue_upper(uint32_t now)
{
    const VisionObservation *o = &vision.upper;
    const VisionObject *zone = NULL;
    UpperFrame frame;
    unsigned i, matches = 0;

    if ((!o->updated_ms && !o->fresh) || o->age_ms == UINT32_MAX ||
        (run.input_seen && run.input_frame == o->frame_id))
        return;
    if (run.input_seen && (int32_t)(o->frame_id - run.input_frame) < 0 &&
        now - run.observed_before <= 400U)
        return;
    memset(&frame, 0, sizeof(frame));
    frame.frame = o->frame_id;
    frame.tick = o->updated_ms - o->age_ms;
    frame.flags = (o->fresh ? 8U : 0U) | (o->available ? 16U : 0U);
    if (o->status != VISION_OBS_OK && o->status != VISION_OBS_EMPTY && o->status != VISION_OBS_STALE) {
        frame.reason = REASON_INVALID;
    } else if (!o->fresh || now - frame.tick >= HUI_OBS_MAX_MS || o->status == VISION_OBS_STALE) {
        frame.reason = REASON_STALE;
    } else if (o->status == VISION_OBS_EMPTY) {
        frame.reason = REASON_EMPTY;
    } else if (!o->available || o->status != VISION_OBS_OK || o->count > VISION_MAX_OBJECTS) {
        frame.reason = REASON_INVALID;
    } else {
        for (i = 0; i < o->count; ++i) {
            if (o->objects[i].kind != run.zone.kind) continue;
            zone = &o->objects[i];
            ++matches;
        }
        if (!matches) frame.reason = REASON_OTHER_ZONE;
        else if (matches != 1) frame.reason = REASON_AMBIGUOUS;
        else {
            frame.flags |= (zone->position_present ? 1U : 0U) | (zone->angle_present ? 2U : 0U) |
                           (zone->distance_valid ? 4U : 0U);
            /* Accept numeric estimates for this test; keep the original valid bit in the log. */
            if ((frame.flags & 3U) != 3U || !(zone->angle >= -180.0f && zone->angle <= 180.0f) ||
                (!zone->x_mm && !zone->y_mm))
                frame.reason = REASON_INVALID;
            frame.x_mm = zone->x_mm;
            frame.y_mm = zone->y_mm;
            frame.angle = zone->angle;
        }
    }
    run.input_seen = 1;
    run.input_frame = frame.frame;
    run.observed_before = now;
    if (run.pending_count == HUI_PENDING_FRAMES) {
        /* The oldest sample still could not be bracketed. Preserve the next
         * radio frame and explicitly account for the unalignable observation. */
        run.pending[run.pending_start].reason = REASON_ALIGN;
        process_pending(now);
    }
    run.pending[(run.pending_start + run.pending_count) % HUI_PENDING_FRAMES] = frame;
    ++run.pending_count;
}

void hui_ku_observe_upper(void)
{
    uint32_t now;
    if (!run.owns || run.stage == HUI_COMPLETE || run.epoch != field_map.epoch) return;
    now = HAL_GetTick();
    enqueue_upper(now);
    process_pending(now);
}

static uint8_t is_settled(uint32_t now)
{
    unsigned i;
    for (i = 0; i < 4; ++i) {
        if (now - encoders[i]->last_tick > 100U ||
            fabsf((float)encoders[i]->speed) * COUNTS_TO_CM > HUI_STOP_CM_S) {
            run.settling = 0;
            return 0;
        }
    }
    if (!run.settling) {
        run.settling = 1;
        run.settling_at = now;
    }
    return now - run.settling_at >= HUI_SETTLE_MS;
}

static Plan next_plan(uint32_t now, uint8_t hazard)
{
    Plan p = { HUI_PROTECT, ACTION_STOP, REASON_NONE, 0, 0, 0 };
    float dx, dy, distance, yaw, c, s, gain;
    uint8_t use_camera, vision_lost;

    if (run.stage == HUI_COMPLETE) {
        p.stage = HUI_COMPLETE;
        p.reason = REASON_SETTLED;
        return p;
    }
    if (run.epoch != field_map.epoch) run.failed = 1;
    if (run.failed) { p.reason = REASON_RESET; return p; }
    if (pose_flags(now) != 3U) { p.reason = REASON_IMU; return p; }
    if (hazard) { p.reason = REASON_HAZARD; return p; }

    dx = goal_x();
    dy = goal_y();
    distance = hypotf(dx, dy);
    yaw = field_map.robot.yaw;
    use_camera = run.obs_reason == REASON_FIX && now - run.last_good_tick < HUI_OBS_MAX_MS;
    vision_lost = !use_camera && run.fixes &&
                  (now - run.last_good_tick >= HUI_LOSS_MS || lost_travel() >= HUI_LOSS_CM);
    /* Advance the measured angle to the current IMU heading. The predicted
     * bearing takes over as soon as the latest accepted evidence is lost. */
    run.bearing = use_camera ? angle_deg(run.good_angle * DEG_RAD + run.good_heading - yaw) :
                  angle_deg(atan2f(run.zone.zx - field_map.robot.x,
                                   run.zone.zy - field_map.robot.y) - yaw);

    if (fabsf(dx) <= HUI_POS_TOL_CM && fabsf(dy) <= HUI_POS_TOL_CM) {
        p.stage = HUI_CONFIRM;
        p.reason = REASON_AT_POINT;
        if (vision_lost) {
            run.settling = 0;
            p.stage = HUI_REACQUIRE;
            p.reason = lost_travel() >= HUI_LOSS_CM ? REASON_LOSS_TRAVEL : REASON_LOSS_TIME;
            return p;
        }
        if (run.fixes) {
            /* Carry the accepted exposure's field bearing across short gaps.
             * Apply the IMU motion since the last encoder-aligned heading. */
            float current_yaw = yaw + (angle_imu - angle_body);
            run.bearing = angle_deg(run.good_heading + run.good_angle * DEG_RAD - current_yaw);
            if (fabsf(run.bearing) <= HUI_ANGLE_TOL_DEG) run.terminal_wait = 1;
        }
        if (fabsf(run.bearing) > HUI_ANGLE_TOL_DEG && !run.terminal_wait) {
            p.action = use_camera ? ACTION_CAMERA : run.fixes ? ACTION_PREDICTED : ACTION_SEARCH;
            p.reason = REASON_TURN;
            p.turn = 2.5f * run.bearing;
            run.settling = 0;
        } else {
            uint8_t settled = is_settled(now);
            if (use_camera && settled && fabsf(run.good_angle) <= HUI_ANGLE_TOL_DEG &&
                fabsf(run.bearing) <= HUI_ANGLE_TOL_DEG) {
                p.stage = HUI_COMPLETE;
                p.reason = REASON_SETTLED;
            }
        }
        return p;
    }

    run.terminal_wait = run.settling = 0;
    if (vision_lost) {
        float sweep = ((now - run.last_good_tick) / HUI_LOSS_MS) % 2U ?
                      HUI_REOBSERVE_DEG : -HUI_REOBSERVE_DEG;
        p.stage = HUI_REACQUIRE;
        p.reason = lost_travel() >= HUI_LOSS_CM ? REASON_LOSS_TRAVEL : REASON_LOSS_TIME;
        p.action = ACTION_SEARCH;
        p.turn = 2.5f * angle_deg((run.bearing + sweep) * DEG_RAD);
        return p;
    }

    p.stage = use_camera ? HUI_VISUAL : run.fixes ? HUI_COAST : HUI_SEEK;
    p.action = use_camera ? ACTION_CAMERA : ACTION_PREDICTED;
    p.reason = use_camera ? REASON_FIX : run.obs_reason ? run.obs_reason : REASON_BEGIN;
    /* Position controls translation independently of the target bearing.
     * This still moves toward the centreline when the camera angle is zero. */
    gain = fminf(2.0f, HUI_MOVE_CM_S / distance);
    c = cosf(yaw);
    s = sinf(yaw);
    p.forward = (dx * s + dy * c) * gain;
    p.right = (dx * c - dy * s) * gain;
    p.turn = 2.5f * run.bearing;
    return p;
}

uint8_t hui_ku_start(uint8_t source, uint8_t blocked)
{
    unsigned i;
    uint32_t now = HAL_GetTick();
    if (!hui_log_begin(now)) return HUI_START_BUSY;

    memset(&run, 0, sizeof(run));
    run.zone = RETURN_TEAM == VISION_RED ?
        (Zone){ 135.0f, 282.0f, 135.0f, 232.0f, VISION_RED_MATERIAL_ZONE } :
        (Zone){ 165.0f, 18.0f, 165.0f, 68.0f, VISION_BLUE_MATERIAL_ZONE };
    run.epoch = field_map.epoch;
    run.started = run.last_good_tick = run.observed_before = now;
    run.good_travel = field_map_travel_cm();
    run.frame = vision.upper.frame_id;
    run.obs_tick = vision.upper.updated_ms - vision.upper.age_ms;
    run.frame_seen = vision.upper.fresh ||
        (vision.upper.updated_ms && vision.upper.age_ms != UINT32_MAX);
    run.input_seen = run.frame_seen;
    run.input_frame = run.frame;
    run.wheel_tick = encoder1.last_tick;
    for (i = 0; i < 4; ++i) run.wheel_count[i] = encoder_get_count(encoders[i]);
    record_start(source, blocked, now);
    if (blocked != HUI_START_OK) {
        run.rejected_start = 1;
        change_stage(HUI_PROTECT, ACTION_STOP,
                     blocked == HUI_START_KEY4_HELD ? REASON_KEY4_HELD : REASON_BUSY, now);
    } else {
        run.owns = 1;
        goto_target.valid = 0;
        step = HUI_KU_TASK_STEP;
        if (pose_flags(now) == 3U)
            change_stage(HUI_SEEK, ACTION_PREDICTED, REASON_BEGIN, now);
        else
            change_stage(HUI_PROTECT, ACTION_STOP, REASON_NO_POSE, now);
    }
    record_nav(now);
    record_wheels(now);
    return blocked;
}

void hui_ku_update(uint8_t hazard)
{
    uint32_t now, nav_period, wheel_period;
    uint8_t changed = 0;

    if (!hui_log_active()) return;
    now = HAL_GetTick();
    if (run.owns) {
        Plan p;
        if (run.stage != HUI_COMPLETE && run.epoch == field_map.epoch) {
            hui_ku_observe_upper();
            if (!run.pending_count && now - run.observed_before >= HUI_OBS_MAX_MS)
                observe_miss(REASON_SILENCE, now, 0U, 0);
        }
        p = next_plan(now, hazard);
        if (p.stage == HUI_PROTECT) run.settling = 0;
        if (p.action == ACTION_STOP) {
            if (Motor_DriveEnabled()) Motor_SetDriveEnabled(0);
        } else {
            Motor_SetDriveEnabled(1);
            body_move(p.forward, p.right, p.turn);
        }
        changed = change_stage(p.stage, p.action, p.reason, now);
        task_info.reason = p.reason == REASON_HAZARD ? TASK_HAZARD :
            p.stage == HUI_PROTECT ? TASK_RETURN_POSE_WAIT :
            (p.stage == HUI_REACQUIRE || (p.stage == HUI_CONFIRM && run.obs_reason != REASON_FIX)) ?
            TASK_RETURN_X_NO_ZONE : TASK_OK;
    }
    nav_period = run.stage == HUI_COMPLETE ? 1000U : 100U;
    wheel_period = run.stage == HUI_COMPLETE ? 1000U : 200U;
    if (changed || now - run.nav_tick >= nav_period) record_nav(now);
    if (encoder1.last_tick - run.wheel_tick >= wheel_period) record_wheels(now);
}

void hui_ku_end(uint8_t source)
{
    unsigned i;
    int16_t targets[4];
    uint32_t now;
    EndLog r;
    if (!hui_log_active()) return;
    now = HAL_GetTick();
    if (run.owns && run.stage != HUI_COMPLETE && run.obs_reason != REASON_FIX)
        remember_loss(now);
    flush_misses(now);
    r.source = source;
    r.stage = run.stage;
    r.reason = run.reason;
    r.outcome = run.rejected_start ? OUTCOME_REJECTED : run.failed ? OUTCOME_FAILED :
                run.stage == HUI_COMPLETE ? OUTCOME_COMPLETE : OUTCOME_INTERRUPTED;
    r.drive = Motor_DriveEnabled();
    r.started_tick = run.started;
    r.stopped_tick = now;
    r.encoder_tick = encoder1.last_tick;
    r.frame = run.frame;
    r.seen = run.seen;
    r.fixes = run.fixes;
    r.rejected = run.rejected;
    r.longest_loss_ms = run.longest_loss_ms;
    r.outputs_zero = !r.drive && body_motion.forward == 0.0f && body_motion.right == 0.0f &&
                     body_motion.turn == 0.0f;
    Motor_GetWheelTargets(targets);
    for (i = 0; i < 4; ++i) {
        r.speed[i] = encoders[i]->speed;
        r.outputs_zero = r.outputs_zero && targets[i] == 0;
    }
    r.x = field_map.robot.x;
    r.y = field_map.robot.y;
    r.yaw = angle_deg(field_map.robot.yaw);
    r.dx = goal_x();
    r.dy = goal_y();
    r.bearing = run.bearing;
    r.longest_loss_cm = run.longest_loss_cm;
    change_stage(HUI_STOPPED, ACTION_STOP, REASON_KEY4, now);
    record_nav(now);
    record_wheels(now);
    hui_log_record(LOG_END, now, &r, sizeof(r));
    hui_log_end();
    run.owns = 0;
}

uint8_t hui_ku_controls(void) { return run.owns; }
HuiKuStage hui_ku_stage(void) { return run.stage; }
