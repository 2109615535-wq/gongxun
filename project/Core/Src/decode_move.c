#include "decode_move.h"
#include "vision.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <float.h>
#include "pid.h"
#include "servo.h"
#include "key.h"
#include "field_map.h"
#include "imu.h"

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

/* Simple low-speed mission: nearest green -> forward + steer -> grip and hold.
 * Nonblocking tasks; missing observations stop motion and wait for recovery. */

uint8_t step = 0;
TaskInfo task_info;
GotoTarget goto_target;
static uint32_t lower_frame, uart_at_start, grip_since;
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
    return o->fresh && observation_age(o) <= 400U &&
           (o->status == VISION_OBS_OK || o->status == VISION_OBS_EMPTY);
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
uint8_t task_goto_start(float x_cm, float y_cm, float speed_cm_s)
{
    if (!(x_cm >= 0 && x_cm <= 300 && y_cm >= 0 && y_cm <= 300 &&
          speed_cm_s > 0 && speed_cm_s <= FLT_MAX)) return 0;
    if (Key_Read() & KEY_4) return 0;
    goto_reset();
    goto_target.x = x_cm; goto_target.y = y_cm; goto_target.speed = speed_cm_s;
    goto_target.valid = 1;
    drive_stop(); pending_kind = VISION_NONE;
    task_info.reason = TASK_OK; step = 50;
    return 1;
}
static void claw(int16_t angle)
{
    if (servo_angle != angle) { servo_angle = angle; Servo_SetAngle((float)angle, 0); }
    if (angle == 50) { task_info.held_kind = VISION_NONE; pending_kind = VISION_NONE; }
}
static void enter_step(uint8_t next)
{
    drive_stop(); step = next;
    if (next == 30) {
        task_info.inside_frames = task_info.outside_frames = 0;
        lower_seen = 1; lower_frame = vision.lower.frame_id; lower_fault = 0;
    }
    if (next == 40) grip_commanded = 0;
}
void task_start(void)
{
    if (task_info.held_kind != VISION_NONE) return; /* KEY1 must not drop cargo. */
    if (step != 0 && step != 90 && step != 99 && step != 50 && step != 51) return;
    goto_target.valid = 0; goto_reset();
    drive_stop(); claw(50); task_info.reason = TASK_OK; enter_step(1);
}
void task_stop(void)
{
    drive_stop(); claw(50); step = 0; goto_target.valid = 0; goto_reset();
    task_info.reason = TASK_STOPPED;
    task_info.inside_frames = task_info.outside_frames = 0;
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
    task_info.reason = TASK_OK; task_info.inside_frames = task_info.outside_frames = 0;
    uart_at_start = vision.uart_errors;
}
/* Called on each received frame; duplicates do not increment ROI counts. */
void task_observe_lower(void)
{
    const VisionObservation *o = &vision.lower;
    unsigned i, green = 0, other = 0;
    if (step != 10 && step != 30 && step != 40 && step != 99) return;
    if (!observation_fresh(o)) {
        lower_fault = TASK_LOWER_WAIT;
        task_info.inside_frames = task_info.outside_frames = 0;
        return;
    }
    if (lower_seen && lower_frame == o->frame_id) return;
    lower_seen = 1; lower_frame = o->frame_id; lower_fault = 0;
    for (i = 0; i < o->count; ++i) {
        const VisionObject *v = &o->objects[i];
        if (v->kind == VISION_BLUE) lower_fault = TASK_HAZARD;
        if (!v->in_roi) continue;
        if (v->kind == VISION_GREEN) ++green;
        else ++other;
    }
    if (!lower_fault && (green > 1 || other)) lower_fault = TASK_AMBIGUOUS;
    if (lower_fault) { task_info.inside_frames = task_info.outside_frames = 0; return; }
    if (green == 1) {
        /* Latch the first valid ROI event, even if a later queued frame loses it. */
        if ((step == 10 || step == 30) && pending_kind == VISION_NONE)
            pending_kind = VISION_GREEN;
        task_info.outside_frames = 0;
        if (task_info.inside_frames < 3) ++task_info.inside_frames;
    } else {
        task_info.inside_frames = 0;
        if (task_info.outside_frames < 3) ++task_info.outside_frames;
    }
}
static uint8_t hazard_visible(void)
{
    unsigned camera, i;
    for (camera = 0; camera < 2; ++camera) {
        const VisionObservation *o = camera ? &vision.lower : &vision.upper;
        if (!observation_fresh(o)) continue;
        for (i = 0; i < o->count; ++i)
            if (o->objects[i].kind == VISION_BLUE || o->objects[i].kind == VISION_UNKNOWN) return 1;
    }
    return 0;
}
/* Re-select the nearest ordinary block every call, without ID/position locking.
 * Estimated distance/angle are allowed; source validity flags stay unchanged. */
static uint8_t select_nearest(void)
{
    const VisionObservation *o = &vision.upper;
    unsigned i;
    int best = -1;
    uint8_t green_seen = 0;
    if (!observation_fresh(o)) { task_info.reason = TASK_UPPER_WAIT; return 0; }
    for (i = 0; i < o->count; ++i) {
        const VisionObject *v = &o->objects[i];
        if (v->kind != VISION_GREEN) continue;
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
        claw(100);
        /* User's mechanism: entering ROI and closing means the object is held.
         * Keep the captured class even when it disappears from both cameras. */
        task_info.held_kind = pending_kind;
        grip_commanded = 1; grip_since = now;
    }
    return now-grip_since >= 500U ? TASK_DONE : TASK_RUNNING;
}
void move(void)
{
    if ((step >= 10 && step <= 40) || step == 99) task_observe_lower();
    if ((step == 10 || step == 30) && pending_kind != VISION_NONE) {
        task_info.reason = TASK_OK;
        enter_step(40); /* Stop and close in this same scheduler call. */
    }
    /* Vision gates apply to driving, not to an already accepted grip or cargo. */
    if (step == 10 || step == 20 || step == 30) {
        if (vision.uart_errors != uart_at_start || !vision.connected) {
            task_info.reason = vision.uart_errors != uart_at_start ? TASK_UART_ERROR : TASK_VISION_LOST;
            uart_at_start = vision.uart_errors;
            drive_stop();
            return;
        }
        if (hazard_visible()) { drive_stop(); task_info.reason = TASK_HAZARD; return; }
        if (step == 30 && (lower_fault || !observation_fresh(&vision.lower))) {
            drive_stop(); task_info.reason = lower_fault ? lower_fault : TASK_LOWER_WAIT;
            return;
        }
    }
    switch (step) {
        case 0: drive_stop(); break;
        case 1: task_prepare(); enter_step(10); break;
        case 10: if (task_search() == TASK_DONE) enter_step(30); break;
        case 20: enter_step(30); break; /* Retired standalone alignment step. */
        case 30: task_approach(); break;
        case 40:
            if (task_grip() == TASK_DONE) { task_info.reason = TASK_OK; enter_step(99); }
            break;
        case 50:
            switch (task_goto(goto_target.x, goto_target.y, goto_target.speed)) {
                case TASK_DONE: step = 51; break;
                case TASK_FAILED: task_info.reason = TASK_BAD_STEP; step = 90; break;
                default: break;
            }
            break;
        case 51: drive_stop(); break; /* Coordinate reached; retain target for display. */
        case 90: drive_stop(); break;
        case 99: drive_stop(); break; /* Hold until an explicit release (KEY4). */
        default: task_stop(); task_info.reason = TASK_BAD_STEP; break;
    }
}
