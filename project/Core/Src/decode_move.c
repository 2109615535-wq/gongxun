#include "decode_move.h"
#include "vision.h"
#include "encoder.h"
#include "motor.h"
#include <math.h>
#include <float.h>
#include "pid.h"
#include "servo.h"
#include "key.h"

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
}

void position_encoder(void)
{
    float dis1,dis2,dis3,dis4;
    float dx,dy,dtheta;
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

    dtheta = (-dis1 + dis2 + dis3 - dis4) / (2.0f * (wheel_X_length + wheel_Y_length));
    dtheta *= 1.020435f;

    mid_angle = angle_body + dtheta / 2.0f;

    //y_body = ( motordistance1 + motordistance2 + motordistance3 + motordistance4) / 4.0f * 1.028666f;
    //x_body = (-motordistance1 + motordistance2 - motordistance3 + motordistance4) / 4.0f * 0.952092f;

    x_body += dx * cosf(mid_angle) + dy * sinf(mid_angle);
    y_body += -dx * sinf(mid_angle) + dy * cosf(mid_angle);

    angle_body += dtheta;
}




void body_move(float vy, float vx, float vtheta)
{
    float v1,v2,v3,v4,peak,scale;

    vx /= 0.952092f;
    vy /= 1.028666f;

    v1 = vy - vx - (wheel_X_length + wheel_Y_length) / 2.0f * vtheta * 3.1415926f / 180.0f;
    v2 = vy + vx + (wheel_X_length + wheel_Y_length) / 2.0f * vtheta * 3.1415926f / 180.0f;
    v3 = vy - vx + (wheel_X_length + wheel_Y_length) / 2.0f * vtheta * 3.1415926f / 180.0f;
    v4 = vy + vx - (wheel_X_length + wheel_Y_length) / 2.0f * vtheta * 3.1415926f / 180.0f;

    v1 *= 40810.0f * 1000.0f / (3.1415926f * wheel_diameter * (float)ENCODER_SPEED_AT_PWM1000);
    v2 *= 40810.0f * 1000.0f / (3.1415926f * wheel_diameter * (float)ENCODER_SPEED_AT_PWM1000);
    v3 *= 40810.0f * 1000.0f / (3.1415926f * wheel_diameter * (float)ENCODER_SPEED_AT_PWM1000);
    v4 *= 40810.0f * 1000.0f / (3.1415926f * wheel_diameter * (float)ENCODER_SPEED_AT_PWM1000);

    /* Only the wheel envelope limits a requested body velocity. Preserve ratios. */
    peak = fabsf(v1);
    if (fabsf(v2) > peak) peak = fabsf(v2);
    if (fabsf(v3) > peak) peak = fabsf(v3);
    if (fabsf(v4) > peak) peak = fabsf(v4);
    if (peak > MOTOR_WHEEL_LIMIT) {
        scale = (float)MOTOR_WHEEL_LIMIT / peak;
        v1 *= scale; v2 *= scale; v3 *= scale; v4 *= scale;
    }

    MotorSetSpeed1((int32_t)v1);
    MotorSetSpeed2((int32_t)v2);
    MotorSetSpeed3((int32_t)v3);
    MotorSetSpeed4((int32_t)v4);
}


/* Simple low-speed mission: nearest green -> forward + steer -> grip and hold.
 * Nonblocking tasks; missing observations stop motion and wait for recovery. */
#define TASK_FORWARD_CM_S 50.0f
#define TASK_ACCEL_CM_S2 30.0f
#define TASK_STEER_KP 1.35f
#define TASK_STEER_KI 0.1f
#define TASK_STEER_KD 0.0f
#define TASK_ANGLE_DEAD_DEG 3.0f

uint8_t step = 0;
TaskInfo task_info;
static uint32_t lower_frame, uart_at_start, grip_since;
static uint8_t lower_seen, lower_fault, grip_commanded, pending_kind;
static float target_angle;
static PID_t pid_steer;
static uint32_t steer_frame;
static uint8_t steer_seen;
static float drive_forward;
static uint32_t drive_updated_ms;

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
/* User-requested forward acceleration only; no body-speed or turn-rate cap. */
static void drive_stop(void)
{
    Motor_SetDriveEnabled(0);
    drive_forward = 0;
    drive_updated_ms = HAL_GetTick();
    PID_Init(&pid_steer);
    steer_seen = 0;
}
static void drive(float forward, float turn)
{
    uint32_t now = HAL_GetTick();
    float max_change = TASK_ACCEL_CM_S2 * ((float)(now - drive_updated_ms) * 0.001f);
    drive_updated_ms = now;
    if (forward - drive_forward > max_change) drive_forward += max_change;
    else if (drive_forward - forward > max_change) drive_forward -= max_change;
    else drive_forward = forward;
    Motor_SetDriveEnabled(1);
    body_move(drive_forward, 0, turn);
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
    if (step != 0 && step != 90 && step != 99) return;
    drive_stop(); claw(50); task_info.reason = TASK_OK; enter_step(1);
}
void task_stop(void)
{
    drive_stop(); claw(50); step = 0;
    task_info.reason = TASK_STOPPED;
    task_info.inside_frames = task_info.outside_frames = 0;
    lower_fault = 0;
}
void task_prepare(void)
{
    drive_stop(); claw(50);
    lower_fault = grip_commanded = 0;
    lower_seen = 1; lower_frame = vision.lower.frame_id;
    task_info.reason = TASK_OK; task_info.inside_frames = task_info.outside_frames = 0;
    uart_at_start = vision.uart_errors;
    pid_steer.kp = TASK_STEER_KP;
    pid_steer.ki = TASK_STEER_KI;
    pid_steer.kd = TASK_STEER_KD;
    /* Existing PID API requires bounds; do not add a practical body-speed cap. */
    pid_steer.output_max = FLT_MAX;
    pid_steer.error_int_max = FLT_MAX;
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
static float steering_pid(void)
{
    if (fabsf(target_angle) <= TASK_ANGLE_DEAD_DEG) {
        PID_Init(&pid_steer); steer_seen = 0;
        return 0;
    }
    if (!steer_seen || steer_frame != vision.upper.frame_id) {
        pid_steer.target = 0;
        pid_steer.current = -target_angle;
        if (!steer_seen) pid_steer.error_now = target_angle; /* No derivative kick on acquisition. */
        PID_Update(&pid_steer);
        steer_frame = vision.upper.frame_id; steer_seen = 1;
    }
    return pid_steer.output;
}
TaskResult task_approach(void)
{
    float turn;
    if (!select_nearest()) { drive_stop(); return TASK_RUNNING; }
    turn = steering_pid();
    drive(TASK_FORWARD_CM_S, turn);
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
        case 90: drive_stop(); break;
        case 99: drive_stop(); break; /* Hold until an explicit release (KEY4). */
        default: task_stop(); task_info.reason = TASK_BAD_STEP; break;
    }
}
