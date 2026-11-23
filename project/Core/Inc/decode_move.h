#ifndef DECODE_MOVE_H
#define DECODE_MOVE_H

#include <stdint.h>
#include "pid.h"

#ifndef RETURN_TEAM
#define RETURN_TEAM VISION_TEAM_BLUE
#endif

/* 长度单位 cm；轮序：左前2、右前1、左后3、右后4。
 * 车体系：右为+x，前为+y；航向角从初始朝向起，顺时针为正。
 */
#define wheel_X_length 23.6f    /* 左右轮中心的完整间距，需核实实车尺寸 */
#define wheel_Y_length 8.8f    /* 前后轮中心的完整间距，需核实实车尺寸 */
#define wheel_diameter 6.0f     /* 轮子直径 cm */



/* 相对初始化时起点的固定坐标系位置；单位cm。 */
extern float x_body;
extern float y_body;
/* 小车的朝向角度 */
extern float angle_body;  /* rad, clockwise positive, relative to the last reset. */
/* 四轮从里程计初始化起的累计有符号滚动距离，单位cm。 */
extern float motordistance1;
extern float motordistance2;
extern float motordistance3;
extern float motordistance4;

/* 在encoder_init之后调用；也可重置里程计原点，不清零编码器。 */
void position_encoder_init(void);
/* 主循环任务中，在本次encoder_process之后调用。 */
void position_encoder(void);
typedef struct {
    float target_forward, target_right, target_turn;
    float forward, right, turn; /* Applied commands, NOT measured wheel feedback. */
    uint32_t updated_ms;
    uint8_t active;
} BodyMotion;
extern BodyMotion body_motion;
/* Retained body target: forward/right cm/s, clockwise turn deg/s.
 * motor_process() applies wheel targets directly; no body speed/yaw cap. */
void body_move(float vy, float vx, float vtheta);
void body_move_process(void);
void body_stop(void); /* Zero target next motor cycle; does not enable the drive. */
void body_move_cancel(void); /* Cancel retained body target ownership. */
/* Motor-layer callback: report applied wheel commands (cm/s), not feedback. */
void body_wheel_applied(float w1, float w2, float w3, float w4);
/* Software-ramp braking distance; zero while acceleration limiting is disabled. */
float body_brake_distance(void);

/* 任务步骤：0 空闲，1 开始，90 失败，99 结束；在主循环中修改。 */
extern uint8_t step;
typedef enum { TASK_RUNNING, TASK_DONE, TASK_FAILED } TaskResult;
#define GOTO_POSITION_TOLERANCE_CM 2.0f
/* Absolute field target (cm): share pursuit steering and
 * forward command, with one alignment to +/-10deg within 25cm per target.
 * No ramp or endpoint slowdown; arrival within 2 cm latches
 * stop. Does not grab/release cargo or require a camera observation. */
TaskResult task_goto(float x_cm, float y_cm, float speed_cm_s);
typedef struct { float x, y, speed; uint8_t valid; } GotoTarget;
extern GotoTarget goto_target;
uint8_t task_goto_start(float x_cm, float y_cm, float speed_cm_s);
typedef enum {
    TASK_OK, TASK_STOPPED, TASK_VISION_LOST, TASK_POSITION_INVALID,
    TASK_TARGET_LOST, TASK_AMBIGUOUS, TASK_HAZARD, TASK_SEARCH_TIMEOUT,
    TASK_ALIGN_TIMEOUT, TASK_APPROACH_TIMEOUT, TASK_TRAVEL_LIMIT,
    TASK_ROI_MISSING, TASK_GRIP_LOST, TASK_GRIP_TIMEOUT, TASK_BAD_STEP,
    TASK_UART_ERROR, TASK_UPPER_WAIT, TASK_LOWER_WAIT,
    TASK_RETURN_POSE_WAIT, TASK_RETURN_ENTRY_WAIT, TASK_RETURN_X_WAIT,
    TASK_RETURN_X_NO_ZONE, TASK_RETURN_X_ALIGN, TASK_RETURN_X_SPREAD, TASK_RETURN_X_SKIPPED
} TaskReason;
typedef struct {
    uint8_t reason, inside_frames, outside_frames;
    uint8_t held_kind; /* VisionKind: NONE=empty, GREEN=ordinary block held; cleared on release. */
} TaskInfo;
extern TaskInfo task_info;
typedef enum {
    TASK_TARGET_AVAILABLE, TASK_TARGET_LOCKED, TASK_TARGET_CARRYING,
    TASK_TARGET_DELIVERED, TASK_TARGET_REVIEW
} TaskTargetState;
typedef enum {
    TARGET_MAPPED, TARGET_SELECTED, TARGET_REPLACED, TARGET_GRIP_VERIFIED,
    TARGET_GRIP_FAILED, TARGET_INTERRUPTED, TARGET_DUPLICATE,
    TARGET_RECHECKED, TARGET_RELEASE_PENDING, TARGET_RELEASE_UNVERIFIED,
    TARGET_DELIVERY_VERIFIED
} TaskTargetReason;
typedef struct {
    uint32_t epoch, track_id, related_id;
    uint32_t evidence_frame, evidence_tick;
    float x, y;
    TaskTargetState state;
    TaskTargetReason reason;
    uint8_t visible, in_safety_zone, candidate;
} TaskTarget;
uint8_t task_target_get(uint32_t epoch, uint32_t track_id, TaskTarget *out);
/* source: 0 stopped/no target, 1 visual target, 2 field coordinate target. */
typedef struct { uint32_t tick; uint8_t source; float p, i, d; } SteerState;
extern PID_t pid_steer;
extern SteerState steer_state;
/* Runtime RAM gains. Resets angle history, does not start/stop the drive. */
uint8_t steering_set_pid(float kp, float ki, float kd);
void task_start(void);
uint8_t task_start_hui_ku(uint8_t source, uint8_t blocked);
void task_stop(void); /* Normal stop and release. */
void task_emergency_stop(void); /* KEY4 immediate stop and release. */
void task_prepare(void);
TaskResult task_search(void);

TaskResult task_approach(void);
TaskResult task_grip(void);
void task_observe_lower(void); /* Main-loop observation notification; no actuator writes. */
void move(void);

#endif /* DECODE_MOVE_H */
