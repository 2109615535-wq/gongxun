#ifndef DECODE_MOVE_H
#define DECODE_MOVE_H

#include <stdint.h>

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
void body_move(float vy, float vx, float vtheta);

/* 任务步骤：0 空闲，1 开始，90 失败，99 结束；在主循环中修改。 */
extern uint8_t step;
typedef enum { TASK_RUNNING, TASK_DONE, TASK_FAILED } TaskResult;
typedef enum {
    TASK_OK, TASK_STOPPED, TASK_VISION_LOST, TASK_POSITION_INVALID,
    TASK_TARGET_LOST, TASK_AMBIGUOUS, TASK_HAZARD, TASK_SEARCH_TIMEOUT,
    TASK_ALIGN_TIMEOUT, TASK_APPROACH_TIMEOUT, TASK_TRAVEL_LIMIT,
    TASK_ROI_MISSING, TASK_GRIP_LOST, TASK_GRIP_TIMEOUT, TASK_BAD_STEP,
    TASK_UART_ERROR, TASK_UPPER_WAIT, TASK_LOWER_WAIT
} TaskReason;
typedef struct {
    uint8_t reason, inside_frames, outside_frames;
    uint8_t held_kind; /* VisionKind: NONE=empty, GREEN=ordinary block held; cleared on release. */
} TaskInfo;
extern TaskInfo task_info;
void task_start(void);
void task_stop(void);
void task_prepare(void);
TaskResult task_search(void);

TaskResult task_approach(void);
TaskResult task_grip(void);
void task_observe_lower(void); /* Main-loop observation notification; no actuator writes. */
void move(void);

#endif /* DECODE_MOVE_H */
