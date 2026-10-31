#ifndef DECODE_MOVE_H
#define DECODE_MOVE_H

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
extern float head_angle;
/* 四轮从里程计初始化起的累计有符号滚动距离，单位cm。 */
extern float motordistance1;
extern float motordistance2;
extern float motordistance3;
extern float motordistance4;

/* 在encoder_init之后调用；也可重置里程计原点，不清零编码器。 */
void position_encoder_init(void);
/* 主循环任务中，在本次encoder_process之后调用。 */
void position_encoder(void);

#endif /* DECODE_MOVE_H */
