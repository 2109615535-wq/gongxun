#ifndef __SERVO_H
#define __SERVO_H

#include "main.h"

/*舵机编号，调用时直接传 1~2*/
#define SERVO_1				1		/*对应 PE5（TIM9_CH1）*/
#define SERVO_2				2		/*对应 PE6（TIM9_CH2）*/

/*脉宽范围（单位：微秒）
  * 0 度对应 SERVO_MIN_PULSE，180 度对应 SERVO_MAX_PULSE
  * 不同品牌舵机的实际行程略有差异，如果转到头会"滋滋"响，
  * 把这两个值往中间收一点（比如 600 和 2400）即可
  */
#define SERVO_MIN_PULSE		500U
#define SERVO_MAX_PULSE		2500U

/*舵机初始化，在上电后调用一次即可*/
void servo_init(void);

/*设置舵机角度
  *   servo_num：舵机编号，1~2
  *   angle    ：角度，0~180（度），超过 180 按 180 处理
  */
void servo_set_angle(uint8_t servo_num, uint8_t angle);

#endif
