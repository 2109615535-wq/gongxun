#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/*电机编号，调用时直接传 1~4*/
#define MOTOR_1				1
#define MOTOR_2				2
#define MOTOR_3				3
#define MOTOR_4				4

/*电机与引脚的对应关系（依据板子引脚表）
  *   电机1：PWM -> PA11（TIM1_CH4）  方向 -> PC0（MOTOR_PH1）  使能 -> PE13（MOTOR_EN1）
  *   电机2：PWM -> PA10（TIM1_CH3）  方向 -> PC1（MOTOR_PH2）  使能 -> PE11（MOTOR_EN2）
  *   电机3：PWM -> PA9 （TIM1_CH2）  方向 -> PC2（MOTOR_PH3）  使能 -> PE9 （MOTOR_EN3）
  *   电机4：PWM -> PA8 （TIM1_CH1）  方向 -> PC3（MOTOR_PH4）  使能 -> PE7 （MOTOR_EN4）
  */

/*电机相关初始化，在上电后调用一次即可*/
void motor_init(void);

/*设置电机速度和方向
  *   motor_num：电机编号，1~4
  *   pwm      ：速度，0~100（百分比），超过 100 按 100 处理
  *   dir      ：方向，0 反转，1 正转
  */
void motor_set_speed(uint8_t motor_num, uint8_t pwm, uint8_t dir);

#endif
