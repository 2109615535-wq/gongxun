#ifndef __GMR_H
#define __GMR_H

#include "main.h"

/* 编码器编号与电机编号对应，定时器保持连续计数。 */
#define GMR_1 1 /* PD12/PD13 -> TIM4，16位 */
#define GMR_2 2 /* PA15/PB3  -> TIM2，32位 */
#define GMR_3 3 /* PB4/PB5   -> TIM3，16位 */
#define GMR_4 4 /* PA0/PA1   -> TIM5，32位 */

#define GMR_TICK_MS   10U  /* SysTick中每10ms读取一次硬件计数 */
#define GMR_SAMPLE_MS 100U /* 每100ms发布一次累加结果 */

/* 按实际电机填写：A相每圈脉冲数、减速比、四倍频计数。 */
#define GMR_PPR           31U
#define GMR_GEAR_RATIO    20U
#define GMR_COUNT_PER_REV (GMR_PPR * 4U * GMR_GEAR_RATIO)

/* 在MX_TIM2_Init~MX_TIM5_Init之后调用。初始化之前滴答调用安全返回。 */
void GMR_init(void);

/* 仅由1ms的SysTick中断调用，不在主循环调用。
 * 16位计数器两次实际采样之间的净计数绝对值必须小于32768；
 * 32位计数器必须小于2147483648。长时间屏蔽中断仍可能丢失回绕信息。
 */
void GMR_tick_1ms(void);

/* 返回最近100ms的有符号脉冲总数，int32_t；读取不会清零或触发采样。
 * 初始化后第一个100ms完成前返回0；编号无效返回0。
 * 结果超出int32_t范围时饱和，不发生软件整数回绕。
 */
int32_t GMR_get_count(uint8_t encoder_num);

/* 返回最近发布周期的输出轴转速，int32_t，单位RPM（转/分钟）。
 * 注意：原接口单位为浮点圈/秒，现在改为整数RPM，小数向零截断。
 * 使用实际毫秒时长和64位中间运算；结果饱和到int32_t范围。
 */
int32_t GMR_get_speed(uint8_t encoder_num);

#endif
