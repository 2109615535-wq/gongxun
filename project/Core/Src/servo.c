#include "servo.h"

/*main.c 里定义的定时器句柄，本文件要用*/
extern TIM_HandleTypeDef htim9;

/*每个舵机对应的定时器通道*/
static const uint32_t servo_channel[2] =
{
	TIM_CHANNEL_1,	/*舵机1 -> PE5*/
	TIM_CHANNEL_2	/*舵机2 -> PE6*/
};

/**
  * 函    数：舵机初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：本函数只做"设置初始角度 + 启动 PWM 输出"，
  *           外设本身（TIM9、GPIO 的模式和时钟）已经在 CubeMX 生成的
  *           MX_TIM9_Init() 和 MX_GPIO_Init() 里配置好了，
  *           这里不再重复初始化，上电后调用一次即可
  */
void servo_init(void)
{
	/*上电先让两个舵机都停在中间位置（90 度），再启动输出*/
	__HAL_TIM_SET_COMPARE(&htim9, servo_channel[0], (SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2U);
	__HAL_TIM_SET_COMPARE(&htim9, servo_channel[1], (SERVO_MIN_PULSE + SERVO_MAX_PULSE) / 2U);
	
	/*两个通道分别启动 PWM 输出（HAL 不支持一次启动多个通道）*/
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
}

/**
  * 函    数：设置舵机角度
  * 参    数：servo_num 舵机编号，范围：1~2
  * 参    数：angle 角度，范围：0~180（度），超过 180 按 180 处理
  * 返 回 值：无
  * 说    明：角度线性映射到脉宽，调用后立刻生效
  */
void servo_set_angle(uint8_t servo_num, uint8_t angle)
{
	uint32_t pulse;
	
	/*编号超出范围，直接返回，不做任何操作*/
	if (servo_num < SERVO_1 || servo_num > SERVO_2)
	{
		return;
	}
	
	/*限幅，避免超过 180 度*/
	if (angle > 180)
	{
		angle = 180;
	}
	
	/*角度换算成脉宽：500 + 2000 × angle ÷ 180，单位微秒*/
	pulse = SERVO_MIN_PULSE + ((SERVO_MAX_PULSE - SERVO_MIN_PULSE) * angle) / 180U;
	
	/*写入对应通道的比较寄存器（TIM9 每个计数正好是 1 微秒）*/
	__HAL_TIM_SET_COMPARE(&htim9, servo_channel[servo_num - 1], pulse);
}
