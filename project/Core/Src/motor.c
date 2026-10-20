#include "motor.h"

/*main.c 里定义的定时器句柄，本文件要用*/
extern TIM_HandleTypeDef htim1;

/*PWM 参数：定时器时钟 168MHz，计数 16800 次为一个周期，即 10kHz
  * 改这里的数值就能改 PWM 频率：周期计数 = 168000000 ÷ 频率
  *   20kHz -> 8400    10kHz -> 16800    5kHz -> 33600
  */
#define MOTOR_PWM_PERIOD		16800U

/*每个电机对应的 PWM 通道（注意：和引脚顺序是反的）*/
static const uint32_t motor_pwm_channel[4] =
{
	TIM_CHANNEL_4,	/*电机1 -> PA11*/
	TIM_CHANNEL_3,	/*电机2 -> PA10*/
	TIM_CHANNEL_2,	/*电机3 -> PA9 */
	TIM_CHANNEL_1	/*电机4 -> PA8 */
};

/*每个电机对应的方向引脚（PH）*/
static GPIO_TypeDef * const motor_ph_port[4] =
{
	motor_ph1_GPIO_Port, motor_ph2_GPIO_Port, motor_ph3_GPIO_Port, motor_ph4_GPIO_Port
};
static const uint16_t motor_ph_pin[4] =
{
	motor_ph1_Pin, motor_ph2_Pin, motor_ph3_Pin, motor_ph4_Pin
};

/*每个电机对应的使能引脚（EN）*/
static GPIO_TypeDef * const motor_en_port[4] =
{
	motor_en1_GPIO_Port, motor_en2_GPIO_Port, motor_en3_GPIO_Port, motor_en4_GPIO_Port
};
static const uint16_t motor_en_pin[4] =
{
	motor_en1_Pin, motor_en2_Pin, motor_en3_Pin, motor_en4_Pin
};

/**
  * 函    数：电机初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：本函数只做"设置引脚初始状态 + 启动 PWM 输出"，
  *           外设本身（TIM1、GPIO 的模式和时钟）已经在 CubeMX 生成的
  *           MX_TIM1_Init() 和 MX_GPIO_Init() 里配置好了，
  *           这里不再重复初始化，上电后调用一次即可
  */
void motor_init(void)
{
	uint8_t i;
	
	/*按上面定义的频率设置自动重装值（ARR = 周期计数 - 1）*/
	__HAL_TIM_SET_AUTORELOAD(&htim1, MOTOR_PWM_PERIOD - 1U);

	/*先把占空比清零、方向设为正转，避免启动瞬间电机突然冲一下*/
	for (i = 0; i < 4; i ++)
	{
		motor_set_speed(i + 1, 0, 1);
	}
	
	/*使能引脚拉高（高电平使能）*/
	for (i = 0; i < 4; i ++)
	{
		HAL_GPIO_WritePin(motor_en_port[i], motor_en_pin[i], GPIO_PIN_SET);
	}
	
	/*四个通道分别启动 PWM 输出（HAL 不支持一次启动多个通道）*/
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

/**
  * 函    数：设置电机的速度和方向
  * 参    数：motor_num 电机编号，范围：1~4
  * 参    数：pwm 速度，范围：0~100（百分比），超过 100 按 100 处理
  * 参    数：dir 方向，0 为反转，1 为正转
  * 返 回 值：无
  * 说    明：先根据 dir 设置方向引脚，再把速度换算成定时器比较值，调用后立刻生效
  */
void motor_set_speed(uint8_t motor_num, uint8_t pwm, uint8_t dir)
{
	uint32_t compare;
	
	/*编号超出范围，直接返回，不做任何操作*/
	if (motor_num < MOTOR_1 || motor_num > MOTOR_4)
	{
		return;
	}
	
	/*限幅，避免超过 100%*/
	if (pwm > 100)
	{
		pwm = 100;
	}
	
	/*设置方向：0 反转（方向引脚拉低），非 0 按正转处理（方向引脚拉高）*/
	if (dir == 0)
	{
		HAL_GPIO_WritePin(motor_ph_port[motor_num - 1], motor_ph_pin[motor_num - 1], GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(motor_ph_port[motor_num - 1], motor_ph_pin[motor_num - 1], GPIO_PIN_SET);
	}
	
	/*百分比换算成比较值：8400 × pwm ÷ 100*/
	compare = (MOTOR_PWM_PERIOD * pwm) / 100U;
	
	/*写入对应通道的比较寄存器*/
	__HAL_TIM_SET_COMPARE(&htim1, motor_pwm_channel[motor_num - 1], compare);
}
