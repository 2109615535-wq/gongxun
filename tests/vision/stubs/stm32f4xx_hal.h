#ifndef TEST_HAL_H
#define TEST_HAL_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
typedef struct { uint32_t CR, SR, DR; } TestRng;
extern TestRng test_rng;
#define RNG (&test_rng)
#define RNG_CR_RNGEN 4U
#define RNG_SR_DRDY 1U
#define RNG_SR_CECS 2U
#define RNG_SR_SECS 4U
#define USART3 ((void *)3)
#define USART6 ((void *)6)
#define UART5 ((void *)5)
typedef struct { uint32_t compare[4]; } TIM_HandleTypeDef;
typedef struct { unsigned unused; } GPIO_TypeDef;
typedef int GPIO_PinState;
#define GPIO_PIN_RESET 0
#define GPIO_PIN_SET 1
#define GPIOC ((GPIO_TypeDef *)3)
#define GPIOD ((GPIO_TypeDef *)4)
#define GPIOE ((GPIO_TypeDef *)5)
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define GPIO_PIN_2 4U
#define GPIO_PIN_3 8U
#define GPIO_PIN_4 16U
#define GPIO_PIN_5 32U
#define GPIO_PIN_6 64U
#define GPIO_PIN_7 128U
#define GPIO_PIN_9 512U
#define GPIO_PIN_11 2048U
#define GPIO_PIN_13 8192U
#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3
#define __HAL_TIM_GET_AUTORELOAD(h) 8399U
#define __HAL_TIM_SET_COMPARE(h,c,p) ((h)->compare[c]=(p))
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t);
void HAL_GPIO_WritePin(GPIO_TypeDef *, uint16_t, GPIO_PinState);
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *, uint32_t);
#define HAL_OK 0
#define HAL_ERROR 1
#define __HAL_RCC_RNG_CLK_ENABLE() ((void)0)
#define __HAL_UART_CLEAR_OREFLAG(h) ((void)(h))
#define __get_PRIMASK() 0U
#define __disable_irq() ((void)0)
#define __set_PRIMASK(x) ((void)(x))
#define __DMB() ((void)0)
uint32_t HAL_GetTick(void);
int HAL_UART_Receive_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_Transmit_IT(UART_HandleTypeDef *, uint8_t *, uint16_t);
int HAL_UART_AbortReceive(UART_HandleTypeDef *);
int HAL_UART_AbortTransmit(UART_HandleTypeDef *);
#endif
