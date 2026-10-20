#include "GMR.h"
#include <limits.h>

#define GMR_USE_INTERNAL_PULLUP 1

#if (GMR_TICK_MS == 0) || (GMR_SAMPLE_MS < GMR_TICK_MS) || (GMR_SAMPLE_MS % GMR_TICK_MS != 0)
#error Invalid_GMR_sampling_period
#endif

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;

static TIM_HandleTypeDef * const GMR_timer[4] = {&htim4, &htim2, &htim3, &htim5};
static const uint8_t GMR_is_32bit[4] = {0, 1, 0, 1};
/* 正方向按机械安装标定；不能用方向系数掩盖计数溢出。 */
static const int8_t GMR_dir_sign[4] = {1, 1, 1, 1};

#if GMR_USE_INTERNAL_PULLUP
/* 2号编码器A/B相跨端口，必须分别设置PA15和PB3。 */
static GPIO_TypeDef * const GMR_port_a[4] = {GPIOD, GPIOA, GPIOB, GPIOA};
static GPIO_TypeDef * const GMR_port_b[4] = {GPIOD, GPIOB, GPIOB, GPIOA};
static const uint8_t GMR_pin_a[4] = {12, 15, 4, 0};
static const uint8_t GMR_pin_b[4] = {13, 3, 5, 1};
#endif

/* 工作状态仅由SysTick使用；初始化时关中断，防止采到半初始化状态。 */
static uint32_t GMR_previous[4];
static int32_t GMR_sum[4];
static uint32_t GMR_tick_divider;
static uint32_t GMR_window_ms;
static uint32_t GMR_window_start;
static volatile uint32_t GMR_ready;
/* SysTick发布，主循环读取；Cortex-M4对齐32位读取是原子的。 */
static volatile int32_t GMR_last_count[4];
static volatile uint32_t GMR_last_span;

static int32_t GMR_clamp_int32(int64_t value)
{
    if (value > INT32_MAX) return INT32_MAX;
    if (value < INT32_MIN) return INT32_MIN;
    return (int32_t)value;
}

void GMR_init(void)
{
    uint8_t i;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    GMR_ready = 0;
    for (i = 0; i < 4; ++i)
    {
#if GMR_USE_INTERNAL_PULLUP
        uint32_t shift_a = GMR_pin_a[i] * 2U;
        uint32_t shift_b = GMR_pin_b[i] * 2U;
        GMR_port_a[i]->PUPDR = (GMR_port_a[i]->PUPDR & ~(3U << shift_a)) | (1U << shift_a);
        GMR_port_b[i]->PUPDR = (GMR_port_b[i]->PUPDR & ~(3U << shift_b)) | (1U << shift_b);
#endif
        __HAL_TIM_SET_COUNTER(GMR_timer[i], 0);
        if (HAL_TIM_Encoder_Start(GMR_timer[i], TIM_CHANNEL_ALL) != HAL_OK)
        {
            __set_PRIMASK(primask);
            Error_Handler();
            return;
        }
        GMR_sum[i] = 0;
        GMR_last_count[i] = 0;
    }
    for (i = 0; i < 4; ++i)
    {
        GMR_previous[i] = __HAL_TIM_GET_COUNTER(GMR_timer[i]);
    }
    GMR_tick_divider = 0;
    GMR_window_ms = 0;
    GMR_last_span = 0;
    GMR_window_start = HAL_GetTick();
    __DMB();
    GMR_ready = 1;
    __set_PRIMASK(primask);
}

void GMR_tick_1ms(void)
{
    uint8_t i;
    uint32_t now, diff, tick;
    int32_t delta;

    if (GMR_ready == 0) return;
    if (++GMR_tick_divider < GMR_TICK_MS) return;
    GMR_tick_divider = 0;

    for (i = 0; i < 4; ++i)
    {
        now = __HAL_TIM_GET_COUNTER(GMR_timer[i]);
        diff = now - GMR_previous[i]; /* 无符号减法允许硬件计数器回绕 */
        GMR_previous[i] = now;
        if (GMR_is_32bit[i] == 0)
        {
            diff &= 0xffffU;
            delta = (diff < 0x8000U) ? (int32_t)diff : (int32_t)diff - 65536;
        }
        else
        {
            /* 显式解码32位补码，避免超范围无符号到有符号转换。 */
            delta = (diff <= INT32_MAX) ? (int32_t)diff : -1 - (int32_t)(UINT32_MAX - diff);
        }
        GMR_sum[i] = GMR_clamp_int32((int64_t)GMR_sum[i] + (int64_t)delta * GMR_dir_sign[i]);
    }

    GMR_window_ms += GMR_TICK_MS;
    if (GMR_window_ms >= GMR_SAMPLE_MS)
    {
        tick = HAL_GetTick();
        for (i = 0; i < 4; ++i)
        {
            GMR_last_count[i] = GMR_sum[i];
            GMR_sum[i] = 0;
        }
        GMR_last_span = tick - GMR_window_start;
        GMR_window_start = tick;
        GMR_window_ms = 0;
    }
}

int32_t GMR_get_count(uint8_t encoder_num)
{
    if (encoder_num < GMR_1 || encoder_num > GMR_4) return 0;
    return GMR_last_count[encoder_num - 1];
}

int32_t GMR_get_speed(uint8_t encoder_num)
{
    int32_t count;
    uint32_t span, primask;
    if (encoder_num < GMR_1 || encoder_num > GMR_4) return 0;

    /* 一起读取计数和时长，防止SysTick在两次读取之间发布新周期。 */
    primask = __get_PRIMASK();
    __disable_irq();
    count = GMR_last_count[encoder_num - 1];
    span = GMR_last_span;
    __set_PRIMASK(primask);
    if (span == 0) return 0;

    /* 整数RPM，小数向零截断；乘法先扩展到64位，避免中间结果溢出。 */
    return GMR_clamp_int32((int64_t)count * 60000 / ((int64_t)GMR_COUNT_PER_REV * span));
}
