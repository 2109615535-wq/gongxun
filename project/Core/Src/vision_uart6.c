#include "vision_uart6.h"
#include "vision_simple.h"
#include "OLED.h"

#include <stdbool.h>
#include <math.h>

#define VISION_RX_SIZE 2048U
#define VISION_RX_MASK (VISION_RX_SIZE - 1U)

extern UART_HandleTypeDef huart6;

typedef struct {
    uint32_t received_ms;
    uint8_t byte;
} VisionRxByte;

static VisionRxByte rx_buffer[VISION_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_fault;
static volatile bool rx_armed;
static bool opened;
static uint8_t rx_byte;
static VSP_Receiver receiver;

volatile uint32_t g_vision_uart6_rx_bytes;
volatile uint32_t g_vision_uart6_overflows;
volatile uint32_t g_vision_uart6_errors;

static void arm_receive(void)
{
    rx_armed = HAL_UART_Receive_IT(&huart6, &rx_byte, 1U) == HAL_OK;
    if (!rx_armed) {
        rx_fault = true;
        g_vision_uart6_errors++;
    }
}

void vision_uart6_init(void)
{
    VSP_Init(&receiver);
    rx_head = rx_tail = 0U;
    rx_fault = false;
    rx_armed = false;
    g_vision_uart6_rx_bytes = 0U;
    g_vision_uart6_overflows = 0U;
    g_vision_uart6_errors = 0U;
    opened = true;
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    arm_receive();
}

void vision_uart6_rx_complete(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != USART6 || !opened) {
        return;
    }
    rx_armed = false;
    g_vision_uart6_rx_bytes++;
    next = (uint16_t)((rx_head + 1U) & VISION_RX_MASK);
    if (next == rx_tail) {
        rx_fault = true;
        g_vision_uart6_overflows++;
    } else if (!rx_fault) {
        rx_buffer[rx_head].byte = rx_byte;
        rx_buffer[rx_head].received_ms = HAL_GetTick();
        __DMB();
        rx_head = next;
    }
    arm_receive();
}

void vision_uart6_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART6 || !opened) {
        return;
    }
    rx_fault = true;
    rx_armed = false;
    g_vision_uart6_errors++;
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    /* The scheduler retries reception after the HAL IRQ handler has returned. */
}

static void invalidate_observations(void)
{
    unsigned source;

    receiver.length = 0U;
    receiver.dropping = 1U; /* Resume at the next complete line boundary. */
    for (source = 0U; source < 2U; source++) {
        receiver.cameras[source].available = 0U;
        receiver.cameras[source].observation.status = VSP_FAULT;
        receiver.cameras[source].observation.count = 0U;
        receiver.cameras[source].observation.age_ms = -1;
    }
}

void vision_uart6_process(void)
{
    unsigned budget = VISION_RX_SIZE;

    while (budget-- > 0U) {
        VisionRxByte sample;
        uint32_t interrupt_state = __get_PRIMASK();
        bool fault;
        bool available;

        __disable_irq();
        fault = rx_fault;
        if (fault) {
            rx_tail = rx_head;
            rx_fault = false;
        }
        if (!rx_armed && opened) {
            arm_receive();
        }
        available = rx_tail != rx_head;
        if (available) {
            sample = rx_buffer[rx_tail];
            rx_tail = (uint16_t)((rx_tail + 1U) & VISION_RX_MASK);
        }
        if (interrupt_state == 0U) {
            __enable_irq();
        }
        if (fault) {
            invalidate_observations();
        }
        if (!available) {
            break;
        }
        VSP_Feed(&receiver, sample.byte, sample.received_ms);
    }
}

static const char *status_name(VSP_Status status)
{
    static const char *const names[] = {
        "OK", "EMPTY", "STALE", "FAULT", "OVERFLOW", "NO DATA"
    };
    return (unsigned)status < sizeof(names) / sizeof(names[0]) ? names[status] : "FAULT";
}

static void show_camera(VSP_Source source, int16_t y, uint32_t now)
{
    VSP_Observation observation;
    const VSP_Object *object;
    const char *label = source == VSP_UPPER ? "UP" : "LOW";
    uint8_t green_indices[VSP_MAX_OBJECTS];
    unsigned green_count = 0U;
    unsigned index;

    (void)VSP_Get(&receiver, source, now, &observation);
    if (rx_fault || !rx_armed) {
        observation.status = VSP_FAULT;
        observation.count = 0U;
    }
    for (index = 0U; index < observation.count; index++) {
        if (observation.objects[index].kind == VSP_GREEN) {
            green_indices[green_count++] = (uint8_t)index;
        }
    }
    if (observation.status != VSP_OK || green_count == 0U) {
        const char *status = (observation.status == VSP_OK || observation.status == VSP_EMPTY)
                             ? "NO GREEN" : status_name(observation.status);
        OLED_Printf1(0, y, "%s %s", label, status);
        OLED_Printf1(0, y + 8, "X:NA Y:NA");
        OLED_Printf1(0, y + 16, "A:NA");
        OLED_Printf1(0, y + 24, "D:NA");
        return;
    }
    index = (now / 2000U) % green_count;
    object = &observation.objects[green_indices[index]];
    OLED_Printf1(0, y, "%s GREEN %u/%u mm", label, index + 1U, green_count);
    if (object->distance_mm >= 0 && (object->angle_present || object->distance_mm == 0)) {
        float radians = object->angle * (3.14159265358979323846f / 180.0f);
        float x = (float)object->distance_mm * sinf(radians);
        float forward = (float)object->distance_mm * cosf(radians);
        long x_mm = (long)(x + (x < 0.0f ? -0.5f : 0.5f));
        long y_mm = (long)(forward + (forward < 0.0f ? -0.5f : 0.5f));
        const char *estimate = object->distance_valid && object->angle_valid ? "" : "~";
        OLED_Printf1(0, y + 8, "X:%s%+ld Y:%s%+ld", estimate, x_mm, estimate, y_mm);
    } else {
        OLED_Printf1(0, y + 8, "X:NA Y:NA");
    }
    if (object->angle_present) {
        long angle = (long)(object->angle * 100.0f + (object->angle < 0.0f ? -0.5f : 0.5f));
        char sign = angle < 0 ? '-' : '+';
        long magnitude = angle < 0 ? -angle : angle;
        OLED_Printf1(0, y + 16, "A:%s%c%ld.%02lddeg", object->angle_valid ? "" : "~",
                     sign, magnitude / 100L, magnitude % 100L);
    } else {
        OLED_Printf1(0, y + 16, "A:NA");
    }
    if (object->distance_mm >= 0) {
        OLED_Printf1(0, y + 24, "D:%s%ldmm", object->distance_valid ? "" : "~",
                     (long)object->distance_mm);
    } else {
        OLED_Printf1(0, y + 24, "D:NA");
    }
}

void vision_uart6_oled_task(void)
{
    uint32_t now = HAL_GetTick();

    OLED_ClearArea(0, 0, 128, 64);
    show_camera(VSP_UPPER, 0, now);
    show_camera(VSP_LOWER, 32, now);
    OLED_Update();
}
