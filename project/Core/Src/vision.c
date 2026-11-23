#include "vision.h"
#include "vision_simple.h"
#include <string.h>
#include "decode_move.h"

/* Simple observations on USART6. The link reports connection and frame freshness. */
#define RX_SIZE 2048U
#define RX_MASK (RX_SIZE - 1U)
extern UART_HandleTypeDef huart6;
VisionResult vision;
static VSP_Receiver receiver;
static uint8_t rx_byte, rx_data[RX_SIZE];
static uint32_t rx_time[RX_SIZE];
static volatile uint16_t rx_head, rx_tail;
static volatile uint8_t rx_fault;
static uint8_t opened, seen_packet;
static uint32_t last_packet_ms;

static VisionKind convert_kind(VSP_Kind kind)
{
    static const VisionKind map[] = {
        VISION_NONE, VISION_GREEN, VISION_ORANGE, VISION_BLUE, VISION_BLACK,
        VISION_RED_CASUALTY_ZONE, VISION_RED_MATERIAL_ZONE,
        VISION_BLUE_CASUALTY_ZONE, VISION_BLUE_MATERIAL_ZONE
    };
    return (unsigned)kind < sizeof(map)/sizeof(map[0]) ? map[kind] : VISION_UNKNOWN;
}

static void update_camera(VSP_Source source, VisionObservation *out, uint32_t now)
{
    VSP_Observation observation;
    unsigned i;
    uint8_t fresh = VSP_Get(&receiver, source, now, &observation);
    memset(out, 0, sizeof(*out));
    out->available = out->fresh = fresh;
    out->frame_id = observation.frame_id;
    out->updated_ms = now;
    out->age_ms = observation.age_ms < 0 ? UINT32_MAX : (uint32_t)observation.age_ms;
    out->status = observation.status == VSP_NO_DATA ? VISION_OBS_FAULT :
                  (VisionObservationStatus)observation.status;
    out->count = fresh ? observation.count : 0;
    for (i = 0; i < out->count; ++i) {
        const VSP_Object *input = &observation.objects[i];
        VisionObject *object = &out->objects[i];
        object->target_id = i + 1U; /* Frame-local display index, NOT tracking. */
        object->kind = convert_kind(input->kind);
        object->distance_mm = input->distance_mm;
        object->distance_valid = input->distance_valid;
        object->angle = input->angle;
        object->angle_present = input->angle_present;
        object->angle_valid = input->angle_valid;
        object->in_roi = input->in_roi;
        object->in_safety_zone = input->in_safety_zone;
    }
}


bool vision_init(void)
{
    if (opened) return true;
    if (huart6.Instance != USART6) return false;
    memset(&vision, 0, sizeof(vision));
    VSP_Init(&receiver);
    rx_head = rx_tail = 0;
    rx_fault = seen_packet = 0;
    vision.danger_flag = 1; /* Simple observations supply per-object results. */
    vision.upper.age_ms = vision.lower.age_ms = UINT32_MAX;
    strcpy(vision.fault_code, "waiting_rv2");
    opened = 1;
    __HAL_UART_CLEAR_OREFLAG(&huart6);
    if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK) {
        opened = 0;
        return false;
    }
    return true;
}

bool vision_set_task(const VisionTask *task)
{
    (void)task;
    return false; /* Simple protocols send observations only. */
}
void vision_set_yaw(float degrees, bool valid) { (void)degrees; (void)valid; }
void vision_uart_tx_complete(UART_HandleTypeDef *huart) { (void)huart; }

void vision_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint16_t next;
    if (huart->Instance != USART6 || !opened) return;
    next = (uint16_t)((rx_head + 1U) & RX_MASK);
    if (next == rx_tail) rx_fault = 1;
    else if (!rx_fault) {
        rx_data[rx_head] = rx_byte;
        rx_time[rx_head] = HAL_GetTick();
        __DMB(); rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1) != HAL_OK) rx_fault = 1;
}
void vision_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6 && opened) rx_fault = 1;
}

void vision_process(void)
{
    unsigned budget = RX_SIZE;
    uint32_t now;
    if (!opened) return;
    while (budget--) {
        uint32_t irq = __get_PRIMASK(), time = 0, errors;
        uint8_t byte = 0, have_byte, fault, complete;
        __disable_irq();
        fault = rx_fault;
        if (fault) {
            (void)HAL_UART_AbortReceive(&huart6);
            rx_tail = rx_head; rx_fault = 0;
            __HAL_UART_CLEAR_OREFLAG(&huart6);
            if (HAL_UART_Receive_IT(&huart6, &rx_byte, 1) != HAL_OK) rx_fault = 1;
        }
        have_byte = rx_tail != rx_head;
        if (have_byte) {
            byte = rx_data[rx_tail]; time = rx_time[rx_tail];
            rx_tail = (uint16_t)((rx_tail + 1U) & RX_MASK);
        }
        __set_PRIMASK(irq);
        if (fault) {
            unsigned source;
            ++vision.uart_errors;
            receiver.length = 0; receiver.dropping = 1;
            for (source = 0; source < 2; ++source) {
                receiver.cameras[source].available = 0;
                receiver.cameras[source].observation.status = VSP_FAULT;
                receiver.cameras[source].observation.count = 0;
                receiver.cameras[source].observation.age_ms = -1;
            }
            seen_packet = 0;
            strcpy(vision.fault_code, "uart_error");
        }
        if (!have_byte) break;
        complete = byte == '\n' && !receiver.dropping && receiver.length > 0;
        errors = receiver.format_errors;
        VSP_Feed(&receiver, byte, time);
        if (receiver.format_errors != errors) strcpy(vision.fault_code, "invalid_packet");
        else if (complete) {
            ++vision.rx_packets; seen_packet = 1; last_packet_ms = time;
            strcpy(vision.fault_code, "rv2_observation");
            /* Process each received observation, including multiple lines in one
             * queue drain. Otherwise an intervening outside frame could be lost. */
            vision.connected = 1;
            update_camera(VSP_LOWER, &vision.lower, HAL_GetTick());
            task_observe_lower();
        }
    }
    now = HAL_GetTick();
    vision.bad_packets = receiver.format_errors;
    vision.ignored_packets = receiver.duplicates;
    vision.connected = seen_packet && (uint32_t)(now - last_packet_ms) < 1000U;
    update_camera(VSP_UPPER, &vision.upper, now);
    update_camera(VSP_LOWER, &vision.lower, now);
    if (rx_fault) {
        vision.connected = 0;
        vision.upper.fresh = vision.upper.available = vision.upper.count = 0;
        vision.lower.fresh = vision.lower.available = vision.lower.count = 0;
    }
    task_observe_lower();
    if (seen_packet && !vision.connected) strcpy(vision.fault_code, "vision_timeout");
    else if (!strcmp(vision.fault_code, "rv2_observation") && !vision.upper.fresh)
        strcpy(vision.fault_code, "upper_unavailable");
}
