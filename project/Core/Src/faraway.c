#include "faraway.h"
#include "vision.h"
#include "key.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "vision_simple.h"
#include "motor.h"
#include "decode_move.h"

#define FARAWAY_RX_SIZE       512U
#define FARAWAY_RX_MASK       (FARAWAY_RX_SIZE - 1U)
#define FARAWAY_TX_SIZE       4096U
#define FARAWAY_TX_CHUNK 64U
#define FARAWAY_TX_GAP_MS 5U
#define FARAWAY_TX_MASK       (FARAWAY_TX_SIZE - 1U)
#define FARAWAY_LINE_SIZE     96U
#define FARAWAY_PROCESS_LIMIT 128U

extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim1;
extern PID_t pid_motor1;
extern PID_t pid_motor2;
extern PID_t pid_motor3;
extern PID_t pid_motor4;

static uint8_t rx_byte;
static uint8_t rx_buffer[FARAWAY_RX_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile bool rx_fault;
static uint8_t tx_buffer[FARAWAY_TX_SIZE];
static volatile uint16_t tx_head;
static volatile uint16_t tx_tail;
static volatile uint16_t tx_length;
static volatile bool opened;
static volatile uint32_t tx_next_ms;
static char line[FARAWAY_LINE_SIZE];
static uint16_t line_length;
static bool discard_line;

volatile uint32_t g_faraway_rx_overflow_count;
volatile uint32_t g_faraway_uart_error_count;
volatile uint32_t g_faraway_tx_drop_count;

/* Called with interrupts masked, or from the UART ISR. The queued storage
 * remains valid until HAL reports that this contiguous part has completed. */
static void faraway_tx_start(void)
{
    uint16_t length;

    if (!opened || tx_length != 0U || tx_tail == tx_head) {
        return;
    }
    length = tx_head > tx_tail ? (uint16_t)(tx_head - tx_tail)
                              : (uint16_t)(FARAWAY_TX_SIZE - tx_tail);
    if ((int32_t)(HAL_GetTick() - tx_next_ms) < 0) return;
    if (length > FARAWAY_TX_CHUNK) length = FARAWAY_TX_CHUNK;
    tx_length = length;
    if (HAL_UART_Transmit_IT(&huart5, &tx_buffer[tx_tail], length) != HAL_OK) {
        tx_length = 0U;
        g_faraway_uart_error_count++;
    }
}

static void faraway_send(const char *text)
{
    uint32_t irq;
    uint16_t length;
    uint16_t free_bytes;
    uint16_t i;
    uint16_t next_head;

    if (!opened) {
        return;
    }
    length = (uint16_t)strlen(text);
    irq = __get_PRIMASK();
    __disable_irq();
    free_bytes = (uint16_t)((tx_tail + FARAWAY_TX_SIZE - tx_head - 1U)
                            & FARAWAY_TX_MASK);
    if (length > free_bytes) {
        g_faraway_tx_drop_count++;
        __set_PRIMASK(irq);
        return;
    }
    next_head = tx_head;
    __set_PRIMASK(irq);
    /* Sole producer is the main loop. Keep RX interrupts enabled while copying
     * a long report; ISR sees the old head until the complete frame is ready. */
    for (i = 0U; i < length; i++) {
        tx_buffer[next_head] = (uint8_t)text[i];
        next_head = (uint16_t)((next_head + 1U) & FARAWAY_TX_MASK);
    }
    irq = __get_PRIMASK();
    __disable_irq();
    __DMB();
    tx_head = next_head;
    faraway_tx_start();
    __set_PRIMASK(irq);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        vision_uart_tx_complete(huart);
        return;
    }
    if (huart->Instance == UART5 && opened) {
        tx_tail = (uint16_t)((tx_tail + tx_length) & FARAWAY_TX_MASK);
        tx_length = 0U;
        tx_next_ms = HAL_GetTick() + FARAWAY_TX_GAP_MS;
    }
}

bool faraway_init(void)
{
    if (opened) {
        return true;
    }
    if (huart5.Instance != UART5) {
        return false;
    }
    rx_head = 0U;
    rx_tail = 0U;
    rx_fault = false;
    tx_head = 0U;
    tx_tail = 0U;
    tx_length = 0U;
    line_length = 0U;
    discard_line = false;
    g_faraway_rx_overflow_count = 0U;
    g_faraway_uart_error_count = 0U;
    g_faraway_tx_drop_count = 0U;
    tx_next_ms = HAL_GetTick();
    opened = true;
    __HAL_UART_CLEAR_OREFLAG(&huart5);
    if (HAL_UART_Receive_IT(&huart5, &rx_byte, 1U) != HAL_OK) {
        opened = false;
        g_faraway_uart_error_count++;
        return false;
    }
    return true;
}

void faraway_uart_rx_complete(UART_HandleTypeDef *huart)
{
    uint16_t next;

    if (huart->Instance != UART5 || !opened) {
        return;
    }
    next = (uint16_t)((rx_head + 1U) & FARAWAY_RX_MASK);
    if (next == rx_tail) {
        rx_fault = true;
        g_faraway_rx_overflow_count++;
    } else {
        rx_buffer[rx_head] = rx_byte;
        rx_head = next;
    }
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        rx_fault = true;
        g_faraway_uart_error_count++;
    }
}

void faraway_uart_error(UART_HandleTypeDef *huart)
{
    if (huart->Instance != UART5 || !opened) {
        return;
    }
    g_faraway_uart_error_count++;
    rx_fault = true;
    (void)HAL_UART_AbortReceive(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    if (HAL_UART_Receive_IT(huart, &rx_byte, 1U) != HAL_OK) {
        g_faraway_uart_error_count++;
    }
}

static char *faraway_trim(char *text)
{
    char *end;

    while (*text == ' ' || *text == '\t') {
        text++;
    }
    end = text + strlen(text);
    while (end > text && (end[-1] == ' ' || end[-1] == '\t')) {
        *--end = '\0';
    }
    return text;
}

/* Compact v2: [kind, distance_mm, angle_centidegrees|null, valid].
 * One outstanding snapshot; CRC detects missing/corrupted wireless bytes. */
static unsigned telemetry_fault(void)
{
    if (!strcmp(vision.fault_code, "waiting_rv2")) return 1;
    if (!strcmp(vision.fault_code, "invalid_packet")) return 2;
    if (!strcmp(vision.fault_code, "uart_error")) return 3;
    if (!strcmp(vision.fault_code, "vision_timeout")) return 4;
    if (!strcmp(vision.fault_code, "upper_unavailable")) return 5;
    return 0;
}
static void faraway_vision_report(void)
{
    static char report[768];
    const VisionObservation *obs = &vision.upper;
    uint32_t now = HAL_GetTick(), elapsed = now - obs->updated_ms;
    uint32_t total_age = obs->age_ms;
    unsigned fresh, status = (unsigned)obs->status, i, count;
    int used, written;
    char age[24], angle[16];
    uint16_t crc;
    if (tx_head != tx_tail) { ++g_faraway_tx_drop_count; return; }
    if (total_age != UINT32_MAX)
        total_age = elapsed > UINT32_MAX - 1U - total_age ? UINT32_MAX - 1U : total_age + elapsed;
    fresh = obs->fresh && total_age != UINT32_MAX && total_age <= VSP_TIMEOUT_MS;
    count = fresh ? obs->count : 0;
    if (!fresh && obs->fresh) status = VISION_OBS_STALE;
    if (total_age == UINT32_MAX) strcpy(age, "null");
    else (void)snprintf(age, sizeof(age), "%lu", (unsigned long)total_age);
    /* V3 array: tag,tick,link,rx,bad,uart,drops,upper_frame,age,status,
     * fresh,fault,upper_objects,servo_angle,lower_observation. */
    used = snprintf(report, sizeof(report),
        "[\"V3\",%lu,%u,%lu,%lu,%lu,%lu,%lu,%s,%u,%u,%u,[",
        (unsigned long)now, vision.connected,
        (unsigned long)vision.rx_packets, (unsigned long)vision.bad_packets,
        (unsigned long)vision.uart_errors, (unsigned long)g_faraway_tx_drop_count,
        (unsigned long)obs->frame_id, age, status, fresh,
        telemetry_fault());
    if (used < 0 || (unsigned)used >= sizeof(report)) goto overflow;
    for (i = 0; i < count; ++i) {
        const VisionObject *o = &obs->objects[i];
        unsigned valid = obs->available && total_age < 150U && o->distance_mm >= 0 &&
                         o->angle_present && o->distance_valid && o->angle_valid;
        if (o->angle_present) {
            long a = (long)(o->angle * 100.0f + (o->angle < 0 ? -0.5f : 0.5f));
            (void)snprintf(angle, sizeof(angle), "%ld", a);
        } else strcpy(angle, "null");
        written = snprintf(report + used, sizeof(report) - (unsigned)used,
                           "%s[%u,%ld,%s,%u]", i ? "," : "", (unsigned)o->kind,
                           (long)o->distance_mm, angle, valid);
        if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
        used += written;
    }
    /* Lower camera stays independent of upper freshness. Compact entries are
     * [kind,in_roi]; coordinates are deliberately not required for ROI testing. */
    {
        const VisionObservation *lower = &vision.lower;
        uint32_t lower_age = lower->age_ms, wait = now - lower->updated_ms;
        unsigned lower_fresh, lower_status = (unsigned)lower->status;
        if (lower_age != UINT32_MAX)
            lower_age = wait > UINT32_MAX - 1U - lower_age ? UINT32_MAX - 1U : lower_age + wait;
        lower_fresh = lower->fresh && lower_age != UINT32_MAX && lower_age <= VSP_TIMEOUT_MS;
        if (!lower_fresh && lower->fresh) lower_status = VISION_OBS_STALE;
        if (lower_age == UINT32_MAX) strcpy(age, "null");
        else (void)snprintf(age, sizeof(age), "%lu", (unsigned long)lower_age);
        written = snprintf(report + used, sizeof(report) - (unsigned)used,
                           "],%d,[%lu,%s,%u,%u,[", (int)servo_angle,
                           (unsigned long)lower->frame_id, age, lower_status, lower_fresh);
        if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
        used += written;
        for (i = 0; lower_fresh && i < lower->count; ++i) {
            written = snprintf(report + used, sizeof(report) - (unsigned)used,
                               "%s[%u,%u]", i ? "," : "", (unsigned)lower->objects[i].kind,
                               lower->objects[i].in_roi);
            if (written < 0 || (unsigned)written >= sizeof(report) - (unsigned)used) goto overflow;
            used += written;
        }
    }
    if ((unsigned)used + 10U > sizeof(report)) goto overflow;
    written = snprintf(report + used, sizeof(report) - (unsigned)used,
                       "]],[%u,%u,%u,%u,%u]]", step, task_info.reason,
                       task_info.inside_frames, task_info.outside_frames, task_info.held_kind);
    if (written < 0 || (unsigned)written + 8U >= sizeof(report) - (unsigned)used) goto overflow;
    used += written;
    crc = VSP_Crc16((const uint8_t *)report, (uint16_t)used);
    (void)snprintf(report + used, sizeof(report) - (unsigned)used, "*%04X\r\n", crc);
    faraway_send(report);
    return;
overflow:
    ++g_faraway_tx_drop_count;
}

void faraway_report(void)
{
    static uint32_t last_report;
    static bool reported;
    uint32_t now = HAL_GetTick();
    if (!opened || (reported && now - last_report < 150U)) return;
    last_report = now; reported = true;
    faraway_vision_report();
}

static void faraway_command(char *command)
{
    /* Motion belongs to the physical start/stop keys and the task state machine. */
    if (!strcmp(faraway_trim(command), "GET")) faraway_report();
    else faraway_send("ERR,TASK_CONTROL\r\n");
}

void faraway_process(void)
{
    uint16_t i;
    uint8_t byte;
    uint32_t irq;

    if (!opened) {
        return;
    }
    for (i = 0U; i < FARAWAY_PROCESS_LIMIT; i++) {
        irq = __get_PRIMASK();
        __disable_irq();
        if (rx_fault) {
            rx_tail = rx_head;
            rx_fault = false;
            line_length = 0U;
            discard_line = true;
            __set_PRIMASK(irq);
            faraway_send("ERR,RX_LOST\r\n");
            break;
        }
        if (rx_tail == rx_head) {
            faraway_tx_start();
            __set_PRIMASK(irq);
            break;
        }
        byte = rx_buffer[rx_tail];
        rx_tail = (uint16_t)((rx_tail + 1U) & FARAWAY_RX_MASK);
        __set_PRIMASK(irq);

        if (byte == '\r' || byte == '\n') {
            if (!discard_line && line_length != 0U) {
                line[line_length] = '\0';
                faraway_command(line);
            }
            line_length = 0U;
            discard_line = false;
        } else if (!discard_line) {
            if (line_length >= FARAWAY_LINE_SIZE - 1U ||
                (byte < 0x20U && byte != '\t') || byte > 0x7EU) {
                line_length = 0U;
                discard_line = true;
                faraway_send("ERR,LINE\r\n");
            } else {
                line[line_length++] = (char)byte;
            }
        }
    }
}
