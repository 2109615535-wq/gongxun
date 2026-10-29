#include "faraway.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "motor.h"

#define FARAWAY_RX_SIZE       512U
#define FARAWAY_RX_MASK       (FARAWAY_RX_SIZE - 1U)
#define FARAWAY_TX_SIZE       1024U
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
    } else {
        for (i = 0U; i < length; i++) {
            tx_buffer[tx_head] = (uint8_t)text[i];
            tx_head = (uint16_t)((tx_head + 1U) & FARAWAY_TX_MASK);
        }
        faraway_tx_start();
    }
    __set_PRIMASK(irq);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5 && opened) {
        tx_tail = (uint16_t)((tx_tail + tx_length) & FARAWAY_TX_MASK);
        tx_length = 0U;
        faraway_tx_start();
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

static bool faraway_read_integer(const char *text, long *value)
{
    char *end;
    long result;

    if (*text == '\0') {
        return false;
    }
    result = strtol(text, &end, 10);
    if (*end != '\0' || result < -1000L || result > 1000L) {
        return false;
    }
    *value = result;
    return true;
}

static bool faraway_read_gain(const char *text, float *value)
{
    char *end;
    double result;

    if (*text == '\0') {
        return false;
    }
    result = strtod(text, &end);
    /* Ordered comparisons also reject NaN and infinity. */
    if (*end != '\0' || !(result >= 0.0 && result <= 100.0)) {
        return false;
    }
    *value = (float)result;
    return true;
}

static void faraway_reset_pid_history(void)
{
    pid_motor2.error_int = 0.0f;
    pid_motor2.error_now = pid_motor2.target - pid_motor2.current;
    pid_motor2.error_last = pid_motor2.error_now;
    pid_motor2.output = 0.0f;
}

void faraway_report(void)
{
    static char report[512];
    static PID_t *const motors[] = {
        &pid_motor1, &pid_motor2, &pid_motor3, &pid_motor4
    };
    static Encoder_t *const encoders[] = {
        &encoder1, &encoder2, &encoder3, &encoder4
    };
    static GPIO_TypeDef *const enable_ports[] = {
        motor1_En_Port, motor2_En_Port, motor3_En_Port, motor4_En_Port
    };
    static const uint16_t enable_pins[] = {
        motor1_En_Pin, motor2_En_Pin, motor3_En_Pin, motor4_En_Pin
    };
    unsigned int i;
    unsigned int id;
    unsigned int used = 1U;
    int length;

    if (!opened) {
        return;
    }
    report[0] = '{';
    for (i = 0U; i < 4U; i++) {
        id = i + 1U;
        length = snprintf(report + used, sizeof(report) - used - 3U,
            "%s\"target%u\":%ld,\"output%u\":%ld,\"speed%u\":%ld,"
            "\"kp%u\":%.4f,\"ki%u\":%.4f,\"kd%u\":%.4f,\"enable%u\":%u",
            i == 0U ? "" : ",",
            id, (long)motors[i]->target, id, (long)motors[i]->output,
            id, (long)encoder_get_speed(encoders[i]),
            id, (double)motors[i]->kp, id, (double)motors[i]->ki,
            id, (double)motors[i]->kd,
            id, (unsigned int)(HAL_GPIO_ReadPin(enable_ports[i], enable_pins[i]) == GPIO_PIN_SET));
        if (length <= 0 || (unsigned int)length >= sizeof(report) - used - 3U) {
            g_faraway_tx_drop_count++;
            return;
        }
        used += (unsigned int)length;
    }
    memcpy(report + used, "}\r\n", 4U);
    faraway_send(report);
}

static void faraway_command(char *command)
{
    char *fields[4];
    char *comma;
    uint8_t count = 0U;
    long value;
    float kp, ki, kd;

    do {
        if (count >= 4U) {
            faraway_send("ERR,FORMAT\r\n");
            return;
        }
        comma = strchr(command, ',');
        if (comma != NULL) {
            *comma = '\0';
        }
        fields[count++] = faraway_trim(command);
        if (comma != NULL) {
            command = comma + 1;
        }
    } while (comma != NULL);

    if (count == 1U && strcmp(fields[0], "GET") == 0) {
        faraway_report();
    } else if (count == 2U && strcmp(fields[0], "TARGET") == 0) {
        if (!faraway_read_integer(fields[1], &value)) {
            faraway_send("ERR,RANGE\r\n");
            return;
        }
        MotorSetSpeed2((int32_t)value);
        faraway_send("OK,TARGET\r\n");
    } else if (count == 4U && strcmp(fields[0], "PID") == 0) {
        if (!faraway_read_gain(fields[1], &kp) ||
            !faraway_read_gain(fields[2], &ki) ||
            !faraway_read_gain(fields[3], &kd)) {
            faraway_send("ERR,RANGE\r\n");
            return;
        }
        pid_motor2.kp = kp;
        pid_motor2.ki = ki;
        pid_motor2.kd = kd;
        faraway_reset_pid_history();
        faraway_send("OK,PID\r\n");
    } else if (count == 2U && strcmp(fields[0], "ENABLE") == 0) {
        if (!faraway_read_integer(fields[1], &value) || (value != 0L && value != 1L)) {
            faraway_send("ERR,RANGE\r\n");
            return;
        }
        MotorEnable2(0U);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
        faraway_reset_pid_history();
        MotorEnable2((uint8_t)value);
        faraway_send("OK,ENABLE\r\n");
    } else if (count == 2U && strcmp(fields[0], "RUN") == 0) {
        if (!faraway_read_integer(fields[1], &value) || (value != 0L && value != 1L)) {
            faraway_send("ERR,RANGE\r\n");
            return;
        }
        MotorEnable1((uint8_t)value);
        MotorEnable2((uint8_t)value);
        MotorEnable3((uint8_t)value);
        MotorEnable4((uint8_t)value);
        faraway_send("OK,RUN\r\n");
    } else {
        faraway_send("ERR,COMMAND\r\n");
    }
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
