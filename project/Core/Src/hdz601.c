#include "hdz601.h"
#include "faraway.h"

#include <stddef.h>

#include "scheduler.h"
#include "stm32f4xx_hal.h"
#include "OLED.h"

#define HDZ601_COMMAND_REPORT_CONTROL       (0x0AU)
#define HDZ601_COMMAND_REPORT_MODE          (0x0BU)
#define HDZ601_COMMAND_TRIGGER_CALIBRATION  (0x10U)
#define HDZ601_COMMAND_SOFT_RESET           (0x12U)
#define HDZ601_COMMAND_CALIBRATE_YAW_SCALE  (0x14U)
#define HDZ601_COMMAND_ATTITUDE_REPORT      (0x01U)

#define HDZ601_ATTITUDE_PAYLOAD_LENGTH (6U)
#define HDZ601_FULL_PAYLOAD_LENGTH     (18U)
#define HDZ601_MAX_PAYLOAD_LENGTH      (32U)
#define HDZ601_UART2_RX_BUFFER_SIZE    (256U)
#define HDZ601_UART2_RX_BUFFER_MASK    (HDZ601_UART2_RX_BUFFER_SIZE - 1U)

extern UART_HandleTypeDef huart2;

typedef enum {
    HDZ601_PARSE_HEADER_1 = 0,
    HDZ601_PARSE_HEADER_2,
    HDZ601_PARSE_DEVICE_ID,
    HDZ601_PARSE_COMMAND,
    HDZ601_PARSE_LENGTH,
    HDZ601_PARSE_DATA,
    HDZ601_PARSE_CHECKSUM
} hdz601_parse_state_t;

typedef union {
    float value;
    uint8_t bytes[4];
} hdz601_float_bytes_t;

static hdz601_parse_state_t hdz601_parse_state = HDZ601_PARSE_HEADER_1;
static uint8_t hdz601_rx_device_id = 0U;
static uint8_t hdz601_rx_command = 0U;
static uint8_t hdz601_rx_length = 0U;
static uint8_t hdz601_rx_index = 0U;
static uint8_t hdz601_rx_checksum = 0U;
static uint8_t hdz601_rx_data[HDZ601_MAX_PAYLOAD_LENGTH];
static bool hdz601_attitude_valid = false;
static volatile bool hdz601_opened = false;
static uint8_t hdz601_uart2_rx_byte;
static uint8_t hdz601_uart2_rx_buffer[HDZ601_UART2_RX_BUFFER_SIZE];
static volatile uint16_t hdz601_uart2_rx_head;
static volatile uint16_t hdz601_uart2_rx_tail;

volatile hdz601_attitude_t g_hdz601_attitude = {0.0f, 0.0f, 0.0f};
volatile uint32_t g_hdz601_valid_frame_count = 0U;
volatile uint32_t g_hdz601_checksum_error_count = 0U;
volatile uint32_t g_hdz601_frame_error_count = 0U;
volatile uint32_t g_hdz601_uart2_rx_overflow_count = 0U;
volatile uint32_t g_hdz601_uart2_error_count = 0U;

static bool hdz601_uart2_init(void)
{
    hdz601_uart2_rx_head = 0U;
    hdz601_uart2_rx_tail = 0U;
    if (HAL_UART_Receive_IT(&huart2, &hdz601_uart2_rx_byte, 1U) != HAL_OK) {
        g_hdz601_uart2_error_count++;
        return false;
    }
    return true;
}

static void hdz601_uart2_clear_rx_buffer(void)
{
    uint32_t interrupt_state = __get_PRIMASK();

    __disable_irq();
    hdz601_uart2_rx_tail = hdz601_uart2_rx_head;
    if (interrupt_state == 0U) {
        __enable_irq();
    }
}

static void hdz601_uart2_send_data(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U)) {
        return;
    }
    if (HAL_UART_Transmit(&huart2, (uint8_t *) data, length, 100U) != HAL_OK) {
        g_hdz601_uart2_error_count++;
    }
}

static void hdz601_uart2_send_byte(uint8_t byte)
{
    hdz601_uart2_send_data(&byte, 1U);
}

static bool hdz601_uart2_read_byte(uint8_t *byte)
{
    uint16_t tail;

    if (byte == NULL) {
        return false;
    }
    tail = hdz601_uart2_rx_tail;
    if (tail == hdz601_uart2_rx_head) {
        return false;
    }
    *byte = hdz601_uart2_rx_buffer[tail];
    hdz601_uart2_rx_tail = (uint16_t) ((tail + 1U) & HDZ601_UART2_RX_BUFFER_MASK);
    return true;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint16_t next_head;

    if (huart->Instance == UART5) {
        faraway_uart_rx_complete(huart);
        return;
    }
    if ((huart->Instance != USART2) || !hdz601_opened) {
        return;
    }
    next_head = (uint16_t) ((hdz601_uart2_rx_head + 1U) & HDZ601_UART2_RX_BUFFER_MASK);
    if (next_head == hdz601_uart2_rx_tail) {
        g_hdz601_uart2_rx_overflow_count++;
    } else {
        hdz601_uart2_rx_buffer[hdz601_uart2_rx_head] = hdz601_uart2_rx_byte;
        hdz601_uart2_rx_head = next_head;
    }
    if (HAL_UART_Receive_IT(huart, &hdz601_uart2_rx_byte, 1U) != HAL_OK) {
        g_hdz601_uart2_error_count++;
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        faraway_uart_error(huart);
        return;
    }
    if ((huart->Instance == USART2) && hdz601_opened) {
        g_hdz601_uart2_error_count++;
        if (HAL_UART_Receive_IT(huart, &hdz601_uart2_rx_byte, 1U) != HAL_OK) {
            g_hdz601_uart2_error_count++;
        }
    }
}

static void hdz601_reset_parser(void)
{
    hdz601_parse_state = HDZ601_PARSE_HEADER_1;
    hdz601_rx_index = 0U;
}

static uint8_t hdz601_calculate_checksum(uint8_t command,
                                         uint8_t length,
                                         const uint8_t *data)
{
    uint16_t checksum = (uint16_t) HDZ601_DEVICE_ID +
                        (uint16_t) command + (uint16_t) length;
    uint8_t i;

    for (i = 0U; i < length; i++) {
        checksum += data[i];
    }

    return (uint8_t) checksum;
}

static void hdz601_send_command(uint8_t command,
                                const uint8_t *data,
                                uint8_t length)
{
    uint8_t header[5];
    uint8_t checksum;

    if (!hdz601_opened || ((data == NULL) && (length != 0U))) {
        return;
    }

    header[0] = HDZ601_FRAME_HEADER_1;
    header[1] = HDZ601_FRAME_HEADER_2;
    header[2] = HDZ601_DEVICE_ID;
    header[3] = command;
    header[4] = length;
    checksum = hdz601_calculate_checksum(command, length, data);

    hdz601_uart2_send_data(header, sizeof(header));
    hdz601_uart2_send_data(data, length);
    hdz601_uart2_send_byte(checksum);
}

static int16_t hdz601_read_i16_le(const uint8_t *data)
{
    return (int16_t) (((uint16_t) data[1] << 8U) | data[0]);
}

static uint16_t hdz601_read_u16_le(const uint8_t *data)
{
    return (uint16_t) (((uint16_t) data[1] << 8U) | data[0]);
}

static void hdz601_parse_attitude(void)
{
    const uint8_t *angle_data;

    if (hdz601_rx_length == HDZ601_ATTITUDE_PAYLOAD_LENGTH) {
        angle_data = hdz601_rx_data;
        g_hdz601_attitude.yaw =
            (float) hdz601_read_u16_le(&angle_data[0]) / 100.0f;
        g_hdz601_attitude.pitch =
            (float) hdz601_read_i16_le(&angle_data[2]) / 100.0f;
        g_hdz601_attitude.roll =
            (float) hdz601_read_i16_le(&angle_data[4]) / 100.0f;
        hdz601_attitude_valid = true;
    } else if (hdz601_rx_length >= HDZ601_FULL_PAYLOAD_LENGTH) {
        angle_data = &hdz601_rx_data[12];
        g_hdz601_attitude.pitch =
            (float) hdz601_read_i16_le(&angle_data[0]) / 100.0f;
        g_hdz601_attitude.roll =
            (float) hdz601_read_i16_le(&angle_data[2]) / 100.0f;
        g_hdz601_attitude.yaw =
            (float) hdz601_read_u16_le(&angle_data[4]) / 100.0f;
        hdz601_attitude_valid = true;
    } else {
        g_hdz601_frame_error_count++;
    }
}

static void hdz601_handle_frame(void)
{
    if (hdz601_rx_command == HDZ601_COMMAND_ATTITUDE_REPORT) {
        hdz601_parse_attitude();
    }
}

static void hdz601_process_byte(uint8_t byte)
{
    switch (hdz601_parse_state) {
        case HDZ601_PARSE_HEADER_1:
            if (byte == HDZ601_FRAME_HEADER_1) {
                hdz601_parse_state = HDZ601_PARSE_HEADER_2;
            }
            break;

        case HDZ601_PARSE_HEADER_2:
            if (byte == HDZ601_FRAME_HEADER_2) {
                hdz601_parse_state = HDZ601_PARSE_DEVICE_ID;
            } else if (byte != HDZ601_FRAME_HEADER_1) {
                hdz601_reset_parser();
            }
            break;

        case HDZ601_PARSE_DEVICE_ID:
            hdz601_rx_device_id = byte;
            hdz601_rx_checksum = byte;
            hdz601_parse_state = HDZ601_PARSE_COMMAND;
            break;

        case HDZ601_PARSE_COMMAND:
            hdz601_rx_command = byte;
            hdz601_rx_checksum = (uint8_t) (hdz601_rx_checksum + byte);
            hdz601_parse_state = HDZ601_PARSE_LENGTH;
            break;

        case HDZ601_PARSE_LENGTH:
            hdz601_rx_length = byte;
            hdz601_rx_checksum = (uint8_t) (hdz601_rx_checksum + byte);
            hdz601_rx_index = 0U;

            if (hdz601_rx_length > HDZ601_MAX_PAYLOAD_LENGTH) {
                g_hdz601_frame_error_count++;
                hdz601_reset_parser();
            } else if (hdz601_rx_length == 0U) {
                hdz601_parse_state = HDZ601_PARSE_CHECKSUM;
            } else {
                hdz601_parse_state = HDZ601_PARSE_DATA;
            }
            break;

        case HDZ601_PARSE_DATA:
            hdz601_rx_data[hdz601_rx_index++] = byte;
            hdz601_rx_checksum = (uint8_t) (hdz601_rx_checksum + byte);
            if (hdz601_rx_index >= hdz601_rx_length) {
                hdz601_parse_state = HDZ601_PARSE_CHECKSUM;
            }
            break;

        case HDZ601_PARSE_CHECKSUM:
            if (byte != hdz601_rx_checksum) {
                g_hdz601_checksum_error_count++;
            } else if (hdz601_rx_device_id != HDZ601_DEVICE_ID) {
                g_hdz601_frame_error_count++;
            } else {
                g_hdz601_valid_frame_count++;
                hdz601_handle_frame();
            }
            hdz601_reset_parser();
            break;

        default:
            hdz601_reset_parser();
            break;
    }
}

/**
 * @brief 初始化HDZ601并启动姿态数据上报。
 */
void hdz601_init(void)
{
    g_hdz601_attitude.yaw = 0.0f;
    g_hdz601_attitude.pitch = 0.0f;
    g_hdz601_attitude.roll = 0.0f;
    g_hdz601_valid_frame_count = 0U;
    g_hdz601_checksum_error_count = 0U;
    g_hdz601_frame_error_count = 0U;
    g_hdz601_uart2_rx_overflow_count = 0U;
    g_hdz601_uart2_error_count = 0U;
    hdz601_attitude_valid = false;
    hdz601_reset_parser();

    (void) hdz601_open();
}

bool hdz601_open(void)
{
    if (hdz601_opened) {
        return true;
    }
    if (huart2.Instance != USART2) {
        g_hdz601_uart2_error_count++;
        return false;
    }
    if ((huart2.gState == HAL_UART_STATE_RESET) &&
        (HAL_UART_Init(&huart2) != HAL_OK)) {
        g_hdz601_uart2_error_count++;
        return false;
    }

    hdz601_attitude_valid = false;
    hdz601_reset_parser();
    hdz601_opened = true;
    if (!hdz601_uart2_init()) {
        hdz601_opened = false;
        (void) HAL_UART_DeInit(&huart2);
        return false;
    }

    hdz601_soft_reset();
    dl_delay(2000U);
    hdz601_uart2_clear_rx_buffer();

    hdz601_calibrate_yaw_scale(HDZ601_CALIBRATE_YAW_DEFAULT);
    hdz601_set_report_mode(HDZ601_REPORT_ATTITUDE);
    hdz601_report_control(true);
    return true;
}

void hdz601_close(void)
{
    if (!hdz601_opened) {
        return;
    }

    hdz601_report_control(false);
    hdz601_opened = false;
    if (HAL_UART_DeInit(&huart2) != HAL_OK) {
        g_hdz601_uart2_error_count++;
    }
    hdz601_uart2_clear_rx_buffer();
    hdz601_reset_parser();
    hdz601_attitude_valid = false;
}


void hdz601_oledshow(void)
{
    hdz601_attitude_t attitude_show;
    if (!hdz601_get(&attitude_show)) {
        return;
    }
    OLED_ShowFloatNum2(0, 0, attitude_show.yaw, 3, 3);
    OLED_ShowFloatNum2(0, 16, attitude_show.pitch, 3, 3);
    OLED_ShowFloatNum2(0, 32, attitude_show.roll, 3, 3);
}







/**
 * @brief 处理USART2缓冲区内已经接收的数据。
 */
void hdz601_process(void)
{
    uint8_t byte;

    if (!hdz601_opened) {
        return;
    }
    while (hdz601_uart2_read_byte(&byte)) {
        hdz601_process_byte(byte);
    }

}

/**
 * @brief 启动或停止周期数据上报。
 */
void hdz601_report_control(bool enable)
{
    uint8_t value = enable ? 0x01U : 0x00U;

    hdz601_send_command(HDZ601_COMMAND_REPORT_CONTROL, &value, 1U);
}

/**
 * @brief 设置全数据模式或仅姿态模式。
 */
void hdz601_set_report_mode(uint8_t mode)
{
    uint8_t value = (mode == HDZ601_REPORT_ATTITUDE) ?
                    HDZ601_REPORT_ATTITUDE : HDZ601_REPORT_FULL;

    hdz601_send_command(HDZ601_COMMAND_REPORT_MODE, &value, 1U);
}

/**
 * @brief 触发模块重新校准。
 */
void hdz601_trigger_calibration(void)
{
    hdz601_send_command(HDZ601_COMMAND_TRIGGER_CALIBRATION, NULL, 0U);
}

/**
 * @brief 软复位HDZ601内部控制器。
 */
void hdz601_soft_reset(void)
{
    hdz601_send_command(HDZ601_COMMAND_SOFT_RESET, NULL, 0U);
}

/**
 * @brief 将实测单圈角度发送给模块进行Yaw比例校准。
 */
void hdz601_calibrate_yaw_scale(float actual_angle_deg)
{
    hdz601_float_bytes_t angle;

    angle.value = actual_angle_deg;
    hdz601_send_command(HDZ601_COMMAND_CALIBRATE_YAW_SCALE,
                        angle.bytes,
                        sizeof(angle.bytes));
}

/**
 * @brief 获取最近一次有效的姿态角，单位为度。
 * @return 已经收到过有效姿态帧时返回true，否则返回false。
 */
bool hdz601_get(hdz601_attitude_t *attitude)
{
    if ((attitude == NULL) || !hdz601_opened || !hdz601_attitude_valid) {
        return false;
    }

    attitude->yaw = g_hdz601_attitude.yaw;
    attitude->pitch = g_hdz601_attitude.pitch;
    attitude->roll = g_hdz601_attitude.roll;
    return true;
}
