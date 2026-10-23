#ifndef HDZ601_H_
#define HDZ601_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HDZ601_FRAME_HEADER_1 (0xAAU) //帧头
#define HDZ601_FRAME_HEADER_2 (0x55U) //帧尾
#define HDZ601_DEVICE_ID      (0x60U)

#define HDZ601_REPORT_FULL     (0x00U)
#define HDZ601_REPORT_ATTITUDE (0x01U)
#define HDZ601_CALIBRATE_YAW_DEFAULT (359.6f)//yaw一圈的校准值，两个模块一个是359.6，另一个361.5

typedef struct {
    float yaw;//偏航
    float pitch;//俯仰
    float roll;//横滚
} hdz601_attitude_t;

extern volatile hdz601_attitude_t g_hdz601_attitude;//主要的变量
extern volatile uint32_t g_hdz601_valid_frame_count;
extern volatile uint32_t g_hdz601_checksum_error_count;
extern volatile uint32_t g_hdz601_frame_error_count;
extern volatile uint32_t g_hdz601_uart2_rx_overflow_count;
extern volatile uint32_t g_hdz601_uart2_error_count;

void hdz601_init(void);//初始化
void hdz601_task(void);//串口数据处理，可以选择处理的时间间隔，即读取间隔
void hdz601_report_control(bool enable);//开始或者停止上报数据
void hdz601_set_report_mode(uint8_t mode);//汇报什么数据，一般就三个角度所以不用
void hdz601_trigger_calibration(void);//重新校准，一般不用
void hdz601_soft_reset(void);//软件重置，一般不用
void hdz601_calibrate_yaw_scale(float actual_angle_deg);//校准yaw，一般不用
bool hdz601_get_attitude(hdz601_attitude_t *attitude);//获取角度,实际上是把g_hdz601_attitude的三个角度取出来

#ifdef __cplusplus
}
#endif

#endif /* HDZ601_H_ */
