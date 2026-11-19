#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>

/* Clockwise-positive body yaw. Keep the existing sign; if a clockwise hand
 * turn makes the displayed angle decrease, set this to -1.0f. */
#define IMU_YAW_SIGN 1.0f
#define IMU_TIMEOUT_MS 400U

typedef struct {
    uint32_t frame, tick;
    float raw_yaw;
    bool ready;
} ImuState;
extern ImuState imu;
extern float angle_imu; /* Continuous radians relative to the reset heading. */
void imu_process(void);
bool imu_init(void); /* Reset reference; wait for first fresh frame if needed. */
bool imu_fresh(uint32_t now);
bool imu_at(uint32_t tick, float *yaw); /* Align yaw to the encoder sample. */

#endif
