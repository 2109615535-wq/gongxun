#include "imu.h"
#include "hdz601.h"
#include "stm32f4xx_hal.h"
#include <stddef.h>

#define IMU_HISTORY_SIZE 64U
#define IMU_DEG_RAD (3.14159265358979323846f / 180.0f)

typedef struct { uint32_t tick; float yaw; } ImuSample;
static ImuSample imu_history[IMU_HISTORY_SIZE];
static unsigned imu_next, imu_count;
ImuState imu;
float angle_imu;

bool imu_fresh(uint32_t now)
{
    return imu.ready && now - imu.tick <= IMU_TIMEOUT_MS;
}

void imu_process(void)
{
    hdz601_attitude_t attitude;
    uint32_t frame = g_hdz601_attitude_count, tick = g_hdz601_attitude_tick;
    float delta;
    if (!hdz601_get(&attitude) || (imu.ready && frame == imu.frame) ||
        HAL_GetTick() - tick > IMU_TIMEOUT_MS ||
        !(attitude.yaw >= 0.0f && attitude.yaw <= 360.0f)) return;
    if (!imu.ready) angle_imu = 0;
    else {
        delta = attitude.yaw - imu.raw_yaw;
        if (delta > 180.0f) delta -= 360.0f;
        else if (delta < -180.0f) delta += 360.0f;
        angle_imu += IMU_YAW_SIGN * delta * IMU_DEG_RAD;
        if (tick - imu.tick > IMU_TIMEOUT_MS) imu_next = imu_count = 0;
    }
    imu.ready = true; imu.raw_yaw = attitude.yaw; imu.frame = frame; imu.tick = tick;
    if (imu_count && imu_history[(imu_next + IMU_HISTORY_SIZE - 1U) % IMU_HISTORY_SIZE].tick == tick) {
        imu_history[(imu_next + IMU_HISTORY_SIZE - 1U) % IMU_HISTORY_SIZE].yaw = angle_imu;
        return;
    }
    imu_history[imu_next].tick = tick; imu_history[imu_next].yaw = angle_imu;
    imu_next = (imu_next + 1U) % IMU_HISTORY_SIZE;
    if (imu_count < IMU_HISTORY_SIZE) ++imu_count;
}

bool imu_init(void)
{
    imu.ready = false; imu_next = imu_count = 0; angle_imu = 0;
    imu_process();
    return imu.ready;
}

bool imu_at(uint32_t tick, float *yaw)
{
    unsigned i, first;
    if (!yaw || !imu_fresh(HAL_GetTick()) || !imu_count) return false;
    first = (imu_next + IMU_HISTORY_SIZE - imu_count) % IMU_HISTORY_SIZE;
    for (i = 0; i < imu_count; ++i) {
        const ImuSample *a = &imu_history[(first + i) % IMU_HISTORY_SIZE];
        if (tick == a->tick) { *yaw = a->yaw; return true; }
        if (i + 1U < imu_count) {
            const ImuSample *b = &imu_history[(first + i + 1U) % IMU_HISTORY_SIZE];
            uint32_t span = b->tick - a->tick, offset = tick - a->tick;
            if (span && offset < span) {
                *yaw = a->yaw + (b->yaw - a->yaw) * ((float)offset / (float)span);
                return true;
            }
        }
    }
    /* Hold the latest sample without extrapolating angular velocity.
     * Never substitute a future heading for an older encoder sample. */
    if (tick - imu.tick <= IMU_TIMEOUT_MS) { *yaw = angle_imu; return true; }
    return false;
}
