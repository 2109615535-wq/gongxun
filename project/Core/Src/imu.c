#include "imu.h"
#include "hdz601.h"

float angle_imu = 0.0f;

static float last_yaw;
static bool imu_ready = false;

void imu_process(void)
{
    hdz601_attitude_t attitude;
    float delta;

    if (!hdz601_get(&attitude))
    {
        return;
    }

    /* The first valid reading establishes the initial zero direction. */
    if (!imu_ready)
    {
        last_yaw = attitude.yaw;
        angle_imu = 0.0f;
        imu_ready = true;
        return;
    }

    delta = attitude.yaw - last_yaw;
    last_yaw = attitude.yaw;

    /* Assumes less than 180 degrees of rotation between readings. */
    if (delta > 180.0f)
    {
        delta -= 360.0f;
    }
    else if (delta < -180.0f)
    {
        delta += 360.0f;
    }

    /* If clockwise rotation decreases module yaw, change += to -=. */
    angle_imu += delta * 3.1415926f / 180.0f;
}

bool imu_init(void)
{
    hdz601_attitude_t attitude;

    if (!hdz601_get(&attitude))
    {
        return false;
    }

    last_yaw = attitude.yaw;
    angle_imu = 0.0f;
    imu_ready = true;

    return true;
}
