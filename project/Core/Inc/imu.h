#ifndef IMU_H
#define IMU_H

#include <stdbool.h>

/* Continuous relative yaw in radians. Verify the sign on the robot. */
extern float angle_imu;

/* Call periodically after hdz601_process(), from the main-loop scheduler. */
void imu_process(void);

/* Use the latest received yaw as zero. False if no attitude is available. */
bool imu_init(void);

#endif /* IMU_H */
