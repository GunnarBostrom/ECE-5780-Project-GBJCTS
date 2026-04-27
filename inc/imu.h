/**
imu.h
LSM6DS3 Driver

*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {

    // raw sensor output (not corrected)
    int16_t gx, gy, gz;
    int16_t ax, ay, az;

    // physical units
    // int16_t gx_mdps, gy_mdps, gz_mdps;
    // int16_t ax_mg, ay_mg, az_mg;
    float gx_dps, gy_dps, gz_dps;
    float ax_g, ay_g, az_g;

} LSM6DS3_t;

extern volatile uint8_t imu_ready;

void imu_init(LSM6DS3_t* imu);
void imu_calibrate(LSM6DS3_t* imu);
void imu_read(LSM6DS3_t* imu);
void imu_read_accel(LSM6DS3_t* imu);
void imu_read_gyro(LSM6DS3_t* imu);
void imu_read_temp(LSM6DS3_t* imu);
void imu_read_all(LSM6DS3_t* imu);

#endif // IMU_H