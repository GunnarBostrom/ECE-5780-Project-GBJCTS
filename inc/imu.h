/**
imu.h
LSM6DS3 Driver

*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    int16_t temp_raw;

    // raw sensor output
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    // physical units
    // int16_t ax_mg;
    // int16_t ay_mg;
    // int16_t az_mg;
    // int16_t gx_mdps;
    // int16_t gy_mdps;
    // int16_t gz_mdps;

    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;

} LSM6DS3_t;

extern volatile uint8_t imu_ready;

void imu_init(LSM6DS3_t* imu);
void imu_read(LSM6DS3_t* imu);
void imu_read_accel(LSM6DS3_t* imu);
void imu_read_gyro(LSM6DS3_t* imu);
void imu_read_temp(LSM6DS3_t* imu);
void imu_read_all(LSM6DS3_t* imu);

#endif // IMU_H
