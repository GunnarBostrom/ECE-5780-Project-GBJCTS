/**
IMU
STM LSM6DS3

*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    uint8_t slave_addr;

    // raw sensor outputs
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;

    // physical units in milli-g and milli-dps
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;
    int8_t temp_c;
} LSM6DS3_t;

extern volatile uint8_t imu_ready;

void imu_init(LSM6DS3* imu, uint8_t slave_addr);
void imu_read(LSM6DS3* imu);
void imu_read_accel(LSM6DS3* imu);
void imu_read_gyro(LSM6DS3* imu);
void imu_read_temp(LSM6DS3* imu);
void imu_read_all(LSM6DS3* imu);
static void imu_convert_units(LSM6DS3* imu)

#endif // IMU_H