#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    uint8_t slave_addr;

    // Raw sensor outputs from the LSM6DS3 registers
    int16_t ax;
    int16_t ay;
    int16_t az;

    int16_t gx;
    int16_t gy;
    int16_t gz;

    int16_t temp_raw;

    // Scaled physical units for filtering/control
    float ax_g;
    float ay_g;
    float az_g;

    float gx_dps;
    float gy_dps;
    float gz_dps;

    float temp_c;
} IMU_t;

extern volatile uint8_t imu_ready;

void imu_init(IMU_t* imu, uint8_t slave_addr);
void imu_read(IMU_t* imu);
void imu_read_accel(IMU_t* imu);
void imu_read_gyro(IMU_t* imu);
void imu_read_temp(IMU_t* imu);
void imu_read_all(IMU_t* imu);

#endif // IMU_H