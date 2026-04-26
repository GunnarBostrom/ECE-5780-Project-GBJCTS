/**
IMU driver
STM LSM6DS3

*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>
#include <stdbool.h>

#define IMU_ERROR_NONE         0U
#define IMU_ERROR_CAL_TIMEOUT  1U
#define IMU_ERROR_CAL_MOTION   2U

typedef struct {
    // raw sensor outputs
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;

    // physical units in milli-g and milli-dps
    int32_t ax_mg, ay_mg, az_mg;
    int32_t gx_mdps, gy_mdps, gz_mdps;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;

    // per-axis gyro bias in raw sensor counts
    int16_t gx_bias;
    int16_t gy_bias;
    int16_t gz_bias;
    bool gyro_bias_valid;

    int8_t temp_c;
} LSM6DS3_t;

extern volatile uint8_t imu_ready;
extern volatile uint8_t imu_error_code;

bool imu_init(LSM6DS3_t* imu);
bool imu_data_ready(void);
void imu_reset_gyro_bias(LSM6DS3_t* imu);
bool imu_calibrate_gyro(LSM6DS3_t* imu, uint16_t sample_count);
void imu_read(LSM6DS3_t* imu);
void imu_read_accel(LSM6DS3_t* imu);
void imu_read_gyro(LSM6DS3_t* imu);
void imu_read_temp(LSM6DS3_t* imu);
void imu_read_all(LSM6DS3_t* imu);

#endif // IMU_H
