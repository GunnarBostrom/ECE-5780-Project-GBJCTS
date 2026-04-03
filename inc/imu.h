/**
IMU
STM LSM6DS3

*/

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

typedef struct {
    uint8_t slave_addr;   // slave address
    uint16_t freq;        // I2C frequency in kHz
    int16_t ax, ay, az;   // accelerometer for each axis in g
    int16_t gx, gy, gz;   // gyroscope for each axis in dps
    float temp;           // temperature in c?
} IMU_t;


/**
* @brief Initializes and configures IMU.
*
* Hard-coded configuration of the output data rate, accelerometer full-scale, 
* gyroscope full-scale, and an interrupt.
*
* @param imu            Pointer to IMU struct
* @param slave_addr     The I2C slave address of the IMU
* @param freq           I2C frequency in kHz
*/ 
void imu_init(IMU_t* imu, uint8_t slave_addr, uint16_t freq); // config IMU outside of init?

// Reads all inertial data.
void imu_read(IMU_t* imu);

// Reads acceleration data.
void imu_read_accel(IMU_t* imu);

// Reads angular rate data.
void imu_read_gyro(IMU_t* imu);

// Reads temperature data.
void imu_read_temp(IMU_t* imu);

// Reads all data the IMU can provide.
void imu_read_all(IMU_t* imu);



// void imu_set_rate(void); // data rate
// void imu_set_range(void); // accelerometer range
// anything else?

#endif // IMU_H