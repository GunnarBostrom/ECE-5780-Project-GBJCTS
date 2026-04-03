/**
IMU
STM LSM6DS3

LSM6DS3 can operate at 100 kHz or 400 kHz.

WHO_AM_I register: 0x0F
Device ID: 0x69
I2C slave address: 0x6B (default), otherwise 0x6A
*/

#include "config.h"
#include "imu.h"

#define WHO_AM_I_REG    0x000F
#define DEVICE_ID       0x69
#define ACCEL_REG       0x0028  // OUTX_L_XL - lowest accel output register
#define GYRO_REG        0x0022  // OUTX_L_G - lowest gyro output register
#define TEMP_REG        0x0020  // OUT_TEMP_L - lowest temp output register
#define ACCEL_CFG       0x0010  // CTRL1_XL - accelerometer control register
#define GYRO_CFG        0x0011  // CTRL2_G - gyroscope control register
#define INT1_CFG        0x000D  // INT1_CTRL - interrupt1 control register
#define INT2_CFG        0x000E  // INT2_CTRL - interrupt2 control register


/* accelerometer config values */
#define ACCEL_ODR_104HZ  (0b0100 << 4)  // 12.5 Hz to 6.66 kHz ODR available
#define ACCEL_ODR_208HZ  (0b0101 << 4)  // default is Power-down
#define ACCEL_ODR_416HZ  (0b0110 << 4)

#define ACCEL_FS_2G      (0b00 << 2)  // ±2 g to ±16 g full-scale available
#define ACCEL_FS_4G      (0b10 << 2)  // default is ±2 g
#define ACCEL_FS_8G      (0b11 << 2)
#define ACCEL_FS_16G     (0b01 << 2)

#define ACCEL_BW_200HZ   (0b01 << 0)  // 50 Hz to 400 Hz anti-aliasing available
#define ACCEL_BW_400HZ   (0b00 << 0)  // default is 400 Hz


/* gyroscope config values */
#define GYRO_ODR_104HZ   (0b0100 << 4)  // 12.5 Hz to 1.66 kHz ODR available
#define GYRO_ODR_208HZ   (0b0101 << 4)  // default is Power-down
#define GYRO_ODR_416HZ   (0b0110 << 4)

#define GYRO_FS_500DPS   (0b10 << 2)  // 125 dps to 2000 dps full-scale available
#define GYRO_FS_2000DPS  (0b11 << 2)  // default is 250 dps
#define GYRO_FS_125DPS   (0b1  << 1)  // 125 dps disabled by default



#if USE_IMU // use REAL hardware

void imu_init(IMU_t* imu, uint8_t slave_addr, uint16_t freq) {
    imu->slave_addr = slave_addr;
    imu->freq = freq;
    
    // spin up I2C
    i2c_init(freq);

    // verify device
    uint8_t buf[1];
    i2c_read(imu->slave_addr, WHO_AM_I_REG, buf, 1);
    if (buf[0] != DEVICE_ID) {
        // wrong device: throw an error
        return;
    }

    // configure IMU
    uint8_t accel_config = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(imu->slave_addr, ACCEL_CFG, &accel_config, 1);

    uint8_t gyro_config = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(imu->slave_addr, GYRO_CFG, &gyro_config, 1);

    uint8_t int1_config = 0b00000011; // bit 1 = DRDY_G, bit 0 = DRDY_XL
    i2c_write(imu->slave_addr, INT1_CFG, &int1_config, 1);
}

// Reads all inertial data (this does not include temperature)
void imu_read(IMU_t* imu) {
    uint8_t buf[12];

    i2c_read(imu->slave_addr, GYRO_REG, buf, 12);
    imu->gx = (int16_t)(buf[0]  | buf[1]  << 8);
    imu->gy = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gz = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->ax = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ay = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->az = (int16_t)(buf[10] | buf[11] << 8);
}

// Reads IMU acceleration data
void imu_read_accel(IMU_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(imu->slave_addr, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | buf[1] << 8);
    imu->ay = (int16_t)(buf[2] | buf[3] << 8);
    imu->az = (int16_t)(buf[4] | buf[5] << 8);
}

// Reads IMU gyroscope data
void imu_read_gyro(IMU_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(imu->slave_addr, GYRO_REG, buf, 6);
    imu->gx = (int16_t)(buf[0] | buf[1] << 8);
    imu->gy = (int16_t)(buf[2] | buf[3] << 8);
    imu->gz = (int16_t)(buf[4] | buf[5] << 8);
}

// Reads IMU temperature data
void imu_read_temp(IMU_t* imu) {
    uint8_t buf[2];

    i2c_read(imu->slave_addr, TEMP_REG, buf, 2);
    imu->temp = (int16_t)(buf[0]  | buf[1]  << 8);
}

// Reads all IMU data (including temperature)
void imu_read_all(IMU_t* imu) {
    uint8_t buf[14];

    i2c_read(imu->slave_addr, TEMP_REG, buf, 14);
    imu->temp = (int16_t)(buf[0]  | buf[1]  << 8);
    imu->gx   = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gy   = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->gz   = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ax   = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->ay   = (int16_t)(buf[10] | buf[11] << 8);
    imu->az   = (int16_t)(buf[12] | buf[13] << 8);
}

#else // use FAKE hardware
void imu_init(IMU_t* imu, uint8_t slave_addr, uint16_t freq) { /* do nothing */ }
void imu_read(IMU_t* imu) { /* do nothing */ }
void imu_read_accel(IMU_t* imu) { /* do nothing */ }
void imu_read_gyro(IMU_t* imu) { /* do nothing */ }
void imu_read_temp(IMU_t* imu) { /* do nothing */ }
void imu_read_all(IMU_t* imu) { /* do nothing */ }
#endif