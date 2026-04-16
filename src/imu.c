/**
IMU

STM LSM6DS3
LSM6DS3 can operate at 100 kHz or 400 kHz.

Data peripheral provides:
    - 3 axis acceleration
    - 3 axis angular rate
    - temperature

I2C slave address: 0x6B (default), otherwise 0x6A
WHO_AM_I register: 0x0F
Device ID: 0x69
*/

#include "config.h"
#include "imu.h"
#include "i2c.h"

// copied in just for debugging with LEDs
#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include "stm32f0xx_it.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>

#define WHO_AM_I_REG    0x0F
#define DEVICE_ID       0x69


/* ––––– output registers ––––– */
#define ACCEL_REG       0x28  // OUTX_L_XL - lowest accel output register
#define GYRO_REG        0x22  // OUTX_L_G - lowest gyro output register
#define TEMP_REG        0x20  // OUT_TEMP_L - lowest temp output register


/* ––––– configuration registers ––––– */
#define ACCEL_CFG       0x10  // CTRL1_XL - accelerometer control register
#define GYRO_CFG        0x11  // CTRL2_G - gyroscope control register
#define INT1_CFG        0x0D  // INT1_CTRL - interrupt1 control register
#define INT2_CFG        0x0E  // INT2_CTRL - interrupt2 control register

#define CTRL3_C         0x12  // these are for interrupts
#define CTRL4_C         0x13


/* ––––– accelerometer config values ––––– */
// 12.5 Hz to 6.66 kHz ODR available, default is Power-down
#define ACCEL_ODR_104HZ  (0b0100 << 4)
#define ACCEL_ODR_208HZ  (0b0101 << 4)
#define ACCEL_ODR_416HZ  (0b0110 << 4)

// ±2 g to ±16 g full-scale available, default is ±2 g
#define ACCEL_FS_2G      (0b00 << 2)
#define ACCEL_FS_4G      (0b10 << 2)
#define ACCEL_FS_8G      (0b11 << 2)
#define ACCEL_FS_16G     (0b01 << 2)

// 50 Hz to 400 Hz anti-aliasing available, default is 400 Hz
#define ACCEL_BW_200HZ   (0b01 << 0)
#define ACCEL_BW_400HZ   (0b00 << 0)


/* ––––– gyroscope config values ––––– */
// 12.5 Hz to 1.66 kHz ODR available, default is Power-down
#define GYRO_ODR_104HZ   (0b0100 << 4)
#define GYRO_ODR_208HZ   (0b0101 << 4)
#define GYRO_ODR_416HZ   (0b0110 << 4)

// 125 dps to 2000 dps full-scale available, default is 250 dps
#define GYRO_FS_500DPS   (0b10 << 2)  
#define GYRO_FS_2000DPS  (0b11 << 2)
#define GYRO_FS_125DPS   (0b1  << 1) // 125 dps enabled overrides other modes


/* ––––– interrupt config values ––––– */
// data ready registers disabled by default
#define INT1_GYRO_EN     (0b1 << 1)  // INT1_DRDY_G
#define INT1_ACCEL_EN    (0b1 << 0)  // INT1_DRDY_XL

#define INT2_TEMP_EN     (0b1 << 2)  // INT2_DRDY_TEMP
#define INT2_GYRO_EN     (0b1 << 1)  // INT2_DRDY_G
#define INT2_ACCEL_EN    (0b1 << 0)  // INT2_DRDY_XL

volatile uint8_t imu_ready = 0;

#if USE_IMU // use REAL hardware

/**
 * @brief LSM6DS3 configuration and initialization.
 * 
 * Hard-coded configs:
 *      - accelerometer: 416Hz data rate, ±8g full-scale, 400Hz anti-aliasing
 *      - gyroscope:     416Hz data rate, 2000 dps
 * 
 * @param imu           Pointer to IMU struct
 * @param slave_addr    I2C slave address of the IMU
 */
void imu_init(IMU_t* imu, uint8_t slave_addr) {
    imu->slave_addr = slave_addr;
    
    // verify device
    uint8_t buf[1];

    i2c_read(imu->slave_addr, WHO_AM_I_REG, buf, 1);
    
    if (buf[0] != DEVICE_ID) {
        // wrong device: throw an error

        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_Delay(200);
        }

        return;
    }


    // Global control: BDU + auto-increment, active-high push-pull INT
    uint8_t ctrl3 = 0x44;  // IF_INC | BDU
    i2c_write(imu->slave_addr, CTRL3_C, &ctrl3, 1);

    // Mask DRDY until filters settle
    uint8_t ctrl4 = 0x08;  // DRDY_MASK
    i2c_write(imu->slave_addr, CTRL4_C, &ctrl4, 1);


    // configure IMU
    uint8_t accel_config = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(imu->slave_addr, ACCEL_CFG, &accel_config, 1);

    uint8_t gyro_config = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(imu->slave_addr, GYRO_CFG, &gyro_config, 1);

    HAL_Delay(100); // from the data sheet:   t_start = max(t_boot, 1/ODR * filter_settling_samples)

    // interrupts
    uint8_t int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(imu->slave_addr, INT1_CFG, &int1_config, 1);

    // uint8_t int2_config = INT2_GYRO_EN | INT2_ACCEL_EN;
    // i2c_write(imu->slave_addr, INT2_CFG, &int2_config, 1);
    
}

/**
 * @brief Reads all inertial data (this does not include temperature).
 * 
 * This IMU has an auto-increment feature, allowing the access all in one
 * shot of the data in a register as well as the n bytes desired after it.
 * Hence the gyro data is accessed first because it is the lowest register.
 */
void imu_read(IMU_t* imu) {
    uint8_t buf[12];

    i2c_read(imu->slave_addr, GYRO_REG, buf, 12);
    imu->gx = (int16_t)(buf[0]  | buf[1]  << 8);
    imu->gy = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gz = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->ax = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ay = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->az = (int16_t)(buf[10] | buf[11] << 8);
    imu_convert_units(imu);
}

//Reads IMU acceleration data
void imu_read_accel(IMU_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(imu->slave_addr, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | buf[1] << 8);
    imu->ay = (int16_t)(buf[2] | buf[3] << 8);
    imu->az = (int16_t)(buf[4] | buf[5] << 8);
    imu->ax_g = (float)imu->ax * 0.000244f;
    imu->ay_g = (float)imu->ay * 0.000244f;
    imu->az_g = (float)imu->az * 0.000244f;
}

// Reads IMU gyroscope data
void imu_read_gyro(IMU_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(imu->slave_addr, GYRO_REG, buf, 6);
    imu->gx = (int16_t)(buf[0] | buf[1] << 8);
    imu->gy = (int16_t)(buf[2] | buf[3] << 8);
    imu->gz = (int16_t)(buf[4] | buf[5] << 8);
    mu->gx_dps = (float)imu->gx * 0.070f;
    imu->gy_dps = (float)imu->gy * 0.070f;
    imu->gz_dps = (float)imu->gz * 0.070f;
}

// Reads IMU temperature data
// This output is raw. The conversion to celsius is:
// celsius_temp = (raw_temp / 16) + 25  (± the offset error)
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
    imu_convert_units(imu);
}

static void imu_convert_units(IMU_t* imu)
{
    const float accel_scale_g_per_lsb = 0.000244f; // ±8 g
    const float gyro_scale_dps_per_lsb = 0.070f;   // 2000 dps

    imu->ax_g = (float)imu->ax * accel_scale_g_per_lsb;
    imu->ay_g = (float)imu->ay * accel_scale_g_per_lsb;
    imu->az_g = (float)imu->az * accel_scale_g_per_lsb;

    imu->gx_dps = (float)imu->gx * gyro_scale_dps_per_lsb;
    imu->gy_dps = (float)imu->gy * gyro_scale_dps_per_lsb;
    imu->gz_dps = (float)imu->gz * gyro_scale_dps_per_lsb;
}

#else // use FAKE hardware
void imu_init(IMU_t* imu, uint8_t slave_addr) { /* do nothing */ }
void imu_read(IMU_t* imu) { /* do nothing */ }
void imu_read_accel(IMU_t* imu) { /* do nothing */ }
void imu_read_gyro(IMU_t* imu) { /* do nothing */ }
void imu_read_temp(IMU_t* imu) { /* do nothing */ }
void imu_read_all(IMU_t* imu) { /* do nothing */ }
#endif