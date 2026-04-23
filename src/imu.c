/**
IMU driver

STM LSM6DS3
LSM6DS3 can operate at 100 kHz or 400 kHz.

Data provided:
    - 3 axis acceleration
    - 3 axis angular rate
    - temperature

*/

#include "config.h"
#include "i2c.h"
#include "imu.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>

#define IMU_ADDR        0x6B  // default I2C slave address
//#define IMU_ADDR      0x6A  // another I2C slave address option
#define WHO_AM_I_REG    0x0F  // 
#define IMU_ID          0x69  // I2C device id, the value in WHO_AM_I_REG

#define ACCEL_REG       0x28
#define GYRO_REG        0x22
#define TEMP_REG        0x20

#define ACCEL_CFG       0x10
#define GYRO_CFG        0x11
#define INT1_CFG        0x0D
#define INT2_CFG        0x0E

#define CTRL3_C         0x12
#define CTRL4_C         0x13

#define ACCEL_ODR_104HZ  (0b0100 << 4)
#define ACCEL_ODR_208HZ  (0b0101 << 4)
#define ACCEL_ODR_416HZ  (0b0110 << 4)

#define ACCEL_FS_2G      (0b00 << 2)
#define ACCEL_FS_4G      (0b10 << 2)
#define ACCEL_FS_8G      (0b11 << 2)
#define ACCEL_FS_16G     (0b01 << 2)

#define ACCEL_BW_200HZ   (0b01 << 0)
#define ACCEL_BW_400HZ   (0b00 << 0)

#define GYRO_ODR_104HZ   (0b0100 << 4)
#define GYRO_ODR_208HZ   (0b0101 << 4)
#define GYRO_ODR_416HZ   (0b0110 << 4)

#define GYRO_FS_500DPS   (0b10 << 2)
#define GYRO_FS_2000DPS  (0b11 << 2)
#define GYRO_FS_125DPS   (0b1  << 1)

#define INT1_GYRO_EN     (0b1 << 1)
#define INT1_ACCEL_EN    (0b1 << 0)

#define INT2_TEMP_EN     (0b1 << 2)  // INT2_DRDY_TEMP
#define INT2_GYRO_EN     (0b1 << 1)  // INT2_DRDY_G
#define INT2_ACCEL_EN    (0b1 << 0)  // INT2_DRDY_XL


/* ––––– scaling to account for lack of FPU on STM32F0 board ––––– */
// accelerometer full-scale conversion mg/LSB --> ±32767 or 32768?
#define A_FS_2      0.061 // 2 g  =  2000 mg
#define A_FS_4      0.122
#define A_FS_8      0.244
#define A_FS_16     0.488

// gyroscope full-scale conversion mdps/LSB --> ±28571
#define G_FS_125    4.375 // 125 dps  =  125000 mdps
#define G_FS_250    8.75
#define G_FS_500    17.5
#define G_FS_1000   35
#define G_FS_2000   70


static void imu_convert_units(LSM6DS3_t* imu);

volatile uint8_t imu_ready = 0;


#if USE_IMU // use REAL hardware

/**
 * @brief LSM6DS3 configuration and initialization.
 * 
 * Hard-coded configs:
 *      - accelerometer: 416Hz data rate, ±8g full-scale, 400Hz anti-aliasing
 *      - gyroscope:     416Hz data rate, 2000 dps
 * 
 * @param imu       Pointer to IMU struct
 * @return          true if peripheral successfully initialized, false otherwise
 */
bool imu_init(LSM6DS3_t* imu) {
    // verify device
    uint8_t device_id;
    i2c_read(IMU_ADDR, WHO_AM_I_REG, &device_id, 1);
    
    if (device_id != IMU_ID) {
        // wrong device: throw an error
        while (1) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_Delay(200);
        }

        return false;
    }

    // freeze registers for read on block data update
    uint8_t ctrl3 = 0x44;  // BDU
    i2c_write(IMU_ADDR, CTRL3_C, &ctrl3, 1);

    // prevent interrupt fire at startup with data ready mask
    uint8_t ctrl4 = 0x08;  // DRDY_MASK
    i2c_write(IMU_ADDR, CTRL4_C, &ctrl4, 1);

<<<<<<< Updated upstream
    uint8_t int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(IMU_ADDR, INT1_CFG, &int1_config, 1);
=======
    // interrupts
//   uint8_t int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
//   i2c_write(IMU_ADDR, INT1_CFG, &int1_config, 1);
>>>>>>> Stashed changes

    // uint8_t int2_config = INT2_GYRO_EN | INT2_ACCEL_EN;
    // i2c_write(IMU_ADDR, INT2_CFG, &int2_config, 1);

    // configure
    uint8_t accel_config = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(IMU_ADDR, ACCEL_CFG, &accel_config, 1);

    uint8_t gyro_config = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(IMU_ADDR, GYRO_CFG, &gyro_config, 1);

//    while (!imu_ready) { }  

    return true;
}

/**
 * @brief Reads all inertial data (this does not include temperature).
 * 
 * This IMU has an auto-increment feature, allowing the access all in one
 * shot of the data in a register as well as the n bytes desired after it.
 * Hence the gyro data is accessed first because it is the lowest register.
 */
void imu_read(LSM6DS3_t* imu) {
    uint8_t buf[12];

    i2c_read(IMU_ADDR, GYRO_REG, buf, 12);
    imu->gx = (int16_t)(buf[0]  | buf[1]  << 8);
    imu->gy = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gz = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->ax = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ay = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->az = (int16_t)(buf[10] | buf[11] << 8);

    imu_convert_units(imu);
}

// Reads IMU acceleration data
void imu_read_accel(LSM6DS3_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(IMU_ADDR, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | buf[1] << 8);
    imu->ay = (int16_t)(buf[2] | buf[3] << 8);
    imu->az = (int16_t)(buf[4] | buf[5] << 8);

    imu_convert_units(imu);
}

// Reads IMU gyroscope data
void imu_read_gyro(LSM6DS3_t* imu) {
    uint8_t buf[6]; // 6 bytes: X, Y, Z (2 bytes each)

    i2c_read(IMU_ADDR, GYRO_REG, buf, 6);
    imu->gx = (int16_t)(buf[0] | buf[1] << 8);
    imu->gy = (int16_t)(buf[2] | buf[3] << 8);
    imu->gz = (int16_t)(buf[4] | buf[5] << 8);

    imu_convert_units(imu);
}

// Reads IMU temperature data
// This output is raw. The conversion to celsius is:
// celsius_temp = (raw_temp / 16) + 25  (± the offset error)
void imu_read_temp(IMU_t* imu) {
    uint8_t buf[2];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 2);
    imu->temp = (int16_t)(buf[0] | buf[1]  << 8);

    imu_convert_units(imu);
}

// Reads all IMU data (including temperature)
void imu_read_all(LSM6DS3_t* imu) {
    uint8_t buf[14];

    i2c_read(IMU_ADDR, TEMP_REG, buf, 14);
    imu->temp = (int16_t)(buf[0]  | buf[1]  << 8);
    imu->gx   = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gy   = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->gz   = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ax   = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->ay   = (int16_t)(buf[10] | buf[11] << 8);
    imu->az   = (int16_t)(buf[12] | buf[13] << 8);

    imu_convert_units(imu);
}

/**
 * This conversion is hard coded for the configs we selected. Accuracy can be
 * increased by using micro g and micro dps.
 */
static void imu_convert_units(LSM6DS3_t* imu) {
    imu->ax_mg   = (int32_t)imu->ax * 244 / 1000; // 244 for ±8g full-scale
    imu->ay_mg   = (int32_t)imu->ay * 244 / 1000;
    imu->az_mg   = (int32_t)imu->az * 244 / 1000;

    imu->gx_mdps = (int32_t)imu->gx * 70 / 1000;  // 70 for ±2000dps full-scale
    imu->gy_mdps = (int32_t)imu->gy * 70 / 1000;
    imu->gz_mdps = (int32_t)imu->gz * 70 / 1000;

    //imu->temp_c  = ((int32_t)imu->temp >> 4) + 25;
}

#else // use FAKE hardware
bool imu_init(LSM6DS3_t* imu) { return true; }
void imu_read(LSM6DS3_t* imu) { /* do nothing */ }
void imu_read_accel(LSM6DS3_t* imu) { /* do nothing */ }
void imu_read_gyro(LSM6DS3_t* imu) { /* do nothing */ }
void imu_read_temp(LSM6DS3_t* imu) { /* do nothing */ }
void imu_read_all(LSM6DS3_t* imu) { /* do nothing */ }
#endif
