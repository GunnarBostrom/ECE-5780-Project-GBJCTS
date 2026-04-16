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
#include "i2c.h"
#include "imu.h"

#include "stm32f072xb.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <sys/_intsup.h>
#include <sys/types.h>

#define WHO_AM_I_REG    0x0F
#define DEVICE_ID       0x69

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

#define INT2_TEMP_EN     (0b1 << 2)
#define INT2_GYRO_EN     (0b1 << 1)
#define INT2_ACCEL_EN    (0b1 << 0)

volatile uint8_t imu_ready = 0;

#if USE_IMU

void imu_init(IMU_t* imu, uint8_t slave_addr)
{
    uint8_t buf[1];

    imu->slave_addr = slave_addr;

    imu->ax = 0;
    imu->ay = 0;
    imu->az = 0;

    imu->gx = 0;
    imu->gy = 0;
    imu->gz = 0;

    imu->temp_raw = 0;

    imu->ax_g = 0.0f;
    imu->ay_g = 0.0f;
    imu->az_g = 0.0f;

    imu->gx_dps = 0.0f;
    imu->gy_dps = 0.0f;
    imu->gz_dps = 0.0f;

    imu->temp_c = 0.0f;

    i2c_read(imu->slave_addr, WHO_AM_I_REG, buf, 1);
    if (buf[0] != DEVICE_ID)
    {
        while (1)
        {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
            HAL_Delay(200);
        }
    }

    uint8_t ctrl3 = 0x44;
    i2c_write(imu->slave_addr, CTRL3_C, &ctrl3, 1);

    uint8_t ctrl4 = 0x08;
    i2c_write(imu->slave_addr, CTRL4_C, &ctrl4, 1);

    uint8_t accel_config = ACCEL_ODR_416HZ | ACCEL_FS_8G | ACCEL_BW_400HZ;
    i2c_write(imu->slave_addr, ACCEL_CFG, &accel_config, 1);

    uint8_t gyro_config = GYRO_ODR_416HZ | GYRO_FS_2000DPS;
    i2c_write(imu->slave_addr, GYRO_CFG, &gyro_config, 1);

    HAL_Delay(100);

    uint8_t int1_config = INT1_GYRO_EN | INT1_ACCEL_EN;
    i2c_write(imu->slave_addr, INT1_CFG, &int1_config, 1);
}

static void imu_convert_units(IMU_t* imu)
{
    const float accel_scale_g_per_lsb = 0.000244f;
    const float gyro_scale_dps_per_lsb = 0.070f;

    imu->ax_g = (float)imu->ax * accel_scale_g_per_lsb;
    imu->ay_g = (float)imu->ay * accel_scale_g_per_lsb;
    imu->az_g = (float)imu->az * accel_scale_g_per_lsb;

    imu->gx_dps = (float)imu->gx * gyro_scale_dps_per_lsb;
    imu->gy_dps = (float)imu->gy * gyro_scale_dps_per_lsb;
    imu->gz_dps = (float)imu->gz * gyro_scale_dps_per_lsb;
}

void imu_read(IMU_t* imu)
{
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

void imu_read_accel(IMU_t* imu)
{
    uint8_t buf[6];

    i2c_read(imu->slave_addr, ACCEL_REG, buf, 6);
    imu->ax = (int16_t)(buf[0] | buf[1] << 8);
    imu->ay = (int16_t)(buf[2] | buf[3] << 8);
    imu->az = (int16_t)(buf[4] | buf[5] << 8);
    imu->ax_g = (float)imu->ax * 0.000244f;
    imu->ay_g = (float)imu->ay * 0.000244f;
    imu->az_g = (float)imu->az * 0.000244f;
}

void imu_read_gyro(IMU_t* imu)
{
    uint8_t buf[6];

    i2c_read(imu->slave_addr, GYRO_REG, buf, 6);
    imu->gx = (int16_t)(buf[0] | buf[1] << 8);
    imu->gy = (int16_t)(buf[2] | buf[3] << 8);
    imu->gz = (int16_t)(buf[4] | buf[5] << 8);
    imu->gx_dps = (float)imu->gx * 0.070f;
    imu->gy_dps = (float)imu->gy * 0.070f;
    imu->gz_dps = (float)imu->gz * 0.070f;
}

void imu_read_temp(IMU_t* imu)
{
    uint8_t buf[2];

    i2c_read(imu->slave_addr, TEMP_REG, buf, 2);
    imu->temp_raw = (int16_t)(buf[0] | (buf[1] << 8));
}

void imu_read_all(IMU_t* imu)
{
    uint8_t buf[14];

    i2c_read(imu->slave_addr, TEMP_REG, buf, 14);
    imu->temp_raw = (int16_t)(buf[0] | (buf[1] << 8));
    imu->gx = (int16_t)(buf[2]  | buf[3]  << 8);
    imu->gy = (int16_t)(buf[4]  | buf[5]  << 8);
    imu->gz = (int16_t)(buf[6]  | buf[7]  << 8);
    imu->ax = (int16_t)(buf[8]  | buf[9]  << 8);
    imu->ay = (int16_t)(buf[10] | buf[11] << 8);
    imu->az = (int16_t)(buf[12] | buf[13] << 8);
    imu_convert_units(imu);
}

#else
void imu_init(IMU_t* imu, uint8_t slave_addr) { (void)imu; (void)slave_addr; }
void imu_read(IMU_t* imu) { (void)imu; }
void imu_read_accel(IMU_t* imu) { (void)imu; }
void imu_read_gyro(IMU_t* imu) { (void)imu; }
void imu_read_temp(IMU_t* imu) { (void)imu; }
void imu_read_all(IMU_t* imu) { (void)imu; }
#endif
